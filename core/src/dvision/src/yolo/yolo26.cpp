#include "dvision/yolo/yolo26.hpp"
#include "dvision/yolo/utils.hpp"

namespace dvision {

static Logger gLogger;

void yolo26::init(std::string engine_name, float threshold_, float nms_threshold_, int batch_size_)
{
    threshold = threshold_;
    nms_threshold = nms_threshold_;
    batch_size = batch_size_;

    data = new float[batch_size * 3 * INPUT_H * INPUT_W];

    std::ifstream file(engine_name, std::ios::binary);
    if (!file.good()) {
        std::cerr << "yolo26: read " << engine_name << " error!" << std::endl;
        return;
    }
    char *trtModelStream = nullptr;
    size_t size = 0;
    file.seekg(0, file.end);
    size = file.tellg();
    file.seekg(0, file.beg);
    trtModelStream = new char[size];
    assert(trtModelStream);
    file.read(trtModelStream, size);
    file.close();

    runtime = createInferRuntime(gLogger);
    assert(runtime != nullptr);
    engine = runtime->deserializeCudaEngine(trtModelStream, size);
    assert(engine != nullptr);
    context = engine->createExecutionContext();
    assert(context != nullptr);
    delete[] trtModelStream;

    // Query I/O bindings dynamically
    assert(engine->getNbBindings() == 2);
    inputIndex = engine->getBindingIndex("images");
    outputIndex = engine->getBindingIndex("output0");

    if (inputIndex < 0 || outputIndex < 0) {
        // fallback: use binding 0 as input, 1 as output
        inputIndex = 0;
        outputIndex = 1;
    }

    CUDA_CHECK(cudaMalloc(&buffers[inputIndex], batch_size * 3 * INPUT_H * INPUT_W * sizeof(float)));
    // Output: [batch, 12, 8400] = [batch, 4+CLASS_NUM, NUM_ANCHORS]
    CUDA_CHECK(cudaMalloc(&buffers[outputIndex], batch_size * (CLASS_NUM + 4) * NUM_ANCHORS * sizeof(float)));

    prob = new float[batch_size * (CLASS_NUM + 4) * NUM_ANCHORS];

    CUDA_CHECK(cudaStreamCreate(&stream));
}

yolo26::~yolo26()
{
    if (data) delete[] data;
    if (prob) delete[] prob;
    if (buffers[0]) cudaFree(buffers[0]);
    if (buffers[1]) cudaFree(buffers[1]);
    if (context) context->destroy();
    if (engine) engine->destroy();
    if (runtime) runtime->destroy();
    cudaStreamDestroy(stream);
}

void yolo26::doInference(IExecutionContext& context, cudaStream_t& stream, void **buffers, float* input, float* output, int batchSize)
{
    CUDA_CHECK(cudaMemcpyAsync(buffers[inputIndex], input, batchSize * 3 * INPUT_H * INPUT_W * sizeof(float), cudaMemcpyHostToDevice, stream));
    context.enqueue(batchSize, buffers, stream, nullptr);
    CUDA_CHECK(cudaMemcpyAsync(output, buffers[outputIndex], batchSize * (CLASS_NUM + 4) * NUM_ANCHORS * sizeof(float), cudaMemcpyDeviceToHost, stream));
    cudaStreamSynchronize(stream);
}

std::vector<Yolo::Detection> yolo26::decodeOutput(const float* output, int img_w, int img_h)
{
    std::vector<Yolo::Detection> detections;

    // Compute letterbox scale/offset (same as preprocess_img)
    float r_w = INPUT_W / (img_w * 1.0f);
    float r_h = INPUT_H / (img_h * 1.0f);
    float scale = std::min(r_w, r_h);
    float pad_x = (INPUT_W - img_w * scale) / 2.0f;
    float pad_y = (INPUT_H - img_h * scale) / 2.0f;

    for (int i = 0; i < NUM_ANCHORS; i++) {
        // YOLOv8 output: [bs, 4+nc, anchors], channel-first
        // For anchor i:
        //   cx = output[0 * NUM_ANCHORS + i]
        //   cy = output[1 * NUM_ANCHORS + i]
        //   w  = output[2 * NUM_ANCHORS + i]
        //   h  = output[3 * NUM_ANCHORS + i]
        //   class scores = output[(4 + c) * NUM_ANCHORS + i]

        float cx = output[0 * NUM_ANCHORS + i];
        float cy = output[1 * NUM_ANCHORS + i];
        float w  = output[2 * NUM_ANCHORS + i];
        float h  = output[3 * NUM_ANCHORS + i];

        // Find max class score (model output already has sigmoid applied)
        float max_score = 0.0f;
        int best_class = -1;
        for (int j = 0; j < CLASS_NUM; j++) {
            float score = output[(4 + j) * NUM_ANCHORS + i];
            if (score > max_score) {
                max_score = score;
                best_class = j;
            }
        }

        if (max_score < threshold || best_class < 0)
            continue;

        // Convert from letterbox coordinates back to original image coordinates
        float x1 = (cx - w / 2.0f - pad_x) / scale;
        float y1 = (cy - h / 2.0f - pad_y) / scale;
        float x2 = (cx + w / 2.0f - pad_x) / scale;
        float y2 = (cy + h / 2.0f - pad_y) / scale;

        // Clip to image bounds
        x1 = std::max(0.0f, std::min(x1, (float)img_w - 1.0f));
        y1 = std::max(0.0f, std::min(y1, (float)img_h - 1.0f));
        x2 = std::max(0.0f, std::min(x2, (float)img_w - 1.0f));
        y2 = std::max(0.0f, std::min(y2, (float)img_h - 1.0f));

        // Filter invalid boxes
        if (x2 <= x1 || y2 <= y1)
            continue;

        Yolo::Detection det;
        // Store as cx, cy, w, h in original image space (for compatibility with downstream code)
        float out_cx = (x1 + x2) / 2.0f;
        float out_cy = (y1 + y2) / 2.0f;
        float out_w = x2 - x1;
        float out_h = y2 - y1;
        det.bbox[0] = out_cx;
        det.bbox[1] = out_cy;
        det.bbox[2] = out_w;
        det.bbox[3] = out_h;
        det.conf = max_score;
        det.class_id = static_cast<float>(best_class);

        detections.push_back(det);
    }

    // Apply NMS
    // Sort by confidence descending
    std::sort(detections.begin(), detections.end(),
        [](const Yolo::Detection& a, const Yolo::Detection& b) {
            return a.conf > b.conf;
        });

    std::vector<bool> suppressed(detections.size(), false);
    std::vector<Yolo::Detection> result;

    for (size_t i = 0; i < detections.size(); i++) {
        if (suppressed[i]) continue;
        result.push_back(detections[i]);

        float ix = detections[i].bbox[0];
        float iy = detections[i].bbox[1];
        float iw = detections[i].bbox[2];
        float ih = detections[i].bbox[3];
        float i_area = iw * ih;

        for (size_t j = i + 1; j < detections.size(); j++) {
            if (suppressed[j]) continue;
            if (detections[j].class_id != detections[i].class_id) continue;

            float jx = detections[j].bbox[0];
            float jy = detections[j].bbox[1];
            float jw = detections[j].bbox[2];
            float jh = detections[j].bbox[3];

            float inter_x1 = std::max(ix - iw / 2.0f, jx - jw / 2.0f);
            float inter_y1 = std::max(iy - ih / 2.0f, jy - jh / 2.0f);
            float inter_x2 = std::min(ix + iw / 2.0f, jx + jw / 2.0f);
            float inter_y2 = std::min(iy + ih / 2.0f, jy + jh / 2.0f);

            if (inter_x2 <= inter_x1 || inter_y2 <= inter_y1) continue;

            float inter_area = (inter_x2 - inter_x1) * (inter_y2 - inter_y1);
            float j_area = jw * jh;
            float iou = inter_area / (i_area + j_area - inter_area);

            if (iou > nms_threshold) {
                suppressed[j] = true;
            }
        }
    }

    return result;
}

std::vector<std::vector<Yolo::Detection>> yolo26::detect(std::vector<cv::Mat> images)
{
    int fcount = std::min((int)images.size(), batch_size);

    // Preprocess: letterbox + normalize (keep BGR as model expects)
    for (int b = 0; b < fcount; b++) {
        cv::Mat img = images[b];

        // Handle stereo camera: if image is very wide (e.g. 2560x720),
        // crop to right half (single camera view at 1280x720)
        if (img.cols > img.rows * 2) {
            int half_w = img.cols / 2;
            std::cout << "yolo26: cropping stereo " << img.cols << "x" << img.rows
                      << " -> right half " << half_w << "x" << img.rows << std::endl;
            img = img(cv::Rect(half_w, 0, half_w, img.rows)).clone();
            images[b] = img;
        }

        cv::Mat pr_img = preprocess_img(img, INPUT_W, INPUT_H);

        int i = 0;
        for (int row = 0; row < INPUT_H; ++row) {
            uchar* uc_pixel = pr_img.data + row * pr_img.step;
            for (int col = 0; col < INPUT_W; ++col) {
                data[b * 3 * INPUT_H * INPUT_W + i] = (float)uc_pixel[0] / 255.0;
                data[b * 3 * INPUT_H * INPUT_W + i + INPUT_H * INPUT_W] = (float)uc_pixel[1] / 255.0;
                data[b * 3 * INPUT_H * INPUT_W + i + 2 * INPUT_H * INPUT_W] = (float)uc_pixel[2] / 255.0;
                uc_pixel += 3;
                ++i;
            }
        }
    }

    // Inference (use fcount, not batch_size, to avoid sending uninitialized data)
    doInference(*context, stream, buffers, data, prob, fcount);

    // Post-process each image in batch
    std::vector<std::vector<Yolo::Detection>> batch_res;
    for (int b = 0; b < fcount; b++) {
        cv::Mat img = images[b];
        float* output_ptr = prob + b * (CLASS_NUM + 4) * NUM_ANCHORS;
        auto detections = decodeOutput(output_ptr, img.cols, img.rows);
        batch_res.push_back(detections);
    }

    return batch_res;
}

}
