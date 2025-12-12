#include "dvision/yolo/yolov5.hpp"

namespace dvision {

// stuff we know about the network and the input/output blobs

static const int INPUT_H = Yolo::INPUT_H;
static const int INPUT_W = Yolo::INPUT_W;
static const int CLASS_NUM = Yolo::CLASS_NUM;
static const int OUTPUT_SIZE = Yolo::MAX_OUTPUT_BBOX_COUNT * sizeof(Yolo::Detection) / sizeof(float) + 1;  // we assume the yololayer outputs no more than MAX_OUTPUT_BBOX_COUNT boxes that conf >= 0.1
const char* INPUT_BLOB_NAME = "data";
const char* OUTPUT_BLOB_NAME = "prob";
static Logger gLogger;

float yolov5::data[BATCH_SIZE * 3 * Yolo::INPUT_H * Yolo::INPUT_W];

void yolov5::init(std::string engine_name, float threshold_, float nms_threshold_, int batch_size_)
{
    threshold=threshold_;
    nms_threshold=nms_threshold_;
    batch_size=batch_size_;
    std::cout<<sizeof(data)<<std::endl;
    std::ifstream file(engine_name, std::ios::binary);
    if (!file.good()) {
        std::cerr <<"yolov5 "<< "read " << engine_name << " error!" << std::endl;
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
    assert(engine->getNbBindings() == 2);

    // In order to bind the buffers, we need to know the names of the input and output tensors.
    // Note that indices are guaranteed to be less than IEngine::getNbBindings()
    const int inputIndex = engine->getBindingIndex(INPUT_BLOB_NAME);
    const int outputIndex = engine->getBindingIndex(OUTPUT_BLOB_NAME);
    assert(inputIndex == 0);
    assert(outputIndex == 1);
    // Create GPU buffers on device
    CUDA_CHECK(cudaMalloc(&buffers[inputIndex], batch_size * 3 * INPUT_H * INPUT_W * sizeof(float)));
    CUDA_CHECK(cudaMalloc(&buffers[outputIndex], batch_size * OUTPUT_SIZE * sizeof(float)));
    // Create stream
    CUDA_CHECK(cudaStreamCreate(&stream));
}

yolov5::~yolov5()
{
    cudaStreamDestroy(stream);
}

void yolov5::doInference(IExecutionContext& context, cudaStream_t& stream, void **buffers, float* input, float* output, int batchSize) {
    // DMA input batch data to device, infer on the batch asynchronously, and DMA output back to host
    CUDA_CHECK(cudaMemcpyAsync(buffers[0], input, batchSize * 3 * INPUT_H * INPUT_W * sizeof(float), cudaMemcpyHostToDevice, stream));
    context.enqueue(batchSize, buffers, stream, nullptr);
    CUDA_CHECK(cudaMemcpyAsync(output, buffers[1], batchSize * OUTPUT_SIZE * sizeof(float), cudaMemcpyDeviceToHost, stream));
    cudaStreamSynchronize(stream);
}

std::vector<std::vector<Yolo::Detection>> yolov5::detect(std::vector<cv::Mat> images){
    int fcount = MIN(images.size(),batch_size);
    
    for(int b = 0; b < fcount; b++){
        cv::Mat img = images[b];
        cv::Mat pr_img = preprocess_img(img, INPUT_W, INPUT_H); // letterbox BGR to RGB
        int i = 0;
        for (int row = 0; row < INPUT_H; ++row) {
            uchar* uc_pixel = pr_img.data + row * pr_img.step;
            for (int col = 0; col < INPUT_W; ++col) {
                data[b * 3 * INPUT_H * INPUT_W + i] = (float)uc_pixel[2] / 255.0;
                data[b * 3 * INPUT_H * INPUT_W + i + INPUT_H * INPUT_W] = (float)uc_pixel[1] / 255.0;
                data[b * 3 * INPUT_H * INPUT_W + i + 2 * INPUT_H * INPUT_W] = (float)uc_pixel[0] / 255.0;
                uc_pixel += 3;
                ++i;
            }
        }
    }
    
    // Run inference
    doInference(*context, stream, buffers, data, prob, batch_size);
    batch_res.clear();
    for (int b = 0; b < fcount; b++) {
        std::vector<Yolo::Detection> res;
        nms(res, &prob[b * OUTPUT_SIZE], threshold, nms_threshold);

        float offset[2]; // x y
        cv::Mat img = images[b];
        int w, h;
        float r_w = INPUT_W / (img.cols*1.0);
        float r_h = INPUT_H / (img.rows*1.0);
        if (r_h > r_w) {
            w = INPUT_W;
            h = r_w * img.rows;
            offset[0] = 0;
            offset[1] = (INPUT_H  - h) / 2;
        } else {
            w = r_h * img.cols;
            h = INPUT_H ;
            offset[0] = (INPUT_W - w) / 2;
            offset[1] = 0;
        }
        for (auto& detection : res){
            detection.bbox[0] = detection.bbox[0] - offset[0];
            detection.bbox[1] = detection.bbox[1] - offset[1];
        }

        // std:: cout << offset[0] << "-----" << offset[1] << std::endl;
        batch_res.push_back(res);
    }
    return batch_res;
}

}//dvision namespace