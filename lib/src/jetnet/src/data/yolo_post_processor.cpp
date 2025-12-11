#include "yolo_post_processor.h"
#include <cmath>
#include <thread>

using namespace jetnet;
using namespace nvinfer1;

YoloPostProcessor::YoloPostProcessor(std::string input_blob_name,
                        Type network_type,
                        std::vector<OutputSpec> output_specs,
                        float thresh,
                        std::shared_ptr<Logger> logger,
                        NmsFunction nms,
                        bool relative_coords) :
    m_input_blob_name(input_blob_name),
    m_network_type(network_type),
    m_output_specs(output_specs),
    m_thresh(thresh),
    m_logger(logger),
    m_nms(nms),
    m_relative_coords(relative_coords)
{
}

bool YoloPostProcessor::OutputSpecInt::init(const OutputSpec& in, const ICudaEngine* engine, YoloPostProcessor& parent)
{
    blob_index = engine->getBindingIndex(in.name.c_str());
    Dims network_dims = engine->getBindingDimensions(blob_index);

    // CHW order
    c = network_dims.d[0];
    h = network_dims.d[1];
    w = network_dims.d[2];

    classes = in.num_classes;
    anchors = in.anchor_priors.size() >> 1;
    coords = (c / anchors) - classes - 1;

    if (c != (coords + 1 + classes) * anchors) {
        parent.m_logger->log(ILogger::Severity::kERROR, "Network output " + in.name + " has " + std::to_string(c) +
                      " channels. This number must equal (1 + coords + classes ) * anchors. Coords = " +
                      std::to_string(coords) + ", classes = " + std::to_string(classes) + ", anchors = " +
                      std::to_string(anchors));
        return false;
    }

    anchor_priors = in.anchor_priors;

    return true;
}

bool YoloPostProcessor::init(const ICudaEngine* engine)
{
    Dims network_input_dims = engine->getBindingDimensions(engine->getBindingIndex(m_input_blob_name.c_str()));

    // CHW order
    m_net_in_w = network_input_dims.d[2];
    m_net_in_h = network_input_dims.d[1];

    switch(m_network_type) {
        case Type::Yolov2:
            m_calc_box_size = [](float x, float prior, float in_size, float out_size)
                                {
                                    (void)in_size;
                                    return (exp(x) * prior) / out_size;
                                };
            break;
        case Type::Yolov3:
            // for yolov3 (tiny), also the width/height output maps are sigmoid applied. To get the final
            // width/height values we need exp(x) where x is the output of the last conv layer (without sigmoid)
            // we can prove that exp(x) = s(x) / (1 - s(x)) where s is the sigmoid operator
            m_calc_box_size = [](float x, float prior, float in_size, float out_size)
                                {
                                    (void)out_size;
                                    return (prior * x) / ((1.0 - x) * in_size);
                                };
            break;
        default:
            m_logger->log(ILogger::Severity::kERROR, "Unknown network type");
            return false;
    }

    for (auto& spec : m_output_specs) {
        OutputSpecInt spec_int;
        if (!spec_int.init(spec, engine, *this))
            return false;

        m_output_specs_int.push_back(spec_int);
    }

    return true;
}

bool YoloPostProcessor::operator()(const std::map<int, GpuBlob>& output_blobs, const std::vector<cv::Size>& image_sizes)
{
    std::vector<std::thread> threads(image_sizes.size());

    // download blob data to CPU for every network output (each blob contains data for multiple batches)
    for (auto& output_spec_int : m_output_specs_int) {
        const int index = output_spec_int.blob_index;

        // create index key if it does not exists
        if (m_cpu_blobs.find(index) == m_cpu_blobs.end()) {
            m_cpu_blobs.insert(std::pair<int, std::vector<float>>(index, std::vector<float>()));
        }

        output_blobs.at(index).download(m_cpu_blobs.at(index));
    }

    if (m_detections.size() != image_sizes.size())
        m_detections.resize(image_sizes.size());

    // start a thread per image
    if (1 == image_sizes.size()) {
        this->process(image_sizes, 0);
    } else {
        for (size_t batch=0; batch<image_sizes.size(); ++batch) {
            threads[batch] = std::thread([=, &image_sizes]()
                                         { this->process(image_sizes, batch); });
        }

        // wait for all threads to complete
        for (size_t batch=0; batch<image_sizes.size(); ++batch) {
            threads[batch].join();
        }
    }

    return true;
}

std::vector<std::vector<Detection>> YoloPostProcessor::get_detections()
{
    return m_detections;
}

void YoloPostProcessor::process(const std::vector<cv::Size>& image_sizes, int batch)
{
    std::vector<Detection> detections;

    // iterate over all network outputs and retrieve there results
    for (auto& output_spec_int : m_output_specs_int) {
        const int out_batch_step = output_spec_int.w * output_spec_int.h * output_spec_int.c;

        calc_detections(&m_cpu_blobs.at(output_spec_int.blob_index).data()[batch * out_batch_step],
                       image_sizes[batch].width, image_sizes[batch].height, output_spec_int, detections);
    }

    if (m_nms)
        m_nms(detections);

    m_detections[batch] = detections;
}

void YoloPostProcessor::calc_detections(const float* input, int image_w, int image_h, const OutputSpecInt& net_out,
                                              std::vector<Detection>& detections)
{
    int x, y, anchor, cls;
    int new_w=0;
    int new_h=0;
    const int out_channel_step = net_out.w * net_out.h;
    float scale_w = image_w;
    float scale_h = image_h;

    // calculate image width/height that the image must have to fit the network while keeping the aspect ratio fixed
    if ((m_net_in_w * image_h) < (m_net_in_h * image_w)) {
        new_w = m_net_in_w;
        new_h = (image_h * m_net_in_w)/image_w;
    } else {
        new_h = m_net_in_h;
        new_w = (image_w * m_net_in_h)/image_h;
    }

    if (m_relative_coords)
        scale_w = scale_h = 1.0;

    for (anchor=0; anchor<net_out.anchors; ++anchor) {
        const int anchor_index = anchor * out_channel_step * (1 + net_out.coords + net_out.classes);
        for (y=0; y<net_out.h; ++y) {
            const int row_index = y * net_out.w + anchor_index;
            for (x=0; x<net_out.w; ++x) {
                const int index = x + row_index;

                // extract objectness
                const float objectness = input[index + net_out.coords * out_channel_step];

                // extract class probs if objectness > threshold
                if (objectness <= m_thresh)
                    continue;

                Detection detection;

                // extract probs for each class label. Probs below the threshold are set to 0
                bool probs_above_thresh = false;
                detection.probabilities.resize(net_out.classes);
                for (cls=0; cls < net_out.classes; ++cls) {
                    float prob = objectness * input[index + (1 + net_out.coords + cls) * out_channel_step];
                    if (prob > m_thresh) {
                        probs_above_thresh = true;
                    } else {
                        prob = 0;
                    }
                    detection.probabilities[cls] = prob;
                }

                // stop early if no class prob is higher than the threshold
                if (!probs_above_thresh)
                    continue;

                // extract box
                cv::Rect2f bbox;
                bbox.x = (x + input[index]) / net_out.w;
                bbox.y = (y + input[index + out_channel_step]) / net_out.h;
                bbox.width = m_calc_box_size(input[index + 2 * out_channel_step],
                                             net_out.anchor_priors[2 * anchor],
                                             m_net_in_w, net_out.w);
                bbox.height = m_calc_box_size(input[index + 3 * out_channel_step],
                                              net_out.anchor_priors[2 * anchor + 1],
                                              m_net_in_h, net_out.h);

                // transform bbox network coords to input image coordinates
                bbox.x = ((bbox.x * m_net_in_w - (m_net_in_w - new_w) / 2.0) * scale_w) / new_w;
                bbox.y = ((bbox.y * m_net_in_h - (m_net_in_h - new_h) / 2.0) * scale_h) / new_h;
                bbox.width  *= (m_net_in_w * scale_w) / new_w;
                bbox.height *= (m_net_in_h * scale_h) / new_h;

                // clip bboxes to image boundaries and convert x,y to top left corner
                float xmin = bbox.x - bbox.width/2.;
                float xmax = bbox.x + bbox.width/2.;
                float ymin = bbox.y - bbox.height/2.;
                float ymax = bbox.y + bbox.height/2.;

                if (xmin < 0) xmin = 0;
                if (ymin < 0) ymin = 0;
                if (xmax > scale_w) xmax = scale_w;
                if (ymax > scale_h) ymax = scale_h;

                detection.bbox.x = xmin;
                detection.bbox.y = ymin;
                detection.bbox.width = xmax - xmin;
                detection.bbox.height = ymax - ymin;

                // add detection to detection list
                detections.push_back(detection);
            }
        }
    }
}
