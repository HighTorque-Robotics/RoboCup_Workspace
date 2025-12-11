#ifndef JETNET_YOLO_POST_PROCESSOR_H
#define JETNET_YOLO_POST_PROCESSOR_H

#include "logger.h"
#include "gpu_blob.h"
#include "nms.h"
#include "detection.h"
#include <NvInfer.h>
#include <functional>
#include <memory>
#include <vector>
#include <string>

namespace jetnet
{

class YoloPostProcessor
{
public:
    enum class Type {
        Yolov2,
        Yolov3
    };

    struct OutputSpec
    {
        std::string name;
        std::vector<float> anchor_priors;       // anchor prior pairs. Its length is used to determine the number of anchors
        size_t num_classes;
    };

    /*
     *  input_blob_name:        name of the input tensor. Needed to know the input dimensions of the network
     *  output_spec:            A list of network output specifications. Each network output needs one spec. For yolov2,
     *                          only one spec is expected. For yolov3, three specs are expected.
     *  thresh:                 detection threshold
     *  logger:                 logger object
     *  nms:                    non maxima suppression function
     *  relative_coords:        Return detections using relative coordinates (between 0.0 and 1.0 for x, y, width and height)
     */
    YoloPostProcessor(std::string input_blob_name,
                        Type network_type,
                        std::vector<OutputSpec> output_specs,
                        float thresh,
                        std::shared_ptr<Logger> logger,
                        NmsFunction nms,
                        bool relative_coords=false);

    /*
     *  Called after the network is deserialized
     *  engine:         containes the deserialized cuda engine
     *  returns true on success, false on failure
     */
    bool init(const nvinfer1::ICudaEngine* engine);

    /* used by model runner template to know the type of the second argument of the operator() function of this class */
    typedef std::vector<cv::Size> Arg;

    /*
     *  Actual post-processing
     *  images:         list of processed images that can be used to draw detection results onto
     *  output_blobs:   output data from the network that needs post-processing
     *  returns true on success, false on failure
     */
    bool operator()(const std::map<int, GpuBlob>& output_blobs, const std::vector<cv::Size>& image_sizes);

    /*
     *  Get last batch of postprocessed detections
     */
    std::vector<std::vector<Detection>> get_detections();

private:
    // internal output specifications structure
    struct OutputSpecInt
    {
        bool init(const OutputSpec& in, const nvinfer1::ICudaEngine* engine, YoloPostProcessor& parent);

        int w;                                  // network output width
        int h;                                  // network output height
        int c;                                  // network output channels
        int coords;                             // number of network coordinates
        int classes;                            // number of network classes
        int anchors;                            // number of anchor boxes
        int blob_index;                         // network output blob index
        std::vector<float> anchor_priors;
    };

    void process(const std::vector<cv::Size>& image_sizes, int batch);
    void calc_detections(const float* input, int image_w, int image_h, const OutputSpecInt& net_out,
                                              std::vector<Detection>& detections);

    std::string m_input_blob_name;
    Type m_network_type;
    std::vector<OutputSpec> m_output_specs;
    float m_thresh;
    std::shared_ptr<Logger> m_logger;
    NmsFunction m_nms;
    bool m_relative_coords;

    int m_net_in_w;
    int m_net_in_h;
    std::vector<OutputSpecInt> m_output_specs_int;
    std::function<float(float, float, float, float)> m_calc_box_size;

    std::map<int, std::vector<float>> m_cpu_blobs;
    std::vector<std::vector<Detection>> m_detections;
};

}

#endif /* JETNET_YOLO_POST_PROCESSOR_H */
