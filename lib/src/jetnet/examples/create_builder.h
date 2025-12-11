#ifndef JETNET_CREATE_BUILDER_H
#define JETNET_CREATE_BUILDER_H

#include "jetnet.h"
#include <memory>

class DarknetBuilderFactory
{
    int m_input_width;
    int m_input_height;
    int m_num_classes;

public:

    DarknetBuilderFactory(int input_width, int input_height, int num_classes) :
        m_input_width(input_width),
        m_input_height(input_height),
        m_num_classes(num_classes)
    {
    }

    std::shared_ptr<jetnet::ModelBuilder> create(std::string model_name, std::string weights_file)
    {
        // yolov2
        if (model_name == "yolov2" || model_name == "yolov2_leaky_plugin")
            return create_yolov2<jetnet::LeakyReluPlugin>(weights_file);
        if (model_name == "yolov2_leaky_native")
            return create_yolov2<jetnet::LeakyReluNative>(weights_file);
        if (model_name == "yolov2_relu")
            return create_yolov2<jetnet::Relu>(weights_file);

        // yolov3
        if (model_name == "yolov3" || model_name == "yolov3_leaky_plugin")
            return create_yolov3<jetnet::LeakyReluPlugin>(weights_file);
        if (model_name == "yolov3_leaky_native")
            return create_yolov3<jetnet::LeakyReluNative>(weights_file);
        if (model_name == "yolov3_relu")
            return create_yolov3<jetnet::Relu>(weights_file);

        // yolov3_tiny
        if (model_name == "yolov3_tiny" || model_name == "yolov3_tiny_leaky_plugin")
            return create_yolov3_tiny<jetnet::LeakyReluPlugin>(weights_file);
        if (model_name == "yolov3_tiny_leaky_native")
            return create_yolov3_tiny<jetnet::LeakyReluNative>(weights_file);
        if (model_name == "yolov3_tiny_relu")
            return create_yolov3_tiny<jetnet::Relu>(weights_file);

        std::cerr << "Error: unknown model type " << model_name << std::endl;
        return nullptr;
    }

    template<typename T>
    std::shared_ptr<jetnet::ModelBuilder> create_yolov2(std::string weights_file)
    {
        const std::string input_blob_name = "data";
        const std::string output_blob_name = "probs";

        return std::make_shared<jetnet::Yolov2Builder<T>>(input_blob_name, output_blob_name, weights_file,
                                                nvinfer1::DimsCHW{3, m_input_height, m_input_width}, 5, 80);
    }

    template<typename T>
    std::shared_ptr<jetnet::ModelBuilder> create_yolov3(std::string weights_file)
    {
        const std::string input_blob_name = "data";
        const std::string output_blob1_name = "probs1";
        const std::string output_blob2_name = "probs2";
        const std::string output_blob3_name = "probs3";

        const int num_anchors = 3;
        const int num_classes = m_num_classes;
        const int num_coords = 4;

        typename jetnet::Yolov3Builder<T>::OutputSpec outspec_large{output_blob1_name,  num_anchors, num_classes, num_coords};
        typename jetnet::Yolov3Builder<T>::OutputSpec outspec_mid{output_blob2_name,    num_anchors, num_classes, num_coords};
        typename jetnet::Yolov3Builder<T>::OutputSpec outspec_small{output_blob3_name,  num_anchors, num_classes, num_coords};

        return std::make_shared<jetnet::Yolov3Builder<T>>(input_blob_name, weights_file, nvinfer1::DimsCHW{3,
                                                m_input_height, m_input_width}, outspec_large, outspec_mid, outspec_small);
    }

    template<typename T>
    std::shared_ptr<jetnet::ModelBuilder> create_yolov3_tiny(std::string weights_file)
    {
        const std::string input_blob_name = "data";
        const std::string output_blob1_name = "probs1";
        const std::string output_blob2_name = "probs2";

        const int num_anchors = 3;
        const int num_classes = m_num_classes;
        const int num_coords = 4;

        typename jetnet::Yolov3TinyBuilder<T>::OutputSpec outspec_large{output_blob1_name,  num_anchors, num_classes, num_coords};
        typename jetnet::Yolov3TinyBuilder<T>::OutputSpec outspec_small{output_blob2_name,  num_anchors, num_classes, num_coords};

        return std::make_shared<jetnet::Yolov3TinyBuilder<T>>(input_blob_name, weights_file, nvinfer1::DimsCHW{3,
                                                m_input_height, m_input_width}, outspec_large, outspec_small);
    }
};

#endif /* JETNET_CREATE_BUILDER_H */
