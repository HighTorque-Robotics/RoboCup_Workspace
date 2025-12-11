#ifndef JETNET_YOLOV2_BUILDER_H
#define JETNET_YOLOV2_BUILDER_H

#include "model_builder.h"
#include "darknet_weights_loader.h"
#include "conv2d_batch_leaky.h"
#include "fp16.h"
#include <NvInferPlugin.h>
#include <NvInfer.h>
#include <memory>
#include <string>

namespace jetnet
{

template<typename TActivation>
class Yolov2Builder : public ModelBuilder
{
public:
    /*
     *  Determine network properties using the constructor
     *  input_blob_name:    name of the input tensor. The same name must be used in the runner
     *                      to identify the input tensor
     *  output_blob_name:   name of the output tensor. The same name must be used in the runner
     *                      to identify the output tensor
     *  weightsfile:        darknet weights file
     *  input_dimenstions:  network input dimensions (num channels, height, width)
     *  num_anchors:        number of anchors the network should support
     *  num_classes:        number of classes the network should be able to produce
     *  num_coords:         number of coordinates the network should be able to produce
     */
    Yolov2Builder(std::string input_blob_name,
                  std::string output_blob_name,
                  std::string weightsfile,
                  nvinfer1::DimsCHW input_dimenstions,
                  int num_anchors,
                  int num_classes,
                  int num_coords=4) :
        m_input_blob_name(input_blob_name),
        m_output_blob_name(output_blob_name),
        m_weightsfile(weightsfile),
        m_input_dimensions(input_dimenstions),
        m_num_anchors(num_anchors),
        m_num_classes(num_classes),
        m_num_coords(num_coords) {}

    nvinfer1::INetworkDefinition* parse(nvinfer1::DataType dt) override;

private:
    /*
     *  Network configuration
     */
    std::string m_input_blob_name;
    std::string m_output_blob_name;
    std::string m_weightsfile;
    nvinfer1::DimsCHW m_input_dimensions;
    int m_num_anchors;
    int m_num_classes;
    int m_num_coords;

    /*
     *  layer states (weights, layers and plugins)
     */
    std::unique_ptr<DarknetWeightsLoader> m_weights;
    Conv2dBatchLeaky<TActivation> m_convs[22];

    void (*nvPluginDeleter)(nvinfer1::plugin::INvPlugin*){[](nvinfer1::plugin::INvPlugin* ptr) { ptr->destroy(); }};
    std::unique_ptr<nvinfer1::plugin::INvPlugin, decltype(nvPluginDeleter)> m_reorg_plugin{nullptr, nvPluginDeleter};
    std::unique_ptr<nvinfer1::plugin::INvPlugin, decltype(nvPluginDeleter)> m_region_plugin{nullptr, nvPluginDeleter};
};

}

#endif /* JETNET_YOLOV2_BUILDER_H */
