#ifndef JETNET_LEAKY_RELU_PLUGIN_H
#define JETNET_LEAKY_RELU_PLUGIN_H

#include <NvInfer.h>
#include <NvInferPlugin.h>
#include <memory>

namespace jetnet
{

class LeakyReluPlugin
{
public:
    nvinfer1::ILayer* operator()(std::string name, nvinfer1::INetworkDefinition* network, nvinfer1::ITensor& input,
                               float negSlope, nvinfer1::DataType dt);

private:
    void (*nvPluginDeleter)(nvinfer1::plugin::INvPlugin*){[](nvinfer1::plugin::INvPlugin* ptr) { ptr->destroy(); }};
    std::unique_ptr<nvinfer1::plugin::INvPlugin, decltype(nvPluginDeleter)> m_plugin{nullptr, nvPluginDeleter};
};

}

#endif /* JETNET_LEAKY_RELU_PLUGIN_H */
