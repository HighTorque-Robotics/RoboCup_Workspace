#include "leaky_relu_plugin.h"

using namespace jetnet;
using namespace nvinfer1;

ILayer* LeakyReluPlugin::operator()(std::string name, INetworkDefinition* network, ITensor& input, float negSlope, DataType dt)
{
    (void)dt;
    // Manage plugin through smart pointer and custom deleter
    m_plugin = std::unique_ptr<::plugin::INvPlugin, decltype(nvPluginDeleter)>(plugin::createPReLUPlugin(negSlope),
                                                                               nvPluginDeleter);
    if (!m_plugin)
        return nullptr;

    // Leaky ReLU through PReLU plugin (not natively supported)
    ITensor *batchnorm_tensor = &input;
    ILayer* activation = network->addPlugin(&batchnorm_tensor, 1, *m_plugin);
    if (!activation)
        return nullptr;

    activation->setName(std::string(name + "_PReLU").c_str());

    return activation;
}
