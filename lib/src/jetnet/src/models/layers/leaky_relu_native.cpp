#include "leaky_relu_native.h"

using namespace jetnet;
using namespace nvinfer1;

ILayer* LeakyReluNative::operator()(std::string name, INetworkDefinition* network, ITensor& input, float negSlope, DataType dt)
{
    const Weights default_weights{dt, nullptr, 0};
    Weights scales_1{dt, nullptr, 1};
    Weights scales_2{dt, nullptr, 1};

    const float scale1 = (1 - negSlope);
    const float scale2 = (negSlope / scale1);

    if (dt == DataType::kHALF) {
        m_scale_value_1_h = __float2half(scale1);
        m_scale_value_2_h = __float2half(scale2);
        scales_1.values = &m_scale_value_1_h;
        scales_2.values = &m_scale_value_2_h;
    } else {
        m_scale_value_1_f = scale1;
        m_scale_value_2_f = scale2;
        scales_1.values = &m_scale_value_1_f;
        scales_2.values = &m_scale_value_2_f;
    }

    ILayer* layer_scale1 = network->addScale(input, ScaleMode::kUNIFORM, default_weights, scales_1, default_weights);
    if (!layer_scale1)
        return nullptr;
    layer_scale1->setName(std::string(name + "_leaky_scale1").c_str());

    ILayer* layer_scale2 = network->addScale(*layer_scale1->getOutput(0), ScaleMode::kUNIFORM, default_weights, scales_2, default_weights);
    if (!layer_scale2)
        return nullptr;
    layer_scale2->setName(std::string(name + "_leaky_scale2").c_str());

    ILayer* relu = network->addActivation(*layer_scale1->getOutput(0), ActivationType::kRELU);
    if (!relu)
        return nullptr;
    relu->setName(std::string(name + "_leaky_relu").c_str());

    ILayer* activation = network->addElementWise(*layer_scale2->getOutput(0), *relu->getOutput(0), ElementWiseOperation::kSUM);
    if (!activation)
        return nullptr;
    activation->setName(std::string(name + "_leaky_add").c_str());

    return activation;
}
