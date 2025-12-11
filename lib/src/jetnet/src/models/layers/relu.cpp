#include "relu.h"

using namespace jetnet;
using namespace nvinfer1;

ILayer* Relu::operator()(std::string name, INetworkDefinition* network, ITensor& input, float negSlope, DataType dt)
{
    (void)dt;
    (void)negSlope;

    ILayer* activation = network->addActivation(input, ActivationType::kRELU);
    if (!activation)
        return nullptr;

    activation->setName(std::string(name + "_RELU").c_str());

    return activation;
}
