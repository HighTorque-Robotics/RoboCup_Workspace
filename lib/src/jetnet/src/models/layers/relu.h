#ifndef JETNET_RELU_H
#define JETNET_RELU_H

#include <NvInfer.h>
#include <memory>

namespace jetnet
{

class Relu
{
public:
    nvinfer1::ILayer* operator()(std::string name, nvinfer1::INetworkDefinition* network, nvinfer1::ITensor& input,
                                 float negSlope, nvinfer1::DataType dt);
};

}

#endif /* JETNET_RELU_H */
