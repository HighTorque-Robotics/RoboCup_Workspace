#ifndef JETNET_CONV2D_BATCH_LEAKY_H
#define JETNET_CONV2D_BATCH_LEAKY_H

#include "darknet_weights_loader.h"
#include <NvInfer.h>
#include <memory>
#include <string>

namespace jetnet
{

template<typename TLeaky>
class Conv2dBatchLeaky
{
public:
    /*
     *  Create a network definition of a convolution layer with batch norm and leaky relu
     *  name:           unique name of the layer
     *  network:        network definition to where the sub layers will be added
     *  weights:        darknet weights loader reference
     *  input:          input tensor to this layer
     *  nbOutputMaps:   number of convolutional filters of this layer
     *  kernelSize:     convolutional kernel size
     *  padding:        convolutional padding
     *  stride:         stride step
     *  negSlope:       negative slope of the leaky relu sub layer
     *  act_impl:       leaky relu implementation type
     */
    nvinfer1::ILayer* operator()(std::string name,
                       nvinfer1::INetworkDefinition* network,
                       DarknetWeightsLoader& weights,
                       nvinfer1::ITensor& input,
                       int nbOutputMaps,
                       nvinfer1::DimsHW kernelSize,
                       nvinfer1::DimsHW padding = nvinfer1::DimsHW{1, 1},
                       nvinfer1::DimsHW stride = nvinfer1::DimsHW{1, 1},
                       float negSlope = 0.1);
private:
    TLeaky m_activation;
};

}

#include "conv2d_batch_leaky_impl.h"

#endif /* JETNET_CONV2D_BATCH_LEAKY_H */
