#ifndef JETNET_LEAKY_RELU_NATIVE_H
#define JETNET_LEAKY_RELU_NATIVE_H

#include <NvInfer.h>
#include <string>
#include "fp16.h"

namespace jetnet
{

class LeakyReluNative
{
    /*
     * Building PReLU using native TensorRT layers. Leaky ReLU can be calulated by:
     *
     *  out = (in * scale1) * scale2 + ReLU((in * scale1))
     *
     *  where scale1 = 1 - negSlope
     *  where scale2 = negSlope / scale1
     *
     * This requires 2 scale operations (the two multiplications), one ReLU operations and an element wise addition.
     * Since optimization can merge scale1 into the previous layer (other scale layer or convolution), this will be
     * optimized to 1 scale operator, one ReLU and one element wise addition.
     * */
public:
    nvinfer1::ILayer* operator()(std::string name, nvinfer1::INetworkDefinition* network, nvinfer1::ITensor& input,
                               float negSlope, nvinfer1::DataType dt);

private:
    float m_scale_value_1_f;
    float m_scale_value_2_f;
    __half m_scale_value_1_h;
    __half m_scale_value_2_h;
};

}

#endif /* JETNET_LEAKY_RELU_NATIVE_H */
