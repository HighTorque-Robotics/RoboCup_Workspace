#ifndef JETNET_CONV2D_BATCH_LEAKY_IMPL_H
#define JETNET_CONV2D_BATCH_LEAKY_IMPL_H

#define FLOAT_EPSILON       0.00001L
#define HALF_EPSILON        0.001L

namespace jetnet
{

template<typename TLeaky>
nvinfer1::ILayer* Conv2dBatchLeaky<TLeaky>::operator()(std::string name,
                                               nvinfer1::INetworkDefinition* network,
                                               DarknetWeightsLoader& weights,
                                               nvinfer1::ITensor& input,
                                               int nbOutputMaps,
                                               nvinfer1::DimsHW kernelSize,
                                               nvinfer1::DimsHW padding,
                                               nvinfer1::DimsHW stride, float negSlope)
{
    nvinfer1::Dims input_dim = input.getDimensions();
    const nvinfer1::Weights default_weights{weights.datatype, nullptr, 0};
    const int num_channels = input_dim.d[0];

    // Read weights for batchnorm layer (biases are applied in batchnorm i.s.o. conv layer)
    std::vector<float> biases = weights.get_floats(nbOutputMaps);
    std::vector<float> bn_raw_scales = weights.get_floats(nbOutputMaps);
    std::vector<float> bn_raw_means = weights.get_floats(nbOutputMaps);
    std::vector<float> bn_raw_variances = weights.get_floats(nbOutputMaps);

    // calculate Batch norm parameters since we implement this layer as a scale layer
    // Batch norm is defined as:
    //
    //      output = (input - mean) / (sqrt(variance) + epsilon) * bn_scale + bias
    //
    // We implement this with the Scale layer from TensorRT that performs the following operation:
    //
    //      output = (input * scale + shift)^power
    //
    // So deriving the input params:
    //
    //      scale = bn_scale / (sqrt(variance) + epsilon)
    //      shift = - mean * scale + bias

    std::vector<float> scale_vals(nbOutputMaps);
    std::vector<float> shift_vals(nbOutputMaps);

    // determine machine epsilon based on target precision
    float epsilon = weights.datatype == nvinfer1::DataType::kHALF ? HALF_EPSILON : FLOAT_EPSILON;

    for (int i=0; i<nbOutputMaps; ++i) {
        // cast to double to avoid error buildup in calculations
        double scale_val = static_cast<double>(bn_raw_scales[i]) / (sqrt(static_cast<double>(bn_raw_variances[i])) + epsilon);
        shift_vals[i] = -static_cast<double>(bn_raw_means[i]) * scale_val + static_cast<double>(biases[i]);
        scale_vals[i] = scale_val;
    }

    const nvinfer1::Weights bn_scales = weights.get(scale_vals);
    const nvinfer1::Weights bn_shifts = weights.get(shift_vals);

    // Read weights for conv layer
    const nvinfer1::Weights conv_weights = weights.get(nbOutputMaps * num_channels * kernelSize.h() * kernelSize.w());

    // conv layer without bias (bias is within batchnorm)
    nvinfer1::IConvolutionLayer* conv = network->addConvolution(input, nbOutputMaps, kernelSize, conv_weights, default_weights);
    if (!conv)
        return nullptr;

    conv->setStride(stride);
    conv->setPadding(padding);
    conv->setName(std::string(name + "_conv").c_str());

    // batch norm layer
    nvinfer1::ILayer* batchnorm = network->addScale(*conv->getOutput(0), nvinfer1::ScaleMode::kCHANNEL, bn_shifts, bn_scales, default_weights);
    if (!batchnorm)
        return nullptr;

    batchnorm->setName(std::string(name + "_bn").c_str());

    // activation layer
    return m_activation(name, network, *batchnorm->getOutput(0), negSlope, weights.datatype);
}

}

#endif /* JETNET_CONV2D_BATCH_LEAKY_IMPL_H */
