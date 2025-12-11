#ifndef JETNET_UPSAMPLE_PLUGIN_H
#define JETNET_UPSAMPLE_PLUGIN_H

#include <NvInferPlugin.h>
#include <cuda_runtime.h>

namespace jetnet
{

class UpsamplePlugin : public nvinfer1::IPlugin
{
public:
    /*
     *  Upsample plugin enlarges the width/height of the input tensor by 'stride', repeating the input values 'stride' times
     *  stride:     upsample stride. The output tensor width = input_width * stride, the output tensor height = input_height * stride
     */
    UpsamplePlugin(int stride);

    /*
     *  Create an upsample plugin from serialized data
     */
    UpsamplePlugin(const void* data, size_t length);

    /*
     *  Function implementations needed by TensorRT. See IPlugin API for documentation
     */
	int getNbOutputs() const override;
    nvinfer1::Dims getOutputDimensions(int index, const nvinfer1::Dims* inputs, int nbInputDims) override;
	void configure(const nvinfer1::Dims* inputDims, int nbInputs, const nvinfer1::Dims* outputDims, int nbOutputs, int maxBatchSize) override;
	int initialize() override;
	void terminate() override;
	size_t getWorkspaceSize(int maxBatchSize) const override;
	int enqueue(int batchSize, const void*const* inputs, void** outputs, void* workspace, cudaStream_t stream) override;
	size_t getSerializationSize() override;
	void serialize(void* buffer) override;

private:
    int m_stride;
    int m_input_channels;
    int m_input_height;
    int m_input_width;
};

}

#endif /* JETNET_UPSAMPLE_PLUGIN_H */
