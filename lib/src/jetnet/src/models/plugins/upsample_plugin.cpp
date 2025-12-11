#include "upsample_plugin.h"
#include "blas_cuda.h"
#include "custom_assert.h"

using namespace jetnet;
using namespace nvinfer1;

#define NUM_PARAMS  4

UpsamplePlugin::UpsamplePlugin(int stride) :
    m_stride(stride)
{
}

UpsamplePlugin::UpsamplePlugin(const void* data, size_t length)
{
    const int* d = reinterpret_cast<const int*>(data);

    ASSERT(length == getSerializationSize());
    m_stride             = d[0];
    m_input_channels     = d[1];
    m_input_height       = d[2];
    m_input_width        = d[3];
}

int UpsamplePlugin::getNbOutputs() const
{
    return 1;
}

Dims UpsamplePlugin::getOutputDimensions(int index, const Dims* inputs, int nbInputDims)
{
    ASSERT(index == 0 && nbInputDims == 1 && inputs[0].nbDims == 3);
    m_input_channels = inputs[0].d[0];
    m_input_height = inputs[0].d[1];
    m_input_width = inputs[0].d[2];
    return DimsCHW(m_input_channels, m_stride * m_input_height, m_stride * m_input_width);
}

void UpsamplePlugin::configure(const Dims* inputDims, int nbInputs, const Dims* outputDims, int nbOutputs, int maxBatchSize)
{
    (void)inputDims;
    (void)nbInputs;
    (void)outputDims;
    (void)nbOutputs;
    (void)maxBatchSize;
}

int UpsamplePlugin::initialize()
{
    return 0;
}

void UpsamplePlugin::terminate()
{
}

size_t UpsamplePlugin::getWorkspaceSize(int maxBatchSize) const
{
    (void)maxBatchSize;
    return 0;
}

int UpsamplePlugin::enqueue(int batchSize, const void*const* inputs, void** outputs, void* workspace, cudaStream_t stream)
{
    (void)workspace;
    (void)stream;

    upsample_gpu(reinterpret_cast<const float*>(inputs[0]), m_input_width, m_input_height, m_input_channels, batchSize,
                 m_stride, reinterpret_cast<float*>(outputs[0]));
    return 0;
}

size_t UpsamplePlugin::getSerializationSize()
{
    return NUM_PARAMS * sizeof(int);
}

void UpsamplePlugin::serialize(void* buffer)
{
    int* d = reinterpret_cast<int*>(buffer);
    d[0] = m_stride;
    d[1] = m_input_channels;
    d[2] = m_input_height;
    d[3] = m_input_width;
}
