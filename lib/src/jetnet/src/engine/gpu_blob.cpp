#include "gpu_blob.h"
#include "custom_assert.h"
#include <cuda_runtime.h>
#include <algorithm>

using namespace jetnet;

GpuBlob::GpuBlob(size_t size) :
    size(size),
    m_cuda_buffer(create_cuda_buffer(size), &delete_cuda_buffer)
{
}

void GpuBlob::upload(const float* data, size_t size) const
{
    // use the minimum to be sure no out of bounds write will occure
    const size_t count = std::min(size, this->size) * sizeof(float);
    CUDA_CHECK( cudaMemcpy(m_cuda_buffer.get(), data, count, cudaMemcpyHostToDevice) );
}

void GpuBlob::upload(const std::vector<float>& data) const
{
    upload(data.data(), data.size());
}

void GpuBlob::download(float* data, size_t size) const
{
    const size_t count = std::min(size, this->size) * sizeof(float);
    CUDA_CHECK( cudaMemcpy(data, m_cuda_buffer.get(), count, cudaMemcpyDeviceToHost) );
}

void GpuBlob::download(std::vector<float>& data) const
{
    // ensure data is of the right size
    if (data.size() != size)
        data.resize(size);

    download(data.data(), data.size());
}

void* GpuBlob::get()
{
    return m_cuda_buffer.get();
}

void* GpuBlob::create_cuda_buffer(size_t size)
{
    void* buffer;
    CUDA_CHECK( cudaMalloc(&buffer, size * sizeof(float)) );

    return buffer;
}

void GpuBlob::delete_cuda_buffer(void* buffer)
{
    CUDA_CHECK( cudaFree(buffer) );
}

