#ifndef JETNET_GPU_BLOB_H
#define JETNET_GPU_BLOB_H

#include <cstdlib>
#include <vector>
#include <memory>

namespace jetnet
{

struct GpuBlob
{
    /*
     *  Create a data blob on the GPU
     *  size:   blob size in number of floats
     */
    GpuBlob(size_t size);

    /*
     *  Upload CPU data to the GPU blob
     *  data:   CPU data to upload
     *  size:   size of the CPU data buffer (<= blob size)
     */
    void upload(const float* data, size_t size) const;

    /*
     *  Upload CPU data to the GPU blob
     *  data:   CPU data to upload
     */
    void upload(const std::vector<float>& data) const;

    /*
     *  Download GPU data blob to the CPU
     *  The user is responsible for mamnaging the CPU data
     *  data:   CPU data buffer to be filled
     *  size:   number of float numbers to download (<= size of data buffer)
     */
    void download(float* data, size_t size) const;

    /*
     *  Download GPU data blob to the CPU
     *  data:   vector with data. Method will resize the vector if necessary
     */
    void download(std::vector<float>& data) const;

    /*
     *  Get a pointer to GPU blob
     */
    void* get();

    /*
     *  Number of float numbers the blob can hold
     */
    const size_t size;

private:
    static void* create_cuda_buffer(size_t size);
    static void delete_cuda_buffer(void* buffer);

    std::unique_ptr<void, decltype(&delete_cuda_buffer)> m_cuda_buffer;
};

}

#endif /* JETNET_GPU_BLOB_H */
