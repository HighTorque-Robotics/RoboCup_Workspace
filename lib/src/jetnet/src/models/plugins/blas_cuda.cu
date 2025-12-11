#include "blas_cuda.h"
#include "custom_assert.h"
#include <cuda_runtime.h>
#include <cublas_v2.h>
#include <curand.h>

#define BLOCK 512

__global__ void upsample_kernel(size_t N, const float *x, int w, int h, int c, int batch, int stride, float *out)
{
    size_t i = (blockIdx.x + blockIdx.y*gridDim.x) * blockDim.x + threadIdx.x;

    if(i >= N)
        return;

    int out_index = i;
    int out_w = i%(w*stride);
    i = i/(w*stride);
    int out_h = i%(h*stride);
    i = i/(h*stride);
    int out_c = i%c;
    i = i/c;
    int b = i%batch;

    int in_w = out_w / stride;
    int in_h = out_h / stride;
    int in_c = out_c;

    int in_index = b*w*h*c + in_c*w*h + in_h*w + in_w;

    out[out_index] = x[in_index];
}

dim3 cuda_gridsize(size_t n){
    size_t k = (n-1) / BLOCK + 1;
    size_t x = k;
    size_t y = 1;
    if (x > 65535) {
        x = ceil(sqrt(k));
        y = (n-1)/(x*BLOCK) + 1;
    }
    dim3 d(x, y);
    return d;
}

void jetnet::upsample_gpu(const float *in, int w, int h, int c, int batch, int stride, float *out)
{
    size_t size = w*h*c*batch*stride*stride;
    upsample_kernel<<<cuda_gridsize(size), BLOCK>>>(size, in, w, h, c, batch, stride, out);
    CUDA_CHECK( cudaPeekAtLastError() );
}
