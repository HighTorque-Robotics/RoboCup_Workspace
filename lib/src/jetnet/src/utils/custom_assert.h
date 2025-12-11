#ifndef JETNET_GPU_ASSERT_H
#define JETNET_GPU_ASSERT_H

#include <cuda_runtime.h>

#define CUDA_CHECK(expr) { if ((expr) != cudaSuccess) { cuda_assert_failed(expr, __FILE__, __LINE__, #expr); } }
#define ASSERT(expr) { if (!(expr)) { assert_failed(__FILE__, __LINE__, #expr); } }

namespace jetnet
{

void cuda_assert_failed(cudaError_t code, const char *file, int line, const char* msg);
void assert_failed(const char* file, int line, const char* msg);

}

#endif /* JETNET_GPU_ASSERT_H */
