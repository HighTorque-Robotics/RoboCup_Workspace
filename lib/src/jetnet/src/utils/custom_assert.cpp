#include "custom_assert.h"
#include <iostream>

void jetnet::cuda_assert_failed(cudaError_t code, const char *file, int line, const char* msg)
{
    std::cerr << "CUDA ASSERT FAILED: " << cudaGetErrorString(code) << " " << " from: " << msg << " " << file << ":" << line << std::endl;
    abort();
}

void jetnet::assert_failed(const char* file, int line, const char* msg)
{
    std::cerr << "ASSERT FAILED: " << msg << " " << file << ":" << line << std::endl;
    abort();
}
