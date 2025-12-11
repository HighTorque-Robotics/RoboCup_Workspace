#ifndef JETNET_FP16_H
#define JETNET_FP16_H

#include <cublas_v2.h>

namespace jetnet
{

// Code added before equivalent code was available via cuda.
// This code needs to be removed when we ship for cuda-9.2.
template<typename T, typename U> T bitwise_cast(U u)
{
    return *reinterpret_cast<T*>(&u);
}

__half __float2half(float f);

float __half2float(__half h);

}

#endif /* JETNET_FP16_H */
