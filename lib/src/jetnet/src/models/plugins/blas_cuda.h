#ifndef JETNET_BLAS_CUDA_H
#define JETNET_BLAS_CUDA_H

namespace jetnet
{
    /*
     *  in:     input tensor
     *  w:      width of the input tensor
     *  h:      height of the input tensor
     *  c:      number of channels of the input tensor
     *  batch:  batch number of the input tensor
     *  stride: upsample stride
     *  out:    output tensor. The output tensor size = w * h * c * batch * stride^2;
     */
    void upsample_gpu(const float *in, int w, int h, int c, int batch, int stride, float *out);
}

#endif /* JETNET_BLAS_CUDA_H */
