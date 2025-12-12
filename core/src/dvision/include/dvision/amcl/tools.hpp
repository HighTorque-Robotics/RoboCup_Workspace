/* Copyright (C) ZJUDancer
 * 2018 - Yusu Pan <xxdsox@gmail.com>
 * 2017 - Wenxing Mei <mwx36mwx@gmail.com>
 */

/**
 * @file tools.hpp
 * @author Yusu Pan, Wenxing Mei
 * @version 2018
 * @date 2018-02-23
 */

#pragma once
#include <random>

namespace dvision {

template <typename T>
class Distribution
{
  public:
    Distribution(float a, float b)
    {
        dist_ = T(a, b);

        std::random_device rd;
        generator_ = std::default_random_engine(rd());
    }
    float sample()
    {
        return dist_(generator_);
    }

  private:
    std::default_random_engine generator_;
    T dist_;
};

using Gaussian = Distribution<std::normal_distribution<float>>;
using Uniform = Distribution<std::uniform_real_distribution<float>>;

// Copied from https://stackoverflow.com/a/10848293
template <typename T>
T
normal_pdf(T x, T m, T s)
{
    static const T inv_sqrt_2pi = 0.3989422804014327;
    T a = (x - m) / s;

    return inv_sqrt_2pi / s * std::exp(-T(0.5) * a * a);
}

} // namespace dvision
