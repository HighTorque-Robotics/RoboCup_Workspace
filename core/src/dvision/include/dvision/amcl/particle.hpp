/* Copyright (C) ZJUDancer
 * 2018 - Yusu Pan <xxdsox@gmail.com>
 * 2017 - Wenxing Mei <mwx36mwx@gmail.com>
 */

/**
 * @file particle.hpp
 * @author Yusu Pan, Wenxing Mei
 * @version 2018
 * @date 2018-02-23
 */

#pragma once
#include "dvision/amcl/pose.hpp"

namespace dvision {
class Particle
{
  public:
    Particle();
    Pose pose;
    double weight;
};
}
