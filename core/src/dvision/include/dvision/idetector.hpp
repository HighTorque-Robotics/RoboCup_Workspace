/* Copyright (C) ZJUDancer
 * 2017 - Yusu Pan <xxdsox@gmail.com>, Yujie Yang <meetyyj@gmail.com>
 */

/**
 * \file idetector.hpp
 * \author Yusu Pan
 * \version 2018
 * \date 2018-02-17
 */

#pragma once
#include <algorithm>
#include <cmath>
#include <opencv2/opencv.hpp>
#include "dancer_geometry/utils.hpp"
#include "dvision/parameters.hpp"
namespace dvision {
//! Base class for all detectors
class IDetector {
 public:
  //! Initialize IDetector
  virtual bool Init() = 0;
  //! IDetector destructor
  virtual ~IDetector(){};
};

}  // namespace dvision
