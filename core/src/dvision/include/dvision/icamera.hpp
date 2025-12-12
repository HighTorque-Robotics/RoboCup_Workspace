/* Copyright (C) ZJUDancer
 * 2019 - Yusu Pan <xxdsox@gmail.com>
 */

/**
 * \file icamera.hpp
 * \author Yusu Pan
 * \version 2019
 * \date 2019-04-26
 */

#pragma once
#include "dvision/frame.hpp"

namespace dvision {
//! Base class for all camera sources
class ICamera {
 public:
  //! IDetector destructor
  virtual ~ICamera() = default;
  /**
   * \brief Capture a frame from camera device
   *
   * \return captured frame
   */
  virtual Frame capture() = 0;
};

}  // namespace dvision
