/* Copyright (C) ZJUDancer
 * 2019 - Yusu Pan <xxdsox@gmail.com>
 */

/**
 * \file gst_camera.hpp
 * \author Yusu Pan
 * \version 2019
 * \date 2019-04-28
 */

#ifdef USE_GST_CAMERA
#pragma once
#include <opencv2/opencv.hpp>
#include <regex>
#include <string>
#include <vector>
#include "dvision/gst_camera_impl.hpp"
#include "dvision/icamera.hpp"

namespace dvision {
class GstCamera : public ICamera {
 public:
  GstCamera(const std::string& device = "/dev/video0", const int& width = 640,
            const int& height = 480);
  ~GstCamera() override;
  bool init(const std::string& device);
  Frame capture() override;

 private:
  GstCameraImpl* camera_;
  int output_width_;
  int output_height_;
};
}  // namespace dvision
#endif
