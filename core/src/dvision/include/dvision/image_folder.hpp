/* Copyright (C) ZJUDancer
 * 2018 - Yusu Pan <xxdsox@gmail.com>
 * 2017 - Yujie Yang <meetyyj@gmail.com>
 */

/**
 * \file image_folder.hpp
 * \author Yusu Pan, Yujie Yang
 * \version 2018
 * \date 2018-02-13
 */

#pragma once
#include <dirent.h>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>
#include "dvision/icamera.hpp"

namespace dvision {
class ImageFolder : public ICamera {
 public:
  ImageFolder(const std::string& image_root, const int& width = 640,
              const int& height = 480);
  bool init();
  Frame capture() override;

 private:
  std::string image_root_;
  int output_width_ = 640;
  int output_height_ = 480;
  std::vector<std::string> image_files_;
  int num_images_ = 0;
  int image_idx_ = 0;
};
}  // namespace dvision
