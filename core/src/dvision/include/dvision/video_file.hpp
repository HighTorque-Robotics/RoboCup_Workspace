/* Copyright (C) ZJUDancer
 * 2018 - Yusu Pan <xxdsox@gmail.com>
 * 2017 - Yujie Yang <meetyyj@gmail.com>
 */

/**
 * \file video_file.hpp
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
class VideoFile : public ICamera {
 public:
  VideoFile(const std::string &video_file, const int &width, const int &height);
  ~VideoFile() override;
  bool init();
  Frame capture() override;

 private:
  std::string video_file_;
  cv::VideoCapture cap_;
  int output_width_ = 640;
  int output_height_ = 480;
  int num_frames_ = 0;
  int frame_idx_ = 0;
};
}  // namespace dvision
