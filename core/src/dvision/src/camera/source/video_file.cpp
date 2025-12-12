/* Copyright (C) ZJUDancer
 * 2018 - Yusu Pan <xxdsox@gmail.com>
 * 2017 - Yujie Yang <meetyyj@gmail.com>
 */

/**
 * \file camera_dummy.cpp
 * \author Yusu Pan, Yujie Yang
 * \version 2018
 * \date 2018-02-13
 */

#include <dvision/video_file.hpp>

namespace dvision {

VideoFile::VideoFile(const std::string &video_file, const int &width,
                     const int &height)
    : video_file_(video_file),
      cap_(video_file),
      output_width_(width),
      output_height_(height) {
  init();
}

VideoFile::~VideoFile() {
  // When everything done, release the video capture object
  cap_.release();
}

bool VideoFile::init() {
  // Check if camera opened successfully
  if (!cap_.isOpened()) {
    ROS_ERROR("Error opening video stream or file: %s", video_file_.c_str());
    return false;
  }
  return true;
}

Frame VideoFile::capture() {
  cv::Mat frame;
  bool ret = cap_.read(frame);
  if (not ret) {
    cap_.set(cv::CAP_PROP_POS_FRAMES, 0);
    return capture();
  }
  cv::resize(frame, frame, cv::Size(output_width_, output_height_));
  return Frame(frame, output_width_, output_height_);
}
}  // namespace dvision
