/* Copyright (C) ZJUDancer
 * 2018 - Yusu Pan <xxdsox@gmail.com>
 * 2017 - Yujie Yang <meetyyj@gmail.com>
 */

/**
 * \file imagr_folder.cpp
 * \author Yusu Pan, Yujie Yang
 * \version 2018
 * \date 2018-02-13
 */

#include <dvision/image_folder.hpp>

namespace dvision {

ImageFolder::ImageFolder(const std::string &image_root, const int &width,
                         const int &height)
    : image_root_(image_root), output_width_(width), output_height_(height) {
  init();
}

bool ImageFolder::init() {
  DIR *dir;
  struct dirent *ent;
  if ((dir = opendir(image_root_.c_str())) != nullptr) {
    /* print all the files and directories within directory */
    while ((ent = readdir(dir)) != nullptr) {
      std::string tmpFileName = ent->d_name;
      if (tmpFileName.length() > 4) {
        auto nPos = tmpFileName.find(".jpg");
        if (nPos != std::string::npos) {
          image_files_.push_back(tmpFileName);
          num_images_++;
        }
      }
    }
    closedir(dir);
  } else {
    /* could not open directory */
    ROS_ERROR("Failed to open image directory as camera: %s",
              image_root_.c_str());
    return EXIT_FAILURE;
  }
  // ROS_INFO("----------imagesNum------------%d", imagesNum);
  return true;
}

Frame ImageFolder::capture() {
  cv::Mat res = cv::imread((image_root_ + image_files_[image_idx_]).c_str());
  if (image_idx_ < num_images_ - 1) {
    image_idx_++;
  }
  cv::resize(res, res, cv::Size(output_width_, output_height_));
  return Frame(res, output_width_, output_height_);
}
}  // namespace dvision
