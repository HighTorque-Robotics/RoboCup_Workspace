/* Copyright (C) ZJUDancer
 * 2019 - Yusu Pan <xxdsox@gmail.com>
 */

/**
 * \file gst_camera.cpp
 * \author Yusu Pan
 * \version 2019
 * \date 2019-04-28
 */

#ifdef USE_GST_CAMERA
#include <dvision/gst_camera.hpp>

namespace dvision {

GstCamera::GstCamera(const std::string& device, const int& width,
                     const int& height)
    : output_width_(width), output_height_(height) {
  init(device);
}

GstCamera::~GstCamera() { delete camera_; }

bool GstCamera::init(const std::string& device) {
  // determine video device number
  std::regex rx("/dev/video([0-9]+)");
  std::smatch match;
  if (!std::regex_match(device, match, rx)) {
    ROS_ERROR("Cannot find camera device at %s", device.c_str());
    return false;
  }
  int device_number = std::stoi(match[1].str());
  if (device_number < 0) {
    ROS_ERROR("[camera] Failed to initialize video device %s", device.c_str());
    return false;
  }
  // initialize
  camera_ = GstCameraImpl::Create(output_width_, output_height_, device_number);
  if (!camera_) {
    ROS_ERROR("[camera] Failed to initialize video device %s", device.c_str());
    return false;
  }
  ROS_INFO("[camera] Successfully initialized video device");
  ROS_INFO("    width:  %u", camera_->GetWidth());
  ROS_INFO("   height:  %u", camera_->GetHeight());
  ROS_INFO("    depth:  %u (bpp)", camera_->GetPixelDepth());
  // start streaming
  if (!camera_->Open()) {
    ROS_ERROR("[camera] Failed to open camera for streaming");
    return false;
  }
  ROS_INFO("[camera] camera open for streaming");
  return true;
}

Frame GstCamera::capture() {
  void* img_cpu = nullptr;
  void* img_cuda = nullptr;

  // get the latest frame
  if (!camera_->Capture(&img_cpu, &img_cuda, 1000)) {
    ROS_ERROR("[camera] Failed to capture frame");
    return capture();
  } else {
    ROS_DEBUG("[camera] received new frame  CPU=0x%p  GPU=0x%p", img_cpu,
              img_cuda);
  }
  // update frame
  return Frame(img_cpu, img_cuda, output_width_, output_height_);
}

}  // namespace dvision
#endif
