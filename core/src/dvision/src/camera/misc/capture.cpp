/* Copyright (C) ZJUDancer
 * 2018 - Yusu Pan <xxdsox@gmail.com>
 * 2017 - Wenxing Mei <mwx36mwx@gmail.com>
 */

/**
 * @file capture.cpp
 * @brief Capture and save frame from camera
 * @author Yusu Pan, Wenxing Mei
 * @version 2018
 * @date 2018-02-24
 */

#include <fmt/format.h>
#include <ros/ros.h>
#include <signal.h>
#include <boost/filesystem.hpp>
#include <cstdlib>
#include <opencv2/opencv.hpp>
#include "dvision/timer.hpp"
#include "dvision/v4l2_camera.hpp"
#include "dvision/parameters.hpp"

void signalHandler(int sig) {
  ROS_WARN("Trying to exit!");
  ros::shutdown();
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "capture");
  ros::NodeHandle nh("~");
  signal(SIGINT, signalHandler);
  // show usage
  std::cout << "keybindings:" << std::endl;
  std::cout << "\tc to capture" << std::endl;
  std::cout << "\tq to capture" << std::endl;
  std::cout << "\ta to switch auto capturing mode" << std::endl;
  dvision::parameters.init(&nh);
  // initialize camera
  dvision::CameraSettings s(&nh);
  dvision::V4L2Camera c(s);
  // initialize variables
  int order = 0;
  bool auto_mode = false;
  dvision::Timer t;
  // prepare output directory
  std::string path(std::string(std::getenv("HOME")) + "/dancer-camera" + "/int_img_" + std::to_string(dvision::parameters.robotId));
  boost::filesystem::path dir(path.c_str());
  if (boost::filesystem::create_directory(dir)) {
    ROS_INFO("Directory Created for captured frames: %s", path.c_str());
  }
  cv::namedWindow("capture", CV_WINDOW_NORMAL);

  while (ros::ok()) {
    // capture and show image
    auto frame = c.capture();
    cv::imshow("capture", frame.bgr());
    // handle keys
    auto key = (char)cv::waitKey(1);
    if (key == 'c') {
      order++;
      std::string filename = fmt::format("{}/{:05d}.png", path, order);
      frame.save(filename);
    } else if (key == 'a') {
      auto_mode = !auto_mode;
      t.restart();
      ROS_INFO("Auto capturing mode: %d", auto_mode);
    } else if (key == 'q') {
      break;
    }
    // capture in auto mode
    if (auto_mode) {
      if (t.getElapsedSec() >= 1) {
        order++;
        std::string filename = fmt::format("{}/{:05d}.png", path, order);
        frame.save(filename);
        t.restart();
      }
    }
  }
}
