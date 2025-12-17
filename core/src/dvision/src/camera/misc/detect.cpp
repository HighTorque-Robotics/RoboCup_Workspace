/**
 * @file detect.cpp
 * @brief Capture and detect features
 * @version 2024
 * @date 2024-04-08
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
#include "dvision/yolo/yolov5.hpp"

using namespace dvision;

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
  std::cout << "\tq to quit" << std::endl;
  std::cout << "\ta to switch auto capturing mode" << std::endl;
  dvision::parameters.init(&nh);
  yolov5 v5;
  v5.init(parameters.object.input_model_file,parameters.object.threshold,
  parameters.object.nms_threshold,parameters.object.batch_size);
  std::cout<<parameters.object.threshold<<std::endl;
  std::cout<<"yolo is created"<<std::endl;
  // initialize camera
  dvision::CameraSettings s(&nh);
  dvision::V4L2Camera c(s);
  // initialize variables
  int order = 0;
  bool auto_mode = false;
  dvision::Timer t;
  // prepare output directory
  std::string path(std::string("/home/nvidia/RoboCup_Workspace/core/src/dvision/camera") + "/int_img_" + std::to_string(dvision::parameters.robotId));
  boost::filesystem::path dir(path.c_str());
  if (boost::filesystem::create_directory(dir)) {
    ROS_INFO("Directory Created for captured frames: %s", path.c_str());
  }
  cv::namedWindow("capture", CV_WINDOW_NORMAL);
  //prepare yolo
  cudaSetDevice(0);
  

  std::vector<cv::Mat> images;
  while (ros::ok()) {
    // capture and show image
    
    for(int b = 0;b<parameters.object.batch_size;b++){
      auto frame = c.capture();
      //cv::Mat img = cv::imread("/home/nvidia/new-yolov5-test/samples/bus.jpg");
      images.push_back(frame.bgr());
    }
    auto detections = v5.detect(images);
    draw_detections(detections[0],images[0]);
    Frame newframe(images[0]);
    images.clear();
    
    cv::imshow("capture", newframe.bgr());
    // handle keys
    auto key = (char)cv::waitKey(1);
    if (key == 'c') {
      order++;
      std::string filename = fmt::format("{}/{:05d}.png", path, order);
      
      newframe.save(filename);
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
        newframe.save(filename);
        t.restart();
      }
    }
  }
}
