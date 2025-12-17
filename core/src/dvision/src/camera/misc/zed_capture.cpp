#include <ros/ros.h>
#include <signal.h>
#include <boost/filesystem.hpp>
#include <cstdlib>
#include <opencv2/opencv.hpp>
#include "dvision/zed_camera.hpp"
#include "dvision/frame.hpp"
#include "dvision/timer.hpp"
#include "dvision/parameters.hpp"
#include <fmt/format.h>

void signalHandler(int sig) {
  ROS_WARN("Trying to exit!");
  ros::shutdown();
}

int main(int argc, char** argv) {
  // 初始化 ROS
  ros::init(argc, argv, "zed_capture");
  ros::NodeHandle nh("~");
  signal(SIGINT, signalHandler);

  std::cout << "keybindings:" << std::endl;
  std::cout << "\tc to capture" << std::endl;
  std::cout << "\tq to exit" << std::endl;
  std::cout << "\ta to switch auto capturing mode" << std::endl;

  // 初始化 ROS 参数
  dvision::parameters.init(&nh);

  // 创建 ZedCamera 对象
  dvision::ZedCamera zed_camera;

  // 初始化 ZED 相机
  if (!zed_camera.initialize()) {
    ROS_ERROR("ZED 相机初始化失败！");
    return -1;
  }

  int order = 0;
  bool auto_mode = false;
  dvision::Timer t;

  // 输出目录
  std::string path(std::string("/home/nvidia/RoboCup_Workspace/core/src/dvision/camera") + "/int_img_" + std::to_string(dvision::parameters.robotId));
  boost::filesystem::path dir(path.c_str());
  if (boost::filesystem::create_directory(dir)) {
    ROS_INFO("Directory Created for captured frames: %s", path.c_str());
  }

  cv::namedWindow("capture", cv::WINDOW_NORMAL);

  while (ros::ok()) {
    // 捕捉图像
    auto frame = zed_camera.capture();

    cv::imshow("capture", frame.bgr());

    // 处理键盘输入
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
      break;  // 按 'q' 退出程序
    }

    // 自动捕捉模式
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