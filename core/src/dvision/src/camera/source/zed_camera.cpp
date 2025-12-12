#include "dvision/zed_camera.hpp"
#include <opencv2/opencv.hpp>
#include <fmt/format.h>
#include <ros/ros.h>
#include <boost/filesystem.hpp>
#include <cstdlib>

namespace dvision {

ZedCamera::ZedCamera() {}

ZedCamera::~ZedCamera() {
  // 关闭 ZED 相机
  zed.close();
}

bool ZedCamera::initialize() {
  sl::InitParameters init_params;
  init_params.camera_resolution = sl::RESOLUTION::VGA;  // 640x480
  init_params.camera_fps = 30;  // 设置帧率
  
  init_params.depth_mode = sl::DEPTH_MODE::QUALITY;  // 设置深度模式
  init_params.sdk_verbose = true;  // 启用 SDK 调试日志
  // 关键设置：坐标系和单位（ROS 标准）
  init_params.coordinate_system = sl::COORDINATE_SYSTEM::RIGHT_HANDED_Z_UP;  // 右手系，Y 向上
  // init_params.coordinate_units = sl::UNIT::METER;                             // 单位：米

  // 打开相机
  sl::ERROR_CODE err = zed.open(init_params);
  if (err != sl::ERROR_CODE::SUCCESS) {
    ROS_ERROR("无法打开 ZED 相机: %s", sl::toString(err).c_str());
    return false;  // 初始化失败
  }

  return true;  // 初始化成功
}

Frame ZedCamera::capture() {
  if (zed.grab() != sl::ERROR_CODE::SUCCESS) {
    ROS_ERROR("ZED 相机抓取失败！");
    return Frame();  // 返回空帧
  }

  // 获取左侧图像（BGR 格式）
  zed.retrieveImage(image, sl::VIEW::LEFT, sl::MEM::CPU, sl::Resolution(0, 0));

  // 转换为 OpenCV 格式（默认是 BGRA，需要转成 BGR）
  cv::Mat cv_image_bgra = cv::Mat(image.getHeight(), image.getWidth(), CV_8UC4, image.getPtr<sl::uchar1>());
  cv::Mat cv_image_bgr;
  cv::cvtColor(cv_image_bgra, cv_image_bgr, cv::COLOR_BGRA2BGR);  // 去掉 Alpha 通道

  // 创建 Frame 对象（确保 Frame 类能处理 BGR 格式）
  Frame frame(cv_image_bgr);

  return frame;
}

}  // namespace dvision