/**
 * @file zed_detect.cpp
 * @brief 使用 ZED 右相机采集图像并进行 YOLO 推理
 * @version 2025
 * @date 2025-05-13
 */

 #include <fmt/format.h>
 #include <ros/ros.h>
 #include <signal.h>
 #include <boost/filesystem.hpp>
 #include <cstdlib>
 #include <opencv2/opencv.hpp>
 #include "dvision/timer.hpp"
 #include "dvision/zed_camera.hpp"
 #include "dvision/parameters.hpp"
 #include "dvision/yolo/yolo26.hpp"
 #include "dvision/yolo/utils.hpp"

 using namespace dvision;

 void signalHandler(int sig) {
   ROS_WARN("Trying to exit!");
   ros::shutdown();
 }

 int main(int argc, char** argv) {
   ros::init(argc, argv, "zed_detect");
   ros::NodeHandle nh("~");
   signal(SIGINT, signalHandler);

   // 打印使用说明
   std::cout << "keybindings:" << std::endl;
   std::cout << "\tc to capture" << std::endl;
   std::cout << "\tq to quit" << std::endl;
   std::cout << "\ta to switch auto capturing mode" << std::endl;

   // 初始化参数
   dvision::parameters.init(&nh);

   // 加载类别名称
   std::vector<std::string> class_names;
   if (!read_text_file(class_names, parameters.object.input_names_file)) {
     ROS_WARN("Failed to read class names file from: %s", parameters.object.input_names_file.c_str());
   }

   // 初始化 YOLO 检测器
   yolo26 v5;
   v5.init(parameters.object.input_model_file,
           parameters.object.threshold,
           parameters.object.nms_threshold,
           parameters.object.batch_size);
   std::cout << "threshold: " << parameters.object.threshold << std::endl;
   std::cout << "yolo is created" << std::endl;

   // 使用 ZED 右相机
   dvision::ZedCamera zedCam;
   if (!zedCam.initialize(sl::VIEW::RIGHT)) {
     ROS_FATAL("ZED 右相机初始化失败！");
     return -1;
   }
   std::cout << "ZED right camera initialized" << std::endl;

   // 初始化变量
   int order = 0;
   bool auto_mode = false;
   dvision::Timer t;

   // 准备图像保存目录
   std::string path(std::string("/home/nvidia/RoboCup_Workspace/core/src/dvision/camera") +
                    "/int_img_" + std::to_string(dvision::parameters.robotId));
   boost::filesystem::path dir(path.c_str());
   if (boost::filesystem::create_directory(dir)) {
     ROS_INFO("Directory Created for captured frames: %s", path.c_str());
   }

   cv::namedWindow("ZED Right Camera - YOLO Detection", cv::WINDOW_NORMAL);
   cudaSetDevice(0);

   std::vector<cv::Mat> images;
   while (ros::ok()) {
     // 根据 batch_size 批量采集图像
     for (int b = 0; b < parameters.object.batch_size; b++) {
       auto frame = zedCam.capture();
       cv::Mat mat = frame.bgr();
       if(mat.empty())
        continue;

       cv::Mat mat3;
       if (mat.channels() == 4) {
         cv::cvtColor(mat, mat3, cv::COLOR_BGRA2BGR);
       } else {
         mat3 = mat;
       }
       images.push_back(mat3);
     }

     // 进行 YOLO 检测并绘制结果
     auto detections = v5.detect(images);
     draw_detections(detections[0], class_names, images[0]);

     // 用第一帧图像构造新 Frame 对象用于显示和保存
     Frame newframe(images[0]);
     images.clear();

     cv::imshow("ZED Right Camera - YOLO Detection", newframe.bgr());
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
     // 自动捕捉模式：每秒保存一张图像
     if (auto_mode && t.getElapsedSec() >= 1) {
       order++;
       std::string filename = fmt::format("{}/{:05d}.png", path, order);
       newframe.save(filename);
       t.restart();
     }
   }
 }