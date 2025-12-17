#include <aruco.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <ros/ros.h>
#include <boost/filesystem.hpp>
#include <opencv2/opencv.hpp>
#include "dmsgs/ActionCommand.h"
#include "dmsgs/BehaviorInfo.h"
#include "dmsgs/MotionInfo.h"
#include "dvision/frame.hpp"
#include "dvision/projection.hpp"
#include "dvision/v4l2_camera.hpp"

void BehaviorCallback(const dmsgs::BehaviorInfo::ConstPtr &behavior_msg);
void MotionCallback(const dmsgs::MotionInfo::ConstPtr &motion_msg);
void getPoints(ros::NodeHandle &nh);

static std::vector<cv::Point2f> real_pos_;
static aruco::MarkerDetector marker_;
static dvision::Projection projection_;
static double plat_pitch_ = 0, plat_yaw_ = 0;
static ros::Subscriber sub_motion_info_;
static ros::Subscriber sub_behavior_info_;
static dvision::V4L2Camera *camera_;
static dvision::Frame frame_;
static std::string path_(std::string("/home/nvidia/RoboCup_Workspace/core/src/dvision/camera"));
static std::string point = "";

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "ext_calib");
  ros::NodeHandle nh("~");

  // initialize camera
  parameters.init(&nh);
  projection_.init(&nh);
  dvision::CameraSettings s(&nh);
  camera_ = new dvision::V4L2Camera(s);

  if (!nh.getParam("pos", point))
    ROS_FATAL("ext_calib get start position error");

  getPoints(nh);

  // initialize aruco
  marker_.setDictionary("ARUCO_MIP_36h12");

  // subcribe topics
  sub_motion_info_ = nh.subscribe("/dmotion_" + std::to_string(parameters.robotId) + "/MotionInfo",
                                  1, &MotionCallback);
  sub_behavior_info_ = nh.subscribe(
      "/dbehavior_" + std::to_string(parameters.robotId) + "/BehaviorInfo", 1, &BehaviorCallback);

  // prepare output directory
  boost::filesystem::path dir(path_.c_str());
  if (boost::filesystem::create_directory(dir)) {
    ROS_INFO("directory created for captured frames: %s", path_.c_str());
  }

  // clear old data
  std::ofstream ext_file(path_ + "/bot" + std::to_string(parameters.robotId) + "_ext_" + point + ".txt");
  ext_file.close();
  std::ofstream in_file(path_ + "/bot" + std::to_string(parameters.robotId) + "_in.txt");
  in_file << std::setprecision(10) << parameters.camera.undistCameraMatrix.at<double>(0, 0) << " "
          << parameters.camera.undistCameraMatrix.at<double>(1, 1) << " "
          << parameters.camera.undistCameraMatrix.at<double>(0, 2) << " "
          << parameters.camera.undistCameraMatrix.at<double>(1, 2);
  in_file.close();

  ROS_INFO("Start get data for calibrating extrinsic parameters.");
  while (ros::ok()) {
    frame_ = camera_->capture();
    ros::spinOnce();
  }

  return 0;
}

void getPoints(ros::NodeHandle &nh) {
  std::vector<double> p_tmp;
  point[0] = tolower(point[0]);
  if (!nh.getParam("dvision/points_" + point, p_tmp))
    ROS_FATAL("ext_calib get param error");
  for (unsigned int i = 0; i < p_tmp.size() / 2; i++) {
    real_pos_.emplace_back(p_tmp[i * 2], p_tmp[i * 2 + 1]);
    ROS_DEBUG("points: %lf, %lf\n", p_tmp[i * 2], p_tmp[i * 2 + 1]);
  }
  point[0] = toupper(point[0]);
}

void MotionCallback(const dmsgs::MotionInfo::ConstPtr &motion_msg) {
  dmsgs::MotionInfo motion_info;

  motion_info = *motion_msg;

// update motion info at present
#ifdef USE_NEO_INFO
  plat_pitch_ = motion_info.curPlat.pitch;
  plat_yaw_ = motion_info.curPlat.yaw;
#else
  plat_pitch_ = motion_info.action.headCmd.pitch;
  plat_yaw_ = motion_info.action.headCmd.yaw;
#endif
}
void BehaviorCallback(const dmsgs::BehaviorInfo::ConstPtr &behavior_msg) {
  static bool save_data_ = false;
  dmsgs::BehaviorInfo behavior_info;

  behavior_info = *behavior_msg;

  if (behavior_info.save_image && !save_data_) {
    save_data_ = true;
    std::string filename(path_ + "/bot" + std::to_string(parameters.robotId) + "_ext_" + point + ".txt");
    std::ofstream ext_file(filename, std::ios::app);

    auto &mat = frame_.bgr();

    // Calibration by aruco markers
    std::vector<aruco::Marker> markers = marker_.detect(frame_.bgr());
    for (auto &m : markers) {
      int id = m.id;
      if (id > (int)real_pos_.size() - 1) {
        ROS_WARN("[dvision] Marker index_%d overflow!", id);
        continue;
      }
      cv::Point2f center = m.getCenter(), undist;
      undist = projection_.undistP(center);
      // if too far, the markers is stand, need to use the lowest y
      if (id >= 40) {
          int max_y = 0, total = 0, count = 0;
          for (auto iter : m.contourPoints) {
              if (iter.y > max_y + 5) {
                  max_y = iter.y;
                  total = iter.y;
                  count = 1;
              } else if (iter.y > max_y - 5) {
                  total += iter.y;
                  count++;
              }
          }
          if (count == 0)
              continue;
          else {
              undist = projection_.undistP(cv::Point2f(center.x, (float)total / count));
              if (parameters.monitor.use_cv_show)
                cv::circle(mat, cv::Point2f(center.x, (float)total / count), 2, cv::Scalar(255, 0, 0), 2, 8);
          }
      }
      ext_file << plat_yaw_ << " " << plat_pitch_ << " " << undist.x << " " << undist.y << " "
               << real_pos_[id].x << " " << real_pos_[id].y << std::endl;
      if (parameters.monitor.use_cv_show) m.draw(mat);
    }
    ext_file.close();

    ROS_INFO("[dvision] Data wrote in %s. p_%lf, y_%lf", filename.c_str(), plat_pitch_, plat_yaw_);
    if (parameters.monitor.use_cv_show) {
      // Show all frames that are used to get data
      cv::namedWindow("ext_calib", CV_WINDOW_NORMAL);
      cv::imshow("ext_calib", mat);
      auto key = (char)cv::waitKey(10);
      if (key == 'q') {
        ros::shutdown();
      }
    }
  }

  save_data_ = behavior_info.save_image;
}
