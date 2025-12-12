#pragma once
#include "ros/ros.h"
#include "opencv2/core/core.hpp"
#include "opencv/highgui.h"
#include "opencv2/opencv.hpp"
#include "dvision/parameters.hpp"
#include "dvision/frame.hpp"
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <fstream>
#include <cstdio>
#include <cstring>  
#include <cstdlib>  
#include <numeric>
#include <dirent.h>
#include <chrono>
#include <vector>

#define PI 3.14159265358

namespace dvision {

//! Information for head pose of each images, especially rotation
class CameraPose {
public:
  //! Information in head_cmd from behavior
  double head_yaw;
  double head_pitch;

  //! IMU info
  double imu_yaw;

  //! Constructor
  CameraPose(double a, double b, double c):
    head_yaw(a), head_pitch(b), imu_yaw(c){}
};


class VisionCompass {
private:
  std::string img_dir;
  std::string paras_path;

  DIR *dp;
  struct dirent *dirp;
  std::vector<std::string> img_names;
  std::vector<cv::Mat> imgs;

  //! camera
  std::vector<CameraPose> camera_poses;
  double fx_, fy_, cx_, cy_;

  //! perpare
  std::vector<std::vector<cv::KeyPoint>> keypoints_list; 
  std::vector<cv::Mat> descriptors_list;
  cv::Ptr<cv::FeatureDetector> detector;
  cv::Ptr<cv::DescriptorExtractor> descriptor;
  cv::Ptr<cv::DescriptorMatcher> matcher;
  int max_keypoints = 0;

  //! correct 
  std::vector<cv::KeyPoint> target_keypoints;
  cv::Mat target_descriptors;
  cv::Point2f psrc, pdst;
  double theta_src, theta_dst, theta_delta;
  std::vector<double>delta_list;

public:
  /**
   * \brief Vision Compass Initialize
   *
   * \param fx - camera intrinsic parameters
   *
   */
  void Init(double fx, double fy, double ucx, double ucy);

  /**
   * \brief Load collected images and infomation
   */
  void loadInfo();

  /**
   * \brief match current frame with pre-built panorama to correct yaw
   *
   * \param target_img - current frame
   * 
   * \param imu_yaw - current imu_yaw
   * 
   * \param head_yaw - part of current head command
   * 
   * \return - error of imu
   *
   */
  double Process(const cv::Mat &target_img, double imu_yaw, double head_yaw);

  /**
   * \brief Calculate raw theta for keypoints under camera coordinate
   *
   * \param point - point in image coordinate
   *
   * \return theta of point in camera coordinate
   */
  double CalRawTheta(cv::Point2f p)
  {
      return atan((p.x - cx_)/fx_)*180 / PI;
  }

};

}  // namespace dvision