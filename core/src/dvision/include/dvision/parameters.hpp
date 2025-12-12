/* Copyright (C) ZJUDancer
 * 2018 - Yusu Pan <xxdsox@gmail.com>
 * 2017 - Wenxing Mei <mwx36mwx@gmail.com>
 */

/**
 * \file parameters.hpp
 * \author Yusu Pan, Wenxing Mei
 * \version 2018
 * \date 2018-02-14
 */

#pragma once
#include <ros/ros.h>

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

namespace dvision {

//! Parameters for camera
struct CameraParameters {
  //! Image size of camera
  cv::Size imageSize;
  //! Intrinsic matrix of camera
  cv::Mat cameraMatrix;
  //! Distort coeffients of camera
  cv::Mat distCoeff;

  //! Undistorted image size of camera
  cv::Size undistImageSize;
  //! Undistorted intrinsic matrix of camera
  cv::Mat undistCameraMatrix;
  //! Undistorted distort coeffients of camera
  cv::Mat undistDistCoeff;

  //! Focal length in x axis
  double fx;
  //! Focal length in y axis
  double fy;
  //! Camera's principal point in x axis
  double cx;
  //! Camera's principal point in y axis
  double cy;

  //! Undistorted camera's principal point in x axis
  double undistCx;
  //! Undistorted camera's principal point in y axis
  double undistCy;
  //! Undistorted camera's origin point in x axis
  double centerInUndistX;  // oringin center in undist
  //! Undistorted camera's origin point in y axis
  double centerInUndistY;

  //! Image width of camera
  int width;
  //! Image height of camera
  int height;

  //! Undistorted image width of camera
  int undistWidth;
  //! Undistorted image height of camera
  int undistHeight;

  // 16 extrinsic parameters, see meaning in Matlab code: main.m
  //! Extrinsic parameters of camera
  std::vector<double> extrinsic_para;
  //! White balance gain
  std::vector<double> bgr_gains = {1, 1, 1};
  //! Flag for debug
  bool debug = false;
};

//! Parameters for calibration
struct CalibParameters {
  //! Flag for odometry calibration
  bool calibOdometer;
};

//! Parameters for kalman filter
struct KalmanFilterParameters {
  //! Maximum second to determine missing
  int maxMissSec;
  //! Flag for whether or not allows reset kalman filter
  bool allowReset;
  // float processNoiseCov;
  // float measurementNoiseCov;
  // float errorCovPost;
};

//! Parameters for object detector
struct ObjectDetectorParameters {
  //! Flag for whether or not enables object detector
  bool enable;
  //! Flag for whether or not shows all detection results
  bool showAllDetections;
  //! Network type (yolov2, yolov3, yolov3-tiny)
  std::string network_type;
  //! Built and serialized TensorRT model file
  std::string input_model_file;
  //! Class names list file
  std::string input_names_file;
  //! Anchor prior file name
  std::string anchors_file;
  //! Enable profiling
  bool enable_profiling;
  //! Detection threshold
  float threshold;
  //! Non-maxima suppression threshold
  float nms_threshold;
  //! Batch size
  int batch_size;
  //! Maximum scale factor of ball in the image
  float ball_max_scale_coff;
  //! Lower ratio of width and height of ball
  float ball_wh_low_ratio;
  //! Higher ratio of width and height of ball
  float ball_wh_high_ratio;
};

//! Parameters for obstacle detector
struct ObstacleDetectorParameters {
  //! Flag for whether or not enables obstacle detector
  bool enable;
  //! Flag for whether or not shows all obstable detections (before post
  //! process)
  bool showAllObstacles;
  //! Flag for whether or not shows only processed obstable results
  bool showResObstacles;
  //! Minimum hue value
  int h0;
  //! Maximum hue value
  int h1;
  //! Minimum saturation value
  int s0;
  //! Maximum saturation value
  int s1;
  //! Minimum value
  int v0;
  //! Maximum value
  int v1;
  float decayConfidence;
  float minValidConfidence;
  float lowPassCoef;
  float maxPossibleJump;
  int dilate_1;
  int erode_1;
  int dilate_2;
  int erode_2;
  float maxDistance;
  int minArea;
  int minDistance;
};

//! Parameters for ball detector
struct BallDetectorParameters {
  bool enable;
  bool showResult;
  bool useKalman;
  float kalmanMaxMiss;
  bool useRadiusCheck;
  float minBallRadiusRatio;
  float maxBallRadiusRatio;
  bool useInFieldCheck;
  float minBallToFieldDist;
  bool useDistCheck;
  float maxSeeBallDist;
  int minQueue;
  int maxQueue;
  bool velocityDebug;
};

//! Parameters for circle detector
struct CircleDetectorParameters {
  bool enable;
  bool useKalman;
  float kalmanMaxMiss;
};

//! Parameters for corner detector
struct CornerDetectorParameters {
  bool enable;
};

//! Parameters for goal
struct GoalDetectorParameters {
  bool enable;
  bool useKalman;
  float kalmanMaxMiss;
  int maxDistFromRobot;
  bool useGoalWidthCheck;
  bool useGoalAngleCheck;
  float minGoalWidthRatio;
  float maxGoalWidthRatio;
  float GoalAngleTh;
};

//! Parameters for line classifier
struct LineClassifierParameters {
  bool enable;
};

//! Parameters for vision yaw correction
struct YawCorrection {
  double angle2HorLine;
  double angle2VerLine;
  double minLineLen;
  int maxDistBothGoal;
  int maxDistSingleGoal;
  int yawCorrectNum;
  double circleLineLen;
  double goalLineLen;
  double otherLineLen;
  double robotMaxDist2GoalLine;
};

//! Parameters for monitor
struct MonitorParameters {
  bool use_cv_show;
  bool transmit_raw_img;
  bool update_gui_img;
  bool update_canny_img;
  bool update_field_binary;
  bool update_goal_binary;
  bool update_line_binary;
  bool update_obstacle_binary;
};

//! Range for HSV
struct HSVRange {
  //! Active flag (deprecated)
  bool active;
  //! Minimum hue value
  int h0;
  //! Maximum hue value
  int h1;
  //! Minimum saturation value
  int s0;
  //! Maximum saturation value
  int s1;
  //! Minimum value
  int v0;
  //! Maximum value
  int v1;
};

//! Parameters for AMCL
struct Amcl {
  bool enable;
  int trust_dist = 200;
  int max_dist = 300;
  double z_hit = 0.95;
  double z_rand = 0.05;
  double sigma_hit = 0.2;
  int max_occ_dist = 25;
  double dist_threshold = 50;
  double alpha_slow = 0.00001;
  double alpha_fast = 0.1;
  int resample_interval = 10;
  int num_particles = 300;
  double good_dist_th = 20;
  double good_angle_th = 5;
  double c_step_cost = 0.0005;
  int estimate_interval = 5;
  double consistency_th = 0.05;
  int max_resample_dist = 450;
  int resample_dist_increase = 10;
  bool debug_on_monitor = false;
};

struct FieldFeaturesPos {
  std::vector<std::vector<cv::Point2f>> all_points;
};

//! Global Parameters
struct Parameters {
  CalibParameters calib;
  ObjectDetectorParameters object;
  ObstacleDetectorParameters obstacle;
  BallDetectorParameters ball;
  CircleDetectorParameters circle;
  CornerDetectorParameters corner;
  GoalDetectorParameters goal;
  FieldFeaturesPos field_features;

  LineClassifierParameters line_classifier;
  YawCorrection yaw_correction;
  CameraParameters camera;
  MonitorParameters monitor;

  KalmanFilterParameters kalman;
  Amcl amcl;

  //! Network address for UDP broadcasting
  std::string udpBroadcastAddress;
  //! Robot ID
  int robotId;
  //! Flag for whether or not in simulation mode
  bool simulation;
  bool offline_record, offline_replay;
  bool broadcast_vision_info = false;
  bool vision_only_mode = false;
  // record has higher priority

  /**
   * \brief Init parameters
   *
   * \param nh - ROS node handler*
   */
  void init(ros::NodeHandle *nh);
  //! Update parameters from server
  void update();

 private:
  //! ROS node handler
  ros::NodeHandle *m_nh;
};

extern Parameters parameters;
}  // namespace dvision
