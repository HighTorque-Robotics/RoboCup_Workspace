/* Copyright (C) ZJUDancer
 * 2018 - Yusu Pan <xxdsox@gmail.com>
 * 2017 - Wenxing Mei <mwx36mwx@gmail.com>
 */

/**
 * \file dvision.hpp
 * \brief Main class of vision module
 * \author Yusu Pan, Wenxing Mei
 * \version 2018
 * \date 2018-02-12
 */

#pragma once
#include <fstream>
#include <mutex>
#include <thread>
#include "dmsgs/ActionCommand.h"
#include "dmsgs/BehaviorInfo.h"
#include "dmsgs/MotionInfo.h"
#include "dmsgs/SaveImg.h"
#include "std_msgs/String.h"
#include <boost/filesystem.hpp>
#include <cstdlib>
#include "dmsgs/VisionInfo.h"
//#include "dmsgs/VisionDebugInfo.h"

#include "dmsgs/ResetParticleLeftTouch.h"
#include "dmsgs/ResetParticlePoint.h"
#include "dmsgs/ResetParticleRandom.h"
#include "dmsgs/ResetParticleRightTouch.h"
#include "dmsgs/ResetParticleTouchLine.h"
#include "dmsgs/ToggleAMCL.h"
#ifdef USE_NEO_INFO
#include "dmsgs/SetInitOrientation.h"
#endif

#include "dancer_geometry/utils.hpp"
#include "dprocess/dconcurrent.hpp"
#include "dprocess/dprocess.hpp"
#include "dtransmit/dtransmit.hpp"
#include "dvision/amcl/amcl.hpp"
#include "dvision/ball_detector.hpp"
#include "dvision/ball_tracker.hpp"
#include "dvision/circle_detector.hpp"
#include "dvision/corner_detector.hpp"
#include "dvision/goal_detector.hpp"
#include "dvision/line_classifier.hpp"
#include "dvision/object_detector.hpp"
#include "dvision/obstacle_detector.hpp"
#include "dvision/projection.hpp"
#include "dvision/ringbuffer.hpp"

#ifdef USE_GST_CAMERA
#include "dvision/gst_camera.hpp"
#endif

#ifdef USE_ZED_CAMERA
#include "dvision/zed_camera.hpp"
#endif

#include "dvision/vision_shared_info.hpp"

#include "dcommon/profiler.hpp"

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>

namespace dvision {

//! Main class of Dvision
class DVision {
 public:
  //! Dvision constructor
  explicit DVision();
  //! Dvision destructor
  ~DVision();
  //! Start dvision module
  void Start();
  //! Join thread
  void Join();

 private:
  // For parallel
  //! Shared buffer for gui image
  RingBuffer<cv::Mat> buffer_gui_image_;
  //! Shared buffer for vision info
  RingBuffer<VisionSharedInfo> buffer_info_;

  //! Thread for step 1
  std::thread t1_;
  //! Thread for step 2
  std::thread t2_;
  //! Thread for transmitting gui image after step 1 and 2
  // std::thread t3_;

 private:
  //! Step for capturing frames and updating camera extrinsic parameters.
  void step1();
  //! Step for detecting objects, localizing and tracking.
  void step2();

 private:
  // For receiving an transmitting

  //! ROS subscriber listening to information from DMotion
  ros::Subscriber sub_motion_info_;
  //! ROS subscriber listening to information from DBehavior
  ros::Subscriber sub_behavior_info_;
  //! ROS subscriber listening to signal from config watchdog
  ros::Subscriber sub_reload_config_;
  //! ROS publisher for broadcasting topic messages of DVision
  ros::Publisher pub_;
  //    //! ROS publisher for broadcasting debug messages of DVision
  //    ros::Publisher pub_debug_;
  //! Image transport publisher for recording image
  image_transport::Publisher pub_image_;
  //! Image transport subscriber for replaying image
  image_transport::Subscriber sub_image_;

  //! ROS service server for toggling AMCL
  ros::ServiceServer serv_toggle_amcl_;
  //! ROS service server for toggling particle on touch line
  ros::ServiceServer serv_reset_particle_touch_line_;
  //! ROS service server for toggling particle left touch
  ros::ServiceServer serv_reset_particle_left_touch_;
  //! ROS service server for toggling particle right touch
  ros::ServiceServer serv_reset_article_right_touch_;
  //! ROS service server for resetting all particles
  ros::ServiceServer serv_reset_particle_point_;
#ifdef USE_NEO_INFO
  //! ROS service server for resetting yaw
  ros::ServiceServer serv_reset_yaw_;
#endif

  //! Pointer to instance of DTransmit
  /*! Used for broadcasting gui images and vision info */
  dtransmit::DTransmit* transmitter_;
  std::mutex lock_vision_yaw_;

 private:
  // Variables for thread1

  //! Profiler for step 1
  dcommon::Profiler profiler_step_1_;

  //! Pointer to Node handler for ROS
  ros::NodeHandle* nh_;
  
  //! Pointer to instance of Camera class
  /*! Used for capturing frames */
#ifdef USE_GST_CAMERA
  GstCamera* camera_;
#elif defined(USE_ZED_CAMERA)
  ZedCamera* camera_;
#else
  V4L2Camera* camera_;
#endif

  //! Instance of Projection class
  /*! Used for calculating projections and updating camera extrinsic parameters */
  Projection projection_;
  //! Instance of DConcurrent class
  /*! Used for implementing concurrent work */
  dprocess::DConcurrent concurrent_;
  //! Workspace for storing processed vision information
  VisionSharedInfo* workspace_;

  //! Instance of Frame class
  /*! Used for storing camera frames in various color spaces and encoding
   * themselves for transmitting
   */
  Frame frame_;

  //! Binary image w.r.t soccer field
  cv::Mat field_binary_;
  //! Binary image w.r.t goal
  cv::Mat goal_binary_;
  //! Binary image w.r.t white lines
  cv::Mat line_binary_;
  //! Binary image w.r.t obstacles in the field
  cv::Mat obstacle_binary_;

  //! Binary image processed by canny edge detector
  cv::Mat canny_img_;
  //! Counter for cycle 1 (deprecated)
  size_t cycle1_ = 0;

  //! Pitch value of camera platform
  double plat_pitch_ = 0;
  //! Yaw value (a.k.a field angle) of camera platform
  double plat_yaw_ = 0;

  //! Detectors for objects, e.g, ball, goal and robots.
  ObjectDetector obj_;
  //! Detectors for obstacles in the field.
  ObstacleDetector obstacle_;

  // Quality of ball (get from detector's probability of ball)
  float ball_quality_;

 private:
  // Variables for thread2

  //! Profiler for step 2
  dcommon::Profiler profiler_step_2_;

  //! Tracker for computing platform angle to make objects in the center of FOV
  Tracker tracker_;
  //! Another instance of Projection class in order to run in separate thread
  Projection projection2_;

  // Motion data
  //! Delta motion data (dx, dy, dt) of robot
  Control delta_;
  //! Previous delta motion data of robot
  geometry_msgs::Vector3 previous_delta_;
  //! Pose of robot (deprecated)
  Pose pose_;

  // Localization
  //! Instance of AMCL
  /*! Used for localization based on information of vision and IMU */
  AMCL amcl_;
  //! Measurement for AMCL
  Measurement measurement_;
  //! Flag for toggling AMCL
  bool enable_amcl_ = true;

  // Detectors
  //! Detector for the ball
  BallDetector ball_;
  //! Detector for the center circle
  CircleDetector circle_;
  //! Detector for the corner
  CornerDetector corner_;
  //! Classifier for the white lines to determine if it's horizontal or vertical
  LineClassifier line_classifier_;
  //! Detector for the goal
  GoalDetector goal_;
  //! Counter for cycle 2 (deprecated)
  size_t cycle2_ = 0;

  //! Velocity of ball in field plane
  cv::Point2f ball_velocity_;
  //! Position queue for velocity
  std::queue<BallPos> ball_pos_history_;
  std::ofstream log_file_;

  // Vision yaw
  //! Flag for whether IMU is initialized
  bool imu_inited_ = false;
  //! Current roll value from IMU
  double imu_roll_ = 0;
  //! Current pitch value from IMU
  double imu_pitch_ = 0;
  //! Previous yaw value from IMU
  double imu_yaw_pre_ = M_PI / 2;
  //! Current yaw value from IMU
  double imu_yaw_cur_ = M_PI / 2;
  //! Delta yaw value between current one and previous one
  double imu_yaw_delta_ = 0;
  //! Corrected yaw value of robot
  double vision_yaw_ = M_PI / 2;  // 机器人 yaw 角度

  //! Previous x position of robot in world coordinate
  double vision_x_pre_ = 0;  // 机器人前一帧的世界坐标系位置 x
  //! Previous y position of robot in world coordinate
  double vision_y_pre_ = 0;  // 机器人前一帧的世界坐标系位置 y
  //! Current x position of robot in world coordinate
  double vision_x_cur_ = 0;  // 机器人当前帧的世界坐标系位置 x
  //! Current y position of robot in world coordinate
  double vision_y_cur_ = 0;  // 机器人当前帧的世界坐标系位置 y
  //! Delta x value between current one and previous one in world coordinate
  double vision_x_delta_ = 0;  // 两帧间的世界坐标系位置差 delta_x
  //! Delta y value between current one and previous one in world coordinate
  double vision_y_delta_ = 0;  // 两帧间的世界坐标系位置差 delta_y

  // using for test
  //! Accumulated delta x value in world coordinate
  double vision_x_sum_ = 0;  // 两帧间的世界坐标系位置差 delta_x
  //! Accumulated delta y value in world coordinate
  double vision_y_sum_ = 0;  // 两帧间的世界坐标系位置差 delta_y
  //! Accumulated delta x value from IMU
  double dx_sum_ = 0;
  //! Accumulated delta y value from IMU
  double dy_sum_ = 0;
  //! Accumulated delta z value from IMU
  double dt_sum_ = 0;
  //! Counter of refreshing GUI image
  int gui_cnt_ = 0;

 private:
  //! Profiler for others
  dcommon::Profiler profiler_others_;

  //! Flag for whether robot is fallen down (deprecated)
  bool fallen_ = false;
  //! Flag for whether to save image per frame
  bool save_data_ = false;

  //! Time of last received motion info
  /*! Used for handling motion disconnected and fall down */
  ros::Time time_last_motion_recv = ros::Time(0);
  //! Time of last received motion connected
  ros::Time time_last_motion_conn = ros::Time(0);
  //! Flag for whether motion module of robot is connected to this computer
  bool motion_connected_ = false;
  //! Flag for whether robot is stable
  bool motion_stable_ = false;

  //! Time since robot start kick
  ros::Time time_start_kick_ = ros::Time(0);
  //! Flag for whether is crouching(best time for localization)
  bool crouching_ = false;

  /**
   * \brief Check connection status of motion module
   *
   * \return Whether or not motion module is connected
   */
  bool CheckMotionConnection();

  // TODO Handle penalty

  // dmsgs::VisionInfo m_vision_info;
  //! Information of vision module for simulation
  dmsgs::VisionInfo m_sim_vision_info;
  //! Information of motion module
  dmsgs::MotionInfo motion_info_;
  //! Information of behavior module
  dmsgs::BehaviorInfo m_behavior_info;

  cv::VideoCapture *cap_;

  //! Initialize values of angles from platform, IMU and vision.
  void InitVisionYaw();

  /**
   * \brief Show debug image with OpenCV imshow
   *
   * \param info - detected information of vision
   */
  void ShowDebugImg(VisionSharedInfo& info);

  // void TrackBall();

  /**
   * \brief Compute view range of robot for simulation
   *
   * \param info - detected information of vision
   */
  void UpdateViewRange(VisionInfo& info);

  /**
   * \brief Callback function for motion topic messages
   *
   * \param msg - received message of motion information
   */
  void MotionCallback(const dmsgs::MotionInfo::ConstPtr& msg);

  /**
   * \brief Callback function for bahvior topic messages
   *
   * \param msg - received message of behavior information
   */
  void BehaviorCallback(const dmsgs::BehaviorInfo::ConstPtr& msg);

  /**
   * \brief Callback function for reloading config message
   *
   * \param msg - received message of reloading config flag
   */
  void ReloadConfigCallback(const std_msgs::String::ConstPtr&);
  void imageCallback(const sensor_msgs::ImageConstPtr& msg);

  // Service
  /**
   * \brief Toggle AMCL module
   *
   * \param req - request of service
   * \param res - response of service
   *
   * \return whether service is operaterd successfully
   */
  bool toggleAMCL(dmsgs::ToggleAMCL::Request& req,
                  dmsgs::ToggleAMCL::Response& res);
  /**
   * \brief Reset particles ion touch line
   *
   * \param req - request of service
   * \param res - response of service
   *
   * \return whether service is operated successfully
   */
  bool ResetParticlesTouchLine(dmsgs::ResetParticleTouchLine::Request& req,
                               dmsgs::ResetParticleTouchLine::Response& res);
  /**
   * \brief Reset particles in left field
   *
   * \param req - request of service
   * \param res - response of service
   *
   * \return whether service is operated successfully
   */
  bool ResetParticlesLeftTouch(dmsgs::ResetParticleLeftTouch::Request& req,
                               dmsgs::ResetParticleLeftTouch::Response& res);
  /**
   * \brief Reset particles in right field
   *
   * \param req - request of service
   * \param res - response of service
   *
   * \return whether service is operated successfully
   */
  bool ResetParticlesRightTouch(dmsgs::ResetParticleRightTouch::Request& req,
                                dmsgs::ResetParticleRightTouch::Response& res);
  /**
   * \brief Reset all particles
   *
   * \param req - request of service
   * \param res - response of service
   *
   * \return whether service is operated successfully
   */
  bool ResetParticlesPoint(dmsgs::ResetParticlePoint::Request& req,
                           dmsgs::ResetParticlePoint::Response& res);
  /**
   * \brief Reset particles randomly
   *
   * \param req - request of service
   * \param res - response of service
   *
   * \return whether service is operated successfully
   */
  bool ResetParticlesRandom(dmsgs::ResetParticleRandom::Request& req,
                            dmsgs::ResetParticleRandom::Response& res);

#ifdef USE_NEO_INFO
  /**
   * \brief Reset yaw
   *
   * \param req - request of service
   * \param res - response of service
   *
   * \return whether service is operated successfully
   */
  bool SetInitOrientation(dmsgs::SetInitOrientation::Request& req,
                          dmsgs::SetInitOrientation::Response& res);
#endif
};
}  // namespace dvision

