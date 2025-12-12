/* Copyright (C) ZJUDancer
 * 2018 - Yusu Pan <xxdsox@gmail.com>
 * 2017 - Wenxing Mei <mwx36mwx@gmail.com>
 */

/**
 * \file parameters.cpp
 * \author Yusu Pan, Wenxing Mei
 * \version 2018
 * \date 2018-02-14
 */

#include "dvision/parameters.hpp"

#include <vector>
using std::vector;
using namespace cv;

namespace dvision {

#define GPARAM(x, y)                                    \
  do {                                                  \
    if (!m_nh->getParam(x, y)) {                        \
      ROS_FATAL("Projection get pararm " #x " error!"); \
    }                                                   \
  } while (0)

void Parameters::init(ros::NodeHandle *nh) {
  m_nh = nh;
  update();
}

void Parameters::update() {
  // Global parameters
  GPARAM("/ZJUDancer/Simulation", parameters.simulation);
  GPARAM("/ZJUDancer/OfflineRecord", parameters.offline_record);
  GPARAM("/ZJUDancer/OfflineReplay", parameters.offline_replay);
  GPARAM("/ZJUDancer/udpBroadcastAddress", parameters.udpBroadcastAddress);
  GPARAM("/ZJUDancer/BroadcastMVInfo", parameters.broadcast_vision_info);
  GPARAM("/ZJUDancer/VisionOnlyMode", parameters.vision_only_mode);

  // Private parameters
  GPARAM("RobotId", parameters.robotId);

  //    GPARAM("dvision/camera/debug", parameters.camera.debug);
  GPARAM("dvision/projection/fx", parameters.camera.fx);
  GPARAM("dvision/projection/fy", parameters.camera.fy);
  GPARAM("dvision/projection/cx", parameters.camera.cx);
  GPARAM("dvision/projection/cy", parameters.camera.cy);
  GPARAM("dvision/projection/extrinsic_para", parameters.camera.extrinsic_para);
  GPARAM("dvision/camera/width", parameters.camera.width);
  GPARAM("dvision/camera/height", parameters.camera.height);
  GPARAM("dvision/camera/bgr_gains", parameters.camera.bgr_gains);

  parameters.camera.cameraMatrix =
      (Mat_<double>(3, 3) << parameters.camera.fx, 0, parameters.camera.cx, 0,
       parameters.camera.fy, parameters.camera.cy, 0, 0, 1);

  vector<double> dist_coeff;
  GPARAM("dvision/projection/dist_coeff", dist_coeff);
  parameters.camera.distCoeff = Mat_<double>(1, dist_coeff.size());
  for (uint32_t i = 0; i < dist_coeff.size(); ++i) {
    parameters.camera.distCoeff.at<double>(0, i) = dist_coeff[i];
  }

  parameters.camera.imageSize =
      Size(parameters.camera.width, parameters.camera.height);

  // Calib
  GPARAM("dvision/calib/calibOdometer", parameters.calib.calibOdometer);

  // Get kalman filter parameters
  GPARAM("dvision/kalman/maxMissSec", parameters.kalman.maxMissSec);
  GPARAM("dvision/kalman/allowReset", parameters.kalman.allowReset);

  // GPARAM("dvision/kalman/processNoiseCov",
  // parameters.kalman.processNoiseCov);
  // GPARAM("dvision/kalman/measurementNoiseCov",
  // parameters.kalman.measurementNoiseCov);
  // GPARAM("dvision/kalman/errorCovPost", parameters.kalman.errorCovPost);

  // Get object detector parameters
  GPARAM("dvision/object_detector/enable", parameters.object.enable);
  GPARAM("dvision/object_detector/showAllDetections",
         parameters.object.showAllDetections);
  GPARAM("dvision/object_detector/network_type",
         parameters.object.network_type);
  GPARAM("dvision/object_detector/input_model_file",
         parameters.object.input_model_file);
  GPARAM("dvision/object_detector/input_names_file",
         parameters.object.input_names_file);
  GPARAM("dvision/object_detector/anchors_file",
         parameters.object.anchors_file);
  GPARAM("dvision/object_detector/enable_profiling",
         parameters.object.enable_profiling);
  GPARAM("dvision/object_detector/threshold", parameters.object.threshold);
  GPARAM("dvision/object_detector/nms_threshold",
         parameters.object.nms_threshold);
  GPARAM("dvision/object_detector/batch_size", parameters.object.batch_size);
  GPARAM("dvision/object_detector/ball_max_scale_coff",
         parameters.object.ball_max_scale_coff);
  GPARAM("dvision/object_detector/ball_wh_low_ratio",
         parameters.object.ball_wh_low_ratio);
  GPARAM("dvision/object_detector/ball_wh_high_ratio",
         parameters.object.ball_wh_high_ratio);

  // Get obstacle detector parameters
  GPARAM("dvision/obstacle_detector/enable", parameters.obstacle.enable);
  GPARAM("dvision/obstacle_detector/showAllObstacles",
         parameters.obstacle.showAllObstacles);
  GPARAM("dvision/obstacle_detector/showResObstacles",
         parameters.obstacle.showResObstacles);
  GPARAM("dvision/obstacle_detector/h0", parameters.obstacle.h0);
  GPARAM("dvision/obstacle_detector/h1", parameters.obstacle.h1);
  GPARAM("dvision/obstacle_detector/s0", parameters.obstacle.s0);
  GPARAM("dvision/obstacle_detector/s1", parameters.obstacle.s1);
  GPARAM("dvision/obstacle_detector/v0", parameters.obstacle.v0);
  GPARAM("dvision/obstacle_detector/v1", parameters.obstacle.v1);
  GPARAM("dvision/obstacle_detector/decayConfidence",
         parameters.obstacle.decayConfidence);
  GPARAM("dvision/obstacle_detector/minValidConfidence",
         parameters.obstacle.minValidConfidence);
  GPARAM("dvision/obstacle_detector/lowPassCoef",
         parameters.obstacle.lowPassCoef);
  GPARAM("dvision/obstacle_detector/maxPossibleJump",
         parameters.obstacle.maxPossibleJump);
  GPARAM("dvision/obstacle_detector/dilate_1", parameters.obstacle.dilate_1);
  GPARAM("dvision/obstacle_detector/erode_1", parameters.obstacle.erode_1);
  GPARAM("dvision/obstacle_detector/dilate_2", parameters.obstacle.dilate_2);
  GPARAM("dvision/obstacle_detector/erode_2", parameters.obstacle.erode_2);
  GPARAM("dvision/obstacle_detector/maxDistance",
         parameters.obstacle.maxDistance);
  GPARAM("dvision/obstacle_detector/minArea", parameters.obstacle.minArea);
  GPARAM("dvision/obstacle_detector/minDistance",
         parameters.obstacle.minDistance);

  // Get ball detector parameters
  GPARAM("dvision/ball_detector/enable", parameters.ball.enable);
  GPARAM("dvision/ball_detector/showResult", parameters.ball.showResult);
  GPARAM("dvision/ball_detector/useKalman", parameters.ball.useKalman);
  GPARAM("dvision/ball_detector/kalmanMaxMiss", parameters.ball.kalmanMaxMiss);
  GPARAM("dvision/ball_detector/useRadiusCheck",
         parameters.ball.useRadiusCheck);
  GPARAM("dvision/ball_detector/minBallRadiusRatio",
         parameters.ball.minBallRadiusRatio);
  GPARAM("dvision/ball_detector/maxBallRadiusRatio",
         parameters.ball.maxBallRadiusRatio);
  GPARAM("dvision/ball_detector/useInFieldCheck",
         parameters.ball.useInFieldCheck);
  GPARAM("dvision/ball_detector/minBallToFieldDist",
         parameters.ball.minBallToFieldDist);
  GPARAM("dvision/ball_detector/useDistCheck", parameters.ball.useDistCheck);
  GPARAM("dvision/ball_detector/maxSeeBallDist",
         parameters.ball.maxSeeBallDist);
  GPARAM("dvision/ball_detector/minQueue", parameters.ball.minQueue);
  GPARAM("dvision/ball_detector/maxQueue", parameters.ball.maxQueue);
  GPARAM("dvision/ball_detector/velocityDebug", parameters.ball.velocityDebug);

  // Get circle detector parameters
  GPARAM("dvision/circle_detector/enable", parameters.circle.enable);
  GPARAM("dvision/circle_detector/useKalman", parameters.circle.useKalman);
  GPARAM("dvision/circle_detector/kalmanMaxMiss",
         parameters.circle.kalmanMaxMiss);

  // Get corner detector parameters
  GPARAM("dvision/corner_detector/enable", parameters.corner.enable);

  // Get all field features
  std::vector<std::string> all_types;
  std::unordered_map<std::string, int> dict;
  GPARAM("dvision/field_features/all_types", all_types);
  parameters.field_features.all_points.resize(all_types.size());
  std::vector<int> tmp;
  for (size_t i = 0; i < all_types.size(); i++) {
    GPARAM("dvision/field_features/" + all_types[i], tmp);
    for (size_t j = 0; j < tmp.size(); j += 2)
      parameters.field_features.all_points[i].emplace_back(
          cv::Point(tmp[j], tmp[j + 1]));
  }

  // Get goal detector parameters
  GPARAM("dvision/goal_detector/enable", parameters.goal.enable);
  GPARAM("dvision/goal_detector/useKalman", parameters.goal.useKalman);
  GPARAM("dvision/goal_detector/kalmanMaxMiss", parameters.goal.kalmanMaxMiss);
  GPARAM("dvision/goal_detector/maxDistFromRobot",
         parameters.goal.maxDistFromRobot);
  GPARAM("dvision/goal_detector/useGoalWidthCheck",
         parameters.goal.useGoalWidthCheck);
  GPARAM("dvision/goal_detector/useGoalAngleCheck",
         parameters.goal.useGoalAngleCheck);
  GPARAM("dvision/goal_detector/GoalAngleTh", parameters.goal.GoalAngleTh);
  GPARAM("dvision/goal_detector/minGoalWidthRatio",
         parameters.goal.minGoalWidthRatio);
  GPARAM("dvision/goal_detector/maxGoalWidthRatio",
         parameters.goal.maxGoalWidthRatio);
  // Get line_classifier detector parameters
  GPARAM("dvision/line_classifier/enable", parameters.line_classifier.enable);

  // Get yaw_correction parameters
  GPARAM("dvision/yaw_correction/angle2HorLine",
         parameters.yaw_correction.angle2HorLine);
  GPARAM("dvision/yaw_correction/angle2VerLine",
         parameters.yaw_correction.angle2VerLine);
  GPARAM("dvision/yaw_correction/minLineLen",
         parameters.yaw_correction.minLineLen);
  GPARAM("dvision/yaw_correction/maxDistBothGoal",
         parameters.yaw_correction.maxDistBothGoal);
  GPARAM("dvision/yaw_correction/maxDistSingleGoal",
         parameters.yaw_correction.maxDistSingleGoal);
  GPARAM("dvision/yaw_correction/yawCorrectNum",
         parameters.yaw_correction.yawCorrectNum);
  GPARAM("dvision/yaw_correction/circleLineLen",
         parameters.yaw_correction.circleLineLen);
  GPARAM("dvision/yaw_correction/goalLineLen",
         parameters.yaw_correction.goalLineLen);
  GPARAM("dvision/yaw_correction/otherLineLen",
         parameters.yaw_correction.otherLineLen);
  GPARAM("dvision/yaw_correction/robotMaxDist2GoalLine",
         parameters.yaw_correction.robotMaxDist2GoalLine);

  // Get monitor parameters
  GPARAM("dvision/monitor/use_cv_show", parameters.monitor.use_cv_show);
  GPARAM("dvision/monitor/transmit_raw_img",
         parameters.monitor.transmit_raw_img);
  GPARAM("dvision/monitor/update_gui_img", parameters.monitor.update_gui_img);
  GPARAM("dvision/monitor/update_canny_img",
         parameters.monitor.update_canny_img);
  GPARAM("dvision/monitor/update_field_binary",
         parameters.monitor.update_field_binary);
  GPARAM("dvision/monitor/update_goal_binary",
         parameters.monitor.update_goal_binary);
  GPARAM("dvision/monitor/update_line_binary",
         parameters.monitor.update_line_binary);
  GPARAM("dvision/monitor/update_obstacle_binary",
         parameters.monitor.update_obstacle_binary);

  // amcl config
  GPARAM("dvision/amcl/trust_dist", parameters.amcl.trust_dist);
  GPARAM("dvision/amcl/max_dist", parameters.amcl.max_dist);
  GPARAM("dvision/amcl/alpha_fast", parameters.amcl.alpha_fast);
  GPARAM("dvision/amcl/alpha_slow", parameters.amcl.alpha_slow);
  GPARAM("dvision/amcl/consistency_th", parameters.amcl.consistency_th);
  GPARAM("dvision/amcl/c_step_cost", parameters.amcl.c_step_cost);
  GPARAM("dvision/amcl/dist_threshold", parameters.amcl.dist_threshold);
  GPARAM("dvision/amcl/enable", parameters.amcl.enable);
  GPARAM("dvision/amcl/estimate_interval", parameters.amcl.estimate_interval);
  GPARAM("dvision/amcl/good_angle_th", parameters.amcl.good_angle_th);
  GPARAM("dvision/amcl/good_dist_th", parameters.amcl.good_dist_th);
  GPARAM("dvision/amcl/resample_dist_increase",
         parameters.amcl.resample_dist_increase);
  GPARAM("dvision/amcl/max_resample_dist", parameters.amcl.max_resample_dist);
  GPARAM("dvision/amcl/num_particles", parameters.amcl.num_particles);
  GPARAM("dvision/amcl/resample_interval", parameters.amcl.resample_interval);
  GPARAM("dvision/amcl/sigma_hit", parameters.amcl.sigma_hit);
  GPARAM("dvision/amcl/z_hit", parameters.amcl.z_hit);
  GPARAM("dvision/amcl/z_rand", parameters.amcl.z_rand);
  GPARAM("dvision/amcl/debug_on_monitor", parameters.amcl.debug_on_monitor);
}

#undef GPARAM

Parameters parameters;
}  // namespace dvision
