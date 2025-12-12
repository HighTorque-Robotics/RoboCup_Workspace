/* Copyright (C) ZJUDancer
 * 2018 - Yusu Pan <xxdsox@gmail.com>
 * 2017 - Yujie Yang <meetyyj@gmail.com>
 */

/**
 * \file line_classifier.hpp
 * \author Yusu Pan, Yujie Yang
 * \version 2018
 * \date 2018-02-17
 */

#pragma once
#include "dancer_geometry/angle.hpp"
#include "dancer_geometry/line_segment.hpp"
#include "dancer_geometry/utils.hpp"
#include "dmsgs/VisionInfo.h"
#include "dvision/kalman.hpp"
#include "dvision/parameters.hpp"
#include "dvision/projection.hpp"
#include "dvision/timer.hpp"

#include <ros/ros.h>
#include <Eigen/StdVector>
#include <algorithm>
#include <cmath>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

namespace dvision {
using dmsgs::VisionInfo;

//! Classifier for determining line type
class LineClassifier {
 public:
  //! LineClassifier constructor
  explicit LineClassifier();
  //! LineClassifier destructor
  ~LineClassifier();

  //! Initialize LineClassifier
  bool Init();
  // dmsgs::VisionInfo classifier;
  // bool Update();

  /**
   * \brief Process
   *
   * \param good_lines - resulted lines from LineDetector
   * \param result_circle - resulted circle center from CircleDetector
   * \param goal_position - resulted goal post points from GoalDetector
   * \param projection - used for projecting from image plane to field plane
   * \param vision_info - used for storaging detected line info
   * \param vision_yaw - corrected yaw value (field angle) of robot
   *
   * \return whether or not process is successful
   */
  bool Process(std::vector<dancer_geometry::LineSegment> &good_lines,
               const cv::Point2f &result_circle,
               const std::vector<cv::Point2f> &goal_position,
               Projection &projection, VisionInfo &vision_info,
               const double &vision_yaw);

 private:
  //! Center lines in the field (across the center circle)
  std::vector<dancer_geometry::LineSegment> center_lines_;
  //! Goal post lines
  std::vector<dancer_geometry::LineSegment> goal_lines_;
  //! Other lines
  std::vector<dancer_geometry::LineSegment> other_lines_;
  //! Bias for correcting vision yaw
  std::vector<dancer_geometry::Angle> yaw_correct_bias_;

  //! Calculate bias for vision yaw
  double CalYawBias();
};
}  // namespace dvision
