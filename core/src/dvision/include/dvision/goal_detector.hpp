/* Copyright (C) ZJUDancer
 * 2017 - Yusu Pan <xxdsox@gmail.com>, Yujie Yang <meetyyj@gmail.com>
 */

/**
 * \file goal_detector.hpp
 * \author Yusu Pan
 * \version 2018
 * \date 2018-02-15
 */

#pragma once
#include <ros/ros.h>
#include <algorithm>
#include <vector>
#include "dmsgs/VisionInfo.h"
#include "dvision/idetector.hpp"
#include "dvision/kalman.hpp"
#include "dvision/projection.hpp"
#include "dvision/timer.hpp"
#include "dvision/amcl/types.hpp"

#include "dvision/yolo/utils.hpp"

using dmsgs::VisionInfo;
namespace dvision {
//! Detector for goal posts
struct Control;
class GoalDetector : public IDetector {
 public:
  //! GoalDetector constructor
  explicit GoalDetector();
  //! GoalDetector destructor
  ~GoalDetector() override = default;
  //! Initialize GoalDetector
  bool Init() override;
  void Detect(const std::vector<BBox>& goalpost_bboxs,
              Projection& projection, VisionInfo& vision_info,
              const Control& delta, Measurement& marks);

  //! Get goal position in field plane
  inline cv::Point2f goal_position() {
    if (parameters.goal.useKalman) {
      return goal_position_kalman_;

    } else {
      return goal_position_;
    }
  }

 private:
  //! Rectangle of image boundary, used for limitting points
  cv::Rect boundary_rect_;
  //! Kalman filter for goal center
  ObjectPosKalmanFilter* goal_kalmanI_;
  //! Position of goal points in fiels plane
  cv::Point2f goal_position_;
  //! Position of goal points in field plane, corrected by kalman filter
  cv::Point2f goal_position_kalman_;
  //! Position of goal point candidates in field plane
  std::vector<cv::Point2f> goal_post_candidate_;

  //! Update prediction of goal position by kalman filter
  bool Update(const Control& delta);

  /**
   * \brief Process goal post detection
   *
   * \param goal_bbox - bounding boxes of detected goal by Object Detector
   * \param canny_img - image processed by canny edge detector
   * \param hsv_img - image from camera in HSV color space
   * \param gui_img - GUI image for drawing detections
   * \param field_hull - field boundary in image plane
   * \param projection - used for projecting from image plane to field plane
   *
   * \return whether or not process is successful
   */
  bool Process(const std::vector<BBox>& goalpost_bboxs,
               Projection& projection, VisionInfo& vision_info);

  /**
   * \brief Check width between goal posts and add save two posts (left goal
   * post first)
   */
  void CheckGoalWidthAngle(Projection &projection);
  float GetAngle(cv::Point2f& lhs, cv::Point2f& rhs);
};
}  // namespace dvision
