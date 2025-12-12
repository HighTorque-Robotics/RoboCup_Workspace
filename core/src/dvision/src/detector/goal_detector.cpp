/* Copyright (C) ZJUDancer
 * 2017 - Yusu Pan <xxdsox@gmail.com>, Yujie Yang <meetyyj@gmail.com>
 */

/**
 * \file goal_detector.cpp
 * \author Yusu Pan
 * \version 2018
 * \date 2018-02-15
 */

#include "dvision/goal_detector.hpp"

#include "dvision/parameters.hpp"

using namespace dancer_geometry;

namespace dvision {
GoalDetector::GoalDetector() {}

bool GoalDetector::Init() {
  goal_kalmanI_ =
      new ObjectPosKalmanFilter(cv::Point2f(UNKNOWN_OBJ_POS, UNKNOWN_OBJ_POS),
                                parameters.goal.kalmanMaxMiss);
  // ROS_DEBUG("GoalDetector Init");
  boundary_rect_.x = 0;
  boundary_rect_.y = 0;
  boundary_rect_.width = parameters.camera.width;
  boundary_rect_.height = parameters.camera.height;

  // left_goal_kalmanI_->Init(cv::Point2f(UNKNOWN_OBJ_POS, UNKNOWN_OBJ_POS));
  // right_goal_kalmanI_->Init(cv::Point2f(UNKNOWN_OBJ_POS, UNKNOWN_OBJ_POS));
  return true;
}

void GoalDetector::Detect(const std::vector<BBox> &goalpost_bboxs,
                          Projection &projection, VisionInfo &vision_info,
                          const Control &delta, Measurement &marks) {
  Timer t;

  // clear
  goal_post_candidate_.clear();

  Process(goalpost_bboxs, projection, vision_info);
  vision_info.see_goal = Update(delta);

  if (vision_info.see_goal) {
    vision_info.goal_field.x = goal_position().x;
    vision_info.goal_field.y = goal_position().y;
    // add goal center to land marks
    marks.goal_center.emplace_back(Measurement::LandMark());
    for (int i = -1; i < 2; i += 2)
      marks.goal_center.back().ref.emplace_back(
          i * dconstant::geometry::field_length_half, 0);
    marks.goal_center.back().pred.x = goal_position().x;
    marks.goal_center.back().pred.y = goal_position().y;
    marks.goal_center.back().weight = Measurement::getWeight(
        40, dancer_geometry::GetDistance(goal_position_),
        parameters.amcl.trust_dist, parameters.amcl.max_dist);
  }
  // goal post as field feature
  for (auto &g : goal_post_candidate_) {
    vision_info.features_field.emplace_back();
    vision_info.features_field.back().feature = dmsgs::FieldFeature::GOAL_POST;
    vision_info.features_field.back().x = g.x;
    vision_info.features_field.back().y = g.y;
    // add goal posts to land marks
    marks.field_points.emplace_back(Measurement::LandMark());
    for (int i = -1; i < 2; i += 2)
      for (int j = -1; j < 2; j += 2)
        marks.field_points.back().ref.emplace_back(
            i * dconstant::geometry::field_length_half,
            j * dconstant::geometry::goal_width_half);
    marks.field_points.back().pred.x = g.x;
    marks.field_points.back().pred.y = g.y;
    cv::Point2f pt(marks.field_points.back().pred);
    marks.field_points.back().type = dmsgs::FieldFeature::GOAL_POST;
    marks.field_points.back().weight = Measurement::getWeight(
        20.0, dancer_geometry::GetDistance(marks.field_points.back().pred),
        parameters.amcl.trust_dist, parameters.amcl.max_dist);
  }
  ROS_DEBUG("goal detect used: %lf ms", t.elapsedMsec());
}

bool GoalDetector::Update(const Control &delta) {
  if (parameters.goal.useKalman) {
    // if see both goal, just update kalman filter for each goal and get
    // correction
    bool see_goal;
    goal_position_kalman_ = cv::Point2f(UNKNOWN_OBJ_POS, UNKNOWN_OBJ_POS);
    // get goal center kalman correction
    see_goal = goal_kalmanI_->Update(goal_position_, delta);
    goal_position_kalman_ = goal_kalmanI_->GetResult();

    return see_goal;
  }
  return !std::isnan(goal_position_.x);
}

bool GoalDetector::Process(const std::vector<BBox> &goalpost_bboxs,
                           Projection &projection, VisionInfo &vision_info) {
  ROS_DEBUG("goal detector tick");
  if (!parameters.goal.enable) {
    return false;
  }
  // insert goal_bbox to goal_post_candidate_
  for (const auto &goal_bbox : goalpost_bboxs) {
    cv::Point2f goal_image_(
        (goal_bbox.left_top.x + goal_bbox.right_bottom.x) / 2.0,
        goal_bbox.right_bottom.y);  // midpoint as the goal post
    cv::Point2f goal_field;
    projection.getOnRealCoordinate(goal_image_, goal_field);
    if (goal_field.x < -10) {
      continue;
    }

    goal_post_candidate_.push_back(goal_field);
  }

  goal_position_ = cv::Point2f(UNKNOWN_OBJ_POS, UNKNOWN_OBJ_POS);
  if (parameters.goal.useGoalWidthCheck || parameters.goal.useGoalAngleCheck) {
    CheckGoalWidthAngle(projection);
  }
  return true;
}

float GoalDetector::GetAngle(cv::Point2f &lhs, cv::Point2f &rhs) {
  float horiz_diff = std::abs(lhs.x - rhs.x);
  float vert_diff = std::abs(lhs.y - rhs.y);
  //(lhs.y - rhs.y > 0) ? lhs.y - rhs.y : rhs.y - lhs.y;
  // ideally, horiz_diff is 0 and vert_diff is goal_width
  return std::atan(horiz_diff / vert_diff) / 3.14159265 * 180;  // in degree
}

void GoalDetector::CheckGoalWidthAngle(Projection &projection) {
  float min_dist = 2., norm_dist;
  float dist_range =
      dconstant::geometry::goalWidth *
      (parameters.goal.maxGoalWidthRatio - parameters.goal.minGoalWidthRatio);
  find_pairwise(
      goal_post_candidate_.begin(), goal_post_candidate_.end(),
      [&](cv::Point2f &lhs, cv::Point2f &rhs) {
        norm_dist = 0;
        if (parameters.goal.useGoalAngleCheck) {
          cv::Point2f lhs_rotated = projection.RotateTowardHeading(lhs);
          cv::Point2f rhs_rotated = projection.RotateTowardHeading(rhs);
          float angle = GetAngle(lhs_rotated, rhs_rotated);
          if (angle > parameters.goal.GoalAngleTh) {
            return;
          }
          norm_dist += angle / parameters.goal.GoalAngleTh;
        }

        if (parameters.goal.useGoalWidthCheck) {
          float goal_dis = GetDistance(lhs, rhs);
          if (goal_dis < dconstant::geometry::goalWidth *
                             parameters.goal.minGoalWidthRatio ||
              goal_dis > dconstant::geometry::goalWidth *
                             parameters.goal.maxGoalWidthRatio) {
            return;
          }
          norm_dist +=
              abs(goal_dis - dconstant::geometry::goalWidth) / dist_range;
        }

        if (norm_dist < min_dist) {
          goal_position_.x = (lhs.x + rhs.x) / 2;
          goal_position_.y = (lhs.y + rhs.y) / 2;
          min_dist = norm_dist;
        }
      });
}

}  // namespace dvision
