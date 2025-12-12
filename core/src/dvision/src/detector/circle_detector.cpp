/* Copyright (C) ZJUDancer
 * 2017 - Yusu Pan <xxdsox@gmail.com>, Yujie Yang <meetyyj@gmail.com>
 */

/**
 * \file circle_detector.cpp
 * \author Yusu Pan, Yujie Yang
 * \version 2018
 * \date 2018-02-15
 */

#include "dvision/circle_detector.hpp"

#include "dconfig/dconstant.hpp"
#include "dvision/parameters.hpp"

using namespace dancer_geometry;

namespace dvision {
CircleDetector::CircleDetector() {}

bool CircleDetector::Init() {
  kalmanI_ = new ObjectPosKalmanFilter(circle_field_kalman_,
                                       parameters.circle.kalmanMaxMiss);
  ROS_DEBUG("CircleDetector Init");
  return true;
}

void CircleDetector::Detect(std::vector<cv::Point2f>& x_intx,
                            Projection& projection, VisionInfo& vision_info,
                            const Control& delta, Measurement& marks) {
  Timer t;
  if (x_intx.size() >= 2) {
    vision_info.see_circle = Process(x_intx, projection, delta);
  }
  if (vision_info.see_circle) {
    vision_info.circle_field.x = result_circle().x;
    vision_info.circle_field.y = result_circle().y;
    // add center circle to land marks
    marks.center_point.emplace_back(Measurement::LandMark());
    marks.center_point.back().ref.emplace_back(0, 0);
    marks.center_point.back().pred.x = result_circle().x;
    marks.center_point.back().pred.y = result_circle().y;
    float dist = dancer_geometry::GetDistance(result_circle_);
    marks.center_point.back().weight = Measurement::getWeight(
        50, dist, parameters.amcl.trust_dist, parameters.amcl.max_dist);
  }
  ROS_DEBUG("circle detect used: %lf ms", t.elapsedMsec());
}
bool CircleDetector::Process(std::vector<cv::Point2f>& x_intx,
                             Projection& projection, const Control& delta) {
  result_circle_.x = UNKNOWN_OBJ_POS;
  result_circle_.y = UNKNOWN_OBJ_POS;
  float norm_dist = 0, min_dist = 2,
        dist_range = dconstant::geometry::centerCircleDiameter * 2 * 0.2;
  LineSegment circle_diameter =
      LineSegment(cv::Point2f(0, dconstant::geometry::center_circle_radius),
                  cv::Point2f(0, -dconstant::geometry::center_circle_radius));
  find_pairwise(
      x_intx.begin(), x_intx.end(), [&](cv::Point2f& pt1, cv::Point2f& pt2) {
        norm_dist = 0;
        dancer_geometry::LineSegment tmp(pt1, pt2);
        projection.RotateTowardHeading(tmp);
        if (parameters.goal.useGoalAngleCheck) {
          float angle = tmp.GetAbsMinAngleDegree(circle_diameter);
          // ROS_INFO("angle: %.2f", angle);
          if (angle > parameters.goal.GoalAngleTh) {
            return;
          }
          norm_dist += angle / parameters.goal.GoalAngleTh;
        }

        if (parameters.goal.useGoalWidthCheck) {
          float diameter = tmp.GetLength();
          ROS_INFO("diameter: %.2f", diameter);
          if (diameter < dconstant::geometry::centerCircleDiameter *
                             parameters.goal.minGoalWidthRatio ||
              diameter > dconstant::geometry::centerCircleDiameter *
                             parameters.goal.maxGoalWidthRatio) {
            return;
          }
          norm_dist +=
              abs(diameter - dconstant::geometry::centerCircleDiameter) /
              dist_range;
        }

        if (norm_dist < min_dist) {
          result_circle_.x = (pt1.x + pt2.x) / 2;
          result_circle_.y = (pt1.y + pt2.y) / 2;
          min_dist = norm_dist;
        }
      });
  bool see_circle = Update(delta);
  return see_circle;
}
bool CircleDetector::Update(const Control& delta) {
  if (parameters.circle.useKalman) {
    bool see_circle = kalmanI_->Update(
        cv::Point2d(result_circle_.x, result_circle_.y), delta);
    circle_field_kalman_ = kalmanI_->GetResult();
    return see_circle;
  }
  return !std::isnan(result_circle_.x);
}
}  // namespace dvision
