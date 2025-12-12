/* Copyright (C) ZJUDancer
 * 2017 - Yusu Pan <xxdsox@gmail.com>
 */

/**
 * \file obstacle_detector.cpp
 * \author Yusu Pan <xxdsox@gmail.com>
 * \version 2018
 * \date 2018-02-17
 */
#include "dvision/obstacle_detector.hpp"

using namespace dancer_geometry;

namespace dvision {
ObstacleObject::ObstacleObject(cv::Point2f position, float confidence, int id)
    : position_(position), confidence_(confidence), id_(id) {}

bool ObstacleObject::DecayConfidence() {
  confidence_ *= parameters.obstacle.decayConfidence;
  return confidence_ > parameters.obstacle.minValidConfidence;
}

bool ObstacleObject::Update(cv::Point2f pos, float conf) {
  // if input was similar to this obstacle then updated self
  if (GetDistance(pos, position_) <= parameters.obstacle.maxPossibleJump) {
    boundry_n(conf, 0.0, 1.0);
    LowPass(pos, position_, conf * parameters.obstacle.lowPassCoef);
    confidence_ = conf;
    return true;
  } else {
    // if input obstacle is not similar to this obstacle then do nothing
    return false;
  }
}

bool ObstacleDetector::Init() { return true; }

void ObstacleDetector::Update() {
  for (auto it = obstacle_objects_.begin(); it != obstacle_objects_.end();) {
    if (!it->DecayConfidence()) {
      it = obstacle_objects_.erase(it);
    } else {
      ++it;
    }
  }
}

void ObstacleDetector::Detect(const cv::Mat& hsv_img, cv::Mat& obstacle_binary,
                              cv::Mat& gui_img,
                              const cv::Mat& field_convex_hull,
                              Projection& projection, VisionInfo& vision_info) {
  if (!parameters.obstacle.enable) {
    return;
  }
  obstacle_binary = cv::Mat::zeros(hsv_img.size(), CV_8UC1);

  Timer t;
  if (vision_info.see_field) {
    GetBinary(hsv_img, field_convex_hull, obstacle_binary);
    vision_info.see_obstacle = Process(obstacle_binary, gui_img, projection);
  }

  if (vision_info.see_obstacle) {
    geometry_msgs::Vector3 tmp;
    for (const auto& obstacle : obstacle_points_) {
      tmp.x = obstacle.x;
      tmp.y = obstacle.y;
      vision_info.obstacles_field.push_back(tmp);
    }
  }
  ROS_DEBUG("obstacle detect used: %lf ms", t.elapsedMsec());
}

bool ObstacleDetector::Process(cv::Mat& obstacle_binary, cv::Mat& gui_img,
                               Projection& projection) {
  // //! Update obstacle objects
  // Update();

  //! Find obstacle contours
  std::vector<std::vector<cv::Point>> obstacle_contour_candidates;
  // cv::Mat obstacle_mask_ = cv::Mat::zeros(obstacle_binary.size(), CV_8UC1);
  cv::findContours(obstacle_binary, obstacle_contour_candidates,
                   CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
  obstacle_points_.clear();
  obstacle_points_.reserve(obstacle_contour_candidates.size());

  //! Iterate through each contour
  for (size_t i = 0; i < obstacle_contour_candidates.size(); i++) {
    //! Remove obstacle whose area is too small
    cv::approxPolyDP(
        obstacle_contour_candidates[i], obstacle_contour_candidates[i],
        cv::arcLength(obstacle_contour_candidates[i], true) * 0.003, true);
    float area = cv::contourArea(obstacle_contour_candidates[i]);
    if (area <= parameters.obstacle.minArea) {
      continue;
    }

    //! Get down point of obstacle
    cv::Rect bounding_rect = cv::boundingRect(obstacle_contour_candidates[i]);
    cv::Point btn_point(bounding_rect.x + bounding_rect.width / 2,
                        bounding_rect.y + bounding_rect.height);
    boundry_n(btn_point.x, 0, parameters.camera.width - 1);
    boundry_n(btn_point.y, 0, parameters.camera.height - 1);
    cv::Point2f btn_real;
    if (!projection.getOnRealCoordinate(btn_point, btn_real)) {
      ROS_ERROR("Error in programming!");
    }

    //! Show obstacle
    if (parameters.monitor.update_gui_img &&
        parameters.obstacle.showAllObstacles) {
      cv::drawContours(gui_img, obstacle_contour_candidates, i, whiteColor(), 2,
                       8);
    }

    //! Filter obstacle with valid distance from robot
    float distance_to_robot = GetDistance(btn_real);
    if (distance_to_robot >= parameters.obstacle.minDistance &&
        distance_to_robot <= parameters.obstacle.maxDistance) {
      obstacle_points_.push_back(btn_real);
      //! Draw on obstacle_mask
      // cv::drawContours(obstacle_mask_, obstacle_contour_candidates, i,
      // grayWhite(), CV_FILLED, 8);
      //! Draw on gui img
      if (parameters.monitor.update_gui_img &&
          parameters.obstacle.showResObstacles) {
        cv::drawContours(gui_img, obstacle_contour_candidates, i, yellowColor(),
                         2, 8);
        cv::circle(gui_img, btn_point, 3, blackColor(), 2);
      }
      // ROS_INFO("distance to robot: %f", distance_to_robot);
      // ROS_INFO("obstacle_point_real (%f, %f)", btn_real.x, btn_real.y);
    }
  }

  std::sort(obstacle_points_.begin(), obstacle_points_.end(),
            [](const cv::Point2f& lhs, const cv::Point2f& rhs) {
              return GetDistance(lhs) < GetDistance(rhs);
            });

  // add down point of obstacles to obstacle_objects_
  // if (obstacle_points_.size() > 0) {
  //     for (auto obstacle_point : obstacle_points_) {
  //         bool registered = false;
  //         // update registered obstacle object
  //         for (auto obstacle_object : obstacle_objects_) {
  //             if (obstacle_object.Update(obstacle_point, 1)) {
  //                 registered = true;
  //                 break;
  //             }
  //         }
  //         // add new obstacle object
  //         if (!registered) {
  //             int id = 1;
  //             if (obstacle_objects_.size() > 0) {
  //                 id = obstacle_objects_[obstacle_objects_.size() - 1].id() +
  //                 1;
  //             }
  //             obstacle_objects_.emplace_back(obstacle_point, 1, id);
  //         }
  //     }
  // }

  // ROS_INFO("detect obstacle: %d", obstacle_points_.size() > 0);
  return !obstacle_points_.empty();
}

void ObstacleDetector::GetBinary(const cv::Mat& hsv_img,
                                 const cv::Mat& field_convex_hull,
                                 cv::Mat& obstacle_binary) {
  if (!parameters.obstacle.enable) {
    return;
  }

  if (field_convex_hull.cols != parameters.camera.width &&
      field_convex_hull.rows != parameters.camera.height) {
    return;
  }

  //! Find obstacles in HSV image
  cv::inRange(hsv_img,
              cv::Scalar(parameters.obstacle.h0, parameters.obstacle.s0,
                         parameters.obstacle.v0),
              cv::Scalar(parameters.obstacle.h1, parameters.obstacle.s1,
                         parameters.obstacle.v1),
              obstacle_binary);

  //! Limit obstacles in the field
  cv::bitwise_and(field_convex_hull, obstacle_binary, obstacle_binary);
  // cv::bitwise_not(obstacle_binary, obstacle_binary);
  // cv::bitwise_and(field_convex_hull, obstacle_binary, obstacle_binary);

  //! Process binary image with close operation
  if (parameters.obstacle.erode_1 > 0) {
    cv::erode(obstacle_binary, obstacle_binary, cv::Mat(), cv::Point(-1, -1),
              parameters.obstacle.erode_1);
  }
  if (parameters.obstacle.dilate_1 > 0) {
    cv::dilate(obstacle_binary, obstacle_binary, cv::Mat(), cv::Point(-1, -1),
               parameters.obstacle.dilate_1);
  }

  if (parameters.obstacle.erode_2 > 0) {
    cv::erode(obstacle_binary, obstacle_binary, cv::Mat(), cv::Point(-1, -1),
              parameters.obstacle.erode_2);
  }
  if (parameters.obstacle.dilate_2 > 0) {
    cv::dilate(obstacle_binary, obstacle_binary, cv::Mat(), cv::Point(-1, -1),
               parameters.obstacle.dilate_2);
  }
}

}  // namespace dvision
