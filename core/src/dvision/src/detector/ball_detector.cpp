/* Copyright (C) ZJUDancer
 * 2017 - Yusu Pan <xxdsox@gmail.com>
 */

/**
 * \file ball_detector.cpp
 * \author Yusu Pan <xxdsox@gmail.com>
 * \version 2018
 * \date 2018-02-15
 */

#include "dvision/ball_detector.hpp"

#include "dancer_geometry/utils.hpp"
#include "dconfig/dconstant.hpp"
#include "dvision/parameters.hpp"

using namespace dancer_geometry;

namespace dvision {
BallDetector::BallDetector()
    : ball_image_(UNKNOWN_OBJ_POS, UNKNOWN_OBJ_POS),
      ball_image_top_(UNKNOWN_OBJ_POS, UNKNOWN_OBJ_POS),
      ball_image_bottom_(UNKNOWN_OBJ_POS, UNKNOWN_OBJ_POS),
      ball_image_radius_(3.0),
      ball_field_(UNKNOWN_OBJ_POS, UNKNOWN_OBJ_POS),
      ball_field_kalman_(UNKNOWN_OBJ_POS, UNKNOWN_OBJ_POS) {}

bool BallDetector::Init() {
  if (!parameters.ball.enable) return false;
  kalmanI_ = new ObjectPosKalmanFilter(ball_field_kalman_,
                                       parameters.ball.kalmanMaxMiss);

  // kalmanI_->Init(ball_field_kalman_);

  ROS_DEBUG("BallDetector Init");
  return true;
}

bool BallDetector::Update(const Control& delta) {
  if (parameters.ball.useKalman) {
    bool see_ball =
        kalmanI_->Update(cv::Point2d(ball_field_.x, ball_field_.y), delta);
    ball_field_kalman_ = kalmanI_->GetResult();
    return see_ball;
  }
  return !std::isnan(ball_field_.x);
}

void BallDetector::Detect(const BBox& ball_position, cv::Mat& gui_img,
                          std::vector<cv::Point2f>& field_hull_real,
                          cv::Mat& field_binary, Projection& projection,
                          VisionInfo& vision_info, const Control& delta) {
  Timer t;

  vision_info.see_ball = Process(ball_position, gui_img, field_hull_real,
                                 field_binary, projection, delta);
  if (vision_info.see_ball) {
    // get ball field
    vision_info.ball_field.x = ball_field().x;
    vision_info.ball_field.y = ball_field().y;

    // get ball velocity from kalman
    // ball_velocity_ = kalmanI_->GetVelocity();
    // vision_info.ball_velocity.x = ball_velocity_.x;
    // vision_info.ball_velocity.y = ball_velocity_.y;

    // ROS_INFO("ball_field_raw (%f, %f)", ball_field_.x, ball_field_.y);
    // ROS_INFO("ball_field_kalman (%f, %f)", ball_field_kalman_.x,
    // ball_field_kalman_.y);

    // std::cerr << ball_field_.x << " " << ball_field_.y << " " <<
    // ball_field_kalman_.x << " " << ball_field_kalman_.y << std::endl;
  }

  ROS_DEBUG("ball detect used %lf ms", t.elapsedMsec());
}

bool BallDetector::Process(const BBox& ball_position, cv::Mat& gui_img,
                           std::vector<cv::Point2f>& field_hull_real,
                           cv::Mat& field_binary, Projection& projection,
                           const Control& delta) {
  // return if not enabled
  if (!parameters.ball.enable) {
    return false;
  }

  if (ball_position.class_label_prob >= 0.5) {
    ball_image_.x =
        (ball_position.left_top.x + ball_position.right_bottom.x) / 2.0;
    ball_image_.y =
        (ball_position.left_top.y + ball_position.right_bottom.y) / 2.0;
    ball_image_top_ =
        cv::Point(ball_position.left_top.x, ball_position.left_top.y);
    ball_image_bottom_ =
        cv::Point(ball_position.right_bottom.x, ball_position.right_bottom.y);
    projection.getOnRealCoordinate(ball_image_, ball_field_,
                                   dconstant::geometry::ballDiameter / 2.0);
    CheckBallDist();
    CheckBallRadius(projection);
    // std::cout<<ball_image_<<"---"<<ball_field_<<std::endl;
  } else {
    // ball_image_.x = parameters.kalman.UNKNOWN_POS;
    // ball_image_.y = parameters.kalman.UNKNOWN_POS;
    // ball_field_.x = static_cast<float>(parameters.kalman.UNKNOWN_POS);
    // ball_field_.y = static_cast<float>(parameters.kalman.UNKNOWN_POS);
    // ball_image_.x = NAN_POS;
    // ball_image_.y = NAN_POS;
    ball_field_.x = UNKNOWN_OBJ_POS;
    ball_field_.y = UNKNOWN_OBJ_POS;
  }

  // update kalman filter
  bool see_ball = Update(delta);

  // check in field
  if (parameters.ball.useInFieldCheck) {
    see_ball &= CheckBallInField(field_hull_real);
  }

  // draw gui image
  if (parameters.monitor.update_gui_img && parameters.ball.showResult &&
      see_ball) {
    // ROS_INFO("ball(%d,%d) radius(%d)", ball_image_.x, ball_image_.y,
    // static_cast<int>(ball_image_radius_));
    cv::circle(gui_img, ball_image_, static_cast<int>(ball_image_radius_),
               redColor(), 2);
  }

  return see_ball;
}

bool BallDetector::CheckBallInField(std::vector<cv::Point2f>& field_hull_real) {
  if (!parameters.ball.useInFieldCheck) {
    return true;
  }

  if (!field_hull_real.empty()) {
    double distance_to_field =
        cv::pointPolygonTest(field_hull_real, ball_field(), true);
    // if (distance_to_field < parameters.ball.minBallToFieldDist) {
    //     ROS_INFO("ball (%f, %f) not in field (distance=%f)", ball_field().x,
    //     ball_field().y, distance_to_field);
    return distance_to_field >= parameters.ball.minBallToFieldDist;
  } else {
    return true;
  }
}

bool BallDetector::CheckBallDist() {
  if (!parameters.ball.useDistCheck) {
    return true;
  }

  if (std::isnan(ball_field_.x) && std::isnan(ball_field_.y)) {
    return false;
  }

  if (GetDistance(ball_field_) <= parameters.ball.maxSeeBallDist) {
    return true;
  } else {
    ball_field_.x = UNKNOWN_OBJ_POS;
    ball_field_.y = UNKNOWN_OBJ_POS;
    return false;
  }
}

bool BallDetector::CheckBallRadius(Projection& projection) {
  if (!parameters.ball.useRadiusCheck) {
    return true;
  }

  if (std::isnan(ball_field_.x) && std::isnan(ball_field_.y)) {
    return false;
  }

  int ball_width = ball_image_bottom_.x - ball_image_top_.x;
  // int ball_height = ball_image_bottom.y - ball_image_top.y;

  cv::Point2f ball_boundary_real;
  projection.getOnRealCoordinate(
      cv::Point(ball_image_.x + ball_width / 2.0, ball_image_.y),
      ball_boundary_real, dconstant::geometry::ballDiameter / 2.0);

  float ball_radius_real = GetDistance(ball_field_, ball_boundary_real);
  // ROS_INFO("ball radius real: %f ~ (%f, %f)",
  //          ball_radius_real,
  //          dconstant::geometry::ballDiameter / 2.0 *
  //          parameters.ball.minBallRadiusRatio,
  //          dconstant::geometry::ballDiameter / 2.0 *
  //          parameters.ball.maxBallRadiusRatio);

  if (ball_radius_real >= dconstant::geometry::ballDiameter / 2.0 *
                              parameters.ball.minBallRadiusRatio &&
      ball_radius_real <= dconstant::geometry::ballDiameter / 2.0 *
                              parameters.ball.maxBallRadiusRatio) {
    ball_field_.x = UNKNOWN_OBJ_POS;
    ball_field_.y = UNKNOWN_OBJ_POS;
    return false;
  }
  return true;
}
}  // namespace dvision
