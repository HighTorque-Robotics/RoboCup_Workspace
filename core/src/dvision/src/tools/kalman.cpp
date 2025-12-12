/* Copyright (C) ZJUDancer
 * 2017 - Yusu Pan <xxdsox@gmail.com>
 */

/**
 * \file kalman.cpp
 * \author Yusu Pan
 * \version 2018
 * \date 2018-02-18
 */

#include "dvision/kalman.hpp"
#include "dancer_geometry/utils.hpp"

namespace dvision {
ObjectPosKalmanFilter::ObjectPosKalmanFilter(const cv::Point2f& p,
                                             float max_miss_sec,
                                             int control_size)
    : last_est_(UNKNOWN_OBJ_POS, UNKNOWN_OBJ_POS),
      curr_est_(UNKNOWN_OBJ_POS, UNKNOWN_OBJ_POS),
      curr_pred_(UNKNOWN_OBJ_POS, UNKNOWN_OBJ_POS),
      reset_(false),
      state_size_(2),
      measure_size_(2),
      control_size_(control_size),
      max_miss_sec_(max_miss_sec) {
  Init(p);
}

ObjectPosKalmanFilter::~ObjectPosKalmanFilter() { delete kalman_; }

void ObjectPosKalmanFilter::Init(const cv::Point2f& p) {
  //! Renew a pointer to KalmanFilter instance
  kalman_ =
      new cv::KalmanFilter(state_size_, measure_size_, control_size_, CV_32F);
  //! Predicted State (x'(k))
  // x(k) = A * x(k-1) + B * u(k)
  // [x, y]
  kalman_->statePre.at<float>(0) = p.x;
  kalman_->statePre.at<float>(1) = p.y;
  kalman_->statePre.at<float>(2) = 0;
  kalman_->statePre.at<float>(3) = 0;

  //! Transition State Matrix A
  // [ 1, 0 ]
  // [ 0, 1 ]
  cv::setIdentity(kalman_->transitionMatrix, cv::Scalar(1));

  //! Measure Matrix H
  // [ 1, 0 ]
  // [ 0, 1 ]
  cv::setIdentity(kalman_->measurementMatrix, cv::Scalar(1));

  // TODO(corenel) add control
  //! Control Matrix B
  // [ -1, 0 ]
  // [  0,-1 ]
  // cv::setIdentity(kalman_->controlMatrix, cv::Scalar(1e-4));
  kalman_->controlMatrix = cv::Mat::zeros(state_size_, measure_size_, CV_32F);
  // kalman_->controlMatrix.at<float>(0) = -1.0f;
  // kalman_->controlMatrix.at<float>(3) = -1.0f;

  //! Process Noise Covariance Matrix Q
  // [ Ex   0  ]
  // [ 0    Ey ]
  cv::setIdentity(kalman_->processNoiseCov, cv::Scalar(1e-3));

  //! MeasureNoise Covariance Matrix R
  cv::setIdentity(kalman_->measurementNoiseCov, cv::Scalar::all(5e-3));

  //! Posterior Error Estimate Covariance Matrix P
  cv::setIdentity(kalman_->errorCovPost, cv::Scalar::all(1e-1));
}

bool ObjectPosKalmanFilter::Update(const cv::Point2f& p, const Control& delta) {
  // ROS_INFO("-----------kalman tick-------------");
  GetPrediction(delta);
  if (std::isnan(p.x) && std::isnan(p.y)) {
    // ROS_WARN("not see ball %lf s", miss_time_.getElapsedSec());
    // TODO(corenel) compare last_est_ and curr_pred_
    curr_est_ = last_est_;
    // curr_est_ = curr_pred_;
    if (miss_time_.getElapsedSec() > max_miss_sec_) {
      reset_ = true;
      return false;
    }
  } else {
    // ROS_INFO("see ball");
    miss_time_.restart();
    if (reset_ && parameters.kalman.allowReset) {
      // ROS_WARN("-----------------Reset Kalman Filter %d", miss_count_);
      Init(p);
      last_est_ = p;
      curr_est_ = p;
      reset_ = false;
    } else {
      cv::Mat measurement(measure_size_, 1, CV_32FC1);

      measurement.at<float>(0) = p.x;
      measurement.at<float>(1) = p.y;

      // Corrected State (x(k))
      // x(k) = x'(k) + G(k) * (z(k) - H * x'(k))
      cv::Mat estimated = kalman_->correct(measurement);
      curr_est_ = cv::Point2f(estimated.at<float>(0), estimated.at<float>(1));
      last_est_ = curr_est_;
    }
  }
  return true;
}

cv::Point2f ObjectPosKalmanFilter::GetPrediction(const Control& delta) {
  // update dT in A (decreasing udpate speed of object position, disabled)
  // float dT = diff_time_.elapsedSec();
  // dT = dT > 0.2 ? 0.2 : dT;
  // kalman_->transitionMatrix.at<float>(2) = dT;
  // kalman_->transitionMatrix.at<float>(7) = dT;
  // ROS_INFO("dT: %f", dT);

  // TODO(corenel) add control matrix
  cv::Mat control = cv::Mat::zeros(measure_size_, 1, CV_32FC1);

  //   cv::Point2f rotated = dancer_geometry::RotateCoordinateAxis(
  //       delta.dt, cv::Point2f(curr_pred_.x - delta.dx, curr_pred_.y - delta.dy));
  //   control.at<float>(0) = rotated.x - curr_pred_.x;
  //   control.at<float>(1) = rotated.y - curr_pred_.y;

  // cv::Mat prediction = kalman_->predict(control);
  cv::Mat prediction = kalman_->predict();
  curr_pred_ = cv::Point2f(prediction.at<float>(0), prediction.at<float>(1));
  return curr_pred_;
}

cv::Point2f ObjectPosKalmanFilter::GetResult() { return curr_est_; }

cv::Point2f ObjectPosKalmanFilter::GetVelocity() {
  // ROS_INFO("ball delta (dx, dy) = (%f, %f)", kalman_->statePre.at<float>(2),
  // kalman_->statePre.at<float>(3));
  float dT = diff_time_.elapsedSec();
  dT = dT > 0.2 ? 0.2 : dT;
  // ROS_INFO("dT: %f", dT);
  // ROS_INFO("ball velocity (vx, vy) = (%f, %f)",
  // kalman_->statePre.at<float>(2) / dT, kalman_->statePre.at<float>(3) / dT);

  // return cv::Point2f(kalman_->statePre.at<float>(2),
  // kalman_->statePre.at<float>(3));
  return cv::Point2f(kalman_->statePre.at<float>(2) / dT,
                     kalman_->statePre.at<float>(3) / dT);
}
}  // namespace dvision
