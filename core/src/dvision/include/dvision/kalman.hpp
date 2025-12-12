/* Copyright (C) ZJUDancer
 * 2017 - Yusu Pan <xxdsox@gmail.com>
 */

/**
 * \file kalman.hpp
 * \author Yusu Pan
 * \version 2018
 * \date 2018-02-18
 */

#pragma once
#include <limits>
#include <vector>
#include "dvision/amcl/types.hpp"
#include "dvision/parameters.hpp"
#include "dcommon/timer.hpp"
#include "opencv2/opencv.hpp"

namespace dvision {
#define UNKNOWN_OBJ_POS std::numeric_limits<float>::quiet_NaN()
// TODO replace OpneCV's kalman filter
//! Kalman fileter for obsject position
class ObjectPosKalmanFilter {
 public:
  //! ObjectPosKalmanFilter constructor
  explicit ObjectPosKalmanFilter(const cv::Point2f& p, float max_miss_sec,
                                 int control_size = 2);
  //! ObjectPosKalmanFilter destructor
  ~ObjectPosKalmanFilter();
  //! Initialize ObjectPosKalmanFilter
  void Init(const cv::Point2f& p);
  //! Update estimation of objcet by kalman filter
  bool Update(const cv::Point2f& p, const Control& delta);

  //! Get prediction of object bt kalman filter
  cv::Point2f GetPrediction(const Control& delta);
  //! Get corrected estimation of object bt kalman filter
  cv::Point2f GetResult();
  //! Get velocity of object bt kalman filter
  cv::Point2f GetVelocity();

 private:
  //! Pointer to instance of kalman filter
  cv::KalmanFilter* kalman_;
  //! Estimation of object in last frame
  cv::Point2f last_est_;
  //! Estimation of object in current frame
  cv::Point2f curr_est_;
  //! Prediction of object in current frame
  cv::Point2f curr_pred_;

  //! Timer for missing object
  dcommon::Timer miss_time_;
  //! Timer for getting velocity from last time on
  dcommon::Timer diff_time_;

  //! Flag for whether or not to reset kalman filter
  bool reset_;
  //! Number of state variables
  int state_size_;
  //! Number of measure variables
  int measure_size_;
  //! Number of control variables
  int control_size_;
  //! Max time allowed for the loss of position
  float max_miss_sec_;
};
}  // namespace dvision
