/* Copyright (C) ZJUDancer
 * 2018 - Yusu Pan <xxdsox@gmail.com>
 * 2017 - Wenxing Mei <mwx36mwx@gmail.com>
 */

/**
 * @file timer.hpp
 * @author Yusu Pan, Wenxing Mei
 * @version 2018
 * @date 2018-02-24
 */

#pragma once

#include <ros/ros.h>

namespace dcommon {

class Timer {
 public:
  /**
   * @brief Timer constructor
   */
  inline Timer() { restart(); }

  /**
   * @brief Restart timer
   */
  inline void restart() { m_startTimestamp = ros::Time::now(); }

  /**
   * @brief Get elapsed time in millisecond and restart timer
   *
   * @return elapsed time in millisecond
   */
  inline double elapsedMsec() {
    auto res = elapsedSec() * 1000;
    restart();
    return res;
  }

  /**
   * @brief Get elapsed time in second and restart timer
   *
   * @return elapsed time in millisecond
   */
  inline double elapsedSec() {
    auto now = ros::Time::now();
    auto res = (now - m_startTimestamp).toSec();
    restart();
    return res;
  }

  /**
   * @brief Get elapsed time in millisecond
   *
   * @return elapsed time in millisecond
   */
  inline double getElapsedMsec() {
    auto res = elapsedSec() * 1000;
    return res;
  }

  /**
   * @brief Get elapsed time in second
   *
   * @return elapsed time in millisecond
   */
  inline double getElapsedSec() {
    auto now = ros::Time::now();
    auto res = (now - m_startTimestamp).toSec();
    return res;
  }

 private:
  //! timestamp of start time
  ros::Time m_startTimestamp;
};
}  // namespace dcommon
