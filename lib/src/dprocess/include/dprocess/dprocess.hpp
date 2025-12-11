/* Copyright (C) ZJUDancer
 * 2018 - Yusu Pan <xxdsox@gmail.com>
 * 2017 - Wenxing Mei <mwx36mwx@gmail.com>
 */

/**
 * @file dprocess.hpp
 * @author Yusu Pan, Wenxing Mei
 * @version 2018
 * @date 2018-03-04
 */

#pragma once
#include <pthread.h>
#include <ros/ros.h>
#include <thread>

namespace dprocess {

template <typename T>
/**
  * @brief Single process wrapper of std::thread for ROS program.
  */
class DProcess {
 public:
  /**
   * @brief DProcess constructor.
   *
   * @param freq - frequency for ROS thread spining
   * @param rt - flag whether use real-time mode
   */
  explicit DProcess(int freq, bool rt = false) : m_freq(freq), m_rt(rt) {}
  /**
   * @brief DProcess destructor
   */
  virtual ~DProcess() {}

  /**
   * @brief Create thread and spin.
   */
  void spin() {
    m_thread = std::thread([=] {
      // TODO  add watch dog, maybe add some timer
      ros::Rate r(m_freq);
      while (ros::ok()) {
        ros::Time begin = ros::Time::now();
        static_cast<T*>(this)->tick();
        ros::spinOnce();
        r.sleep();

        if (m_attemptShutdown) {
          prepareShutdown();
          break;
        }

        ros::Time end = ros::Time::now();
        ROS_DEBUG("Thread tick used %lf ms.", (end - begin).toSec() * 1000);
      }
    });

    set_policy();
  }

  /**
   * @brief Set highest scheduled priority.
   */
  void set_policy() {
    if (m_rt) {
      sched_param param;
      param.sched_priority = 99;
      // TODO, needs sudo
      if (pthread_setschedparam(m_thread.native_handle(), SCHED_FIFO, &param)) {
        ROS_INFO("Set REAL TIME policy success.");
      } else {
        ROS_WARN("I can't run in REAL TIME.");
      }
    }
  }

  // TODO(MWX): need boosting
  /**
   * @brief Spin once. Only for test.
   */
  void spinOnce() {
    m_thread = std::thread([=] { static_cast<T*>(this)->tick(); });
  }

  /**
   * @brief Join thread.
   */
  void join() { m_thread.join(); }

  /**
   * @brief Set flag for attempting to shutdown this process
   */
  void attemptShutdown() { m_attemptShutdown = true; }

  /**
   * @brief Tick.
   */
  virtual void tick() {}

 protected:
  /**
   * @brief Flag for attempting to shutdown this process.
   */
  bool m_attemptShutdown = false;

  /**
   * @brief Prepare to shutdown this process.
   */
  virtual void prepareShutdown() {
    m_attemptShutdown = true;
    // do stuff before shutdown
  }

 private:
  /**
   * @brief Frequency for ROS thread spining
   */
  int m_freq;
  /**
   * @brief Flag whether use real-time mode
   */
  bool m_rt;
  /**
   * @brief Thread instance.
   */
  std::thread m_thread;
};
}
