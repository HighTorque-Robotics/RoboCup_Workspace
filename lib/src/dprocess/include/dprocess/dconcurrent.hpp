/* Copyright (C) ZJUDancer
 * 2018 - Yusu Pan <xxdsox@gmail.com>
 * 2017 - Wenxing Mei <mwx36mwx@gmail.com>
 */

/**
 * @file dconcurrent.hpp
 * @author Yusu Pan, Wenxing Mei
 * @version 2018
 * @date 2018-03-04
 */

#pragma once
#include <array>
#include <condition_variable>
#include <functional>
#include <mutex>
#include <thread>

namespace dprocess {

static const int MAX_THREADS = 10;

/**
 * @brief wrapper for std::thread for concurrent running
 */
class _Thread {
 public:
  /**
   * @brief _Thread constructor.
   */
  _Thread();
  /**
   * @brief Stop thread.
   */
  void stop();
  /**
   * @brief Set function.
   *
   * @param f - given function
   */
  void setf(std::function<void(void)> f);
  /**
   * @brief Spin once.
   */
  void spinOnce();
  /**
   * @brief Join thread.
   */
  void join();

 private:
  //! Mutex lock for in condition
  std::mutex lock_in;
  //! Variable for in condition
  std::condition_variable cond_in;
  //! Mutex lock for out condition
  std::mutex lock_out;
  //! Variable for out condition
  std::condition_variable cond_out;

  //! Flag for ready
  bool ready;
  //! Flag for done
  bool done;
  //! Flag for terminated
  bool terminate;

  //! Function in this thread
  std::function<void(void)> func;
  //! Thread instance
  std::thread thread;
};

/**
 * @brief Load all threads at first and run in async way.
 *
 * Avoid overhead of std::async of always launching new threads.
 *
 */
class DConcurrent {
 public:
  /**
   * @brief DConcurrent constructor.
   */
  DConcurrent();
  /**
   * @brief DConcurrent destructor.
   */
  ~DConcurrent();

  /**
   * @brief Add new function as thread.
   *
   * @param f - given function
   */
  void push(std::function<void(void)> f);
  /**
   * @brief Spin once. 
   */
  void spinOnce();
  /**
   * @brief Join thread. 
   */
  void join();

 private:
  /**
   * @brief Container for threads.
   *
   * Using array because std::vector needs destructing when expanding space,
   * and it will terminate the threads.
   */
  std::array<_Thread, MAX_THREADS> m_threads;
  //! Number of threads
  int m_num;
};
}
