/* Copyright (C) ZJUDancer
 * 2018 - Yusu Pan <xxdsox@gmail.com>
 * 2017 - Wenxing Mei <mwx36mwx@gmail.com>
 */

/**
 * @file dconcurrent.cpp
 * @author Yusu Pan, Wenxing Mei
 * @version 2018
 * @date 2018-03-04
 */

#include <ros/ros.h>
#include <iostream>
#include "dprocess/dconcurrent.hpp"

namespace dprocess {

_Thread::_Thread() : ready(false), done(false), terminate(false) {}

void _Thread::setf(std::function<void(void)> f) {
  func = f;
  thread = std::thread([this]() {
    while (!terminate) {
      std::unique_lock<std::mutex> in(this->lock_in);
      while (!ready) {
        cond_in.wait(in);
      }

      // work
      this->func();
      ready = false;
      in.unlock();

      // join
      std::lock_guard<std::mutex> lock2(this->lock_out);
      done = true;
      cond_out.notify_one();
    }
  });
}

void _Thread::stop() {
  terminate = true;
  spinOnce();
  thread.join();
}

void _Thread::spinOnce() {
  std::lock_guard<std::mutex> lk(lock_in);
  cond_in.notify_one();
  ready = true;
  done = false;
}

void _Thread::join() {
  std::unique_lock<std::mutex> lk(lock_out);
  while (!done) cond_out.wait(lk);
}

DConcurrent::DConcurrent() : m_num(0) {}

DConcurrent::~DConcurrent() {
  for (int i = 0; i < m_num; ++i) {
    m_threads[i].stop();
  }
}

void DConcurrent::push(std::function<void(void)> f) {
  if (m_num < MAX_THREADS) {
    m_threads[m_num++].setf(f);
  } else {
    std::cerr << "MAXIMUM DConcurrent Threads meets" << std::endl;
    std::terminate();
  }
}

void DConcurrent::spinOnce() {
  for (int i = 0; i < m_num; ++i) {
    m_threads[i].spinOnce();
  }
}

void DConcurrent::join() {
  for (int i = 0; i < m_num; ++i) {
    m_threads[i].join();
  }
}
}
