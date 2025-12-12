/* Copyright (C) ZJUDancer
 * 2018 - Yusu Pan <xxdsox@gmail.com>
 * 2017 - Wenxing Mei <mwx36mwx@gmail.com>
 */

/**
 * @file ringbuffer.hpp
 * @author Yusu Pan, Wenxing Mei
 * @version 2018
 * @date 2018-02-24
 */

#pragma once
#include <condition_variable>
#include <deque>
#include <mutex>
#include <ros/ros.h>
#include <vector>

/*
A simple one producer, one consumer ring buffer.

User side                             Worker side

Consume <-- Deqeque --  [ Queue 1 ] <-- Enqueue -- New buffer
|                                                 |
|                                                 |
Release -- Enqueue -->  [ Queue 2 ] -- Dequeue -->  Work ...
    Buffer

*/

// TODO(MWX): move this to dlib

namespace dvision {

/**
  * @brief A simple one producer, one consumer ring buffer
  *
  * @tparam T - type of buffer
  */
template <typename T>
class RingBuffer
{
  public:
    //! RingBuffer constructor (blank)
    RingBuffer();

    /**
     * @brief RingBuffer constructor
     *
     * @param ring_size - size of ring buffer
     * @param timeout - timeout for ring buffer
     */
    explicit RingBuffer(size_t ring_size, size_t timeout);

    /**
     * @brief Initialize ring buffer
     *
     * @param ring_size - size of ring buffer
     * @param timeout - timeout for ring buffer
     */
    void Init(size_t ring_size, size_t timeout);

    /**
     * @brief Request from user
     *
     * @return reference to element in buffer
     */
    T& UserRequest();

    /**
     * @brief Request from worker
     *
     * @return reference to element in buffer
     */
    T& WorkerRequest();

    /**
     * @brief Release from user
     */
    void UserRelease();

    /**
     * @brief Release from worker
     */
    void WorkerRelease();

  private:
    //! Double end queue for ring buffer
    typedef std::deque<size_t> Queue;

  private:
    /**
     * @brief Request element in queue by index 
     *
     * @param index - element index
     * @param queue - queue for buffer
     * @param lock - lock for buffer
     * @param cond - avaliable condition for buffer
     *
     * @return desired element in queue
     */
    T& request(size_t& index, Queue& queue, std::mutex& lock, std::condition_variable& cond);
    /**
     * @brief Release element in queue by index 
     *
     * @param index - element index
     * @param queue - queue for buffer
     * @param lock - lock for buffer
     * @param cond - avaliable condition for buffer
     */
    void release(size_t& index, Queue& queue, std::mutex& lock, std::condition_variable& cond);

  private:
    //! Queue for data
    Queue dataQueue_;
    //! Queue for workspace
    Queue workspaceQueue_;

    //! Lock for data
    std::mutex dataLock_;
    //! Lock for workspace
    std::mutex workspaceLock_;

    //! Avaliable condition for data
    std::condition_variable dataAvailable_;
    //! Avaliable condition for workspace
    std::condition_variable workspaceAvailable_;

    //! Contrainer of buffer
    std::vector<T> buffer_;
    //! Maximum time for waiting
    size_t maxWaitTime_;

    //! Index for last user
    size_t lastUserIndex_;
    //! Index for last worker
    size_t lastWorkerIndex_;
};

template <typename T>
RingBuffer<T>::RingBuffer()
{
}

template <typename T>
void
RingBuffer<T>::Init(size_t ring_size, size_t timeout)
{
    buffer_.resize(ring_size);
    maxWaitTime_ = timeout;
    lastUserIndex_ = UINT_MAX;
    lastWorkerIndex_ = UINT_MAX;
    workspaceQueue_.clear();
    for (size_t i = 0; i < buffer_.size(); ++i)
        workspaceQueue_.push_back(i);
}

template <typename T>
RingBuffer<T>::RingBuffer(size_t ring_size, size_t timeout)
{
    Init(ring_size, timeout);
}

template <typename T>
T&
RingBuffer<T>::request(size_t& index, Queue& queue, std::mutex& lock, std::condition_variable& cond)
{
    std::unique_lock<std::mutex> lk(lock);
    while (queue.empty()) {
        if (cond.wait_for(lk, std::chrono::milliseconds(maxWaitTime_)) == std::cv_status::timeout) {
            // FIXME(MWX): handle timeout
            // ROS_ERROR("Request timeout!");
            lk.unlock();
            return request(index, queue, lock, cond);
        }
    }

    index = queue.front();
    queue.pop_front();

    assert(index < buffer_.size());
    return buffer_.at(index);
}

template <typename T>
void
RingBuffer<T>::release(size_t& index, Queue& queue, std::mutex& lock, std::condition_variable& cond)
{
    if (index == UINT_MAX)
        return;

    std::lock_guard<std::mutex> lk(lock);
    queue.push_back(index);
    index = UINT_MAX;

    if (queue.size() == 1)
        cond.notify_one();
}

template <typename T>
T&
RingBuffer<T>::UserRequest()
{
    return request(lastUserIndex_, dataQueue_, dataLock_, dataAvailable_);
}

template <typename T>
void
RingBuffer<T>::UserRelease()
{
    release(lastUserIndex_, workspaceQueue_, workspaceLock_, workspaceAvailable_);
}

template <typename T>
T&
RingBuffer<T>::WorkerRequest()
{
    return request(lastWorkerIndex_, workspaceQueue_, workspaceLock_, workspaceAvailable_);
}

template <typename T>
void
RingBuffer<T>::WorkerRelease()
{
    release(lastWorkerIndex_, dataQueue_, dataLock_, dataAvailable_);
}

} // namespace dvision
