/* Copyright (C) ZJUDancer
 * 2018 - Yusu Pan <xxdsox@gmail.com>
 * 2017 - Wenxing Mei <mwx36mwx@gmail.com>
 */

/**
 * \file v4l2_camera.hpp
 * \author Yusu Pan, Wenxing Mei
 * \version 2018
 * \date 2018-02-13
 */

#pragma once
#include <linux/videodev2.h>
#include "dvision/camera_settings.hpp"
#include "dvision/icamera.hpp"

namespace dvision {
//! Video4Linux2 camera capture streaming class
class V4L2Camera : public ICamera {
 public:
  //! V4L2Camera constructor
  /*! Use default/saved camera parameters */
  explicit V4L2Camera(const std::string& device = "/dev/video0");

  //! V4L2Camera constructor
  /*! Set camera parameters explicitly */
  explicit V4L2Camera(const CameraSettings&);

  //! V4L2Camera destructor
  ~V4L2Camera() override;

  /**
   * \brief Capture a frame from camera device
   *
   * \return captured frame
   */
  Frame capture() override;

  /**
   * \brief Set camera query control
   *
   * \param id - control id
   * \param value - control value
   */
  void setControl(V4L2CID id, const uint32_t& value);

  /**
   * \brief Set camera query control
   *
   * \param cid - control id
   * \param value - control value
   */
  void setControl(const uint32_t& cid, const uint32_t& value);

  /**
   * \brief Get control value of camera
   *
   * \param cid - control id
   *
   * \return control value
   */
  v4l2_queryctrl getControl(V4L2CID cid);

  /**
   * \brief Get control value of camera
   *
   * \param id - control id
   *
   * \return control value
   */
  v4l2_queryctrl getControl(uint32_t cid);

  void enableDebugWindow();

  // prohibit copy/move
  V4L2Camera(const V4L2Camera&) = delete;
  V4L2Camera(V4L2Camera&&) = delete;
  V4L2Camera& operator=(const V4L2Camera&) = delete;
  V4L2Camera& operator=(V4L2Camera&&) = delete;

 private:
  //! Init camera with settings
  void init();
  //! Stop IO and close camera device
  void deInit();
  //! Find and open camera device
  void openDevice();
  //! Close opened camera device
  void closeDevice();
  //! Init V4L2 device format
  void initFmt();
  //! Set frame rate
  void setFrameRate(uint32_t, uint32_t);
  //! Start camera capturing
  void startIO();
  //! Stop camera capturing
  void stopIO();
  //! Pull frames from camera into dequeued buffer
  void doIO();
  //! Set camera control with settings
  void setCameraControl();
  //! Reset camera query control
  void resetControl();
  //! Init memory mapping for camera device
  void initMmap();

 private:
  //! Camera device name
  std::string device_;
  //! Camera device file descriptor
  int fd_;
  //! Camera settings
  CameraSettings setting_;

  //! Maximum buffer size of frame
  static const int NUM_FRAME_BUFFERS = 3;
  //! Wrapped buffer struct
  struct {
    uint8_t* start;
    size_t length;
  } buffers[NUM_FRAME_BUFFERS];

  //! Indicator for buffers
  uint32_t n_buffers_;
  //! Dequeued raw camera buffer
  v4l2_buffer last_dequeued_;

  //! Pointer to last dequeued buffer
  void* raw_yuv_;
  //! Size of last dequeued buffer
  uint32_t raw_yuv_size_;
  //! Flag for whether or not showing debug window
  bool showDebugWindow_ = false;
};
}  // namespace dvision
