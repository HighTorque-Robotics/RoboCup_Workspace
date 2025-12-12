/* Copyright (C) ZJUDancer
 * 2018 - Yusu Pan <xxdsox@gmail.com>
 * 2017 - Wenxing Mei <mwx36mwx@gmail.com>
 */

/**
 * \file v4l2_camera.cpp
 * \author Yusu Pan, Wenxing Mei
 * \version 2018
 * \date 2018-02-13
 */

#include "dvision/v4l2_camera.hpp"
#include <errno.h>
#include <poll.h>
#include <sys/fcntl.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/stat.h>

#define CLEAR(x) memset(&(x), 0, sizeof((x)))

namespace dvision {

// ioctl
static int xioctl(int fd, int request, void* arg) {
  int status;
  do {
    status = ioctl(fd, request, arg);
  } while (-1 == status && EINTR == errno);
  return status;
}

V4L2Camera::V4L2Camera(const std::string& device) : device_(device) {
  init();
  resetControl();
}

V4L2Camera::V4L2Camera(const CameraSettings& c)
    : device_(c.device), setting_(c) {
  init();
}

V4L2Camera::~V4L2Camera() { deInit(); }

void V4L2Camera::enableDebugWindow() { showDebugWindow_ = true; }

void V4L2Camera::init() {
  try {
    openDevice();
    usleep(1000);
    initFmt();
    setFrameRate(1, setting_.frameRate);
    initMmap();
    startIO();
    doIO();
    resetControl();
    setCameraControl();
  } catch (std::exception& e) {
    ROS_ERROR("[dvision] Init camera error: %s", e.what());
    closeDevice();
    sleep(1);
    init();
  }
}

void V4L2Camera::deInit() {
  stopIO();
  closeDevice();
}

void V4L2Camera::initFmt() {
  // check camera capability
  struct v4l2_capability cap;
  CLEAR(cap);
  if (xioctl(fd_, VIDIOC_QUERYCAP, &cap) < 0)
    throw std::runtime_error("Not a v4l2 device.");
  if (!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE))
    throw std::runtime_error("Not a video capture device");

  // initialize camera format
  v4l2_format fmt;
  CLEAR(fmt);
  fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  fmt.fmt.pix.field = V4L2_FIELD_NONE;
  fmt.fmt.pix.width = setting_.width;
  fmt.fmt.pix.height = setting_.height;
  __u32 pixfmt = V4L2_PIX_FMT_YUYV;
  fmt.fmt.pix.pixelformat = pixfmt;

  // try to set V4L2 format
  if (xioctl(fd_, VIDIOC_S_FMT, &fmt) < 0) {
    printf("v4l2 -- failed to set video format of device (errno=%i) (%s)\n",
           errno, strerror(errno));
    throw std::runtime_error("Set camera device format error");
  }

  // try to get V4L2 format
  if (xioctl(fd_, VIDIOC_G_FMT, &fmt) < 0) {
    ROS_ERROR("v4l2 -- failed to get video format of device (errno=%i) (%s)\n",
              errno, strerror(errno));
    throw std::runtime_error("Get camera device format error");
  }

  // check if formats are matched
  if ((int)fmt.fmt.pix.width != setting_.width ||
      (int)fmt.fmt.pix.height != setting_.height ||
      fmt.fmt.pix.pixelformat != pixfmt)
    throw std::runtime_error("Set camera device format error");

  ROS_INFO("[dvision] Set camera device fmt success.");
}

void V4L2Camera::setFrameRate(uint32_t numerator, uint32_t denominator) {
  v4l2_streamparm parm;
  CLEAR(parm);

  parm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if (xioctl(fd_, VIDIOC_G_PARM, &parm) < 0)
    throw std::runtime_error("Get fps error");

  parm.parm.capture.timeperframe.numerator = numerator;
  parm.parm.capture.timeperframe.denominator = denominator;
  parm.parm.capture.capability = V4L2_CAP_TIMEPERFRAME;
  if (xioctl(fd_, VIDIOC_S_PARM, &parm) < 0)
    ROS_ERROR("[dvision] Set fps error!");

  // confirm
  if (xioctl(fd_, VIDIOC_G_PARM, &parm) < 0)
    throw std::runtime_error("Get fps error");
  if (parm.parm.capture.timeperframe.numerator != numerator ||
      parm.parm.capture.timeperframe.denominator != denominator)
    ROS_ERROR("[dvision] Set fps error!");

  ROS_INFO("[dvision] Set fps to %d/%d", numerator, denominator);
}

void V4L2Camera::initMmap() {
  struct v4l2_requestbuffers req;
  CLEAR(req);

  req.count = NUM_FRAME_BUFFERS;
  req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  req.memory = V4L2_MEMORY_MMAP;

  if (xioctl(fd_, VIDIOC_REQBUFS, &req) < 0) {
    if (errno == EINVAL)
      ROS_ERROR("[dvision] Memory mapping not supported on this device!");
    else
      throw std::runtime_error("REQBUFS");
  }

  if (req.count < 1)
    throw std::runtime_error(
        "[dvision] Insufficient buffer memory for camera.");

  CLEAR(buffers);

  for (n_buffers_ = 0; n_buffers_ < req.count; n_buffers_++) {
    v4l2_buffer buf;
    CLEAR(buf);

    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.index = n_buffers_;

    if (xioctl(fd_, VIDIOC_QUERYBUF, &buf) < 0)
      throw std::runtime_error("QUERYBUF");
    buffers[n_buffers_].length = buf.length;
    buffers[n_buffers_].start = reinterpret_cast<uint8_t*>(
        mmap(NULL, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED, fd_,
             buf.m.offset));
    if (MAP_FAILED == buffers[n_buffers_].start)
      throw std::runtime_error("MMAP");
  }

  // initial release all buffer space
  last_dequeued_.index = UINT_MAX;
  for (unsigned int i = 0; i < n_buffers_; i++) {
    struct v4l2_buffer buf;
    CLEAR(buf);
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.index = i;
    buf.timecode.type = 3;
    if (xioctl(fd_, VIDIOC_QBUF, &buf) < 0) throw std::runtime_error("QBUF");
  }
  ROS_INFO("Init MMAP success.");
}

void V4L2Camera::startIO() {
  enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if (xioctl(fd_, VIDIOC_STREAMON, &type) < 0) {
    printf("v4l2 -- failed to start streaming (errno=%i) (%s)\n", errno,
           strerror(errno));
    throw std::runtime_error("Failed start camera capturing.");
  }
}

void V4L2Camera::stopIO() {
  enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if (xioctl(fd_, VIDIOC_STREAMOFF, &type) < 0) {
    printf("v4l2 -- failed to stop streaming (errno=%i) (%s)\n", errno,
           strerror(errno));
    throw std::runtime_error("Failed stop camera capturing.");
  }
}

void V4L2Camera::setControl(V4L2CID id, const uint32_t& value) {
  setControl(static_cast<uint32_t>(id), value);
  usleep(1000);
}

void V4L2Camera::setControl(const uint32_t& controlId,
                            const uint32_t& controlValue) {
  v4l2_queryctrl queryctrl;
  CLEAR(queryctrl);
  queryctrl.id = controlId;
  if (xioctl(fd_, VIDIOC_QUERYCTRL, &queryctrl) < 0) {
    if (errno != EINVAL)
      ROS_ERROR("[dvision] camera query control error");
    else
      ROS_WARN("[dvision] camera control id %d not supported", controlId);
  } else if (queryctrl.flags & V4L2_CTRL_FLAG_DISABLED) {
    ROS_WARN("[dvision] camera query control %s not supported", queryctrl.name);
  } else {
    v4l2_control control, current;
    CLEAR(control), CLEAR(current);
    control.id = controlId;
    control.value = controlValue;
    current.id = controlId;

    if (xioctl(fd_, VIDIOC_S_CTRL, &control) < 0) {
      ROS_WARN("[dvision] Set camera query control %s error", queryctrl.name);
    } else {
      xioctl(fd_, VIDIOC_G_CTRL, &current);
      ROS_INFO("[dvision] Set camera query control %s to %d", queryctrl.name,
               current.value);
    }
  }
}

void V4L2Camera::setCameraControl() {
  ROS_INFO("Testing exposure auto mode:");
  setControl(V4L2CID::exposure_auto, 0);
  setControl(V4L2CID::exposure_auto, 1);
  setControl(V4L2CID::exposure_auto, 2);
  setControl(V4L2CID::exposure_auto, 3);
  setControl(V4L2CID::exposure_auto, setting_.exposure_auto);
  setControl(V4L2CID::exposure_absolute, setting_.exposure_absolute);

  // setControl(V4L2CID::focus_auto, m_setting.focus_auto);
  // setControl(V4L2CID::focus_absolute, m_setting.focus_absolute);
  setControl(V4L2CID::brightness, setting_.brightness);
  setControl(V4L2CID::contrast, setting_.contrast);
  setControl(V4L2CID::saturation, setting_.saturation);
  setControl(V4L2CID::sharpness, setting_.sharpness);
  setControl(V4L2CID::gain, setting_.gain);
  setControl(V4L2CID::hue, setting_.hue);
  setControl(V4L2CID::gamma, setting_.gamma);

  setControl(V4L2CID::white_balance_auto, setting_.whitebalance_auto);
  setControl(V4L2CID::white_balance_temperature,
             setting_.whitebalance_absolute);
}

v4l2_queryctrl V4L2Camera::getControl(V4L2CID cid) {
  struct v4l2_queryctrl retv, queryctrl;
  retv.minimum = 0;
  retv.maximum = 0;  // min == max means error

  memset(&queryctrl, 0, sizeof(queryctrl));
  queryctrl.id = static_cast<uint32_t>(cid);
  if (xioctl(fd_, VIDIOC_QUERYCTRL, &queryctrl) < 0) {
    if (errno == EINVAL) return retv;
    ROS_ERROR("queryctrl error");
  } else {
    if (queryctrl.flags & V4L2_CTRL_FLAG_DISABLED) return retv;
    retv = queryctrl;
  }
  return retv;
}

v4l2_queryctrl V4L2Camera::getControl(uint32_t cid) {
  return getControl((V4L2CID)cid);
}

void V4L2Camera::resetControl() {
  ROS_INFO("[dvision] Reset camera control");
  struct v4l2_queryctrl queryInfo;
  static const int EXP_AUTO = 3;  // 1:manual 3:auto
  setControl(V4L2_CID_EXPOSURE_AUTO, EXP_AUTO);
  // auto focus
  queryInfo = getControl(V4L2_CID_FOCUS_AUTO);
  if (queryInfo.minimum != queryInfo.maximum)  // support auto-focus
  {
    setControl(V4L2_CID_FOCUS_AUTO, 0);
    queryInfo = getControl(V4L2_CID_FOCUS_ABSOLUTE);
    setControl(V4L2_CID_FOCUS_ABSOLUTE, queryInfo.default_value);
  } else {
    ROS_INFO("auto focus disabled");
  }

  queryInfo = getControl(V4L2_CID_EXPOSURE_ABSOLUTE);
  if (queryInfo.minimum != queryInfo.maximum) {
    setControl(V4L2_CID_EXPOSURE_ABSOLUTE, queryInfo.default_value);
  }

  queryInfo = getControl(V4L2_CID_BRIGHTNESS);
  if (queryInfo.minimum != queryInfo.maximum) {
    setControl(V4L2_CID_BRIGHTNESS, queryInfo.default_value);
  }

  queryInfo = getControl(V4L2_CID_CONTRAST);
  if (queryInfo.minimum != queryInfo.maximum) {
    setControl(V4L2_CID_CONTRAST, queryInfo.default_value);
  }

  queryInfo = getControl(V4L2_CID_SATURATION);
  if (queryInfo.minimum != queryInfo.maximum) {
    setControl(V4L2_CID_SATURATION, queryInfo.default_value);
  }

  queryInfo = getControl(V4L2_CID_GAIN);
  if (queryInfo.minimum != queryInfo.maximum) {
    setControl(V4L2_CID_GAIN, queryInfo.default_value);
  } else {
    setControl(V4L2_CID_AUTOGAIN, 2);
  }

  queryInfo = getControl(V4L2_CID_WHITE_BALANCE_TEMPERATURE);
  if (queryInfo.minimum != queryInfo.maximum) {
    setControl(V4L2_CID_WHITE_BALANCE_TEMPERATURE, queryInfo.default_value);
  }

  queryInfo = getControl(V4L2_CID_GAMMA);
  if (queryInfo.minimum != queryInfo.maximum) {
    setControl(V4L2_CID_GAMMA, queryInfo.default_value);
  }

  queryInfo = getControl(V4L2_CID_HUE);
  if (queryInfo.minimum != queryInfo.maximum) {
    setControl(V4L2_CID_HUE, queryInfo.default_value);
  }

  queryInfo = getControl(V4L2_CID_SHARPNESS);
  if (queryInfo.minimum != queryInfo.maximum) {
    setControl(V4L2_CID_SHARPNESS, queryInfo.default_value);
  }

  // set auto white balance
  setControl(V4L2_CID_AUTO_WHITE_BALANCE, 1);
  ROS_INFO("Reset camera control end.");
}

void V4L2Camera::doIO() {
  try {
    // Get frames from camera device
    auto beginTime = ros::Time::now();
    struct pollfd pollfd = {fd_, POLLIN | POLLPRI, 0};
    // wait for 6 frame for a new frame
    int polled = poll(&pollfd, 1, 1000.0 / setting_.frameRate * 20);

    if (polled < 0)
      throw std::runtime_error("Cannot poll from camera");
    else if (polled == 0)
      throw std::runtime_error("Poll timeout in camera");
    else if (pollfd.revents & (POLLERR | POLLNVAL))
      throw std::runtime_error("Polling failed in camera");
    auto stop1 = ros::Time::now();

    // Release last buffer
    if (last_dequeued_.index != UINT_MAX)
      if (xioctl(fd_, VIDIOC_QBUF, &last_dequeued_) < 0)
        throw std::runtime_error("Release frame buffer error");

    // Dequeue raw camera buffer, we got one new raw frame, yuv
    CLEAR(last_dequeued_);
    last_dequeued_.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    last_dequeued_.memory = V4L2_MEMORY_MMAP;

    if (xioctl(fd_, VIDIOC_DQBUF, &last_dequeued_) < 0)
      throw std::runtime_error("DQBUFF");
    auto stop2 = ros::Time::now();
    assert(last_dequeued_.index < n_buffers_);

    raw_yuv_ = buffers[last_dequeued_.index].start;
    raw_yuv_size_ = last_dequeued_.bytesused;

    // Complain about IO time
    auto ioTime = stop1 - beginTime;
    auto dequeTime = stop2 - stop1;
    ROS_DEBUG("[dvision] Camera I/O: %lf ms, dequeue: %lf ms, size: %u",
              ioTime.toSec() * 1000, dequeTime.toSec() * 1000, raw_yuv_size_);

    auto t = (stop2 - beginTime).toSec() * 1000;
    if (t > 40) {
      ROS_WARN("[dvision] Camera I/O is getting slow, used %lf ms.", t);
    }

  } catch (std::exception& e) {
    ROS_ERROR("[dvision] Camera I/O failed, error: %s, trying to restart.",
              e.what());
    sleep(1);
    // stopIO();
    closeDevice();
    init();
  }
};

void V4L2Camera::openDevice() {
  struct stat buf;
  // Find camera device
  if (-1 == stat(device_.c_str(), &buf)) {
    ROS_ERROR("[dvision] Failed to stat camera device %s", device_.c_str());
    std::string devname = "/dev/video";
    int i = 0;
    while (true) {
      // try to open video0 up to video50
      int rc = 0;
      std::string tmp = devname + std::to_string(i);
      rc = stat(tmp.c_str(), &buf);
      if (rc == -1) {
        usleep(10000);
      } else {
        ROS_INFO("[dvision] Found camera device: %s", tmp.c_str());
        device_ = tmp;
        break;
      }
      if (++i > 50) {
        ROS_WARN("[dvision] No camera device available!");
        i = 0;
      }
    }
  }

  // Open camera device
  if (0 == S_ISCHR(buf.st_mode)) {
    throw std::runtime_error("Not a v4l2 camera device.");
  }

  fd_ = open(device_.c_str(), O_CLOEXEC | O_RDWR);
  if (fd_ < 0) {
    throw std::runtime_error("Failed to open camera device");
  } else {
    ROS_INFO("[dvision] Opened camera device %s", device_.c_str());
  }
}

void V4L2Camera::closeDevice() {
  if (fd_ > 0) {
    close(fd_);
  }
  fd_ = -1;
}

Frame V4L2Camera::capture() {
  Timer t;
  //    if(showDebugWindow_) {
  //        cv::namedWindow("camera", CV_WINDOW_NORMAL);
  //
  //        cv::createTrackbar("exposure", "camera",
  //        &m_setting.exposure_absolute, 255); cv::createTrackbar("brightness",
  //        "camera", &m_setting.brightness, 255);
  //        cv::createTrackbar("contrast", "camera", &m_setting.contrast, 255);
  //        cv::createTrackbar("saturation", "camera", &m_setting.saturation,
  //        255); cv::createTrackbar("sharpness", "camera",
  //        &m_setting.sharpness, 3); cv::createTrackbar("gain", "camera",
  //        &m_setting.gain, 255); cv::createTrackbar("hue", "camera",
  //        &m_setting.hue, 255); cv::createTrackbar("gamma", "camera",
  //        &m_setting.gamma, 255); cv::createTrackbar("white_balance",
  //        "camera", &m_setting.whitebalance_absolute, 5000); cv::waitKey(1);
  //
  //        setCameraControl();
  //    }
  doIO();
  ROS_DEBUG("[dvision] camera capture frame used: %lf ms", t.elapsedMsec());
  return Frame(static_cast<uint8_t*>(raw_yuv_), setting_.width,
               setting_.height);
}
}  // namespace dvision
