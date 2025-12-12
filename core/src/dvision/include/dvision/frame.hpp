/* Copyright (C) ZJUDancer
 * 2018 - Yusu Pan <xxdsox@gmail.com>
 * 2017 - Wenxing Mei <mwx36mwx@gmail.com>
 */

/**
 * \file frame.hpp
 * \author Yusu Pan, Wenxing Mei
 * \version 2018
 * \date 2018-02-13
 */

#pragma once
#include "cv_bridge/cv_bridge.h"
#include "dvision/parameters.hpp"
#include "dvision/timer.hpp"
//#include "ros_h264_streamer/h264_decoder.h"
//#include "ros_h264_streamer/h264_encoder.h"
#include <ros/ros.h>
#include <memory>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui_c.h>
#include <string>

namespace dvision {
/**
 * \brief Receiving, converting and encoding images from camera
 */
class Frame {
 public:
  //! Frame constructor (blank)
  inline Frame() = default;

  /**
   * \brief Frame constructor (from camera)
   *
   * \param yuv - YUV image from camera
   * \param width - image width
   * \param height - image height
   */
  inline explicit Frame(uint8_t* yuv, int width, int height)
      : yuv_(yuv), width_(width), height_(height), converted_(false) {
    time_stamp_ = ros::Time::now();
  }

#ifdef USE_CUDA
  inline explicit Frame(void* img_cpu, void* img_cuda, const int& width,
                        const int& height)
      : width_(width), height_(height) {
    bgr_ = cv::Mat(height_, width_, CV_8UC3, img_cpu);
    bgr_cuda_ = cv::cuda::GpuMat(height_, width_, CV_8UC3, img_cuda);
    time_stamp_ = ros::Time::now();
  }
#endif

  /**
   * \brief Frame constructor (from file)
   *
   * \param filepath image path
   */
  explicit inline Frame(const std::string& filepath) {
    bgr_ = cv::imread(filepath);
    width_ = bgr_.size().width;
    height_ = bgr_.size().height;
    converted_ = true;
  }
  explicit inline Frame(const cv::Mat& img) {
    bgr_ = img;
    width_ = bgr_.size().width;
    height_ = bgr_.size().height;
    converted_ = false;
  }

  /**
   * \brief Frame constructor (from cv::Mat)
   *
   * \param mat - cv::Mat image from camera
   * \param width - image width
   * \param height - image height
   */
  inline explicit Frame(cv::Mat& mat, int width, int height)
      : bgr_(mat), width_(width), height_(height), converted_(true) {
    cv::cvtColor(bgr_, hsv_, CV_BGR2HSV);
    time_stamp_ = ros::Time::now();
  }

  //! Frame destructor
  inline ~Frame() = default;

  /**
   * @brief Update frame in offline replaying mode.
   *
   * @param img - image from replaying bag file
   */
  inline void offlineUpdate(cv::Mat& img) {
    Timer t;
    bgr_ = img.clone();
    cv::cvtColor(bgr_, hsv_, CV_BGR2HSV);
    ROS_DEBUG("bgr to hsv used: %lf ms", t.elapsedMsec());
  }

  /**
   * \brief Get image in BGR color space
   *
   * \return reference to BGR image
   */
  inline cv::Mat& bgr() {
    cvt();
    return bgr_;
  }

  /**
   * \brief Get image in BGR color space
   *
   * \return BGR image
   */
  inline cv::Mat bgr_raw() {
    cvt();
    return bgr_;
  }

#ifdef USE_CUDA
  /**
   * \brief Get image in BGR color space
   *
   * \return reference to BGR image
   */
  // inline cv::cuda::GpuMat bgr_cuda() {
  //   cvt_cuda();
  //   return bgr_cuda_;
  // }
#endif

  /**
   * \brief Get image in HSV color space
   *
   * \return HSV image
   */
  inline cv::Mat& hsv() {
    cvt();
    return hsv_;
  }

#ifdef USE_CUDA
  /**
   * \brief Get image in HSV color space
   *
   * \return HSV image
   */
  // inline cv::cuda::GpuMat& hsv_cuda() {
  //   cvt_cuda();
  //   return hsv_cuda_;
  // }
#endif

  inline ros::Time& time_stamp() { return time_stamp_; }

  //!  Convert original YUV image to other color spaces
  inline void cvt() {
    if (converted_) return;

    Timer t;
    if (parameters.offline_replay && !parameters.offline_record) {
      if (bgr_.rows <= 0) return;
      cv::cvtColor(bgr_, hsv_, CV_BGR2HSV);
      converted_ = true;
      ROS_DEBUG("bgr to hsv used: %lf ms", t.elapsedMsec());
      return;
    }
    if (yuv_ != nullptr && bgr_.rows <= 0) {
      cv::Mat yuvMat(height_, width_, CV_8UC2, yuv_);
      cv::cvtColor(yuvMat, bgr_, CV_YUV2BGR_YUYV);
      ROS_DEBUG("yuv to bgr used: %lf ms", t.elapsedMsec());
      // Correct white balance
      t.restart();
      // std::vector<cv::Mat> split_bgr;
      // cv::split(bgr_, split_bgr);
      // for (uint i=0;i<split_bgr.size();i++)
      //   split_bgr[i] *= parameters.camera.bgr_gains[i];
      // cv::merge(split_bgr, bgr_);
      ROS_DEBUG("white balance correction used: %lf ms", t.elapsedMsec());
    }

    if (bgr_.rows > 0) {
      t.restart();
      cv::cvtColor(bgr_, hsv_, CV_BGR2HSV);
      converted_ = true;
      ROS_DEBUG("bgr to hsv used: %lf ms", t.elapsedMsec());
    }
  }

#ifdef USE_CUDA
  //! Convert original YUV image to other color spaces with CUDA acceleration
  // inline void cvt_cuda() {
  //   if (converted_cuda_) return;
  //   cv::cuda::cvtColor(bgr_cuda_, hsv_cuda_, CV_BGR2HSV);
  //   converted_cuda_ = true;
  // }
#endif

  //! Show current frame with OpenCV
  void show();

  /**
   * \brief Save current frame to disk
   *
   * \param path - filepath to save
   */
  void save(const std::string&);

  // h264 encode & decode
  // Mat --> cv_bridge::CvImage --> ImageMsg --> Encoding --> H264EncoderResult
  // --> raw buffer(pointer, size) --> Decoding --> ImageMsg -->
  // cv_bridge::CvImagePtr --> Mat

  //! Initialize H.264 encoder
  // static void initEncoder();

  /**
   * \brief Encode current frame
   *
   * \param length - length of encoded frame
   *
   * \return Pointer to encoded result
   */
  // std::unique_ptr<uint8_t> encode(int& length);

  /**
   * \brief Static method for encoding given image
   *
   * \param src - given image
   * \param length - length of encoded frame
   *
   * \return Pointer to encoded result
   */
  // static std::unique_ptr<uint8_t> encode(const cv::Mat& src, int& length);

  /**
   * \brief Decode buffer into image
   *
   * \param buffer - received raw buffer
   */
  // void decode(void* buffer);

  /**
   * @brief Decode buffer into ROS image message.
   *
   * \param [in] buffer - received raw buffer
   * @param [out] out - ROS image message
   */
  // static void decodeBuffer(void* buffer, sensor_msgs::ImagePtr& out);

  /**
   * @brief Convert ROS image message into OpenCV Mat image.
   *
   * @param [in] out - ROS image message
   * @param [out] bgr - OpenCV Mat image.
   */
  // static void convertMessage(const sensor_msgs::ImagePtr& out, cv::Mat& bgr);

 private:
  //! Raw image in YUV color space
  uint8_t* yuv_ = nullptr;  // raw yuv image
  //! image in BGR color space
  cv::Mat bgr_;
  //! image in HSV color space
  cv::Mat hsv_;
#ifdef USE_CUDA
  //! gpu pointer
  cv::cuda::GpuMat bgr_cuda_;
  cv::cuda::GpuMat hsv_cuda_;
#endif

  //! frame timestamp
  ros::Time time_stamp_;
  //! image width
  int width_ = 640;
  //! image height
  int height_ = 480;
  //! Whether or not frame is converted to RGB and HSV color spaces
  bool converted_ = false;
  //! Whether or not GPU frame is converted to RGB and HSV color spaces
  bool converted_cuda_ = false;

  // h264 encoding
  //! Decoder for H.264 Streaming
  // static ros_h264_streamer::H264Decoder* decoder_;
  //! Encoder for H.264 Streaming
  // static ros_h264_streamer::H264Encoder* encoder_;
};
}  // namespace dvision
