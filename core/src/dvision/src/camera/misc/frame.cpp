/* Copyright (C) ZJUDancer
 * 2018 - Yusu Pan <xxdsox@gmail.com>
 * 2017 - Wenxing Mei <mwx36mwx@gmail.com>
 */

/**
 * \file frame.cpp
 * \author Yusu Pan, Wenxing Mei
 * \version 2018
 * \date 2018-02-13
 */

#include "dvision/frame.hpp"
#include <iostream>
#include <opencv2/opencv.hpp>
#include <string>

namespace dvision {
// ros_h264_streamer::H264Encoder* Frame::encoder_ = nullptr;
// ros_h264_streamer::H264Decoder* Frame::decoder_ = nullptr;
//
// void Frame::initEncoder() {
//  std_msgs::Header header;
//  // FIXME(MWX): remove magic number
//  cv::Mat m(480, 640, CV_8UC3, cv::Scalar(0, 0, 0));
//  cv_bridge::CvImage cvmsg(header, "bgr8", m);
//  sensor_msgs::ImagePtr msg = cvmsg.toImageMsg();
//  // API: H264Encoder::H264Encoder(int width, int height, int quality_level,
//  int
//  // fps_num, int fps_den, const std::string & encoding, bool streaming)
//  encoder_ = new ros_h264_streamer::H264Encoder(msg->width, msg->height, 20,
//  30,
//                                                1, msg->encoding);
//  decoder_ = new ros_h264_streamer::H264Decoder(msg->width, msg->height);
//}
//
// std::unique_ptr<uint8_t> Frame::encode(int& length) {
//  // Check whether or not encoder is initialized
//  if (!encoder_) {
//    ROS_ERROR("Encoder not initialised!");
//    return std::unique_ptr<uint8_t>();
//  }
//
//  // Convert frame (cv::Mat) into ROS Image message
//  std_msgs::Header header;
//  cv_bridge::CvImage cvmsg(header, "bgr8", getBGR());
//  sensor_msgs::ImagePtr msg = cvmsg.toImageMsg();
//
//  // Encode ROS Image message in H.264 format
//  // ros::Time t1 = ros::Time::now();
//  auto res = encoder_->encode(msg);
//  // ros::Time t2 = ros::Time::now();
//  // ros::Duration d = t2 -  t1;
//  // ROS_INFO("[dvision] Image encoding: size: %d, time: %lf", res.frame_size,
//  // d.toSec());
//
//  // Add encoded result into buffer
//  std::unique_ptr<uint8_t> buf(new uint8_t[res.frame_size + 4]);
//  memcpy(buf.get(), &res.frame_size, sizeof(uint32_t));
//  memcpy(&buf.get()[4], res.frame_data, sizeof(uint8_t) * res.frame_size);
//
//  length = res.frame_size + sizeof(uint32_t);
//  return buf;
//}
//
// std::unique_ptr<uint8_t> Frame::encode(const cv::Mat& src, int& length) {
//  // Check whether or not encoder is initialized
//  if (!encoder_) {
//    ROS_ERROR("[dvision] Encoder not initialized!");
//    return std::unique_ptr<uint8_t>();
//  }
//
//  // Convert frame (cv::Mat) into ROS Image message
//  std_msgs::Header header;
//  cv_bridge::CvImage cvmsg(header, "bgr8", src.clone());
//  sensor_msgs::ImagePtr msg = cvmsg.toImageMsg();
//
//  // Encode ROS Image message in H.264 format
//  // ros::Time t1 = ros::Time::now();
//  auto res = encoder_->encode(msg);
//  // ros::Time t2 = ros::Time::now();
//  // ros::Duration d = t2 - t1;
//  // ROS_INFO("Image encoding: size: %d, time: %lf", res.frame_size,
//  d.toSec());
//
//  // Add encoded result into buffer
//  std::unique_ptr<uint8_t> buf(new uint8_t[res.frame_size + 4]);
//  memcpy(buf.get(), &res.frame_size, sizeof(uint32_t));
//  memcpy(&buf.get()[4], res.frame_data, sizeof(uint8_t) * res.frame_size);
//
//  length = res.frame_size + sizeof(uint32_t);
//  return buf;
//}
//
// void Frame::decode(void* buffer) {
//  // Check whether or not encoder is initialized
//  if (!decoder_) {
//    ROS_WARN("Decoder not initialised!");
//    return;
//  }
//
//  // Copy encoded buffer and construct ROS Image message
//  int size;  // !? 32t
//  memcpy(&size, buffer, sizeof(uint32_t));
//  sensor_msgs::ImagePtr out(new sensor_msgs::Image);
//
//  // Decode ROS Image message in H.264 format
//  //    ros::Time t1 = ros::Time::now();
//  decoder_->decode(size, &static_cast<uint8_t*>(buffer)[4], out);
//  //    ros::Time t2 = ros::Time::now();
//  //    ROS_INFO("Image decoding: size: %d, time: %lf", size, (t2 -
//  //    t1).toSec());
//
//  // Convert ROS Image message into frame (cv::Mat)
//  cv_bridge::CvImagePtr cvout = cv_bridge::toCvCopy(out);
//  bgr_ = cvout->image;
//  converted_ = true;
//}
//
// void Frame::decodeBuffer(void* buffer, sensor_msgs::ImagePtr& out) {
//  // Check whether or not encoder is initialized
//  if (!decoder_) {
//    ROS_WARN("Decoder not initialised!");
//    return;
//  }
//
//  // Copy encoded buffer and construct ROS Image message
//  int size;  // !? 32t
//  memcpy(&size, buffer, sizeof(uint32_t));
//  // sensor_msgs::ImagePtr out(new sensor_msgs::Image);
//
//  // Decode ROS Image message in H.264 format
//  decoder_->decode(size, &static_cast<uint8_t*>(buffer)[4], out);
//}
//
// void Frame::convertMessage(const sensor_msgs::ImagePtr& out, cv::Mat& bgr) {
//  cv_bridge::CvImagePtr cvout = cv_bridge::toCvCopy(out);
//  bgr = cvout->image;
//}

void Frame::save(const std::string& filename) {
  cvt();
  cv::imwrite(filename, bgr_);
  ROS_INFO("[dvision] Frames saved as %s", filename.c_str());
}

void Frame::show() {
  cvt();
  cv::namedWindow("camera", CV_WINDOW_NORMAL);
  cv::imshow("camera", bgr_);
}
}  // namespace dvision
