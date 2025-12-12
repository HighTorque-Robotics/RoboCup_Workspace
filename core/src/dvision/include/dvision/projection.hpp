/* Copyright (C) ZJUDancer
 * 2018 - Yusu Pan <xxdsox@gmail.com>
 * 2017 - Wenxing Mei <mwx36mwx@gmail.com>
 */

/**
 * \file projection.hpp
 * \author Yusu Pan, Wenxing Mei
 * \version 2018
 * \date 2018-02-15
 */

// TODO(mwx) Version 1: only consider camera yaw and pitch

#pragma once
#include <ros/ros.h>

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <vector>

#include "dancer_geometry/line_segment.hpp"
#include "dancer_geometry/utils.hpp"
#include "dvision/distortionModel.hpp"
#include "dvision/ipm.hpp"

namespace dvision {
class Projection {
 public:
  //! Projection constructor
  explicit Projection();
  //! Projection destructor
  ~Projection();
  //! Initialize Projection class
  void init(ros::NodeHandle *);

 public:
  /**
   * \brief Update extrinsic parameters
   *
   * \param pitch - current pitch value of camera pose
   * \param yaw - current yaw value of camera pose
   */
  void updateExtrinsic(double pitch, double yaw);

  /**
   * \brief Undistort a point of distorted image (float version)
   *
   * \param point - point in distorted image
   *
   * \return resulted float point in undistorted image
   */
  inline cv::Point2f undistP(const cv::Point2f &point) {
    return dist_.undistort(point.x, point.y);
  }

  /**
   * \brief Undistort a set of points of distorted image
   *
   * \param points - points in distorted image
   * \param res - resulted points in undistorted image
   *
   * \return Whether or not undistortion is successful
   */
  inline bool undistort(const std::vector<cv::Point> &points,
                        std::vector<cv::Point> &res) {
    return dist_.undistort(points, res);
  }

  /**
   * \brief Undistort a set of points of distorted image (float version)
   *
   * \param points - points in distorted image
   * \param res - resulted float points in undistorted image
   *
   * \return Whether or not undistortion is successful
   */

  inline bool undistort(const std::vector<cv::Point> &points,
                        std::vector<cv::Point2f> &res) {
    return dist_.undistort(points, res);
  }

  /**
   * \brief Distort a set of points of undistorted image
   *
   * \param points - points in undistorted image
   * \param res - resulted float points in distorted image
   *
   * \return Whether or not distortion is successful
   */
  inline bool distort(const std::vector<cv::Point> &points,
                      std::vector<cv::Point> &res) {
    dist_.distort(points, res);
    return true;
  }

  // TODO what's the difference of distort and distortP?
  /**
   * \brief Distort a set of points of undistorted image
   *
   * \param points - points in undistorted image
   * \param res - resulted float points in distorted image
   *
   * \return Whether or not distortion is successful
   */
  inline bool distortP(const std::vector<cv::Point> &points,
                       std::vector<cv::Point> &res) {
    dist_.distortP(points, res);
    return true;
  }

  // points
  /**
   * \brief Project points from real coordinate into image coordinate
   *
   * \param points - points in real coordinate
   * \param res_points - resulted points in image coordinate
   *
   * \return Whether or not projection is successful
   */
  bool getOnImageCoordinate(const std::vector<cv::Point2f> &points,
                            std::vector<cv::Point> &res_points);
  /**
   * \brief Project points from image coordinate into real coordinate
   *
   * \param points - points in image coordinate
   * \param res_points - resulted points in real coordinate (i.e. field plane)
   * \param z_real - corresponding position z of those points
   *
   * \return Whether or not projection is successful
   */
  bool getOnRealCoordinate(const std::vector<cv::Point> &points,
                           std::vector<cv::Point2f> &res_points,
                           float z_real = 0);

  // lines
  /**
   * \brief Project lines from real coordinate into image coordinate
   *
   * \param lines - lines in real coordinate
   * \param res_lines - resulted lines in image coordinate
   *
   * \return Whether or not projection is successful
   */
  bool getOnImageCoordinate(
      const std::vector<dancer_geometry::LineSegment> &lines,
      std::vector<dancer_geometry::LineSegment> &res_lines);
  /**
   * \brief Project lines from image coordinate into real coordinate
   *
   * \param lines - lines in image coordinate
   * \param res_lines - resulted lines in real coordinate (i.e. field plane)
   * \param z_real - corresponding position z of those lines
   *
   * \return Whether or not projection is successful
   */
  bool getOnRealCoordinate(
      const std::vector<dancer_geometry::LineSegment> &lines,
      std::vector<dancer_geometry::LineSegment> &res_lines, float z_real = 0);

  // single point
  /**
   * \brief Project single point from real coordinate into image coordinate
   *
   * \param point - point in real coordinate
   * \param res_point - resulted point in image coordinate
   *
   * \return Whether or not projection is successful
   */
  bool getOnImageCoordinate(const cv::Point2f &point, cv::Point &res_point);
  /**
   * \brief Project single point from image coordinate into real coordinate
   *
   * \param point - single point in image coordinate
   * \param res_point - resulted single point in real coordinate (i.e. field
   * plane)
   * \param z_real - corresponding position z of the point
   *
   * \return Whether or not projection is successful
   */
  bool getOnRealCoordinate(const cv::Point &point, cv::Point2f &res_point,
                           float z_real = 0);

  // rotate
  /**
   * \brief Rotate points towards robot heading direction in real coordinate
   *
   * \param in - points in real coordinate
   *
   * \return rotated points in real coordinate
   */
  std::vector<cv::Point2f> RotateTowardHeading(
      const std::vector<cv::Point2f> &in);
  // TODO remove cv::Point2d, only remain cv::Point2f
  /**
   * \brief Rotate single point (double) towards robot heading direction in real
   * coordinate
   *
   * \param in - sinfle point in real coordinate
   *
   * \return rotated point in real coordinate
   */
  cv::Point2d RotateTowardHeading(const cv::Point2d &in);
  /**
   * \brief Rotate single point (float) towards robot heading direction in real
   * coordinate
   *
   * \param in - sinfle point in real coordinate
   *
   * \return rotated point in real coordinate
   */
  cv::Point2f RotateTowardHeading(const cv::Point2f &in);
  /**
   * \brief Rotate lines towards robot heading direction in real coordinate
   *
   * \param in - lines in real coordinate
   *
   * \return rotated lines in real coordinate
   */
  dancer_geometry::LineSegment RotateTowardHeading(
      const dancer_geometry::LineSegment &in);
  /**
   * \brief Rotate lines towards robot heading direction in real coordinate
   *
   * \param in - lines in real coordinate
   *
   * \return rotated lines in real coordinate
   */
  std::vector<dancer_geometry::LineSegment> RotateTowardHeading(
      const std::vector<dancer_geometry::LineSegment> &in);

  /**
   * \brief Calculate heading offset (i.e. field angle) by vision method
   *
   * Used for correcting odometry
   *
   * \param result_lines - detected white lines in real coordinate
   * \param circle_detected - flag for whether or not circle is detected
   * \param result_circle - detected center of circle in real coordinate
   * \param goal_position - detected goal points position in real coordinate
   *
   * \return whether or not process is successful
   */
  bool CalcHeadingOffset(
      std::vector<dancer_geometry::LineSegment> &result_lines,
      bool circle_detected, const cv::Point2d &result_circle,
      const std::vector<cv::Point2f> &goal_position);
  /**
   * \brief Update heading offset with IMU yaw and vision bias
   *
   * \param in_yaw yaw value (i.e. field angle) from odometry by IMU
   *
   * \return whether or not process is successful
   */
  bool UpdateHeadingOffset(double &in_yaw);
  /**
   * \brief Get heading offset (i.e. field angle)
   *
   * \return heading offset corrected by vision bias
   */
  inline double GetHeading() {
    // In Radian
    // if (std::abs(Radian2Degree(m_heading_offset)) > 90) {
    //     ROS_WARN("Heading offset flip prevented!");
    //     m_heading_offset = 0;
    // }
    // TODO(corenel) how to get heading offset?
    return dancer_geometry::CorrectAngleRadian360(0 + heading_offset_);
    // return CorrectAngleRadian360(headingData.heading + m_heading_offset);
  }

  /**
   * \brief Get distortion model
   *
   * \return instance of distortion model
   */
  inline DistortionModel *dist() { return &dist_; }
  /**
   * \brief Set heading offset bias (i.e. vision bias)
   *
   * \param bias - desired vision bias in degree
   */
  void SetHeadingOffsetBias(double bias);
  /**
   * \brief Clear heading offset bias
   */
  inline void ClearHeadingOffsetBias() { heading_offset_bias_ = 0; }

 private:
  //! Blank initialzation
  void init();

 private:
  //! Instance of distortion model
  DistortionModel dist_;
  //! Instance of IPM
  IPM ipm_;

  // VERSION 1, use only yaw and pitch, 2017/5/29
  //! Yaw value of camera pose
  double yaw_;
  //! Pitch value of camera pose
  double pitch_;
  //! Heading offset of robot in radian
  double heading_offset_;  // in radian
  //! Heading offset bias of robot in radian
  double heading_offset_bias_;  // in radian
  //! Extrinsic parameter matrix of camera
  Eigen::MatrixXd extrinsic_;

  //! Homography from image plane to real plane
  cv::Mat homo_img_to_real_;  // real = homo * img
  //! Homography from real plane to image plane
  cv::Mat homo_real_to_img_;  // img = homo * real
};
}  // namespace dvision
