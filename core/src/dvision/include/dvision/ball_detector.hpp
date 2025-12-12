/* Copyright (C) ZJUDancer
 * 2017 - Yusu Pan <xxdsox@gmail.com>
 */

/**
 * \file ball_detector.hpp
 * \author Yusu Pan <xxdsox@gmail.com>
 * \version 2018
 * \date 2018-02-15
 */

#pragma once
#include <ros/ros.h>
#include <fstream>
#include <queue>
#include <string>
#include <vector>
#include "dancer_geometry/utils.hpp"
#include "dmsgs/VisionInfo.h"
#include "dvision/idetector.hpp"
#include "dvision/kalman.hpp"
#include "dvision/projection.hpp"
#include "dvision/timer.hpp"

#include "yolo/utils.hpp"


using dmsgs::VisionInfo;
namespace dvision {

//! Ball detector
struct Control;

struct BallPos {
  BallPos(cv::Point2f p, double t) : pos(p), time(t) {}
  cv::Point2f pos;
  double time;
};

class BallDetector : public IDetector {
 public:
  //! BallDetector constructor
  explicit BallDetector();
  //! BallDetector destructor
  ~BallDetector() override = default;
  //! Initialize BallDetector
  bool Init() override;
  /**
   * \brief Detect ball position in the image
   *
   * using bounding boxes from ObjectDetector and remove false positive
   * detection
   *
   * \param ball_position - bounding boxes of detected ball
   * \param gui_img - GUI image for drawing detections
   * \param field_hull_real - field boundary
   * \param field_binary - binary image of field
   * \param projection - used for projecting from image plane to field plane
   * \param vision_info - used for storing detected ball info
   */
  void Detect(const BBox& ball_position, cv::Mat& gui_img,
              std::vector<cv::Point2f>& field_hull_real, cv::Mat& field_binary,
              Projection& projection, VisionInfo& vision_info,
              const Control& delta);

  //! Get ball position in image coordinate
  inline cv::Point ball_image() { return ball_image_; }

  //! Get ball position in real coordinate (i.e field plane)
  inline cv::Point2f ball_field() {
    if (parameters.ball.useKalman) {
      return ball_field_kalman_;
    } else {
      return ball_field_;
    }
  }

 private:
  //! Center point of ballin image plane
  cv::Point ball_image_;
  //! Top left point of ball in image plane
  cv::Point ball_image_top_;
  //! Bottom right point of ball in image plane
  cv::Point ball_image_bottom_;
  //! Radius of ball in image plane
  float ball_image_radius_;
  //! Center point of ball in field plane
  cv::Point2f ball_field_;
  //! Center point of ball in field plane, corrected by Kalman Filter
  cv::Point2f ball_field_kalman_;

  //! Instance of Kalman Filter
  ObjectPosKalmanFilter *kalmanI_;

  //! Update prediction of ball position by kalman filter
  bool Update(const Control& delta);
  /**
   * \brief Process ball detection
   *
   * \param ball_position - bounding boxes of detected ball
   * \param gui_img - GUI image for drawing detections
   * \param field_hull_real - field boundary
   * \param field_binary - binary image of field
   * \param projection - reference to Projection instance for get position in
   * field plane
   *
   * \return whether or not process is successful
   */
  bool Process(const BBox& ball_position, cv::Mat& gui_img,
               std::vector<cv::Point2f>& field_hull_real, cv::Mat& field_binary,
               Projection& projection, const Control& delta);
  /**
   * \brief Find center of ball
   *
   * \param field_binary - binary image of field
   * \param ball_image_top - top left point of ball in image plane
   * \param ball_image_bottom - bottom right point of ball in image plane
   * \param scale - scaling factor of bbox area to find center
   * \param projection - reference to Projection instance for get position in
   * field plane
   *
   * \return whether or not process is successful
   */
  bool FindBallCenter(const cv::Mat& field_binary,
                      const cv::Point& ball_image_top,
                      const cv::Point& ball_image_bottom, const float& scale,
                      Projection& projection);
  /**
   * \brief Check whether or not ball is in field
   *
   * \param field_hull_real - field boundary
   *
   * \return whether or not ball is in field
   */
  bool CheckBallInField(std::vector<cv::Point2f>& field_hull_real);
  /**
   * \brief Check whether or not the distance between ball and robot is valid
   *
   * \return whether or not the distance between ball and robot is valid
   */
  bool CheckBallDist();
  /**
   * \brief Check whether or not radius of ball is valid
   *
   * \param projection - reference to Projection instance for get position in
   * field plane
   *
   * \return whether or not radius of ball is valid
   */
  bool CheckBallRadius(Projection& projection);
};
}  // namespace dvision
