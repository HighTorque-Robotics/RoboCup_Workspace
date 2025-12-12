/* Copyright (C) ZJUDancer
 * 2017 - Yusu Pan <xxdsox@gmail.com>
 */

/**
 * \file obstacle_detector.hpp
 * \author Yusu Pan <xxdsox@gmail.com>
 * \version 2018
 * \date 2018-02-17
 */

#pragma once
#include <string>
#include <vector>
#include "dmsgs/VisionInfo.h"
#include "dvision/idetector.hpp"
#include "dvision/projection.hpp"
#include "dvision/timer.hpp"
#include <opencv2/imgproc/types_c.h>

using dmsgs::VisionInfo;

namespace dvision {
//! Obstacle objcet
class ObstacleObject {
 public:
  //! ObstacleObject constructor
  explicit ObstacleObject(cv::Point2f position, float confidence = 1.0,
                          int id = 0);
  //! ObstacleObject destructor
  ~ObstacleObject() = default;

  //! Get position of obstacle object
  inline cv::Point2f position() { return position_; }

  //! Get confidence of obstacle object
  inline float confidence() {
    boundry_n(confidence_, 0.0, 1.0);
    return confidence_;
  }

  //! Get id of obstacle object
  inline int id() { return id_; }

  //! Decay confidence of object by time
  bool DecayConfidence();
  /**
   * \brief Update position and confidence of object
   *
   * \param pos - position of objectj
   * \param conf - confidence of object
   *
   * \return whether or not process is successful
   */
  bool Update(cv::Point2f pos, float conf);

 private:
  //! Position of obstacle object
  cv::Point2f position_;
  //! Confidence of obstacle object
  float confidence_;
  //! ID of obstacle object
  int id_;
};

//! Detector for obstacles
class ObstacleDetector : public IDetector {
 public:
  //! ObstacleDetector constructor
  explicit ObstacleDetector() = default;
  //! ObstacleDetector destructor
  ~ObstacleDetector() override = default;

  //! Initialize ObstacleDetector
  bool Init() override;
  /**
   * \brief Detect obstacles in the image
   *
   * \param hsv_img - frame from camera in HSV color space
   * \param obstacle_binary - binary image for obstacles
   * \param gui_img - GUI image for drawing detections
   * \param field_convex_hull - image with convex hull of field
   * \param projection - used for projecting from image plane to field plane
   * \param vision_info - used for storaging detected obstacle info
   */
  void Detect(const cv::Mat& hsv_img, cv::Mat& obstacle_binary,
              cv::Mat& gui_img, const cv::Mat& field_convex_hull,
              Projection& projection, VisionInfo& vision_info);

  //! Get obstacle bottom points
  inline std::vector<cv::Point2f> obstacle_points() { return obstacle_points_; }

  //! Get obstacle objects
  inline std::vector<ObstacleObject> obstacle_objects() {
    return obstacle_objects_;
  }

  //! Get obstacle mask image
  inline cv::Mat obstacle_mask() { return obstacle_mask_; }

 private:
  //! Bottom points of obstacles
  std::vector<cv::Point2f> obstacle_points_;
  //! Obstacle objects
  std::vector<ObstacleObject> obstacle_objects_;
  //! Binary image for obstacles
  cv::Mat obstacle_mask_;

  //! Update position and confidence of objects
  void Update();

  /**
   * \brief Process obstacle detection
   *
   * \param obstacle_binary - binary image for obstacles
   * \param gui_img - GUI image for drawing detections
   * \param projection - used for projecting from image plane to field plane
   *
   * \return whether or not field is detected
   */
  bool Process(cv::Mat& obstacle_binary, cv::Mat& gui_img,
               Projection& projection);

  /**
   * \brief Get binary image for obstacles
   *
   * \param hsv_img - frame from camera in HSV color space
   * \param field_convex_hull - image with convex hull of field
   * \param obstacle_binary - binary image for obstacles
   */
  void GetBinary(const cv::Mat& hsv_img, const cv::Mat& field_convex_hull,
                 cv::Mat& obstacle_binary);
};

}  // namespace dvision
