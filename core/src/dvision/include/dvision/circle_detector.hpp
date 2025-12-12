/* Copyright (C) ZJUDancer
 * 2017 - Yusu Pan <xxdsox@gmail.com>, Yujie Yang <meetyyj@gmail.com>
 */

/**
 * \file circle_detector.hpp
 * \author Yusu Pan, Yujie Yang
 * \version 2018
 * \date 2018-02-15
 */

#pragma once
#include <ros/ros.h>

#include <vector>

#include "dancer_geometry/line_segment.hpp"
#include "dmsgs/VisionInfo.h"
#include "dvision/idetector.hpp"
#include "dvision/kalman.hpp"
#include "dvision/projection.hpp"
#include "dvision/timer.hpp"

using dmsgs::VisionInfo;
namespace dvision {
//! Detector for center circle
class CircleDetector : public IDetector {
 public:
  //! CircleDetector constructor
  explicit CircleDetector();
  //! CircleDetector destructor
  ~CircleDetector() override = default;
  //! Initialize CircleDetector
  bool Init() override;
  /**
   * \brief Detect corner from x intersections
   *
   * \param x_intx - detected x intersection from CornerDetector
   * \param projection - used for projecting from image plane to field plane
   * \param vision_info - used for storing detected circle info
   * \param delta - robot move delta for kalman control input
   * \param marks - put detected circle into landmarks for localization
   */
  void Detect(std::vector<cv::Point2f>& x_intx, Projection& projection,
              VisionInfo& vision_info, const Control& delta,
              Measurement& marks);
  /**
   * \brief Process center circle detection
   *
   * \param x_intx - detected x intersection from CornerDetector
   * \param projection - used for projecting from image plane to field plane
   * \param delta - robot move delta for kalman control input
   *
   * \return whether or not center circle is detected
   */

  bool Process(std::vector<cv::Point2f>& x_intx, Projection& projection,
               const Control& delta);

  //! Update prediction of circle position by kalman filter
  bool Update(const Control& delta);

  //! Get center circle in field plane
  inline cv::Point2f& result_circle() {
    if (parameters.circle.useKalman) {
      return circle_field_kalman_;
    } else {
      return result_circle_;
    }
  }

 private:
  //! Center circle position in field plane
  cv::Point2f result_circle_;
  //! Center point of circle in field plane, corrected by Kalman Filter
  cv::Point2f circle_field_kalman_;

  //! Instance of Kalman Filter
  ObjectPosKalmanFilter* kalmanI_;
};
}  // namespace dvision
