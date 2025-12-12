/* Copyright (C) ZJUDancer
 * 2017 - Yusu Pan <xxdsox@gmail.com>
 */

/**
 * \file object_detector.hpp
 * \author Yusu Pan
 * \version 2018
 * \date 2018-02-17
 */

#pragma once

#include <ctime>
#include <iomanip>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <sstream>
#include <string>
#include <vector>

#include "dvision/idetector.hpp"
#include "dvision/timer.hpp"
#include "dvision/yolo/yolov5.hpp"
#include "dvision/yolo/utils.hpp"


namespace dvision {

struct FieldFeatures {
  std::vector<BBox> l;
  std::vector<BBox> t;
  std::vector<BBox> x;
  std::vector<BBox> penalty;
  void clear() {
    l.clear();
    t.clear();
    x.clear();
    penalty.clear();
  }
};
class ObjectDetector : public IDetector {
 public:
  //! ObjectDetector constructor
  explicit ObjectDetector() = default;
  //! ObjectDetector destructor
  ~ObjectDetector() override = default;
  //! Initialize ObjectDetector
  bool Init() override;

  /**
   * \brief Detect objects in the image by
   *
   * \param frame - frame img from camera
   * \param gui_img - GUI image for drawing detections
   * \param ball_quality - output of found ball quality
   */
#ifdef USE_GST_CAMERA
  void Detect(const cv::cuda::GpuMat &frame, cv::Mat &gui_img,
              float &ball_quality);
#else
  void Detect(const cv::Mat &frame, cv::Mat &gui_img, float &ball_quality);
#endif
  /**
   * \brief Process object detection
   *
   * \param frame - frame img from camera
   * \param gui_img - GUI image for drawing detections
   * \param ball_quality - output of found ball quality
   *
   * \return
   */
#ifdef USE_GST_CAMERA
  bool Process(const cv::cuda::GpuMat &frame, cv::Mat &gui_img,
               float &ball_quality);
#else
  bool Process(const cv::Mat &frame, cv::Mat &gui_img, float &ball_quality);
#endif

  //! Get bounding boxes of all objects
  inline std::vector<BBox> &object_position() {
    return object_position_;
  }

  //! Get position of ball
  inline BBox &ball_position() {
    if (ball_detected_) {
      return ball_position_;
    } else {
      return unknown_position_;
    }
  }

  //! Get position of goal
  inline std::vector<BBox> goal_position() {
    if (goal_detected_) {
      return goal_position_;
    } else {
      std::vector<BBox> unknowns;
      // unknowns.push_back(unknown_position_);
      return unknowns;
    }
  }

  //! Get positions of obstacles
  inline std::vector<BBox> obstacle_position() {
    if (obstacle_detected_) {
      return obstacle_positions_;
    } else {
      std::vector<BBox> unknowns;
      return unknowns;
    }
    return obstacle_positions_;
  }

  inline FieldFeatures field_features() {
    return field_features_;
  }

 private:
  std::vector<std::string> class_names_;
  std::vector<float> anchor_priors_;
  yolov5 v5;
#ifdef USE_CUDA
  
#endif

  //! Bounding boxes of objects
  std::vector<BBox> object_position_;
  //! Bounding box of ball
  BBox ball_position_;
  //! Bounding box of goal
  std::vector<BBox> goal_position_;
  //! Bounding box of unknown object
  BBox unknown_position_;
  //! Bounding boxes of obstacles
  std::vector<BBox> obstacle_positions_;
  //! Bounding boxes of field features
  FieldFeatures field_features_;

  //! Flag for whether or not ball is detected
  bool ball_detected_ = false;
  //! Flag for whether or not goal is detected
  bool goal_detected_ = false;
  //! Flag for whether or not any obstacle is detected
  bool obstacle_detected_ = false;
  //! Flag for whether or not t corner is detected
  bool tcorner_detected_ = false;
  bool xcorner_detected_ = false;
  bool lcorner_detected_ = false;
  bool penalty_detected_ = false;

  //! Re-initialize
  bool ReInit();

  /**
   * \brief Check object validation by its scale factor with the whole image
   * size
   *
   * \param left - position of left boundary of object
   * \param right - position of right boundary of object
   * \param top - position of top boundary of object
   * \param bottom - position of bottom boundary of object
   * \param scale_coff - given scale factor
   *
   * \return whether or not object is valid w.r.t scale
   */
  bool CheckValidScale(const int &left, const int &right, const int &top,
                       const int &bottom, const float &scale_coff);

  /**
   * \brief Check object validation by its ratio of width and height
   *
   * \param left - position of left boundary of object
   * \param right - position of right boundary of object
   * \param top - position of top boundary of object
   * \param bottom - position of bottom boundary of object
   * \param low_ratio - minimum valid ratio
   * \param high_ratio - maximum valid ratio
   *
   * \return whether or not object is valid w.r.t ratio
   */
  bool CheckValidRatio(const int &left, const int &right, const int &top,
                       const int &bottom, const float &low_ratio,
                       const float &high_ratio);
};


}  // namespace dvision


