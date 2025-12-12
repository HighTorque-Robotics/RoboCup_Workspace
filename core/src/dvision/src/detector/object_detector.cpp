/* Copyright (C) ZJUDancer
 * 2017 - Yusu Pan <xxdsox@gmail.com>
 */

/**
 * \file object_detector.cpp
 * \author Yusu Pan
 * \version 2018
 * \date 2018-02-17
 */

#include "dvision/object_detector.hpp"

namespace dvision {
#ifdef USE_GST_CAMERA
void ObjectDetector::Detect(const cv::cuda::GpuMat &frame, cv::Mat &gui_img,
                            float &ball_quality) {
#else
void ObjectDetector::Detect(const cv::Mat &frame, cv::Mat &gui_img,
                            float &ball_quality) {
#endif
  Timer t;
  Process(frame, gui_img, ball_quality);
  ROS_DEBUG("object detect used %lf ms", t.elapsedMsec());
}

bool ObjectDetector::Init() {
  if (!parameters.object.enable) return false;
  unknown_position_.class_label_prob = 0.0;
#ifdef USE_CUDA
  // read class names
  if (!read_text_file(class_names_,
                              parameters.object.input_names_file)) {
    ROS_ERROR("Failed to read names file");
    return false;
  }
  // construct YOLO model
  v5.init(parameters.object.input_model_file,parameters.object.threshold,
  parameters.object.nms_threshold,parameters.object.batch_size);

#else
  ROS_WARN("GPU-accelerated object detection is not in use.");
#endif

  ROS_DEBUG("ObjectDetector Init");
  return true;
}

#ifdef USE_GST_CAMERA
bool ObjectDetector::Process(const cv::cuda::GpuMat &frame, cv::Mat &gui_img,
                             float &ball_quality) {
#else
bool ObjectDetector::Process(const cv::Mat &frame, cv::Mat &gui_img,
                             float &ball_quality) {
#endif
  if (!parameters.object.enable) {
    ball_quality = 0;
    return false;
  }

  // ROS_DEBUG("BallDetector Tick");
  // reinit container
  ReInit();

#ifdef USE_CUDA
// create list of images to feed
#ifdef USE_GST_CAMERA
  std::vector<cv::cuda::GpuMat> images;
#else
  std::vector<cv::Mat> images;
#endif

  // process the same image multiple times if batch size > 1
  for (int i = 0; i < parameters.object.batch_size; ++i) {
    images.push_back(frame);
  }
  // register images to the preprocessor
  auto detections = v5.detect(images);

  // detection classification
  float ball_max_prob = 0.0;
  // float obstacle_max_prob = 0.0;
  obstacle_positions_.clear();
  goal_position_.clear();
  field_features_.clear();
  for (auto detection : detections[0]) {
    BBox bbox;
    bbox.left_top = cv::Point(static_cast<int>(detection.bbox[0]-detection.bbox[2]/2),
                              static_cast<int>(detection.bbox[1]-detection.bbox[3]/2));
    bbox.right_bottom =
        cv::Point(static_cast<int>(detection.bbox[0]+detection.bbox[2]/2),
                  static_cast<int>(detection.bbox[1]+detection.bbox[3]/2));
    bbox.class_label_index = detection.class_id;
    bbox.class_label_prob = detection.conf;

    if (bbox.class_label_index == 0 && bbox.class_label_prob > ball_max_prob &&
        CheckValidScale(bbox.left_top.x, bbox.right_bottom.x, bbox.left_top.y,
                        bbox.right_bottom.y,
                        parameters.object.ball_max_scale_coff) &&
        CheckValidRatio(bbox.left_top.x, bbox.right_bottom.x, bbox.left_top.y,
                        bbox.right_bottom.y,
                        parameters.object.ball_wh_low_ratio,
                        parameters.object.ball_wh_high_ratio)) {
      ball_detected_ = true;
      ball_max_prob = bbox.class_label_prob;
      ball_position_ = bbox;
    } else if (bbox.class_label_index == 1) {
      goal_detected_ = true;
      goal_position_.push_back(bbox);
    } else if (bbox.class_label_index == 2) {
      // && bbox.class_label_prob > obstacle_max_prob &&
      //  std::abs(bbox.left_top.x - bbox.right_bottom.x) <
      //  parameters.camera.width * 0.5 && std::abs(bbox.left_top.y -
      //  bbox.right_bottom.y) < parameters.camera.height * 0.5) {
      obstacle_detected_ = true;
      // obstacle_max_prob = bbox.class_label_prob;
      obstacle_positions_.push_back(bbox);
    } else if (bbox.class_label_index == 3) {
      field_features_.t.push_back(bbox);
    } else if (bbox.class_label_index == 4) {
      field_features_.l.push_back(bbox);
    } else if (bbox.class_label_index == 5) {
      field_features_.x.push_back(bbox);
    } else if (bbox.class_label_index == 6) {
      field_features_.penalty.push_back(bbox);
    }
  }

  // show detection results
  if (parameters.monitor.update_gui_img &&
      parameters.object.showAllDetections) {
    draw_detections(detections[0], class_names_, gui_img);
  }

  ball_quality = ball_max_prob;
#endif

  // return if any object is detected
  return object_position_.empty();
}

bool ObjectDetector::ReInit() {
  object_position_.clear();
  ball_detected_ = false;
  goal_detected_ = false;
  obstacle_detected_ = false;
  return true;
}

bool ObjectDetector::CheckValidScale(const int &left, const int &right,
                                     const int &top, const int &bottom,
                                     const float &scale_coff) {
  return (std::abs(left - right) < parameters.camera.width * scale_coff &&
          std::abs(top - bottom) < parameters.camera.height * scale_coff);
}

bool ObjectDetector::CheckValidRatio(const int &left, const int &right,
                                     const int &top, const int &bottom,
                                     const float &low_ratio,
                                     const float &high_ratio) {
  // Pass
  return true;
  int width = std::abs(right - left);
  int height = std::abs(bottom - top);
  if (width != 0 && height != 0) {
    float ratio = static_cast<float>(width) / static_cast<float>(height);
    return (ratio > low_ratio) && (ratio < high_ratio);
  } else {
    return false;
  }
}

}  // namespace dvision
