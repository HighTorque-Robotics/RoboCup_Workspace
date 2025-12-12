#pragma once
#include <math.h>
#include <ros/ros.h>

#include <vector>

#include "dancer_geometry/line_segment.hpp"
#include "dmsgs/VisionInfo.h"
#include "dvision/amcl/types.hpp"
#include "dvision/idetector.hpp"
#include "dvision/object_detector.hpp"
#include "dvision/projection.hpp"
#include "dvision/timer.hpp"

#include "dvision/yolo/utils.hpp"


using dmsgs::VisionInfo;
namespace dvision {
//! Detector for corner
class CornerDetector : public IDetector {
 public:
  //! CornerDetector constructor
  explicit CornerDetector();
  //! CornerDetector destructor
  ~CornerDetector();
  //! Initialize CornerDetector
  bool Init();
  /**
   * \brief Detect corner from lines
   *
   * \param *_boxxes - bounding boxes from yolo
   * \param result_lines - detected lines from LineDetector
   * \param projection - used for projecting from image plane to field plane
   * \param vision_info - used for storaging detected corner info
   */
  void Detect(FieldFeatures& field_features, Projection& projection,
              VisionInfo& vision_info, cv::Mat& gui_img, Measurement& mark);
  /**
   * \brief Process corner detection
   *
   * \param result_lines - detected lines from LineDetector
   * \param projection - used for projecting from image plane to field plane
   *
   * \return whether or not corner is detected
   */
  bool Process(Projection& projection, cv::Mat& gui_img,
               VisionInfo& vision_info);

  inline std::vector<cv::Point2f>& getXIntx() { return x_center_; }

 private:
  cv::Point2f result_corner_;

  //! yolo detect lines
  std::vector<dancer_geometry::LineSegment> yolo_lines_;
  //! yolo detect field features
  std::vector<cv::Point2f> t_center_;
  std::vector<cv::Point2f> x_center_;
  std::vector<cv::Point2f> l_center_;
  std::vector<cv::Point2f> penalty_center_;

  //! All feature points in the real playground
  std::vector<std::vector<cv::Point2f>> single_features_;
  std::vector<std::vector<cv::Point2f>> combined_features_;

  // Boundary check for t, l intx and penalty mark
  int min_x_, max_x_, min_y_, max_y_;
  inline bool boxInBoundary(const BBox& box) {
    if (box.left_top.x < min_x_) return true;
    if (box.left_top.y < min_y_) return true;
    if (box.right_bottom.x > max_x_) return true;
    if (box.right_bottom.y > max_y_) return true;
    return false;
  }

  void yoloDetect(FieldFeatures& field_features, VisionInfo& vision_info,
                  Projection& projection);

  void classifyIntx(Measurement& mark, Projection& projection);
};
}  // namespace dvision
