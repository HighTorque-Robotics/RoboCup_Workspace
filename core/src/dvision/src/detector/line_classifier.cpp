/* Copyright (C) ZJUDancer
 * 2018 - Yusu Pan <xxdsox@gmail.com>
 * 2017 - Yujie Yang <meetyyj@gmail.com>
 */

/**
 * \file line_classifier.cpp
 * \author Yusu Pan, Yujie Yang
 * \version 2018
 * \date 2018-02-17
 */

#include "dvision/line_classifier.hpp"
#include <fmt/format.h>

using dancer_geometry::Angle;
using namespace dancer_geometry;

namespace dvision {

LineClassifier::LineClassifier() {}

LineClassifier::~LineClassifier() {}

bool LineClassifier::Init() {
  // loc_cnt = 0;
  // ROS_INFO("LineClassifier Init() finished");
  return true;
}

// bool
// LineClassifier::Update()
// {
//     return true;
// }

bool LineClassifier::Process(
    std::vector<dancer_geometry::LineSegment> &good_lines,
    const cv::Point2f &result_circle,
    const std::vector<cv::Point2f> &goal_position, Projection &projection,
    VisionInfo &vision_info, const double &vision_yaw) {
  if (!parameters.line_classifier.enable) {
    return false;
  }
  Timer t;
  center_lines_.clear();
  goal_lines_.clear();
  other_lines_.clear();
  // cv::Mat m_linec_img = cv::Mat(500, 500, CV_8UC3, cv::Scalar(0, 0, 0));
  if (vision_info.see_field && vision_info.see_line) {
    std::vector<dancer_geometry::LineSegment> good_lines_rotated =
        projection.RotateTowardHeading(good_lines);

    // drawContours
    // double offssx = 100;
    // double offssy = 250;
    // double ratioo = 0.3;
    // std::vector<cv::Point> field_hull_real_rotated_ratio;
    // for (size_t i = 0; i < field_hull_real_rotated.size(); i++) {
    //     field_hull_real_rotated_ratio.push_back(cv::Point(field_hull_real_rotated[i].x
    //     * ratioo + offssx, -field_hull_real_rotated[i].y * ratioo + offssy));
    // }
    // std::vector<std::vector<cv::Point>> hulls_real(1,
    // field_hull_real_rotated_ratio);
    // cv::drawContours(m_linec_img, hulls_real, -1, orangeColor(), 1, 8);

    LineSegment HorLine(cv::Point(0, -10), cv::Point(0, 10));
    LineSegment VerLine(cv::Point(10, 0), cv::Point(-10, 0));

    for (size_t i = 0; i < good_lines_rotated.size(); i++) {
      LineSegment line_seg = good_lines_rotated[i];
      // res_lines.push_back(line_seg);
      if (line_seg.GetLength() > parameters.yaw_correction.minLineLen) {
        // 取中点mid
        // cv::Point2d mid = line_seg.GetMiddle();
        if (line_seg.GetAbsMinAngleDegree(VerLine) <
            parameters.yaw_correction.angle2VerLine) {
          other_lines_.push_back(line_seg);
          // If the vertical line is long enough, use it to correct yaw
          if (line_seg.GetLength() >= parameters.yaw_correction.otherLineLen) {
            double angle_diff_hor = line_seg.GetExteriorAngleDegree(VerLine);
            if (angle_diff_hor < -90) angle_diff_hor += 180;
            if (angle_diff_hor > 90) angle_diff_hor += -180;
            yaw_correct_bias_.emplace_back(angle_diff_hor);
          }
          // 添加竖直线
        } else if (line_seg.GetAbsMinAngleDegree(HorLine) <
                   parameters.yaw_correction.angle2HorLine) {
          // 添加水平线
          // 检测到中心圆，且线片段到其距离小于30cm
          if (vision_info.see_circle &&
              DistanceFromLineSegment(line_seg, projection.RotateTowardHeading(
                                                    result_circle)) < 30 &&
              line_seg.GetLength() > parameters.yaw_correction.circleLineLen) {
            center_lines_.push_back(line_seg);
            // 增加视觉 对 陀螺仪的修正
            double angle_diff_hor = line_seg.GetExteriorAngleDegree(HorLine);
            if (angle_diff_hor < -90) angle_diff_hor += 180;
            if (angle_diff_hor > 90) angle_diff_hor += -180;
            // fmt::print("add bias: {:.2f}\n", angle_diff_hor);
            yaw_correct_bias_.emplace_back(angle_diff_hor);

            center_lines_.push_back(line_seg);

          // } else if (!goal_position.empty() &&
          //            line_seg.GetLength() >=
          //                parameters.yaw_correction.goalLineLen) {
          //   bool is_goal_line = false;
          //   // 旋转球门柱
          //   std::vector<cv::Point2f> goal_position_rotated;
          //   goal_position_rotated =
          //       projection.RotateTowardHeading(goal_position);
          //   // 其实可以写成for循环的
          //   if (goal_position_rotated.size() == 2) {
          //     is_goal_line =
          //         line_seg.DistanceFromLine(goal_position_rotated[0]) <
          //             parameters.yaw_correction.maxDistBothGoal &&
          //         line_seg.DistanceFromLine(goal_position_rotated[1]) <
          //             parameters.yaw_correction.maxDistBothGoal;
          //   }
          //   //                        else if (goal_position_rotated.size() ==
          //   //                        1) {
          //   //                            is_goal_line =
          //   //                            line_seg.DistanceFromLine(goal_position_rotated[0])
          //   //                            <
          //   //                            parameters.yaw_correction.maxDistSingleGoal;
          //   //                        }
          //   if (is_goal_line) {
          //     double angle_diff_hor = line_seg.GetExteriorAngleDegree(HorLine);
          //     if (angle_diff_hor < -90) angle_diff_hor += 180;
          //     if (angle_diff_hor > 90) angle_diff_hor += -180;
          //     // fmt::print("add bias: {:.2f}\n", angle_diff_hor);
          //     yaw_correct_bias_.emplace_back(angle_diff_hor);
          //     goal_lines_.push_back(line_seg);
          //   } else {
          //     if (line_seg.GetLength() >=
          //         parameters.yaw_correction.otherLineLen) {
          //       double angle_diff_hor =
          //           line_seg.GetExteriorAngleDegree(HorLine);
          //       if (angle_diff_hor < -90) angle_diff_hor += 180;
          //       if (angle_diff_hor > 90) angle_diff_hor += -180;
          //       yaw_correct_bias_.emplace_back(angle_diff_hor);
          //     }
          //     other_lines_.push_back(line_seg);
          //   }

          } else {
            if (line_seg.GetLength() >=
                parameters.yaw_correction.otherLineLen) {
              double angle_diff_hor = line_seg.GetExteriorAngleDegree(HorLine);
              if (angle_diff_hor < -90) angle_diff_hor += 180;
              if (angle_diff_hor > 90) angle_diff_hor += -180;
              yaw_correct_bias_.emplace_back(angle_diff_hor);
            }
            other_lines_.push_back(line_seg);
          }
          if (yaw_correct_bias_.size() >=
              static_cast<unsigned int>(
                  parameters.yaw_correction.yawCorrectNum)) {
            projection.SetHeadingOffsetBias(CalYawBias());
          }
        }
      }
      // 线片段长度大于预设值
    }

    // cv::line(m_linec_img, cv::Point(offssx, offssy), cv::Point(50 + offssx,
    // offssy), yellowColor(), 2, 8);
    // cv::line(m_linec_img, cv::Point(offssx, offssy), cv::Point(offssx, -50 +
    // offssy), redColor(), 2, 8);
    // for (size_t i = 0; i < other_lines_.size(); ++i) {
    //
    //     cv::line(m_linec_img,
    //              cv::Point(other_lines_[i].P1.x * ratioo + offssx,
    //              -other_lines_[i].P1.y * ratioo + offssy),
    //              cv::Point(other_lines_[i].P2.x * ratioo + offssx,
    //              -other_lines_[i].P2.y * ratioo + offssy),
    //              cv::Scalar(150, 145, 151),
    //              2,
    //              8);
    // }
    // for (size_t i = 0; i < center_lines.size(); ++i) {
    //
    //     cv::line(m_linec_img,
    //              cv::Point(center_lines[i].P1.x * ratioo + offssx,
    //              -center_lines[i].P1.y * ratioo + offssy),
    //              cv::Point(center_lines[i].P2.x * ratioo + offssx,
    //              -center_lines[i].P2.y * ratioo + offssy),
    //              cv::Scalar(0, 255, 255),
    //              2,
    //              8);
    // }
    //
    // for (size_t i = 0; i < goal_lines.size(); ++i) {
    //     cv::line(m_linec_img,
    //              cv::Point(goal_lines[i].P1.x * ratioo + offssx,
    //              -goal_lines[i].P1.y * ratioo + offssy),
    //              cv::Point(goal_lines[i].P2.x * ratioo + offssx,
    //              -goal_lines[i].P2.y * ratioo + offssy),
    //              cv::Scalar(0, 0, 255),
    //              2,
    //              8);
    // }
    //
    // cv::imshow("m_linec_img", m_linec_img);
  }
  ROS_DEBUG("line classifier used: %lf ms", t.elapsedMsec());
  return true;
}

double LineClassifier::CalYawBias() {
  auto bias_avg = Angle::mean(yaw_correct_bias_);

  yaw_correct_bias_.erase(
      std::remove_if(yaw_correct_bias_.begin(), yaw_correct_bias_.end(),
                     [bias_avg](Angle bias) {
                       ROS_DEBUG("[CalYawBias] bias: %lf avg: %lf dev: %lf",
                                 bias.get_signed(), bias_avg.get_signed(),
                                 (bias - bias_avg).get_signed());
                       return std::fabs((bias - bias_avg).get_signed()) > 15;
                     }),
      yaw_correct_bias_.end());

  if (!yaw_correct_bias_.empty()) {
    bias_avg = Angle::mean(yaw_correct_bias_);
    ;
  } else {
    bias_avg = Angle(0);
  }
  yaw_correct_bias_.clear();
  ROS_DEBUG("[CalYawBias] Final bias avg: %lf", bias_avg.get_signed());

  return bias_avg.get_signed();
}
}  // namespace dvision
