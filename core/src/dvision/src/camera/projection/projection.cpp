/* Copyright (C) ZJUDancer
 * 2018 - Yusu Pan <xxdsox@gmail.com>
 * 2017 - Wenxing Mei <mwx36mwx@gmail.com>
 */

/**
 * \file projection.cpp
 * \author Yusu Pan, Wenxing Mei
 * \version 2018
 * \date 2018-02-15
 */

#include "dvision/projection.hpp"

#include <fmt/format.h>

#include <numeric>

#include "dancer_geometry/angle.hpp"
#include "dvision/parameters.hpp"
#include "dvision/timer.hpp"

using namespace std;
using namespace cv;
using namespace dancer_geometry;

namespace dvision {
Projection::Projection() : heading_offset_bias_(0) {}

void Projection::init(ros::NodeHandle *nh) {
  dist_.init();
  ipm_.Init(parameters.camera.extrinsic_para, parameters.camera.fx,
            parameters.camera.fy, parameters.camera.undistCx,
            parameters.camera.undistCy);
}

Projection::~Projection() = default;

void Projection::updateExtrinsic(double pitch, double yaw) {
  if (-90 <= pitch && pitch <= 90 && -180 <= yaw && yaw <= 180) {
    Timer t;
    ipm_.updateDeg(pitch, yaw);
    ROS_DEBUG("updateExtrinsic used: %lf ms", t.elapsedMsec());
  } else {
    throw std::runtime_error("Pitch and yaw error");
  }
}

// Single point

bool Projection::getOnImageCoordinate(const cv::Point2f &point,
                                      cv::Point &resPoint) {
  cv::Point2d undist_point = ipm_.project(point.x, point.y);
  dist_.distortP(undist_point, resPoint);
  return true;
}

bool Projection::getOnRealCoordinate(const cv::Point &point,
                                     cv::Point2f &resPoint, float z_real) {
  cv::Point undist_point = dist_.undistort(point.x, point.y);
  resPoint = ipm_.inverseProject(undist_point.x, undist_point.y, z_real);
  return true;
}

// Points

bool Projection::getOnImageCoordinate(const vector<cv::Point2f> &points,
                                      vector<cv::Point> &resPoints) {
  resPoints.resize(points.size());
  for (uint32_t i = 0; i < points.size(); ++i) {
    getOnImageCoordinate(points[i], resPoints[i]);
  }
  return true;
}

bool Projection::getOnRealCoordinate(const vector<cv::Point> &points,
                                     vector<cv::Point2f> &resPoints,
                                     float z_real) {
  resPoints.resize(points.size());
  for (uint32_t i = 0; i < points.size(); ++i) {
    getOnRealCoordinate(points[i], resPoints[i], z_real);
  }
  return true;
}

// Lines
bool Projection::getOnImageCoordinate(
    const std::vector<dancer_geometry::LineSegment> &lines,
    std::vector<dancer_geometry::LineSegment> &res_lines) {
  res_lines.resize(lines.size());
  cv::Point tmp;
  for (uint32_t i = 0; i < lines.size(); ++i) {
    getOnImageCoordinate(lines[i].P1, tmp);
    res_lines[i].P1.x = tmp.x;
    res_lines[i].P1.y = tmp.y;

    getOnImageCoordinate(lines[i].P2, tmp);
    res_lines[i].P2.x = tmp.x;
    res_lines[i].P2.y = tmp.y;

    res_lines[i].SetProbability(lines[i].GetProbability());
  }
  return true;
}

bool Projection::getOnRealCoordinate(
    const std::vector<dancer_geometry::LineSegment> &lines,
    std::vector<dancer_geometry::LineSegment> &res_lines, float z_real) {
  res_lines.resize(lines.size());
  cv::Point2f tmp;
  for (uint32_t i = 0; i < lines.size(); ++i) {
    getOnRealCoordinate(lines[i].P1, tmp, z_real);
    res_lines[i].P1.x = tmp.x;
    res_lines[i].P1.y = tmp.y;

    getOnRealCoordinate(lines[i].P2, tmp, z_real);
    res_lines[i].P2.x = tmp.x;
    res_lines[i].P2.y = tmp.y;

    res_lines[i].SetProbability(lines[i].GetProbability());
  }
  return true;
}

std::vector<cv::Point2f> Projection::RotateTowardHeading(
    const std::vector<cv::Point2f> &in) {
  std::vector<cv::Point2f> out(in.size());
  for (size_t i = 0; i < in.size(); i++) {
    out[i] = RotateTowardHeading(in[i]);
  }
  return out;
}

cv::Point2d Projection::RotateTowardHeading(const cv::Point2d &in) {
  return RotateAroundPoint(in, -Radian2Degree(GetHeading()));
}

cv::Point2f Projection::RotateTowardHeading(const cv::Point2f &in) {
  return RotateAroundPoint(in, -Radian2Degree(GetHeading()));
}

dancer_geometry::LineSegment Projection::RotateTowardHeading(
   const dancer_geometry::LineSegment &in) {
  dancer_geometry::LineSegment out;
  out = LineSegment(RotateTowardHeading(in.P1), RotateTowardHeading(in.P2),
                    in.GetProbability());
  return out;
}

std::vector<dancer_geometry::LineSegment> Projection::RotateTowardHeading(
    const std::vector<dancer_geometry::LineSegment> &in) {
  std::vector<dancer_geometry::LineSegment> out(in.size());
  for (size_t i = 0; i < in.size(); i++) {
    out[i] = RotateTowardHeading(in[i]);
  }
  return out;
}
// NOTE(daiz): Calculated in line classifier now, not used
bool Projection::CalcHeadingOffset(
    std::vector<dancer_geometry::LineSegment> &result_lines,
    bool circle_detected, const cv::Point2d &result_circle,
    const std::vector<cv::Point2f> &goal_position) {
  LineSegment VerLine(cv::Point(0, -10), cv::Point(0, 10));
  std::vector<double> heading_offset_vec;
  for (auto line_seg : result_lines) {
    if (line_seg.GetLength() > parameters.yaw_correction.minLineLen) {
      double angle_diff_ver = line_seg.GetExteriorAngleDegree(VerLine);
      if (angle_diff_ver < -90) angle_diff_ver += 180;
      if (angle_diff_ver > 90) angle_diff_ver += -180;

      if (circle_detected &&
          DistanceFromLineSegment(line_seg, result_circle) < 50) {
        heading_offset_vec.push_back(angle_diff_ver);
        // cout << "CalcHeadingOffset: HorCenter angle " <<
        // heading_offset_vec.back()
        //      << endl;
      }
      if (goal_position.size() == 2 &&
          line_seg.DistanceFromLine(goal_position[0]) <
              parameters.yaw_correction.maxDistBothGoal &&
          line_seg.DistanceFromLine(goal_position[1]) <
              parameters.yaw_correction.maxDistBothGoal) {
        heading_offset_vec.push_back(angle_diff_ver);
        // cout << "CalcHeadingOffset: goal line angle " <<
        // heading_offset_vec.back()
        //      << endl;
      }
    }
  }
  if (!heading_offset_vec.empty()) {
    double heading_offset_avg =
        accumulate(heading_offset_vec.begin(), heading_offset_vec.end(), 0.0) /
        heading_offset_vec.size();
    // remove angle far from average
    int valid_counter = 0;
    double sum = 0;
    for (size_t counter = 0; counter < heading_offset_vec.size(); counter++) {
      if (std::abs(heading_offset_vec[counter] - heading_offset_avg) < 15) {
        sum += heading_offset_vec[counter];
        valid_counter++;
      }
    }
    if (valid_counter > 0) {
      heading_offset_avg = sum / valid_counter;
      // cout << "CalcHeadingOffset: " << heading_offset_avg << endl;
      heading_offset_ = M_PI / 180 * heading_offset_avg;
    }
  }
  return true;
}
bool Projection::UpdateHeadingOffset(double &in_yaw) {
  ROS_DEBUG("[YAW] raw %lf, bias %lf, corrected %lf",
            dancer_geometry::rad2deg(in_yaw),
            dancer_geometry::rad2deg(heading_offset_bias_),
            dancer_geometry::rad2deg(in_yaw + heading_offset_bias_));
  if (parameters.simulation) {
    heading_offset_bias_ = 0;
  }
  in_yaw += heading_offset_bias_;
  heading_offset_bias_ = 0;
  heading_offset_ = in_yaw;
  return true;
}
void Projection::SetHeadingOffsetBias(double bias) {
  if (parameters.simulation) {
    heading_offset_bias_ = dancer_geometry::deg2rad(bias);
  } else {
    heading_offset_bias_ += dancer_geometry::deg2rad(bias);
  }
  ROS_INFO("Vision yaw bias added %lf",
           dancer_geometry::rad2deg(heading_offset_bias_));
}

}  // namespace dvision
