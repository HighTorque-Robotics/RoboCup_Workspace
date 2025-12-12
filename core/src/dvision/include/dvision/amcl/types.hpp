/* Copyright (C) ZJUDancer
 * 2018 - Yusu Pan <xxdsox@gmail.com>
 * 2017 - Wenxing Mei <mwx36mwx@gmail.com>
 */

/**
 * @file types.cpp
 * @author Yusu Pan, Wenxing Mei
 * @version 2018
 * @date 2018-02-23
 */

#pragma once
#include <algorithm>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <vector>

#include "dancer_geometry/line_segment.hpp"
#include "dconfig/dconstant.hpp"

namespace dvision {
struct Control {
  Control() : dx(0.f), dy(0.f), dt(0.f) {}
  double dx;
  double dy;
  double dt;
};

struct Measurement {
  Measurement() {}
  struct LandMark {
    // Mark position on robot coord.
    cv::Point2f pred;
    // Possible global position on global coord.
    std::vector<cv::Point2f> ref;
    float weight = 0;
    int type;
  };

  static float getWeight(float basic_w, int dist, int trust_dist, int max_dist) {
    if (dist < trust_dist) return basic_w;
    float ratio = 1 - float(dist - trust_dist) / (max_dist - trust_dist);
    if (ratio < 0)
      return 0.;
    else
      return basic_w * ratio;
  }

  std::vector<LandMark> goal_center;
  std::vector<LandMark> center_point;
  std::vector<LandMark> corner_points;
  std::vector<LandMark> field_points;
};

struct Matrix {
  Matrix() {
    for (int i = 0; i < 3; ++i)
      for (int j = 0; j < 3; ++j) m[i][j] = 0.0;
  }
  double m[3][3];
};

}  // namespace dvision
