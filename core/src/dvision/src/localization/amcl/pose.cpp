/* Copyright (C) ZJUDancer
 * 2018 - Yusu Pan <xxdsox@gmail.com>
 * 2017 - Wenxing Mei <mwx36mwx@gmail.com>
 */

/**
 * @file pose.cpp
 * @author Yusu Pan, Wenxing Mei
 * @version 2018
 * @date 2018-02-23
 */

#include "dvision/amcl/pose.hpp"
#include <cmath>
#include "dancer_geometry/utils.hpp"

using namespace std;
using namespace dancer_geometry;

namespace dvision {
Pose::Pose() : x_(0), y_(0), heading_(0) {}

Pose::Pose(double x, double y, double heading)
    : x_(x), y_(y), heading_(heading) {}

Pose::Pose(const Pose& other)
    : x_(other.x()), y_(other.y()), heading_(other.heading()) {}

bool Pose::operator==(const Pose& oth) const {
  if (x_ != oth.x()) return false;

  if (y_ != oth.y()) return false;

  return heading_ == oth.heading();
}

bool Pose::operator<(const Pose& oth) const { return x_ < oth.x(); }

double& Pose::operator[](int index) {
  assert(index < 2);
  if (index == 0) return x_;
  if (index == 1) return y_;
  // FIXME(MWX): ..
  return y_;
}

double Pose::x() const { return x_; }

double Pose::y() const { return y_; }

double Pose::heading() const { return heading_; }

double Pose::headingR() const { return Degree2Radian(heading_); }

double Pose::length() const { return sqrt(x_ * x_ + y_ * y_); }

void Pose::setX(double x) { x_ = x; }

void Pose::setY(double y) { y_ = y; }

void Pose::setHeading(double h) { heading_ = h; }

void Pose::setHeadingR(double h) { heading_ = h / M_PI * 180.0; }

void Pose::rotate(double t) {
  double s = sin(t * 180.f / M_PI);
  double c = cos(t * 180.f / M_PI);

  double tmpx = c * x_ - s * y_;
  double tmpy = s * x_ + c * y_;

  x_ = tmpx;
  y_ = tmpy;
}
}
