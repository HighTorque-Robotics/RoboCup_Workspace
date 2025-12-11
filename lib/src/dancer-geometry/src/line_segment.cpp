/* Copyright (C) ZJUDancer
 * 2017 - Yusu Pan <xxdsox@gmail.com>
 */

/**
 * \file line_segment.cpp
 * \author Yusu Pan
 * \version 2018
 * \date 2018-02-18
 */

#include "dancer_geometry/line_segment.hpp"

#include <iostream>

namespace dancer_geometry {
#define FLOAT_EQE(x, v, e) ((((v) - (e)) < (x)) && ((x) < ((v) + (e))))

LineSegment::LineSegment(const cv::Point2d p1, const cv::Point2d p2,
                         double _probability)
    : P1(p1), P2(p2) {
  SetProbability(_probability);
}

LineSegment::LineSegment(const cv::Point2d center, double angle, double length,
                         double _probability) {
  double len2 = length / 2.;
  double x1 = center.x + std::cos(angle) * len2;
  double y1 = center.y + std::sin(angle) * len2;
  double x2 = center.x - std::cos(angle) * len2;
  double y2 = center.y - std::sin(angle) * len2;

  P1 = cv::Point2d(x1, y1);
  P2 = cv::Point2d(x2, y2);
  SetProbability(_probability);
}

LineSegment::LineSegment(const LineSegment& l)
    : P1(l.P1), P2(l.P2), probability_(l.GetProbability()) {}

LineSegment::LineSegment()
    : P1(cv::Point2f(0, 0)), P2(cv::Point2f(0, 0)), probability_(0) {}

LineSegment::~LineSegment() {}

// Setter
void LineSegment::SetProbability(const double& _in) {
  probability_ = std::max(0.0, std::min(_in, 1.0));
}

void LineSegment::SetDownPoint(const cv::Point2f& alter_down) {
  if (P1.y >= P2.y) {
    P1 = alter_down;
  } else {
    P2 = alter_down;
  }
}

void LineSegment::SetUpPoint(const cv::Point2f& alter_up) {
  if (P1.y <= P2.y) {
    P1 = alter_up;
  } else {
    P2 = alter_up;
  }
}

// Getter
double LineSegment::GetProbability() const { return probability_; }

double LineSegment::GetLength() const {
  float num1 = P1.x - P2.x;
  float num2 = P1.y - P2.y;
  return std::sqrt(static_cast<double>(num1) * static_cast<double>(num1) +
                   static_cast<double>(num2) * static_cast<double>(num2));
}

// Copied From
// http://stackoverflow.com/questions/1811549/perpendicular-on-a-line-from-a-given-point
cv::Point2f LineSegment::GetClosestPointOnLineSegment(cv::Point2f p) {
  // first convert line to normalized unit std::vector
  double dx = P2.x - P1.x;
  double dy = P2.y - P1.y;
  double mag = std::sqrt(dx * dx + dy * dy);
  dx /= mag;
  dy /= mag;

  // translate the point and get the dot product
  double lambda = (dx * (p.x - P1.x)) + (dy * (p.y - P1.y));
  return cv::Point2f((dx * lambda) + P1.x, (dy * lambda) + P1.y);
}

cv::Point2f LineSegment::GetMiddle() {
  return cv::Point2f((P1.x + P2.x) / 2., (P1.y + P2.y) / 2.);
}

cv::Point2f LineSegment::GetUpPoint() { return (P1.y > P2.y) ? P2 : P1; }

cv::Point2f LineSegment::GetDownPoint() { return (P1.y < P2.y) ? P2 : P1; }

// The direction of the line, the norm of which is 1
void LineSegment::GetDirection(cv::Point2d& res) const {
  float num1 = P2.x - P1.x;
  float num2 = P2.y - P1.y;
  float num3 = static_cast<float>(
      std::sqrt(static_cast<double>(num1) * static_cast<double>(num1) +
                static_cast<double>(num2) * static_cast<double>(num2)));
  res = cv::Point2d(num1 / num3, num2 / num3);
}

// Obtain the Y value from the X value using first degree interpolation
float LineSegment::GetYByX(const float& x) const {
  cv::Point2d pointF = P1;
  cv::Point2d direction;
  GetDirection(direction);
  return (x - pointF.x) / direction.x * direction.y + pointF.y;
}

// Determin which side of the line the 2D point is at
// 1 if on the right hand side;
//         0 if on the line;
//        -1 if on the left hand side;
int LineSegment::GetSide(const cv::Point2d& point) const {
  double x0 = static_cast<double>(point.x);
  double y0 = static_cast<double>(point.y);
  double x1 = static_cast<double>(P1.x);
  double y1 = static_cast<double>(P1.y);
  double x2 = static_cast<double>(P2.x);
  double y2 = static_cast<double>(P2.y);
  float num =
      static_cast<double>((x2 - x1) * (y0 - y1) - (x0 - x1) * (y2 - y1));
  if (num > 0.0) return 1;
  return num >= 0.0 ? 0 : -1;
}

// Get the exterior angle between this line and otherLine
// Result is betwenn -180 ~ 180
// the order is important for the result sign
double LineSegment::GetExteriorAngleDegree(const LineSegment& otherLine) const {
  cv::Point2d direction1;
  this->GetDirection(direction1);
  cv::Point2d direction2;
  otherLine.GetDirection(direction2);
  double num = (std::atan2(static_cast<double>(direction2.y),
                           static_cast<double>(direction2.x)) -
                std::atan2(static_cast<double>(direction1.y),
                           static_cast<double>(direction1.x))) *
               57.2957795130823;
  if (num <= -180.0) return num + 360.0;
  if (num <= 180.0) return num;
  return num - 360.0;
}

// Result is between 0 ~ 90
double LineSegment::GetAbsMinAngleDegree(const LineSegment& otherLine) const {
  double diffAngle = GetExteriorAngleDegree(otherLine);
  double res = std::min(std::abs(diffAngle), 180 - std::abs(diffAngle));
  if (res < 0 || res > 90) {
    std::cerr << "GetAbsMinAngleDegree Error" << std::endl;
  }
  return res;
}

bool LineSegment::GetSlope(double& slope) {
  if (std::abs(P2.x - P1.x) < 0.00001) {
    return false;
  }
  slope = (P2.y - P1.y) / (P2.x - P1.x);
  return true;
}

double LineSegment::GetRadianFromX() {
  double res = std::atan2((P2.y - P1.y), (P2.x - P1.x));
  return res;
}

double LineSegment::GetDegreeFromX() {
  return (GetRadianFromX() / M_PI) * (180);
}

std::vector<LineSegment> LineSegment::GetMidLineSegments(const int& count) {
  // Means that lines count will be 2^count
  std::vector<LineSegment> lsList, lsListTmp;
  lsList.push_back(LineSegment(P1, P2));

  for (int counter = 0; counter < count; counter++) {
    lsListTmp.clear();
    for (size_t i = 0; i < lsList.size(); i++) {
      LineSegment tmp = lsList[i];
      cv::Point2d midPoint = tmp.GetMiddle();
      lsListTmp.push_back(LineSegment(tmp.P1, midPoint));
      lsListTmp.push_back(LineSegment(tmp.P2, midPoint));
    }
    lsList = lsListTmp;
  }
  return lsList;
}

std::vector<cv::Point2d> LineSegment::GetMidPoints(const int& count,
                                                   const bool& sortthis) {
  std::vector<LineSegment> lsList, lsListTmp;
  lsList.push_back(LineSegment(P1, P2));
  std::vector<cv::Point2d> res;
  res.push_back(P1);
  res.push_back(P2);
  for (int counter = 0; counter < count; counter++) {
    lsListTmp.clear();
    for (size_t i = 0; i < lsList.size(); i++) {
      LineSegment tmp = lsList[i];
      cv::Point2d midPoint = tmp.GetMiddle();
      res.push_back(midPoint);
      lsListTmp.push_back(LineSegment(tmp.P1, midPoint));
      lsListTmp.push_back(LineSegment(tmp.P2, midPoint));
    }
    lsList = lsListTmp;
  }

  if (sortthis) {
    sort(res.begin(), res.end(),
         std::bind(&LineSegment::SortbyDistance, this, std::placeholders::_1,
                   std::placeholders::_2));
  }

  return res;
}

// Operation
float LineSegment::DistanceFromLine(const cv::Point2f& p) {
  double x1 = P1.x;
  double y1 = P1.y;
  double x2 = P2.x;
  double y2 = P2.y;

  double dx = x2 - x1;
  double const Epsilon = 0.0001;
  if (std::abs(dx) < Epsilon) {
    return std::abs(p.x - x1);
  }

  double m = (y2 - y1) / (x2 - x1);
  double b = -m * x1 + y1;
  // y=mx+b

  double dist =
      static_cast<double>(std::abs(p.y - m * p.x - b) / std::sqrt(m * m + 1));
  return dist;
}

// Copy from:
// http://stackoverflow.com/questions/563198/how-do-you-detect-where-two-line-segments-intersect
bool LineSegment::Intersect(LineSegment L, cv::Point2d& res) {
  float s02_x, s02_y, s10_x, s10_y, s32_x, s32_y, s_numer, t_numer, denom, t;
  s10_x = P2.x - P1.x;
  s10_y = P2.y - P1.y;
  s32_x = L.P2.x - L.P1.x;
  s32_y = L.P2.y - L.P1.y;

  denom = s10_x * s32_y - s32_x * s10_y;
  if (static_cast<int>(denom) == 0) {
    // Collinear
    return false;
  }
  bool denomPositive = denom > 0;

  s02_x = P1.x - L.P1.x;
  s02_y = P1.y - L.P1.y;
  s_numer = s10_x * s02_y - s10_y * s02_x;
  if ((s_numer < 0) == denomPositive) {
    // No collision
    return false;
  }

  t_numer = s32_x * s02_y - s32_y * s02_x;
  if ((t_numer < 0) == denomPositive) {
    // No collision
    return false;
  }

  if (((s_numer > denom) == denomPositive) ||
      ((t_numer > denom) == denomPositive)) {
    // No collision
    return false;
  }

  // Collision detected
  t = t_numer / denom;
  res.x = P1.x + (t * s10_x);
  res.y = P1.y + (t * s10_y);
  return true;
}

bool LineSegment::IntersectLineForm(LineSegment L, cv::Point2d& res) {
  cv::Point2d x = L.P1 - P1;
  cv::Point2d d1 = P2 - P1;
  cv::Point2d d2 = L.P2 - L.P1;

  double cross = d1.x * d2.y - d1.y * d2.x;
  if (std::abs(cross) < 1e-8) {
    // Lines are parallel
    return false;
  }

  double t1 = (x.x * d2.y - x.y * d2.x) / cross;
  res = P1 + (d1 * t1);
  return true;
}

LineSegment LineSegment::PerpendicularLineSegment(const double& scale) {
  double angle = GetRadianFromX();
  angle += M_PI / 2;
  cv::Point2d mid = GetMiddle();
  double len = GetLength() / 2;
  len *= scale;
  double x1 = mid.x + std::cos(angle) * len;
  double y1 = mid.y + std::sin(angle) * len;
  double x2 = mid.x - std::cos(angle) * len;
  double y2 = mid.y - std::sin(angle) * len;
  return LineSegment(cv::Point2d(x1, y1), cv::Point2d(x2, y2));
}

LineSegment LineSegment::PerpendicularLineSegment(const double& len,
                                                  const cv::Point2d& mid) {
  double angle = GetRadianFromX();
  angle += M_PI / 2;
  double x1 = mid.x + std::cos(angle) * len;
  double y1 = mid.y + std::sin(angle) * len;
  double x2 = mid.x - std::cos(angle) * len;
  double y2 = mid.y - std::sin(angle) * len;
  return LineSegment(cv::Point2d(x1, y1), cv::Point2d(x2, y2));
}

cv::Point2d LineSegment::ExtensionPointDown(const double& len) {
  cv::Point down = GetDownPoint();
  double angle = std::abs(GetRadianFromX());
  if (angle > M_PI / 36) {
    // angle = M_PI / 36;
    angle = 0;
  }
  double x1 = down.x + std::sin(angle) * len;
  double y1 = down.y + std::cos(angle) * len;
  return cv::Point2d(x1, y1);
}

cv::Point2d LineSegment::ExtensionPointUp(const double& len) {
  cv::Point up = GetUpPoint();
  double angle = std::abs(GetRadianFromX());
  if (angle > M_PI / 36) {
    // angle = M_PI / 36;
    angle = 0;
  }
  double x1 = up.x - std::sin(angle) * len;
  double y1 = up.y - std::cos(angle) * len;
  return cv::Point2d(x1, y1);
}

LineSegment LineSegment::ExtensionCordDown(const double& len) {
  cv::Point down = GetDownPoint();
  return LineSegment(down, ExtensionPointDown(len));
}

LineSegment LineSegment::ExtensionCordUp(const double& len) {
  cv::Point up = GetUpPoint();
  return LineSegment(up, ExtensionPointUp(len));
}

LineSegment LineSegment::Scale(const double& _in) {
  double angle = GetRadianFromX();
  cv::Point2d mid = GetMiddle();
  double len = GetLength() / 2;
  len *= _in;
  double x2 = mid.x + std::cos(angle) * len;
  double y2 = mid.y + std::sin(angle) * len;
  double x1 = mid.x - std::cos(angle) * len;
  double y1 = mid.y - std::sin(angle) * len;
  return LineSegment(cv::Point2d(x1, y1), cv::Point2d(x2, y2));
}

bool LineSegment::SortbyDistance(const cv::Point2d& a, const cv::Point2d& b) {
  double x = std::abs(P1.x - a.x);
  double y = std::abs(P1.y - a.y);
  double distanceA = std::sqrt(x * x + y * y);

  x = std::abs(P1.x - b.x);
  y = std::abs(P1.y - b.y);
  double distanceB = std::sqrt(x * x + y * y);

  return distanceA < distanceB;
}

bool LineSegment::IsOnThis(const cv::Point2f& ptTest, float flEp) {
  bool bTestX = true;
  const float flX = P2.x - P1.x;
  if (FLOAT_EQE(flX, 0.0f, flEp)) {
    // vertical line -- ptTest.X must equal ptL1.X to continue
    if (!FLOAT_EQE(ptTest.x, P1.x, flEp)) {
      return false;
    }
    bTestX = false;
  }
  bool bTestY = true;
  const float flY = P2.y - P1.y;
  if (FLOAT_EQE(flY, 0.0f, flEp)) {
    // horizontal line -- ptTest.Y must equal ptL1.Y to continue
    if (!FLOAT_EQE(ptTest.y, P1.y, flEp)) {
      return false;
    }
    bTestY = false;
  }
  // found here: http://stackoverflow.com/a/7050309
  // x = x1 + (x2 - x1) * p
  // y = y1 + (y2 - y1) * p
  // solve for p:
  const float pX = bTestX ? ((ptTest.x - P1.x) / flX) : 0.5f;
  const float pY = bTestY ? ((ptTest.y - P1.y) / flY) : 0.5f;
  return Within(pX, 0.0f, 1.0f, flEp) && Within(pY, 0.0f, 1.0f, flEp);
}

void LineSegment::Clip(cv::Rect boundry) {
  double minXPossible = boundry.x;
  double minYPossible = boundry.y;
  double maxXPossible = boundry.x + boundry.width - 1;
  double maxYPossible = boundry.y + boundry.height - 1;

  P1.x = std::min(maxXPossible, std::max(minXPossible, P1.x));
  P1.y = std::min(maxYPossible, std::max(minYPossible, P1.y));

  P2.x = std::min(maxXPossible, std::max(minXPossible, P2.x));
  P2.y = std::min(maxYPossible, std::max(minYPossible, P2.y));
}

bool LineSegment::Within(const float& fl, const float& flLow, const float& flHi,
                         const float& flEp) {
  if ((fl > flLow) && (fl < flHi)) {
    return true;
  }
  if (FLOAT_EQE(fl, flLow, flEp) || FLOAT_EQE(fl, flHi, flEp)) {
    return true;
  }
  return false;
}

bool LineSegment::checkLength(float std_len, float err) {
  float min_len = std_len * (1 - err), max_len = std_len * (1 + err);
  float len = GetLength();
  if (len < max_len && len > min_len)
    return true;
  else
    return false;
}

LinearInterpolator::LinearInterpolator(LineSegment line) : line_(line) {}

LinearInterpolator::LinearInterpolator(const cv::Point2d& p1,
                                       const cv::Point2d& p2) {
  line_.P1 = p1;
  line_.P2 = p2;
}

double LinearInterpolator::Interpolate(const double& x) {
  return (line_.P1.y + (line_.P2.y - line_.P1.y) * (x - line_.P1.x) /
                           (line_.P2.x - line_.P1.x));
}

LinearInterpolator::~LinearInterpolator() {}

bool LinearBoundaryChecker::checkValidity() {
  cv::Point2d tmp;
  return !LLow.Intersect(LHigh, tmp);
}

LinearBoundaryChecker::LinearBoundaryChecker(const LineSegment& _LLow,
                                             const LineSegment& _LHigh)
    : LLow(_LLow), LHigh(_LHigh) {
  if (!checkValidity()) {
    // HAF_ERROR_THROTTLE(1, "Conflict in input for LinearBoundaryChecker");
  }
}

LinearBoundaryChecker::LinearBoundaryChecker(
    const double& nearDistance, const double& nearMin, const double& nearMax,
    const double& farDistance, const double& farMin, const double& farMax) {
  LLow.P1 = cv::Point2d(nearDistance, nearMin);
  LLow.P2 = cv::Point2d(farDistance, farMin);

  LHigh.P1 = cv::Point2d(nearDistance, nearMax);
  LHigh.P2 = cv::Point2d(farDistance, farMax);
  if (!checkValidity()) {
    // HAF_ERROR_THROTTLE(1, "Conflict in input for LinearBoundaryChecker");
  }
}

bool LinearBoundaryChecker::CheckExtrapolation(const double& distance,
                                               const double& value) {
  LineSegment verLine(cv::Point2d(distance, -10), cv::Point2d(distance, 10));
  cv::Point2d resLowerBand, resUpperBand;
  if (LLow.IntersectLineForm(verLine, resLowerBand) &&
      LHigh.IntersectLineForm(verLine, resUpperBand)) {
    if (value >= resLowerBand.y && value <= resUpperBand.y) {
      return true;
    }
  } else {
    std::cerr << "CheckExtrapolation Error: distance " << distance << std::endl;
  }

  return false;
}

bool LinearBoundaryChecker::GetExtrapolation(const double& distance,
                                             double& min, double& max) {
  LineSegment verLine(cv::Point2d(distance, -10), cv::Point2d(distance, 10));
  cv::Point2d resLowerBand, resUpperBand;

  if (LLow.IntersectLineForm(verLine, resLowerBand) &&
      LHigh.IntersectLineForm(verLine, resUpperBand)) {
    if (resUpperBand.y > resLowerBand.y) {
      min = resLowerBand.y;
      max = resUpperBand.y;
      return true;
    }
  } else {
    std::cerr << "GetExtrapolation Error: distance " << distance << std::endl;
  }
  return false;
}

bool LinearBoundaryChecker::CheckInside(const double& distance,
                                        const double& value) {
  double max_y = std::max(
      LLow.P1.y, std::max(LLow.P2.y, std::max(LHigh.P1.y, LHigh.P2.y)));
  LineSegment verLine(cv::Point2d(distance, 0), cv::Point2d(distance, max_y));
  cv::Point2d resLowerBand, resUpperBand;
  // cout << "distance: " << distance << endl;
  // cout << "value: " << value << endl;
  if (LLow.Intersect(verLine, resLowerBand) &&
      LHigh.Intersect(verLine, resUpperBand)) {
    // cout << "resLowerBand: " << resLowerBand << endl;
    // cout << "resUpperBand: " << resUpperBand << endl;
    if (value >= resLowerBand.y && value <= resUpperBand.y) {
      return true;
    }
  }
  return false;
}

LinearBoundaryChecker::~LinearBoundaryChecker() {}

}  // namespace dancer_geometry
