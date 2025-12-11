/* Copyright (C) ZJUDancer
 * 2017 - Yusu Pan <xxdsox@gmail.com>
 */

/**
 * \file line_segment.hpp
 * \author Yusu Pan
 * \version 2018
 * \date 2018-02-18
 */

#pragma once
#include <algorithm>
#include <cmath>
#include <functional>
#include <opencv2/opencv.hpp>
#include <vector>

namespace dancer_geometry {

#define DEFFLEQEPSILON 0.001

//! Class for line segment
class LineSegment {
 public:
  //! LineSegment constructor by two points
  LineSegment(const cv::Point2d p1, const cv::Point2d p2,
              double _probability = 0.0);
  //! LineSegment constructor by single point, direction and length
  LineSegment(const cv::Point2d center, double angle, double length,
              double _probability = 0.0);
  //! LineSegment constructor by another line segment
  LineSegment(const LineSegment& l);
  //! LineSegment constructor (explicit)
  LineSegment();
  //! LineSegment destructor
  ~LineSegment();

  // Setter
  //! Set probability of line segment
  void SetProbability(const double& _in);
  //! Set downside point of line segment
  void SetDownPoint(const cv::Point2f& alter_down);
  //! Set upside point of line segment
  void SetUpPoint(const cv::Point2f& alter_up);

  // Getter
  //! Get probability of line segment
  double GetProbability() const;
  //! Get length of line segment
  double GetLength() const;
  //! Get middle point of line segment
  cv::Point2f GetMiddle();
  //! Get upside point of line segment
  cv::Point2f GetUpPoint();
  //! Get downside point of line segment
  cv::Point2f GetDownPoint();

  /**
   * \brief Get the closest point of given point on line segment
   *
   * \param p - given point not in the line segment
   *
   * \return the closest point of given point on line segment
   */
  cv::Point2f GetClosestPointOnLineSegment(cv::Point2f p);
  /**
   * \brief Get direction vector of line segment, the norm of which is 1
   *
   * \param res - direction vector of line segment
   */
  void GetDirection(cv::Point2d& res) const;
  /**
   * \brief Obtain the Y value from the X value using first degree interpolation
   *
   * \param x - given x position
   *
   * \return corresponding y value of x
   */
  float GetYByX(const float& x) const;
  /**
   * \brief Determin which side of the line the 2D point is at
   *
   * - 1 if on the right hand side
   * - 0 if on the line
   * - -1 if on the left hand side
   *
   * \param point - given point
   *
   * \return which side of the line the 2D point is at
   */
  int GetSide(const cv::Point2d& point) const;
  /**
   * \brief Get the exterior angle between this line and otherLine
   *
   * Result is in [-180,180], the order is important for the result sign
   *
   * \param otherLine - given line
   *
   * \return the exterior angle between two lines
   */
  double GetExteriorAngleDegree(const LineSegment& otherLine) const;
  /**
   * \brief Get the absolute minimum angle between this line and otherLine
   *
   * Result is in [0,90]
   *
   * \param otherLine - given line
   *
   * \return the absolute minimum angle between two lines
   */
  double GetAbsMinAngleDegree(const LineSegment& otherLine) const;
  /**
   * \brief Get slope of lien segment
   *
   * \param slope - calculated slope
   *
   * \return whether or not slope is valid
   */
  bool GetSlope(double& slope);
  /**
   * \brief Get angle between line segment and x axis in radian
   *
   * \return angle in radian
   */
  double GetRadianFromX();
  /**
   * \brief Get angle between line segment and x axis in degree
   *
   * \return angle in degree
   */
  double GetDegreeFromX();
  /**
   * \brief Get set of line segments divided by 2^n middle points
   *
   * \param count - n for middle points (2^n)
   *
   * \return set of line segments divided by 2^n middle points
   */
  std::vector<LineSegment> GetMidLineSegments(const int& count);
  /**
   * \brief Get 2^n middle points of line segment
   *
   * \param count - n for middle points (2^n)
   * \param sortthis - whether or not sort middle points
   *
   * \return 2^n middle points of line segment
   */
  std::vector<cv::Point2d> GetMidPoints(const int& count,
                                        const bool& sortthis = true);

  // Operation
  /**
   * \brief Calculate distance between given point and line segment
   *
   * \param p - given point
   *
   * \return distance between given point and line segment
   */
  float DistanceFromLine(const cv::Point2f& p);
  /**
   * \brief Get intersection of two line segment
   *
   * \param L - given line segment
   * \param res - reuslted intersect point
   *
   * \return whether or not intersect point is existed
   */
  bool Intersect(LineSegment L, cv::Point2d& res);
  /**
   * \brief Get intersection of this line segment and a line
   *
   * \param L - given line segment
   * \param res - reuslted intersect point
   *
   * \return whether or not intersect point is existed
   */
  bool IntersectLineForm(LineSegment L, cv::Point2d& res);
  /**
   * \brief Get perpendicular line segment in middle point of this line segment
   *
   * \param scale - length scale factor of perpendicular line segment
   *
   * \return perpendicular line segment
   */
  LineSegment PerpendicularLineSegment(const double& scale = 1);
  /**
   * \brief Get perpendicular line segment in given point of this line segment
   *
   * \param len - length of perpendicular line segment
   * \param mid - given point
   *
   * \return perpendicular line segment
   */
  LineSegment PerpendicularLineSegment(const double& len,
                                       const cv::Point2d& mid);
  /**
   * \brief Extend downside point
   *
   * \param len - given length to extend
   *
   * \return extended downside point
   */
  cv::Point2d ExtensionPointDown(const double& len);
  /**
   * \brief Extend upside point
   *
   * \param len - given length to extend
   *
   * \return extended upside point
   */
  cv::Point2d ExtensionPointUp(const double& len);
  /**
   * \brief Extend downside point and get extended line segment
   *
   * \param len - given length to extend
   *
   * \return extended line segment (from original downside point to extended
   * point)
   */
  LineSegment ExtensionCordDown(const double& len);
  /**
   * \brief Extend upside point and get extended line segment
   *
   * \param len - given length to extend
   *
   * \return extended line segment (from original upside point to extended
   * point)
   */
  LineSegment ExtensionCordUp(const double& len);
  /**
   * \brief Scale line segment at middle point
   *
   * \param _in - scale factor
   *
   * \return scaled line segment
   */
  LineSegment Scale(const double& _in);
  /**
   * \brief Determine which point is closer to the line segment
   *
   * \param a - given point A
   * \param b - given point B
   *
   * \return whether or not point A is closer to the line segment
   */
  bool SortbyDistance(const cv::Point2d& a, const cv::Point2d& b);
  /**
   * \brief Determine whether or not given point is on the line segment
   *
   * \param ptTest - given point to test
   * \param flEp - float epsilon for veritical linek condition
   *
   * \return whether or not given point is on the line segment
   */
  bool IsOnThis(const cv::Point2f& ptTest, float flEp = DEFFLEQEPSILON);
  /**
   * \brief Clip line segment
   *
   * \param boundry - boundary rectangle
   */
  void Clip(cv::Rect boundry);

  bool checkLength(float std_len, float err = 0.05);

  // TODO(corenel) make m_P1, m_P2 private
  cv::Point2d P1, P2;

 private:
  //! Probability of line segment
  double probability_;
  /**
   * \brief Determine whether or not given float value is within an interval
   *
   * \param fl - given float value
   * \param flLow - the upper bound of interval
   * \param flHi - the upper bound of interval
   * \param flEp - epsilon of float
   *
   * \return whether or not given float value is within an interval
   */
  bool Within(const float& fl, const float& flLow, const float& flHi,
              const float& flEp = DEFFLEQEPSILON);
};

//! class for linear interpolator of line segment
class LinearInterpolator {
 public:
  //! LinearInterpolator constructor (explicit)
  explicit LinearInterpolator(LineSegment line);
  /**
   * \brief LinearInterpolator constructor (w/ two endpoints)
   *
   * \param p1 - point 1 of line segment
   * \param p2 - point 2 of line segment
   */
  LinearInterpolator(const cv::Point2d& p1, const cv::Point2d& p2);
  //! LinearInterpolator destructor
  ~LinearInterpolator();
  /**
   * \brief Get y value of line by linear interpolation
   *
   * \param x - given x value
   *
   * \return interpolated y value
   */
  double Interpolate(const double& x);

 private:
  //! Line segment to interpolate
  LineSegment line_;
};

//! class for linear boundary checker (experimental)
class LinearBoundaryChecker {
 public:
  /**
   * \brief LinearBoundaryChecker constructor with two boundary line segments
   *
   * \param _LLow - lower bound line segment
   * \param _LHigh - higher bound line segment
   */
  LinearBoundaryChecker(const LineSegment& _LLow, const LineSegment& _LHigh);
  /**
   * \brief LinearBoundaryChecker constructor
   *
   * \param nearDistance - length of near boundary
   * \param nearMin - minimum point for near boundary
   * \param nearMax - maximum point for near boundary
   * \param farDistance - length of far boundary
   * \param farMin - minimum point for far boundary
   * \param farMax - maximum point for far boundary
   */
  LinearBoundaryChecker(const double& nearDistance, const double& nearMin,
                        const double& nearMax, const double& farDistance,
                        const double& farMin, const double& farMax);
  //! LinearBoundaryChecker destructor
  ~LinearBoundaryChecker();

  /**
   * \brief check validity of boundary
   *
   * \return validity of boundary
   */
  bool checkValidity();
  /**
   * \brief Check extrapolation
   *
   * \param distance - distance to check
   * \param value - value at given distance
   *
   * \return whether or not extrapolation is valid
   */
  bool CheckExtrapolation(const double& distance, const double& value);
  /**
   * \brief Get extrapolation
   *
   * \param distance - distance to check
   * \param min - minimum value at given distance
   * \param max - maximum value at given distance
   *
   * \return whether or not given distance is valid and has boundary
   */
  bool GetExtrapolation(const double& distance, double& min, double& max);
  /**
   * \brief Check given distance and vlaue is inside boundary
   *
   * \param distance - distance to check
   * \param value - value to check
   *
   * \return whether or not given distance and value is in the boundary
   */
  bool CheckInside(const double& distance, const double& value);

 private:
  //! Lower bound line segment of boundary
  LineSegment LLow;
  //! Higher bound line segment of boundary
  LineSegment LHigh;
};

}  // namespace dancer_geometry
