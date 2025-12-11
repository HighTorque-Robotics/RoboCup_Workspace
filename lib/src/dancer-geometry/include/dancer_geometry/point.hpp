/* Copyright (C) ZJUDancer
 * 2018 - Yusu Pan <xxdsox@gmail.com>
 */

/**
 * \file point.hpp
 * \author Yusu Pan
 * \version 2018
 * \date 2018-06-07
 */

#pragma once

#include "dancer_geometry/angle.hpp"

namespace dancer_geometry {

class Point {
 public:
  /**
   * @brief x value of point
   */
  double x;
  /**
   * @brief y value of point
   */
  double y;

  /**
   * @brief Initialize with default position (0,0)
   */
  Point();
  /**
   * @brief Initialize with given point
   * @param other - given point
   */
  Point(const Point &other);
  /**
   * @brief Initialize with given x and y value
   * @param x - x value
   * @param y - y value
   */
  Point(double x, double y);

  /**
   * @brief Get theta of point to origin and x axis
   * @return theta of point
   */
  dancer_geometry::Angle theta() const;
  /**
   * @brief Get distance from self to given point
   * @param other - given point
   * @return distance from self to given point
   */
  double dist(const Point &other) const;
  /**
   * @brief Get distance from point oto origin
   * @return
   */
  double length() const;

  /**
   * @brief Normalize point length to given value
   * @param len - given length
   * @return normalized point
   */
  Point &normalize(double len = 1);

  /**
   * @brief Determine
   * @param other
   * @return
   */
  bool is_collinear(const Point &other) const;

  /**
   * @brief Get a point rotated 90 degrees
   * @return a point rotated 90 degrees
   */
  Point perpendicular() const;
  /**
   * @brief Rotate point by the center of the origin
   * @param a - given angle
   * @return rotated point
   */
  Point rotate(const dancer_geometry::Angle &a) const;
  /**
   * @brief Rotate point by the center of the origin
   * @param a - given angle
   * @param center - rotation center
   * @return rotated point
   */
  Point rotate(const dancer_geometry::Angle &a, const Point &center) const;

  /**
   * @brief Perform a dot product of two points
   * @param p1 - point 1
   * @param p2 - point 2
   * @return result of dot product
   */
  static double dot(const Point &p1, const Point &p2);
  /**
   * @brief Perfoem a perp dot product of two points
   * @param p1 - point 1
   * @param p2 - point 2
   * @return result of prep dot product
   */
  static double perp_dot(const Point &p1, const Point &p2);

  /**
   * @brief Get opposite point
   * @return opposite point
   */
  Point operator-() const;
  /**
   * @brief Plus a point
   * @param other - given point
   * @return result point
   */
  Point operator+(const Point &other) const;
  /**
   * @brief Minus a point
   * @param other - given point
   * @return result point
   */
  Point operator-(const Point &other) const;
  /**
   * @brief Multiply point with given ratio
   * @param ratio - multiply ratio
   * @return result point
   */
  Point operator*(double ratio) const;
  /**
   * @brief Divide point by given ratio
   * @param ratio - divide ratio
   * @return result point
   */
  Point operator/(double ratio) const;

  /**
   * @brief Plus a point in place
   * @param other - given point
   * @return result point
   */
  Point &operator+=(const Point &other);
  /**
   * @brief Minus a point in place
   * @param other - given point
   * @return result point
   */
  Point &operator-=(const Point &other);
  /**
   * @brief Multiply point with given ratio in place
   * @param ratio - multiply ratio
   * @return result point
   */
  Point &operator*=(double ratio);
  /**
   * @brief Divide point by given ratio in place
   * @param ratio - divide ratio
   * @return result point
   */
  Point &operator/=(double ratio);
  /**
   * @brief Determine whether self is exactly equal to given point
   * @param other - given point
   * @return equality
   */
  bool operator==(const Point &other) const;
};
}  // namespace dancer_geometry
