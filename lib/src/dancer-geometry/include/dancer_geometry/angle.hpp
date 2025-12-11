/* Copyright (C) ZJUDancer
 * 2018 - Yusu Pan <xxdsox@gmail.com>
 */

/**
 * \file angle.hpp
 * \author Yusu Pan
 * \version 2018
 * \date 2018-06-07
 */

#pragma once

#include <ostream>
#include <vector>

namespace dancer_geometry {

class Angle {
 protected:
  double value;

 public:
  /**
   * @brief Initialization with defualt value 0.0;
   */
  Angle();
  /**
   * @brief Initialization with given degree angle value normalized to
   * [-180,180]
   * @param degree - given degree angle
   */
  Angle(double degree);
  /**
   * @brief Initialization with another Angle instance
   * @param other - given Angle instance
   */
  Angle(const Angle &other);

  /**
   * @brief Get signed value in [-180, 180]
   * @return signed angle value in degree
   */
  inline double get_signed() const { return value; };

  /**
   * @brief Get unsigned value in [0, 360]
   * @return unsigned angle value in degree
   */
  inline double get_unsigned() const { return value + 180; };

  /**
   * @brief Get Angle from a vector
   * reference to
   * http://en.wikipedia.org/wiki/Directional_statistics#The_fundamental_difference_between_linear_and_circular_statistics
   * @param x - x position of vector
   * @param y - y position of vector
   * @return Angle initialized from a vector
   */
  static Angle from_xy(double x, double y);
  /**
   * @brief Compute mean and std value of angles
   * @param angles - given angles
   * @param std - std value
   * @return mean value of angles
   */
  static Angle mean(const std::vector<Angle> &angles, double *std);
  /**
   * @brief Get mean value of angles
   * @param angles - given angles
   * @return mean value of angles
   */
  static Angle mean(const std::vector<Angle> &angles);
  /**
   * @brief Get std value of angles
   * @param angles - given angles
   * @return std value of angles
   */
  static double std(const std::vector<Angle> &angles);

  /**
   * @brief Determine whether self value is roughly equal to given angle
   * @param other - given angle
   * @return whether self value is equal to given angle
   */
  bool is_equal(const Angle &other) const;

  /**
   * @brief Compute arcsin of given number
   * @param x - given number
   * @return arcsin of given number
   */
  static Angle asin(double x);

  /**
   * @brief Compute arccos of given number
   * @param x - given number
   * @return arccos of given number
   */
  static Angle acos(double x);
  /**
   * @brief Compute weighted angle
   * @param a1 - angle 1
   * @param w1 - weight for angle 1
   * @param a2 - angle 2
   * @param w2 - weight for angle 2
   * @return weighted angle
   */
  static Angle weighted_average(const Angle &a1, double w1, const Angle &a2,
                                double w2);

  /**
   * @brief Determine whether self value is exactly equal to given angle
   * @param a - given angle
   * @return whether self value is exactly equal to given angle
   */
  bool operator==(const Angle &a) const;
  /**
   * @brief Minus given angle
   * @param a - given angle
   * @return angle result
   */
  Angle operator-(const Angle &a) const;
  /**
   * @brief Plus given angle
   * @param a - given angle
   * @return angle result
   */
  Angle operator+(const Angle &a) const;
};

/**
 * @brief Get opposite angle
 * @param a - given angle
 * @return opposite angle
 */
Angle operator-(const Angle &a);
/**
 * @brief Multiply angle by number x
 * @param x - given number
 * @param a - given angle
 * @return result angle
 */
Angle operator*(double x, const Angle &a);
/**
 * @brief Multiply angle by number x
 * @param a - given angle
 * @param x - given number
 * @return result angle
 */
Angle operator*(const Angle &a, double x);
/**
 * @brief Divide angle value by number x
 * @param a - given angle
 * @param x - given number
 * @return result angle
 */
Angle operator/(const Angle &a, double x);

/**
 * @brief Feed value to out stream
 * @param out - out stream
 * @param a - given value
 * @return out stream
 */
std::ostream &operator<<(std::ostream &out, const Angle &a);

/**
 * @brief Wrapper of sin function for Angle
 * @param a - given angle
 * @return result value
 */
double sin(const Angle &a);
/**
 * @brief Wrapper of cos function for Angle
 * @param a - given angle
 * @return result value
 */
double cos(const Angle &a);
/**
 * @brief Wrapper of tan function for Angle
 * @param a - given angle
 * @return result value
 */
double tan(const Angle &a);

/**
 * @brief Convert angle value from degree to radian
 * @param radian - given angle in degree
 * @return angle in radian
 */
double deg2rad(double degree);
/**
 * @brief Convert angle value from radian to degree
 * @param radian - given angle in radian
 * @return angle in degree
 */
double rad2deg(double radian);

/**
 * @brief Get the normalized angle in [-PI,PI]
 * @param radian - given angle in radian
 * @return the normalized angle
 */
double normalize_rad(double radian);

}  // namespace dancer_geometry
