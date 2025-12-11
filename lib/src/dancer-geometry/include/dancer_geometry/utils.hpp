/* Copyright (C) ZJUDancer
 * 2018 - Yusu Pan <xxdsox@gmail.com>
 * 2017 - Wenxing Mei <mwx36mwx@gmail.com>
 */

/**
 * @file utils.hpp
 * @author Yusu Pan, Wenxing Mei
 * @version 2018
 * @date 2018-02-24
 */

#pragma once
#include <geometry_msgs/Vector3.h>
#include <ros/ros.h>
#include <algorithm>
#include <opencv2/opencv.hpp>
#include <vector>
#include "dancer_geometry/line_segment.hpp"

namespace dancer_geometry {
#define min_n(a, b) ((a) < (b) ? (a) : (b))
#define max_n(a, b) ((a) > (b) ? (a) : (b))
#define boundry_n(n, a, b) \
  {                        \
    n = min_n(b, n);       \
    n = max_n(a, n);       \
  }

/**
 * @brief Rotate coordinate axis and get corresponding point
 *
 * @param alpha - angle in degree to rotate
 * @param p - position of given point
 *
 * @return corresponding position of point after rotation
 */
cv::Point2f RotateCoordinateAxis(const double& alpha, const cv::Point2f& p);

/**
 * @brief Rotate coordinate axis and get corresponding point
 *
 * @param [in] alpha - angle in degree to rotate
 * @param [in] p - position of given point
 * @param [out] res - corresponding position of point after rotation
 */
void RotateCoordinateAxis(const double& alpha, const cv::Point2d& p,
                          cv::Point2d& res);

/**
 * @brief Rotate given point around original point
 *
 * @param pQuery - position of given point
 * @param alpha - angel to rotate
 *
 * @return corresponding position of point after rotation
 */
cv::Point2f RotateAroundPoint(const cv::Point2f& pQuery, const double& alpha);

/**
 * @brief Rotate given point around center point
 *
 * @param pQuery - position of given point
 * @param alpha - angel to rotate
 * @param pCenter - position of center point
 *
 * @return corresponding position of point after rotation
 */
cv::Point2f RotateAroundPoint(const cv::Point2f& pQuery, const double& alpha,
                              const cv::Point2f& pCenter);

/**
 * @brief Rotate given point around original point
 *
 * @param [in] pQuery - position of given point
 * @param [in] alpha - angel to rotate
 * @param [out] res - corresponding position of point after rotation
 */
void RotateAroundPoint(const cv::Point2d& pQuery, const double& alpha,
                       cv::Point2d& res);

/**
 * @brief Convert angle from radian to degree
 *
 * @param d - given angle in redian
 *
 * @return corresponding angle in degree
 */
double Radian2Degree(const double& r);

/**
 * @brief Convert angle from degree to radian
 *
 * @param d - given angle in degree
 *
 * @return corresponding angle in radian
 */
double Degree2Radian(const double& d);

/**
 * @brief Low-pass filter for 3D point
 *
 * @param [in] newrec - new record
 * @param [out] res - result of low pass
 * @param [in] coef - coeffient of low pass
 */
void LowPass(const cv::Point3f& newrec, cv::Point3f& res, const float& coef);

/**
 * @brief Low-pass filter for 2d point
 *
 * @param [in] newrec - new record
 * @param [out] res - result of low pass
 * @param [in] coef - coeffient of low pass
 */
void LowPass(const cv::Point2f& newrec, cv::Point2f& res, const float& coef);

/**
 * @brief Low-pass filter for float number
 *
 * @param [in] newrec - new record
 * @param [out] res - result of low pass
 * @param [in] coef - coeffient of low pass
 */
void LowPass(const float& newrec, float& res, const float& coef);

/**
 * @brief Get distance between given point and original point
 *
 * @param p - position of point 1
 *
 * @return distance between given point and original point
 */
double GetDistance(const cv::Point2d& p);

/**
 * @brief Get distance between given point and original point
 *
 * @param p - position of point 1
 *
 * @return distance between given point and original point
 */
float GetDistance(const cv::Point2f& p);

/**
 * @brief Get distance between two points
 *
 * @param p - position of point 1
 * @param p2 - position of point 2
 *
 * @return distance between two points
 */
double GetDistance(const cv::Point2d& p, const cv::Point2d& p2);

/**
 * @brief Get distance between two points
 *
 * @param p - position of point 1
 * @param p2 - position of point 2
 *
 * @return distance between two points
 */
float GetDistance(const cv::Point2f& p, const cv::Point2f& p2);

/**
 * @brief Get distance between two points
 *
 * @param p - position of point 1
 * @param p2 - position of point 2
 *
 * @return distance between two points
 */
float GetDistance(const cv::Point& p, const cv::Point& p2);

/**
 * @brief Get top boundary of given rectangle
 *
 * @param rec - given rectangle
 *
 * @return top boundary of given rectangle
 */
int Top(const cv::Rect& rec);

/**
 * @brief Get bottom boundary of given rectangle
 *
 * @param rec - given rectangle
 *
 * @return bottom boundary of given rectangle
 */
int Bottom(const cv::Rect& rec);

/**
 * @brief Get left boundary of given rectangle
 *
 * @param rec - given rectangle
 *
 * @return left boundary of given rectangle
 */
int Left(const cv::Rect& rec);

/**
 * @brief Get right boundary of given rectangle
 *
 * @param rec - given rectangle
 *
 * @return right boundary of given rectangle
 */
int Right(const cv::Rect& rec);

/**
 * @brief Get gray white color for drawing
 *
 * @return gray white color
 */
cv::Scalar grayWhite();

/**
 * @brief Get pink color for drawing
 *
 * @return pink color
 */
cv::Scalar pinkColor();

/**
 * @brief Get pink melo color for drawing
 *
 * @return pink melo color
 */
cv::Scalar pinkMeloColor();

/**
 * @brief Get white color for drawing
 *
 * @return white color
 */
cv::Scalar whiteColor();

/**
 * @brief Get red color for drawing
 *
 * @return red color
 */
cv::Scalar redColor();

/**
 * @brief Get magent normal color for drawing
 *
 * @return magent normal color
 */
cv::Scalar magentaNormalColor();

/**
 * @brief Get orange color for drawing
 *
 * @return orange color
 *
 */
cv::Scalar orangeColor();

/**
 * @brief Get dark orange color for drawing
 *
 * @return dark orange color
 */
cv::Scalar darkOrangeColor();

/**
 * @brief Get red melo color for drawing
 *
 * @return red melo color
 */
cv::Scalar redMeloColor();

/**
 * @brief Get green color for drawing
 *
 * @return green color
 */
cv::Scalar greenColor();

/**
 * @brief Get yellow color for drawing
 *
 * @return yellow color
 */
cv::Scalar yellowColor();

/**
 * @brief Get blue color for drawing
 *
 * @return blue color
 */
cv::Scalar blueColor();

/**
 * @brief Get blue melo color for drawing
 *
 * @return bluw melo color
 */
cv::Scalar blueMeloColor();

/**
 * @brief Get blue normal color for drawing
 *
 * @return blue normal color
 */
cv::Scalar blueNormalColor();

/**
 * @brief Get blue light color for drawing
 *
 * @return blue light color
 */
cv::Scalar blueLightColor();

/**
 * @brief Get blue gray color for drawing
 *
 * @return blue gray color
 */
cv::Scalar blueGrayColor();

/**
 * @brief Get black color for drawing
 *
 * @return black color
 */
cv::Scalar blackColor();

/**
 * @brief Get black gray color for drawing
 *
 * @return black gray color
 */
cv::Scalar blackGary();

/**
 * @brief Merge line segments until no more line segments can be merged
 *
 * @param [in] resLinesReal - input line segments
 * @param [in] maxDegree - maximum degree between two line segments to merge
 * @param [in] maxDistance - maximum distance between two line segments to merge
 * @param [in] minCollinearLength - minimum line length for two collinear
 * segments to merge
 * @param [out] clusteredLines - merged line segments
 * @param [in] box - bounding box for merged lines to clip
 * @param [in] useBounding - flag for whether or not use rotated bbox to merge
 * line segments
 *
 * @return whether or not merging is successful
 */
bool MergeLinesMax(std::vector<LineSegment> resLinesReal,
                   const double& maxDegree, const double& maxDistance,
                   const double& minCollinearLength,
                   std::vector<LineSegment>& clusteredLines,
                   const cv::Rect& box, const bool& useBounding = false);

/**
 * @brief Merge line segments once
 *
 * @param [in] resLinesReal - input line segments
 * @param [in] maxDegree - maximum degree between two line segments to merge
 * @param [in] maxDistance - maximum distance between two line segments to merge
 * @param [in] minCollinearLength - minimum line length for two collinear
 * segments to merge
 * @param [out] clusteredLines - merged line segments
 * @param [in] box - bounding box for merged lines to clip
 * @param [in] useBounding - flag for whether or not use rotated bbox to merge
 * line segments
 *
 * @return whether or not merging is successful
 */
bool MergeLinesOnce(std::vector<LineSegment> resLinesReal,
                    const double& maxDegree, const double& maxDistance,
                    const double& minCollinearLength,
                    std::vector<LineSegment>& clusteredLines,
                    const cv::Rect& box, const bool& useBounding = false);

/**
 * @brief Calculate distance in 3D between two line segments
 *
 * @param S1 - line segment 1
 * @param S2 - line segment 2
 *
 * @return distance in 3D between two line segments
 */
float dist3D_Segment_to_Segment(const LineSegment& S1, const LineSegment& S2);

/**
 * @brief Correct angle in degree to [0,360)
 *
 * @param x - given angle
 *
 * @return angle in degree corrected to [0,360)
 */
double CorrectAngleDegree360(const double& x);

/**
 * @brief Correct angle in degree to [-180,180)
 *
 * @param x - given angle
 *
 * @return angle in degree corrected to [-180,180)
 */
double CorrectAngleDegree180(const double& x);

/**
 * @brief Correct angle in radian to [0,2*PI)
 *
 * @param x - given angle
 *
 * @return angle in radian corrected to [0,2*PI)
 */
double CorrectAngleRadian360(const double& x);

/**
 * @brief Correct angle in radian to [-PI,PI)
 *
 * @param x - given angle
 *
 * @return angle in radian corrected to [-PI,PI)
 */
double CorrectAngleRadian180(const double& x);

/**
 * @brief Get angle diff in degree and normalize it to [-180,180)
 *
 * @param first - first angle
 * @param second - second angle
 *
 * @return angle diff in degree and normalize it to [-180,180)
 */
float AngleDiffDegree180(const float& first, const float& second);

/**
 * @brief Get angle diff in radian and normalize it to [-PI,PI)
 *
 * @param first - first angle
 * @param second - second angle
 *
 * @return angle diff in radian and normalize it to [-PI,PI)
 */
float AngleDiffRadian180(const float& first, const float& second);

/**
 * @brief Get average of two points
 *
 * @param p0 - poisiton of point 0
 * @param p1 - position of point 1
 *
 * @return
 */
cv::Point2f GetAverage(const cv::Point2f& p0, const cv::Point2f& p1);

/**
 * @brief Get weighted average of two points
 *
 * @param p0 - position of point 0
 * @param p1 - position of point 1
 * @param w0 - weight of point 0
 * @param w1 - weight of point 1
 *
 * @return weighted average of two points
 */
cv::Point2f GetWeightedAverage(const cv::Point2f& p0, const cv::Point2f& p1,
                               const float& w0, const float& w1);

/**
 * @brief Get weighted average of two 1D point
 *
 * @param p0 - position of point 0
 * @param p1 - position of point 1
 * @param w0 - weight of point 0
 * @param w1 - weight of point 1
 *
 * @return weighted average of two 1D point
 */
float GetWeightedAverage(const float& p0, const float& p1, const float& w0,
                         const float& w1);

/**
 * @brief Calculate distance between line segment and given point
 *
 * @param line - line segment
 * @param p - given point
 *
 * @return distance between line segment and given point
 */
float DistanceFromLineSegment(const LineSegment& line, const cv::Point2f& p);

bool SortFuncDescending(const std::vector<cv::Point>& i,
                        const std::vector<cv::Point>& j);

/**
 * @brief Get position on global coordinate
 *
 * @param robot_pos - robot pose (x,y,t) on global coordinate
 * @param in_lines - given line segments on robot coordinate
 *
 * @return corresponding line segments on global coordinate
 */
std::vector<LineSegment> getOnGlobalCoordinate(
    const cv::Point3d& robot_pos, const std::vector<LineSegment>& in_lines);

/**
 * @brief Get position on global coordinate
 *
 * @param robot_pos - robot pose (x,y,t) on global coordinate
 * @param in_points - given points on robot coordinate
 *
 * @return corresponding points on global coordinate
 */
std::vector<cv::Point2f> getOnGlobalCoordinate(
    const cv::Point3d& robot_pos, const std::vector<cv::Point2f>& in_points);

cv::Point2f getOnGlobalCoordinate(const cv::Point3d& robot_pos,
                                  const cv::Point2f& in_point);

/**
 * @brief Get position on global coordinate
 *
 * @param robot_pos - robot pose (x,y,t) on global coordinate
 * @param x - position x of given point
 * @param y - position y of given point
 *
 * @return  corresponding point on global coordinate
 */
cv::Point2f getOnGlobalCoordinate(const cv::Point3d& robot_pos, float x,
                                  float y);

/**
 * @brief Get position on global coordinate
 *
 * @param robot_pos - robot pose (x,y,t) on global coordinate
 * @param x - position x of given point
 * @param y - position y of given point
 *
 * @return corresponding point on global coordinate
 */
cv::Point2f getOnGlobalCoordinate(const geometry_msgs::Vector3& robot_pos,
                                  float x, float y);

/**
 * @brief Get position on global coordinate
 *
 * @param robot_pos - robot pose (x,y,t) on global coordinate
 * @param in_point - given point (x,y) on robot coordinate
 *
 * @return corresponding point on global coordinate
 */
cv::Point2f getOnGlobalCoordinate(const geometry_msgs::Vector3& robot_pos,
                                  const cv::Point2f& in_point);

/**
 * @brief Get position on global coordinate
 *
 * @param robot_pos - robot pose (x,y,t) on global coordinate
 * @param in_point - given point (x,y) on robot coordinate
 *
 * @return corresponding point on global coordinate
 */
cv::Point2f getOnGlobalCoordinate(const geometry_msgs::Vector3& robot_pos,
                                  const geometry_msgs::Vector3& in_point);

/**
 * @brief Get position on global coordinate
 *
 * @param robot_pos - robot pose (x,y,t) on global coordinate
 * @param in_point - given point (x,y) on robot coordinate
 *
 * @return corresponding point on global coordinate
 */
cv::Point2d getOnGlobalCoordinate(const cv::Point3d& robot_pos,
                                  const cv::Point2d& in_point);

/**
 * @brief Get position on global coordinate
 *
 * @param robot_pos - robot pose (x,y,t) on global coordinate
 * @param in_point - given point (x,y) on robot coordinate
 *
 * @return corresponding point on global coordinate
 */
std::vector<double> getOnGlobalCoordinate(const std::vector<double>& robot_pos,
                                          const std::vector<double>& in_point);

/**
 * @brief Get position on robot coordinate
 *
 * @param robot_pos - robot pose (x,y,t) on global coordinate
 * @param in_point - given point (x,y) on global coordinate
 *
 * @return corresponding point on robot coordinate
 */
cv::Point2d getOnRobotCoordinate(const cv::Point3d& robot_pos,
                                 const cv::Point2d& in_point);

/**
 * @brief Get position on robot coordinate
 *
 * @param robot_pos - robot pose (x,y,t) on global coordinate
 * @param in_point - given point (x,y) on global coordinate
 *
 * @return corresponding point on robot coordinate
 */
std::vector<double> getOnRobotCoordinate(const std::vector<double>& robot_pos,
                                         const std::vector<double>& in_point);

/**
 * @brief Get angle between two vectors
 *
 * @param v1 - vector 1
 * @param v2 - vector 2
 *
 * @return angle between two vectors
 */
inline float getAngleBetweenVectors(const cv::Point2f& v1,
                                    const cv::Point2f& v2) {
  float dot = v1.x * v2.x + v1.y * v2.y;
  float det = v1.x * v2.y - v1.y * v2.x;
  return Radian2Degree(atan2f(det, dot));
}

/**
 * @brief Run given function pairwise in all elements of given container
 *
 * @tparam Iter - type of iterator for container
 * @tparam Func - type of function
 * @param first - iterator to first element in container
 * @param last - iterator to last element in container
 * @param func - function to do pairwise
 */
template <typename Iter, typename Func>
void find_pairwise(Iter first, Iter last, Func func) {
  for (; first != last; ++first)
    for (Iter next = std::next(first); next != last; ++next)
      func(*first, *next);
}

}  // namespace dancer_geometry
