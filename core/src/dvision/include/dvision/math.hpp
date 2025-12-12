/* Copyright (C) ZJUDancer
 * 2018 - Yusu Pan <xxdsox@gmail.com>
 * 2017 - Wenxing Mei <mwx36mwx@gmail.com>
 */

/**
 * @file math.hpp
 * @author Yusu Pan, Wenxing Mei
 * @version 2018
 * @date 2018-02-24
 */

#pragma once
#include <Eigen/Dense>
#include <cmath>
#include <opencv2/opencv.hpp>

using std::cos;
using std::sin;
using Eigen::MatrixXd;

namespace dvision {

// refer to https://en.wikipedia.org/wiki/rotation_matrix

/**
  * @brief Get rotation matrix for rotating around X axis
  *
  * @param r - angle to rotate
  *
  * @return rotation matrix for rotating around X axis
  */
inline MatrixXd
rotateX(double r)
{
    MatrixXd R(4, 4);
    R << 1, 0, 0, 0, 0, cos(r), -sin(r), 0, 0, sin(r), cos(r), 0, 0, 0, 0, 1;
    return R;
}

/**
  * @brief Get rotation matrix for rotating around Y axis
  *
  * @param r - angle to rotate
  *
  * @return rotation matrix for rotating around Y axis
  */
inline MatrixXd
rotateY(double r)
{
    MatrixXd R(4, 4);
    R << cos(r), 0, sin(r), 0, 0, 1, 0, 0, -sin(r), 0, cos(r), 0, 0, 0, 0, 1;
    return R;
}

/**
  * @brief Get rotation matrix for rotating around Z axis
  *
  * @param r - angle to rotate
  *
  * @return rotation matrix for rotating around Z axis
  */
inline MatrixXd
rotateZ(double r)
{
    MatrixXd R(4, 4);
    R << cos(r), -sin(r), 0, 0, sin(r), cos(r), 0, 0, 0, 0, 1, 0, 0, 0, 0, 1;
    return R;
}

/**
 * @brief Get translation matrix
 *
 * @param x - translation in X axis
 * @param y - translation in Y axis
 * @param z - translation in Z axis
 *
 * @return translation matrix
 */
inline MatrixXd
dtranslate(double x, double y, double z)
{
    MatrixXd T(4, 4);
    T << 1, 0, 0, x, 0, 1, 0, y, 0, 0, 1, z, 0, 0, 0, 1;
    return T;
}
}
