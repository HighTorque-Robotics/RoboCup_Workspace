/* Copyright (C) ZJUDancer
 * 2018 - Yusu Pan <xxdsox@gmail.com>
 * 2017 - Wenxing Mei <mwx36mwx@gmail.com>
 * 2017 - Yujie Yang <meetyyj@gmail.com>
 */

/**
 * \file ipm.hpp
 * \author Yusu Pan, Wenxing Mei, Yujie Yang
 * \version 2018
 * \date 2018-02-14
 */

// Note: IPM must be inited after DistortionModel initialize
#pragma once
#include "dvision/math.hpp"
#include "dvision/parameters.hpp"
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <vector>

using Eigen::MatrixXd;
using dvision::parameters;
namespace dvision {
//! Inverse Projection Mapping
class IPM
{
  public:
    /**
     * \brief Init IPM, set camera matrix and extrinsic parameters
     *
     * \param extrinsic_para - extrinsic parameters of camera
     * \param undistFx - undistorted fx of camera
     * \param undistFy - undistorted fy of camera
     * \param undistCx - undistorted cx of camera
     * \param undistCy - undistorted cy of camera
     */
    void Init(std::vector<double> extrinsic_para, double undistFx, double undistFy, double undistCx, double undistCy);

    /**
     * \brief Update yaw and pitch, then first calculate extrinsic and then calculate m_A and m_invA
     *
     * \param pitch - pitch in radian
     * \param yaw - yaw in radian
     */
    void update(double pitch, double yaw);

    /**
     * \brief Update yaw and pitch (in degree), then first calculate extrinsic and then calculate m_A and m_invA
     *
     * \param pitch - pitch in degreen
     * \param yaw - yaw in degree
     */
    void updateDeg(double pitch, double yaw);

    /**
     * \brief Use m_A to project a 3D point into image plane
     *
     * Matlab code: projection.m
     *
     * \param x_real - position x in real coordinate (i.e. robot coordinate)
     * \param y_real - position x in real coordinate
     * \param z_real - position x in real coordinate
     *
     * \return 2D point in image plane
     */
    cv::Point2d project(double x_real, double y_real, double z_real = 0);

    /**
     * \brief Use m_invA to inverse project a point on image plane onto the z_real plane
     *
     * Normally z_real equals 0 meaning that  the object is on the ground
     * when considering the height, such as the center of the ball, z_real can be used
     * Matlab code: calc_xy.m
     *
     * \param u - position u in image plane
     * \param v - position v in image plane
     * \param z_real - position z in real coordinate
     *
     * \return 2D point on the z_real plane (i.e. field plane)
     */
    cv::Point2d inverseProject(double u, double v, double z_real = 0);

  private:
    /**
     * \brief Calculate extrinsic matrix
     *
     * \param pitch pitch value in radian
     * \param yaw - yaw value in radian
     */
    void calc_extrinsic(double pitch, double yaw);

    //! Intrinsic parameter matrix of camera
    Eigen::MatrixXd camera_matrix_;       // fx fy cx cy
    //! Extrinsic parameters of camera
    std::vector<double> extrinsic_para_; // 16 parameters to calculate extrinsic

    //! Extrinsic parameter matrix of camera
    Eigen::MatrixXd extrinsic_matrix_;
    //! Product of camera matrix and extrinsic matrix
    Eigen::MatrixXd A_;    // camera matrix * extrinsic matrix
    //! Inverse of A
    Eigen::MatrixXd inv_A_; // inverse of m_A;
};
} // namespace dvision
