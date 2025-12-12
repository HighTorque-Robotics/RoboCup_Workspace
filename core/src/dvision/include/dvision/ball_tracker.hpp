/* Copyright (C) ZJUDancer
 * 2018 - Yusu Pan <xxdsox@gmail.com>
 * 2017 - Yujie Yang <meetyyj@gmail.com>
 */

/**
 * \file ball_tracker.hpp
 * \author Yusu Pan, Yujie Yang
 * \version 2018
 * \date 2018-02-17
 */

#pragma once
#include "dvision/math.hpp"
#include "dvision/parameters.hpp"
#include <Eigen/Dense>

using namespace std;
using Eigen::MatrixXd;

namespace dvision {
// TODO rename this class, avoiding duplicate name with object tracker
//! Tracker for computing desired pitch and yaw to make object in the center of FOV
class Tracker
{
  public:
    /**
     * \brief Initialize Tracker
     *
     * \param extrinsic_para - extrinsic parameters of camera
     * \param fx - focal length in x axis
     * \param fy - focal length in y axis
     * \param cx - camera's principal point in x axis
     * \param cy - camera's principal point in y axis
     * \param undist_pos_x - position x in undistorted image
     * \param undist_pos_y - position y in undistorted image
     *
     * \return whether or not process is successful
     */
    bool Init(std::vector<double> extrinsic_para, double fx, double fy, double cx, double cy, double undist_pos_x, double undist_pos_y);

    // 球在场地上相对机器人的位置
    // 当前的pitch,yaw的值
    // bool Process(double in_ball_field_x, double in_ball_field_y, double in_pitch, double in_yaw);
    /**
     * \brief Process traking
     *
     * \param in_ball_field_x - ball position in robot ego coordinate
     * \param in_ball_field_y - ball position in robot ego coordinate
     *
     * \return whether or not process is successful
     */
    bool Process(double in_ball_field_x, double in_ball_field_y);

    //! Cal_theta_Asin_Bcos_C (WTF is it?)
    double Cal_theta_Asin_Bcos_C(double _a, double _b, double _c, double theta_raw);

    //! Get desired pitch value
    inline double out_pitch()
    {
        return out_pitch_;
    }
    //! Get desired yaw value
    inline double out_yaw()
    {
        return out_yaw_;
    }

  private:
    //! Desired pitch value for tracking
    double out_pitch_;
    //! Desired yaw value for tracking
    double out_yaw_;
    //! Instrinsic matrix of camera
    MatrixXd camera_matrix_;

    MatrixXd w2p_;
    MatrixXd p2c_;
    MatrixXd c2i_;
    int undist_center_u_;
    int undist_center_v_;
    double Xw2p_;
    double Yw2p_;
    double Zw2p_;
    double RXw2p_;
    double RYw2p_;
    double RZw2p_;
    double Xp2c_;
    double Yp2c_;
    double Zp2c_;
    double RXp2c_;
    double RYp2c_;
    double RZp2c_;
    double scale_yaw_;
    double scale_pitch_;
    double bias_yaw_;
    double bias_pitch_;
};
}
