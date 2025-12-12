/* Copyright (C) ZJUDancer
 * 2018 - Yusu Pan <xxdsox@gmail.com>
 * 2017 - Yujie Yang <meetyyj@gmail.com>
 */

/**
 * \file ball_tracker.cpp
 * \author Yusu Pan, Yujie Yang
 * \version 2018
 * \date 2018-02-17
 */

#include "dvision/ball_tracker.hpp"

namespace dvision {
bool
Tracker::Init(std::vector<double> extrinsic_para, double fx, double fy, double cx, double cy, double undist_pos_x, double undist_pos_y)
{
    undist_center_u_ = undist_pos_x;
    undist_center_v_ = undist_pos_y;

    Xw2p_ = extrinsic_para[0];
    Yw2p_ = extrinsic_para[1];
    Zw2p_ = extrinsic_para[2];

    RXw2p_ = extrinsic_para[3];
    RYw2p_ = extrinsic_para[4];
    RZw2p_ = extrinsic_para[5];
    Xp2c_ = extrinsic_para[6];
    Yp2c_ = extrinsic_para[7];
    Zp2c_ = extrinsic_para[8];

    RXp2c_ = extrinsic_para[9];
    RYp2c_ = extrinsic_para[10];
    RZp2c_ = extrinsic_para[11];

    scale_yaw_ = extrinsic_para[12];
    scale_pitch_ = extrinsic_para[13];
    bias_yaw_ = extrinsic_para[14];
    bias_pitch_ = extrinsic_para[15];

    MatrixXd c2i(4, 4); // camera to image
    c2i << 0, -1, 0, 0, 0, 0, -1, 0, 1, 0, 0, 0, 0, 0, 0, 1;
    c2i_ = c2i;

    MatrixXd cameraMatrix(3, 4);
    cameraMatrix << fx, 0, cx, 0, 0, fy, cy, 0, 0, 0, 1, 0;

    camera_matrix_ = cameraMatrix;

    w2p_ = rotateZ(RZw2p_) * rotateY(RYw2p_) * rotateX(RXw2p_) * dtranslate(-Xw2p_, -Yw2p_, -Zw2p_);

    p2c_ = rotateZ(RZp2c_) * rotateY(RYp2c_) * rotateX(RXp2c_) * dtranslate(-Xp2c_, -Yp2c_, -Zp2c_);

    return true;
}

// bool BalllTracker::Process(double in_ball_field_x, double in_ball_field_y,
//                                double in_pitch, double in_yaw) {
//
//
//     in_pitch = (in_pitch + biasPitch) * scalePitch;
//     in_yaw = (in_yaw + biasYaw) * scaleYaw;
//
//
//     // m_out_pitch = -in_pitch;
//     // m_out_yaw = -in_yaw;
//
//     MatrixXd field_xy0(4, 1);
//     field_xy0 << in_ball_field_x,
//                  in_ball_field_y,
//                  0,
//                  1;
//
//     double approx_yaw;
//     if(in_ball_field_x < 0.000001 && in_ball_field_x > 0.000001){
//       if(in_ball_field_y > 0){
//         approx_yaw = M_PI / 2;
//       } else {
//         approx_yaw = -M_PI / 2;
//       }
//     } else if(in_ball_field_x > 0){
//       approx_yaw = atan(in_ball_field_y / in_ball_field_x);
//     } else if(in_ball_field_y > 0){
//       approx_yaw = atan(in_ball_field_y / in_ball_field_x) + M_PI;
//     } else {
//       approx_yaw = atan(in_ball_field_y / in_ball_field_x) - M_PI;
//     }
//     // cout << "approx_yaw: " << approx_yaw / M_PI * 180 << endl;
//     int repeat_times = 2;
//     for (int i = 0; i < repeat_times; ++i) {
//       //cal yaw
//       MatrixXd m_y = m_cameraMatrix * m_c2i * m_p2c * rotateY(0);
//       // MatrixXd m_y = m_cameraMatrix * m_c2i * m_p2c * rotateY(m_out_pitch);
//       MatrixXd n_y = m_w2p * field_xy0;
//       double A_y = -undist_center_u * m_y(2, 0) * n_y(1, 0) + undist_center_u * m_y(2, 1) * n_y(0, 0) + m_y(0, 0) * n_y(1, 0) - m_y(0, 1) * n_y(0, 0);
//       double B_y = undist_center_u * m_y(2, 0) * n_y(0, 0) + undist_center_u * m_y(2, 1) * n_y(1, 0) - m_y(0, 0) * n_y(0, 0) - m_y(0, 1) * n_y(1, 0);
//       double C_y = undist_center_u * m_y(2, 2) * n_y(2, 0) + undist_center_u * m_y(2, 3) * n_y(3, 0) - m_y(0, 2) * n_y(2, 0) - m_y(0, 3) * n_y(3, 0);
//       m_out_yaw = Cal_theta_Asin_Bcos_C(A_y, B_y, C_y, -approx_yaw);
//       //
//       // MatrixXd s_uv = m_y * rotateZ(m_out_yaw) * n_y;
//       // cout << "-----------" << endl;
//       // cout << "第" << i << "次调整 左右:" << endl;
//       // cout << "u: " << s_uv(0, 0)/s_uv(2, 0) << endl;
//       // cout << "v: " << s_uv(1, 0)/s_uv(2, 0) << endl;
//       // cout << "pitch: " << m_out_pitch / M_PI * 180 << endl;
//       // cout << "yaw: " << m_out_yaw / M_PI * 180 << endl;
//       // cout << "-----------" << endl;
//       //cal pitch
//         MatrixXd m_p = m_cameraMatrix * m_c2i * m_p2c;
//         MatrixXd n_p = rotateZ(m_out_yaw) * m_w2p * field_xy0;
//
//         double A_p = undist_center_v * n_p(2, 0) * m_p(2, 0) - undist_center_v * n_p(0, 0) * m_p(2, 2) - n_p(2, 0) * m_p(1, 0) + n_p(0, 0) * m_p(1, 2);
//         double B_p = undist_center_v * n_p(0, 0) * m_p(2, 0) + undist_center_v * n_p(2, 0) * m_p(2, 2) - n_p(0, 0) * m_p(1, 0) - n_p(2, 0) * m_p(1, 2);
//         double C_p = undist_center_v * n_p(1, 0) * m_p(2, 1) + undist_center_v * n_p(3, 0) * m_p(2, 3) - n_p(1, 0) * m_p(1, 1) - n_p(3, 0) * m_p(1, 3);
//         // m_out_pitch = Cal_theta_Asin_Bcos_C(A_p, B_p, C_p, m_out_pitch);
//         m_out_pitch = Cal_theta_Asin_Bcos_C(A_p, B_p, C_p, 0);
//
//         // MatrixXd s_v = m_p * rotateY(m_out_pitch) * n_p;
//         // cout << "-----------" << endl;
//         // cout << "第" << i << "次调整 俯仰:" << endl;
//         // cout << "u: " << s_v(0, 0)/s_v(2, 0) << endl;
//         // cout << "v: " << s_v(1, 0)/s_v(2, 0) << endl;
//         // cout << "pitch: " << m_out_pitch / M_PI * 180 << endl;
//         // cout << "yaw: " << m_out_yaw / M_PI * 180 << endl;
//         // cout << "-----------" << endl;
//     }
//
//     m_out_pitch = - m_out_pitch / scalePitch - biasPitch;
//     m_out_yaw = - m_out_yaw / scaleYaw - biasYaw;
//     cout << "final pitch: " << m_out_pitch / M_PI * 180 << endl;
//     cout << "final yaw: " << m_out_yaw / M_PI * 180 << endl;
//   //  if (m_out_pitch >= 0 && m_out_pitch < M_PI / 2
//   //     && m_out_yaw >= -M_PI / 2 && m_out_yaw <= M_PI/2){
//   //       return true;
//   //  }
//   //  else{
//   //    return false;
//   //  }
//
//     return true;
// }

bool
Tracker::Process(double in_ball_field_x, double in_ball_field_y)
{
    if (std::isnan(in_ball_field_x) || std::isnan(in_ball_field_y)) {
        out_yaw_ = 0.0;
        out_pitch_ = 0.0;
        return true;
    }

    MatrixXd field_xy0(4, 1);
    field_xy0 << in_ball_field_x, in_ball_field_y, 0, 1;

    double approx_yaw;
    if (in_ball_field_x < 0.000001 && in_ball_field_x > 0.000001) {
        if (in_ball_field_y > 0) {
            approx_yaw = M_PI / 2;
        } else {
            approx_yaw = -M_PI / 2;
        }
    } else if (in_ball_field_x > 0) {
        approx_yaw = atan(in_ball_field_y / in_ball_field_x);
    } else if (in_ball_field_y > 0) {
        approx_yaw = atan(in_ball_field_y / in_ball_field_x) + M_PI;
    } else {
        approx_yaw = atan(in_ball_field_y / in_ball_field_x) - M_PI;
    }
    //  cout << "x: " << in_ball_field_x << endl;
    //  cout << "y: " << in_ball_field_y << endl;

    // cout << "approx_yaw: " << approx_yaw / M_PI * 180 << endl;
    out_yaw_ = -approx_yaw;
    out_pitch_ = 0;
    int repeat_times = 2;
    for (int i = 0; i < repeat_times; ++i) {
        // cal yaw
        MatrixXd m_y = camera_matrix_ * c2i_ * p2c_ * rotateY(out_pitch_);
        MatrixXd n_y = w2p_ * field_xy0;
        double A_y = -undist_center_u_ * m_y(2, 0) * n_y(1, 0) + undist_center_u_ * m_y(2, 1) * n_y(0, 0) + m_y(0, 0) * n_y(1, 0) - m_y(0, 1) * n_y(0, 0);
        double B_y = undist_center_u_ * m_y(2, 0) * n_y(0, 0) + undist_center_u_ * m_y(2, 1) * n_y(1, 0) - m_y(0, 0) * n_y(0, 0) - m_y(0, 1) * n_y(1, 0);
        double C_y = undist_center_u_ * m_y(2, 2) * n_y(2, 0) + undist_center_u_ * m_y(2, 3) * n_y(3, 0) - m_y(0, 2) * n_y(2, 0) - m_y(0, 3) * n_y(3, 0);
        out_yaw_ = Cal_theta_Asin_Bcos_C(A_y, B_y, C_y, out_yaw_);
        //
        // MatrixXd s_uv = m_y * rotateZ(m_out_yaw) * n_y;
        // cout << "-----------" << endl;
        // cout << "第" << i << "次调整 左右:" << endl;
        // cout << "u: " << s_uv(0, 0)/s_uv(2, 0) << endl;
        // cout << "v: " << s_uv(1, 0)/s_uv(2, 0) << endl;
        // cout << "pitch: " << m_out_pitch / M_PI * 180 << endl;
        // cout << "yaw: " << m_out_yaw / M_PI * 180 << endl;
        // cout << "-----------" << endl;
        // cal pitch
        MatrixXd m_p = camera_matrix_ * c2i_ * p2c_;
        MatrixXd n_p = rotateZ(out_yaw_) * w2p_ * field_xy0;

        double A_p = undist_center_v_ * n_p(2, 0) * m_p(2, 0) - undist_center_v_ * n_p(0, 0) * m_p(2, 2) - n_p(2, 0) * m_p(1, 0) + n_p(0, 0) * m_p(1, 2);
        double B_p = undist_center_v_ * n_p(0, 0) * m_p(2, 0) + undist_center_v_ * n_p(2, 0) * m_p(2, 2) - n_p(0, 0) * m_p(1, 0) - n_p(2, 0) * m_p(1, 2);
        double C_p = undist_center_v_ * n_p(1, 0) * m_p(2, 1) + undist_center_v_ * n_p(3, 0) * m_p(2, 3) - n_p(1, 0) * m_p(1, 1) - n_p(3, 0) * m_p(1, 3);
        out_pitch_ = Cal_theta_Asin_Bcos_C(A_p, B_p, C_p, out_pitch_);

        // MatrixXd s_v = m_p * rotateY(m_out_pitch) * n_p;
        // cout << "-----------" << endl;
        // cout << "第" << i << "次调整 俯仰:" << endl;
        // cout << "u: " << s_v(0, 0)/s_v(2, 0) << endl;
        // cout << "v: " << s_v(1, 0)/s_v(2, 0) << endl;
        // cout << "pitch: " << m_out_pitch / M_PI * 180 << endl;
        // cout << "yaw: " << m_out_yaw / M_PI * 180 << endl;
        // cout << "-----------" << endl;
    }

    out_pitch_ = -out_pitch_ / scale_pitch_ - bias_pitch_;
    out_yaw_ = -out_yaw_ / scale_yaw_ - bias_yaw_;
    // cout << "final pitch: " << m_out_pitch / M_PI * 180 << endl;
    // cout << "final yaw: " << m_out_yaw / M_PI * 180 << endl;
    //  if (m_out_pitch >= 0 && m_out_pitch < M_PI / 2
    //     && m_out_yaw >= -M_PI / 2 && m_out_yaw <= M_PI/2){
    //       return true;
    //  }
    //  else{
    //    return false;
    //  }

    return true;
}

double
Tracker::Cal_theta_Asin_Bcos_C(double _a, double _b, double _c, double theta_raw)
{
    double sin_theta_1 = (-2 * _a * _c + sqrt(4 * _a * _a * _c * _c - 4 * (_a * _a + _b * _b) * (_c * _c - _b * _b))) / (2 * (_a * _a + _b * _b));
    double sin_theta_2 = (-2 * _a * _c - sqrt(4 * _a * _a * _c * _c - 4 * (_a * _a + _b * _b) * (_c * _c - _b * _b))) / (2 * (_a * _a + _b * _b));
    double res, res1 = 999, res2 = 999;
    if (abs(sin_theta_1) <= 1) {
        res1 = asin(sin_theta_1);
        if (res1 >= 0) {
            res1 = (abs(_a * sin(res1) + _b * cos(res1) + _c) < abs(_a * sin(M_PI - res1) + _b * cos(M_PI - res1) + _c)) ? res1 : M_PI - res1;
        } else {
            res1 = (abs(_a * sin(res1) + _b * cos(res1) + _c) < abs(_a * sin(-M_PI - res1) + _b * cos(-M_PI - res1) + _c)) ? res1 : -M_PI - res1;
        }
        // cout << "res1: " << res1 / M_PI * 180 << endl;
    }
    if (abs(sin_theta_2) <= 1) {
        res2 = asin(sin_theta_2);
        if (res2 >= 0) {
            res2 = (abs(_a * sin(res2) + _b * cos(res2) + _c) < abs(_a * sin(M_PI - res2) + _b * cos(M_PI - res2) + _c)) ? res2 : M_PI - res2;
        } else {
            res2 = (abs(_a * sin(res2) + _b * cos(res2) + _c) < abs(_a * sin(-M_PI - res2) + _b * cos(-M_PI - res2) + _c)) ? res2 : -M_PI - res2;
        }
        // cout << "res2: " << res2 / M_PI * 180 << endl;
    }
    //    cout << _a * sin(res1) + _b * cos(res1) + _c << endl;
    //    cout << _a * sin(res2) + _b * cos(res2) + _c << endl;
    res = (abs(theta_raw - res1) < abs(abs(theta_raw - res1) - 2 * M_PI) ? abs(theta_raw - res1) : abs(abs(theta_raw - res1) - 2 * M_PI)) <
              (abs(theta_raw - res2) < abs(abs(theta_raw - res2) - 2 * M_PI) ? abs(theta_raw - res2) : abs(abs(theta_raw - res2) - 2 * M_PI))
            ? res1
            : res2;
    // res = (abs(theta_raw - res1) < abs(theta_raw - res2)) ? res1 : res2;
    // cout << "res: " << res / M_PI * 180 << endl;
    //    cout << _a * sin(res) + _b * cos(res) + _c << endl;
    return res;
}
}
