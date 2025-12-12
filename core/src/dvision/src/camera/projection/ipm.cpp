/* Copyright (C) ZJUDancer
 * 2018 - Yusu Pan <xxdsox@gmail.com>
 * 2017 - Wenxing Mei <mwx36mwx@gmail.com>
 * 2017 - Yujie Yang <meetyyj@gmail.com>
 */

/**
 * \file ipm.cpp
 * \author Yusu Pan, Wenxing Mei, Yujie Yang
 * \version 2018
 * \date 2018-02-14
 */

#include "dvision/ipm.hpp"
using namespace cv;
using namespace std;
using namespace Eigen;

namespace dvision {
void
IPM::Init(std::vector<double> extrinsic_para, double fx, double fy, double cx, double cy)
{
    extrinsic_para_ = extrinsic_para;
    assert(extrinsic_para_.size() == 16);

    camera_matrix_ = MatrixXd(4, 4);
    camera_matrix_ << fx, 0, cx, 0, 0, fy, cy, 0, 0, 0, 1, 0, 0, 0, 0, 1;

    // debug
    cout << "camera matrix: \n" << camera_matrix_ << endl;
    cout << "extrinsic para: [" << endl;
    for_each(extrinsic_para_.begin(), extrinsic_para_.end(), [](double& x) { cout << x << endl; });
    cout << "]" << endl;
}

void
IPM::update(double pitch, double yaw)
{
    calc_extrinsic(pitch, yaw);
    A_ = camera_matrix_ * extrinsic_matrix_;
    inv_A_ = A_.inverse();
}

void
IPM::updateDeg(double pitch, double yaw)
{
    update(pitch / 180.0 * M_PI, yaw / 180.0 * M_PI);
}

Point2d
IPM::project(double x_real, double y_real, double z_real)
{
    MatrixXd real(4, 1);
    real << x_real, y_real, z_real, 1;

    MatrixXd pointImage = A_ * real;
    double u = pointImage(0) / pointImage(2);
    double v = pointImage(1) / pointImage(2);

    return Point2d(u, v);
}

Point2d
IPM::inverseProject(double u, double v, double z_real)
{
    double c1 = inv_A_(2, 0);
    double c2 = inv_A_(2, 1);
    double c3 = inv_A_(2, 2);
    double c4 = inv_A_(2, 3);
    double s = (z_real - c4) / (u * c1 + v * c2 + c3);

    MatrixXd foo(4, 1);
    foo << s * u, s * v, s, 1;

    MatrixXd res(4, 1);
    res = inv_A_ * foo;

    double x = res(0);
    double y = res(1);
    return Point2d(x, y);
}

void
IPM::calc_extrinsic(double pitchRad, double yawRad)
{
    auto Xw2p = extrinsic_para_[0];
    auto Yw2p = extrinsic_para_[1];
    auto Zw2p = extrinsic_para_[2];

    auto RXw2p = extrinsic_para_[3];
    auto RYw2p = extrinsic_para_[4];
    auto RZw2p = extrinsic_para_[5];
    auto Xp2c = extrinsic_para_[6];
    auto Yp2c = extrinsic_para_[7];
    auto Zp2c = extrinsic_para_[8];

    auto RXp2c = extrinsic_para_[9];
    auto RYp2c = extrinsic_para_[10];
    auto RZp2c = extrinsic_para_[11];

    auto scaleYaw = extrinsic_para_[12];
    auto scalePitch = extrinsic_para_[13];
    auto biasYaw = extrinsic_para_[14];
    auto biasPitch = extrinsic_para_[15];

    double pitch = (pitchRad + biasPitch) * scalePitch;
    double yaw = (yawRad + biasYaw) * scaleYaw;

    MatrixXd w2p(4, 4);
    w2p = rotateZ(RZw2p) * rotateY(RYw2p) * rotateX(RXw2p) * dtranslate(-Xw2p, -Yw2p, -Zw2p);

    MatrixXd p2c(4, 4);
    p2c = rotateZ(RZp2c) * rotateY(RYp2c) * rotateX(RXp2c) * dtranslate(-Xp2c, -Yp2c, -Zp2c) * rotateY(-pitch) * rotateZ(-yaw);

    MatrixXd c2i(4, 4); // camera to image
    c2i << 0, -1, 0, 0, 0, 0, -1, 0, 1, 0, 0, 0, 0, 0, 0, 1;

    extrinsic_matrix_ = c2i * p2c * w2p;
}
} // namespace dvision
