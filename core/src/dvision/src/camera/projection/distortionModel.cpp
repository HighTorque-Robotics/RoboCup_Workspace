/* Copyright (C) ZJUDancer
 * 2018 - Yusu Pan <xxdsox@gmail.com>
 * 2017 - Wenxing Mei <mwx36mwx@gmail.com>
 */

/**
 * \file distortionModel.cpp
 * \author Yusu Pan, Wenxing Mei, Yujie Yang
 * \version 2018
 * \date 2018-02-14
 */

#include "dvision/distortionModel.hpp"

#include "dvision/parameters.hpp"

using namespace std;
using namespace cv;

static const int INVALID = -9999;

namespace dvision {
DistortionModel::DistortionModel() {}

void DistortionModel::init() {
  int W = parameters.camera.width;
  int H = parameters.camera.height;

  std::vector<Point> center, resCenter;
  center.push_back(Point(W / 2, H / 2));
  undistort_slow(center, resCenter);
  std::vector<Point> src, res;

  for (int y = 0; y < H; ++y)
    for (int x = 0; x < W; ++x) src.push_back(Point(x, y));

  undistort_slow(src, res);

  int maxH = -1;
  int maxW = -1;

  for (uint32_t i = 0; i < res.size(); ++i) {
    int tmpW = max(maxW, abs(res[i].x - resCenter[0].x));
    int tmpH = max(maxH, abs(res[i].y - resCenter[0].y));
    if (tmpW > W * 2 || tmpH > H * 2) {
      ROS_ERROR("[dvision] undistort_error %d %d", tmpW, tmpH);
      continue;
    }
    maxW = tmpW;
    maxH = tmpH;
  }

  parameters.camera.undistWidth = maxW * 2 + 1;
  parameters.camera.undistHeight = maxH * 2 + 1;
  parameters.camera.undistImageSize = Size(maxW * 2 + 1, maxH * 2 + 1);
  std::cout << "Undistorted image size:" << parameters.camera.undistImageSize
            << std::endl;

  int offsetX = maxW - resCenter[0].x;
  int offsetY = maxH - resCenter[0].y;

  //    int offsetX = (m_undistImageSize.width - W) / 2.;
  //    int offsetY = (m_undistImageSize.height - H) / 2.;

  parameters.camera.undistCameraMatrix = parameters.camera.cameraMatrix.clone();
  parameters.camera.undistCameraMatrix.at<double>(0, 2) += offsetX;
  parameters.camera.undistCameraMatrix.at<double>(1, 2) += offsetY;
  parameters.camera.undistCx =
      parameters.camera.undistCameraMatrix.at<double>(0, 2);
  parameters.camera.undistCy =
      parameters.camera.undistCameraMatrix.at<double>(1, 2);

  vector_undistortion_.resize(W * H);
  vector_distortion_.resize(parameters.camera.undistWidth *
                            parameters.camera.undistHeight);

  // init
  for_each(vector_distortion_.begin(), vector_distortion_.end(), [&](Point& p) {
    p.x = INVALID;
    p.y = INVALID;
  });

  for (int y = 0; y < H; ++y) {
    for (int x = 0; x < W; ++x) {
      int index = y * W + x;
      int undistX = res[index].x + offsetX;
      int undistY = res[index].y + offsetY;
      vector_undistortion_[index] = cv::Point(undistX, undistY);

      // int undistIndex = undistY * parameters.camera.undistWidth + undistX;
      // m_distortionVector[undistIndex] = Point(x, y);
    }
  }

  int centerIndex = (H / 2 * W) + W / 2;
  parameters.camera.centerInUndistX = vector_undistortion_[centerIndex].x;
  parameters.camera.centerInUndistY = vector_undistortion_[centerIndex].y;

  // init distortion vector, from undist point to dist point
  // for (int y = 0; y < parameters.camera.undistHeight; ++y) {
  //     for (int x = 0; x < parameters.camera.undistWidth; ++x) {
  //         int index = y * parameters.camera.undistWidth + x;
  //
  //         if (m_distortionVector[index].x == INVALID &&
  //         m_distortionVector[index].y == INVALID) {
  //             int find_region = 2; // pixel
  //             int cnt = 0;
  //             int sumx = 0;
  //             int sumy = 0;
  //             for (int dx = find_region; dx >= -find_region; --dx) {
  //                 for (int dy = -find_region; dy <= find_region; ++dy) {
  //                     int nx = x + dx;
  //                     int ny = y + dy;
  //                     if (0 <= nx && nx < parameters.camera.undistWidth && 0
  //                     <= ny && ny < parameters.camera.undistHeight) {
  //                         int idx = ny * parameters.camera.undistWidth + nx;
  //                         if (m_distortionVector[idx].x != INVALID &&
  //                         m_distortionVector[idx].y != -INVALID) {
  //                             ++cnt;
  //                             sumx += m_distortionVector[idx].x;
  //                             sumy += m_distortionVector[idx].y;
  //                         }
  //                     }
  //                 }
  //             }
  //             if (cnt != 0) {
  //                 m_distortionVector[index].x = sumx / cnt;
  //                 m_distortionVector[index].y = sumy / cnt;
  //             }
  //         }
  //     }
  // }
  cv::initUndistortRectifyMap(
      parameters.camera.cameraMatrix, parameters.camera.distCoeff, cv::Mat(),
      parameters.camera.undistCameraMatrix, parameters.camera.undistImageSize,
      CV_16SC2, map1_, map2_);

  std::cout << "Undist camera matrix" << std::endl;
  std::cout << parameters.camera.undistCameraMatrix << std::endl;
  printf("%.13lf %.13lf %.13lf %.13lf\n",
         parameters.camera.undistCameraMatrix.at<double>(0, 0),
         parameters.camera.undistCameraMatrix.at<double>(1, 1),
         parameters.camera.undistCameraMatrix.at<double>(0, 2),
         parameters.camera.undistCameraMatrix.at<double>(1, 2));
}

void DistortionModel::undistortImage(const cv::Mat& rawImg, cv::Mat& res) {
  remap(rawImg, res, map1_, map2_, INTER_LINEAR);
}

void DistortionModel::undistortImage2(const cv::Mat& rawImg, cv::Mat& res) {
  const int W = parameters.camera.imageSize.width;
  const int H = parameters.camera.imageSize.height;

  std::vector<cv::Point> p(static_cast<unsigned long>(H * W)), resP;
  int ctmp = 0;
  for (int y = 0; y < H; y++) {
    for (int x = 0; x < W; x++) {
      p[ctmp++] = Point(x, y);
    }
  }

  undistort(p, resP);

  const int siX = parameters.camera.undistImageSize.width;
  const int siY = parameters.camera.undistImageSize.height;
  res = cv::Mat::zeros(Size(siX, siY), CV_8UC3);

  int counter = 0;

  const int rawChannels = rawImg.channels();
  const int rawSize = rawImg.rows * rawImg.cols;
  uchar* raw_D = rawImg.data;
  uchar* tmp_D = res.data;

  for (int i = 0; i < rawSize; i++) {
    int x = resP[counter].x;
    int y = resP[counter].y;
    counter++;
    raw_D += rawChannels;
    if (x < 0 || y < 0 || y >= siY || x >= siX) {
      continue;
    }
    uchar* currentP_tmp_D = tmp_D + (((y * siX) + x) * rawChannels);
    currentP_tmp_D[0] = raw_D[0];
    currentP_tmp_D[1] = raw_D[1];
    currentP_tmp_D[2] = raw_D[2];
  }
}

void DistortionModel::undistortImage3(const cv::Mat& rawImg, cv::Mat& res) {
  const int siX = parameters.camera.undistImageSize.width;
  const int siY = parameters.camera.undistImageSize.height;
  res = cv::Mat::zeros(Size(siX, siY), CV_8UC3);

  const int rawChannels = rawImg.channels();
  uchar* raw_D = rawImg.data;
  uchar* tmp_D = res.data;

  for (int y = 0; y < siY; ++y) {
    for (int x = 0; x < siX; ++x) {
      cv::Point origin = vector_distortion_[y * siX + x];
      if (origin.x != INVALID && origin.y != INVALID) {
        uchar* currentP_tmp_D = tmp_D + ((y * siX) + x) * rawChannels;
        uchar* current_raw_D =
            raw_D +
            ((origin.y * parameters.camera.width) + origin.x) * rawChannels;
        currentP_tmp_D[0] = current_raw_D[0];
        currentP_tmp_D[1] = current_raw_D[1];
        currentP_tmp_D[2] = current_raw_D[2];
      }
    }
  }
}

bool DistortionModel::undistort(const std::vector<cv::Point>& points,
                                std::vector<cv::Point2f>& res) {
  res.resize(points.size());
  int W = parameters.camera.width;
  int H = parameters.camera.height;
  for (uint32_t i = 0; i < points.size(); ++i) {
    int x = points[i].x;
    int y = points[i].y;
    if (x < 0 || x >= W || y < 0 || y >= H) {
      ROS_ERROR("Error in undistort (%d, %d)\n", x, y);
      return false;
    }
    int xx = x == W ? x - 1 : x;
    int yy = y == H ? y - 1 : y;
    res[i] = vector_undistortion_[yy * W + xx];
  }
  return true;
}

bool DistortionModel::undistort(const std::vector<cv::Point>& points,
                                std::vector<cv::Point>& res) {
  res.resize(points.size());
  int W = parameters.camera.width;
  int H = parameters.camera.height;
  for (uint32_t i = 0; i < points.size(); ++i) {
    int x = points[i].x;
    int y = points[i].y;
    if (x < 0 || x >= W || y < 0 || y >= H) {
      ROS_ERROR("Error in undistort (%d, %d)\n", x, y);
      return false;
    }
    int xx = x == W ? x - 1 : x;
    int yy = y == H ? y - 1 : y;
    res[i] = vector_undistortion_[yy * W + xx];
  }
  return true;
}

cv::Point2f DistortionModel::undistort(int x, int y) {
  int W = parameters.camera.width;
  int H = parameters.camera.height;
  if (x < 0 || x > W || y < 0 || y > H) {
    ROS_ERROR("error in undistort: (%d, %d)", x, y);
    return Point2f(-9999, -9999);
  }
  int xx = x == W ? x - 1 : x;
  int yy = y == H ? y - 1 : y;
  return vector_undistortion_[yy * W + xx];
}

bool DistortionModel::undistort_slow(const std::vector<cv::Point>& points,
                                     std::vector<cv::Point>& resPoints)

{
  cv::Mat src = Mat(1, points.size(), CV_32FC2);
  cv::Mat res = src;

  for (uint32_t i = 0; i < points.size(); ++i) {
    src.at<Vec2f>(0, i)[0] = points[i].x;
    src.at<Vec2f>(0, i)[1] = points[i].y;
  }

  cv::undistortPoints(src, res, parameters.camera.cameraMatrix,
                      parameters.camera.distCoeff, noArray(),
                      parameters.camera.cameraMatrix);

  resPoints.resize(points.size());
  for (uint32_t i = 0; i < points.size(); ++i) {
    int x = res.at<Vec2f>(0, i)[0];
    int y = res.at<Vec2f>(0, i)[1];
    resPoints[i] = Point(x, y);
  }
  return true;
}

void DistortionModel::distort(const std::vector<cv::Point>& points,
                              std::vector<cv::Point>& res) {
  res.resize(points.size());
  for (uint32_t i = 0; i < points.size(); ++i) {
    res[i] = distort(points[i].x, points[i].y);
  };
}

bool DistortionModel::distortP(const cv::Point& point, cv::Point& resPoint) {
  std::vector<cv::Point> single_point, single_res;
  single_point.push_back(point);
  single_res.push_back(resPoint);
  if (distortP(single_point, single_res)) {
    resPoint = single_res.back();
    return true;
  }
  return false;
}

bool DistortionModel::distortP(const std::vector<cv::Point>& contour,
                               std::vector<cv::Point>& resCountour) {
  resCountour.resize(contour.size());  // allocate result
  std::vector<cv::Point2f> resCountourFloat(contour.size());
  const int W = parameters.camera.width;
  const int H = parameters.camera.height;

  const int siX = parameters.camera.undistWidth;
  const int siY = parameters.camera.undistHeight;

  int offsetx = static_cast<int>((siX - W) / 2.);
  int offsety = static_cast<int>((siY - H) / 2.);

  double fx = parameters.camera.cameraMatrix.at<double>(0, 0);
  double fy = parameters.camera.cameraMatrix.at<double>(1, 1);
  double cx = parameters.camera.cameraMatrix.at<double>(0, 2);
  double cy = parameters.camera.cameraMatrix.at<double>(1, 2);

  std::vector<cv::Point3f> contour3f(contour.size());
  for (size_t i = 0; i < contour.size(); i++) {
    contour3f[i] = cv::Point3f(
        static_cast<float>((contour[i].x - (offsetx + cx)) / fx),
        static_cast<float>((contour[i].y - (offsety + cy)) / fy), 1);
  }

  cv::Mat rVec(3, 1, cv::DataType<double>::type);  // Rotation vector
  rVec.at<double>(0) = 0;
  rVec.at<double>(1) = 0;
  rVec.at<double>(2) = 0;

  cv::Mat tVec(3, 1, cv::DataType<double>::type);  // Translation vector
  tVec.at<double>(0) = 0;
  tVec.at<double>(1) = 0;
  tVec.at<double>(2) = 0;

  cv::projectPoints(contour3f, rVec, tVec, parameters.camera.cameraMatrix,
                    parameters.camera.distCoeff, resCountourFloat);

  for (uint32_t i = 0; i < resCountourFloat.size(); i++) {
    resCountour[i] =
        cv::Point(std::max(std::min((int)round(resCountourFloat[i].x),
                                    parameters.camera.width - 1),
                           0),
                  std::max(std::min((int)round(resCountourFloat[i].y),
                                    parameters.camera.height - 1),
                           0));
  }
  return true;
}

cv::Point DistortionModel::distort(int x, int y) {
  if (x < 0 || x >= parameters.camera.undistWidth || y < 0 ||
      y >= parameters.camera.undistHeight) {
    ROS_ERROR("error in distort: (%d %d)", x, y);
    return cv::Point(INVALID, INVALID);
  }
  return vector_distortion_[y * parameters.camera.undistWidth + x];
}

}  // namespace dvision
