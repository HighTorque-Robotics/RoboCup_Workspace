/* Copyright (C) ZJUDancer
 * 2018 - Yusu Pan <xxdsox@gmail.com>
 * 2017 - Wenxing Mei <mwx36mwx@gmail.com>
 */

/**
 * \file distortionModel.hpp
 * \author Yusu Pan, Wenxing Mei, Yujie Yang
 * \version 2018
 * \date 2018-02-14
 */

// Copied from Nimbro
#pragma once
#include <opencv2/opencv.hpp>
#include <vector>

namespace dvision {

//! Model for camera distortion and undistortion
class DistortionModel
{
  public:
    //! DistortionModel constructor
    DistortionModel();
    //! Initialize DistortionModel
    void init();

    /**
     * \brief Undistort points (x,y)
     *
     * \param x - x positon in distorted image
     * \param y - y positon in distorted image
     *
     * \return corresponding point position in undistorted image
     */
    cv::Point2f undistort(int x, int y);

    /**
     * \brief Undistort cv::Point
     *
     * \param points - point in distorted image
     * \param res - result point in undistorted image
     *
     * \return whether points are undistorted successfully
     */

    bool undistort(const std::vector<cv::Point>& points, std::vector<cv::Point>& res);

    /**
     * \brief Undistort cv::Point2f
     *
     * \param points - point in distorted image
     * \param res - result point in undistorted image
     *
     * \return whether points are undistorted successfully
     */
    bool undistort(const std::vector<cv::Point>& points, std::vector<cv::Point2f>& res);

    /**
     * \brief Undistort image by remap
     *
     * \param src - source image
     * \param dst - destination image
     */
    void undistortImage(const cv::Mat& src, cv::Mat& dst);

    /**
     * \brief Undistort image by undistort function
     *
     * \param src - source image
     * \param dst - destination image
     */
    void undistortImage2(const cv::Mat& src, cv::Mat& dst);

    /**
     * \brief Undistort image by hand
     *
     * \param src - source image
     * \param dst - destination image
     */
    void undistortImage3(const cv::Mat& src, cv::Mat& dst);

    /**
     * \brief Distort a vector of cv::Point
     *
     * \param points - points in undistorted image
     * \param res - points in distorted image
     */
    void distort(const std::vector<cv::Point>& points, std::vector<cv::Point>& res);

    /**
     * \brief Distort point (x,y)
     *
     * \param x - x positon in undistorted image
     * \param y - y positon in undistorted image
     *
     * \return point in distorted image
     */
    cv::Point distort(int x, int y);

    /**
     * \brief Distort cv::Point
     *
     * \param point - point in undistorted image
     *
     * \return points in distorted image
     */
    inline cv::Point distort(const cv::Point point)
    {
        return distort(point.x, point.y);
    }

    /**
     * \brief Distort cv::Point
     *
     * \param point - point in undistorted image
     * \param resPoint - point in distorted image
     *
     * \return whether points are distorted successfully
     */
    bool distortP(const cv::Point& point, cv::Point& resPoint);

    /**
     * \brief Distort contour
     *
     * \param contour - points of contour in undistorted image
     * \param resCountour - points of contour in distorted image
     *
     * \return whether points are distorted successfully
     */
    bool distortP(const std::vector<cv::Point>& contour, std::vector<cv::Point>& resCountour);

  private:
    /**
     * \brief Undistort cv::Point2f by slow method
     *
     * \param points - point in distorted image
     * \param resPoints - result point in undistorted image
     *
     * \return whether points are undistorted successfully
     */
    bool undistort_slow(const std::vector<cv::Point>& points, std::vector<cv::Point>& resPoints);

  private:
    //! Vector mapping points from undistorted image to distorted image
    std::vector<cv::Point> vector_undistortion_; // undist to dist
    //! Vector mapping points from distorted image to undistorted image
    std::vector<cv::Point> vector_distortion_;   // dist to undist
    //! The first output map in remapping
    cv::Mat map1_;
    //! The second output map in remapping
    cv::Mat map2_;
};
} // namespace dvision
