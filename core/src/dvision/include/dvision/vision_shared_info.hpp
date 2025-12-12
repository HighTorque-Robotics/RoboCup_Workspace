/* Copyright (C) ZJUDancer
 * 2018 - Yusu Pan <xxdsox@gmail.com>
 * 2017 - Wenxing Mei <mwx36mwx@gmail.com>
 */

/**
 * @file vision_shared_info.hpp
 * @author Yusu Pan, Wenxing Mei
 * @version 2018
 * @date 2018-02-24
 */

#pragma once
#include "dmsgs/VisionInfo.h"
#include "dancer_geometry/line_segment.hpp"
#include "dvision/projection.hpp"


#include "dvision/yolo/utils.hpp"


namespace dvision {

/**
  * @brief Shared info for vision module, used in dvision for concurrency
  */
struct VisionSharedInfo
{
    //! Message for vision module
    dmsgs::VisionInfo visionInfo;
    //! frame image from camera in HSV color space 
    cv::Mat hsvImg;
    // cv::Mat grayImg;
    //! image for GUI manager
    cv::Mat guiImg;

    //! Binary image w.r.t obstacles
    cv::Mat obstacleMask;
    //! Image with convex hull of field
    cv::Mat convexHull;
    //! Binary image w.r.t soccer field
    cv::Mat fieldBinaryRaw;

    //! Points of field hull in image coordinate
    std::vector<cv::Point> hullField;
    //! Points of field hull in robot coordinate
    std::vector<cv::Point2f> fieldHullReal;

    //! bounding box of ball
    BBox ballPosition;
    //! bounding box of goal
    std::vector<BBox> goalPosition;
    //! bounding box of robots
    std::vector<BBox> robotPosition;
    //! bounding box of t, x, l corners and penalty marks
    FieldFeatures fieldFeatures;
    //! Current pitch value of camera pose
    double pitch;
    //! Current yaw value of camera pose
    double yaw;
};
}
