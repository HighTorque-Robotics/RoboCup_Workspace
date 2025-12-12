/* Copyright (C) ZJUDancer
 * 2018 - Yusu Pan <xxdsox@gmail.com>
 * 2017 - Wenxing Mei <mwx36mwx@gmail.com>
 */

/**
 * \file cameraSettings.hpp
 * \author Yusu Pan, Wenxing Mei
 * \version 2018
 * \date 2018-02-14
 */

#pragma once
#include <cstdint>
#include <linux/videodev2.h>
#include <ros/ros.h>
#include <string>

namespace dvision {
//! Settings for V4L2 camera device
struct CameraSettings
{
    inline explicit CameraSettings()
      : device("/dev/video0")
      , width(640)
      , height(480)
      , frameRate(30)
      , brightness(128)
      , contrast(128)
      , saturation(128)
      , hue(0)
      , sharpness(0)
      , gain(255)
      , gamma(27)
      , whitebalance_auto(0)
      , whitebalance_absolute(4500)
      , exposure_auto(1)
      , exposure_absolute(150)
      , focus_auto(1)
      , focus_absolute(0)
    {}

// macro to get parameters
#define GPARAM(x)                                                                                                                                                                                      \
    do {                                                                                                                                                                                               \
        if (!nh->getParam("dvision/camera/" #x, x)) {                                                                                                                                                  \
            ROS_FATAL("[dvision] CameraSettings get pararm error");                                                                                                                                    \
            throw std::runtime_error("Camera settings get param error!");                                                                                                                              \
        }                                                                                                                                                                                              \
    } while (0)

    inline explicit CameraSettings(ros::NodeHandle* nh)
    {
        GPARAM(device);
        GPARAM(width);
        GPARAM(height);
        GPARAM(frameRate);
        GPARAM(brightness);
        GPARAM(contrast);
        GPARAM(saturation);
        GPARAM(hue);
        GPARAM(sharpness);
        GPARAM(gain);
        GPARAM(gamma);
        GPARAM(whitebalance_auto);
        GPARAM(whitebalance_absolute);
        GPARAM(exposure_auto);
        GPARAM(exposure_absolute);
        GPARAM(focus_auto);
        GPARAM(focus_absolute);

        ROS_INFO("[dvision] Camera settings, width: %d height: %d", width, height);
    }
#undef GPARAM

    //! Device name of camera
    std::string device;
    //! Image width of camera
    int width;
    //! Image height of camera
    int height;
    //! Frame rate of camera
    int frameRate;
    //! Brightness of camera
    int brightness;
    //! Contrast of camera
    int contrast;
    //! Saturation of camera
    int saturation;
    //! Hue of camera
    int hue;
    //! Sharpness of camera
    int sharpness;
    //! Gain of camera
    int gain;
    //! Gamma of camera
    int gamma;

    //! Flag for auto white balance
    int whitebalance_auto;
    //! Mannual white balance value
    int whitebalance_absolute;
    //! Flag for auto exposure
    int exposure_auto;
    //! Mannual exposure value
    int exposure_absolute;
    //! Flag for auto focus
    int focus_auto;
    //! Mannual focus value
    int focus_absolute;
};

//! V4L2 control id set 
enum class V4L2CID
{
    white_balance_auto = V4L2_CID_AUTO_WHITE_BALANCE,
    white_balance_temperature = V4L2_CID_WHITE_BALANCE_TEMPERATURE,
    exposure_auto = V4L2_CID_EXPOSURE_AUTO,
    exposure_absolute = V4L2_CID_EXPOSURE_ABSOLUTE,
    focus_auto = V4L2_CID_FOCUS_AUTO,
    focus_absolute = V4L2_CID_FOCUS_ABSOLUTE,
    brightness = V4L2_CID_BRIGHTNESS,
    contrast = V4L2_CID_CONTRAST,
    saturation = V4L2_CID_SATURATION,
    sharpness = V4L2_CID_SHARPNESS,
    gain = V4L2_CID_GAIN,
    hue = V4L2_CID_HUE,
    gamma = V4L2_CID_GAMMA
};
} // namespace dvision
