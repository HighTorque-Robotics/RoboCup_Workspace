#ifndef JETNET_VISUAL_H
#define JETNET_VISUAL_H

#include "detection.h"
#include <opencv2/opencv.hpp>

namespace jetnet
{
    /*
     *  Draw detections on an image
     *  detections:     list of detection boxes
     *  image:          the image to draw the detections on
     */
    void draw_detections(const std::vector<Detection>& detections, std::vector<std::string> class_labels, cv::Mat& image);
}

#endif /* JETNET_VISUAL_H */
