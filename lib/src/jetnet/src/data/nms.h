#ifndef JETNET_NMS_H
#define JETNET_NMS_H

#include "detection.h"
#include <opencv2/opencv.hpp>
#include <vector>
#include <functional>

namespace jetnet
{
    typedef std::function<void(std::vector<Detection>&)> NmsFunction;

    /*
     *  Calculate box IOU
     */
    float box_iou(cv::Rect2f a, cv::Rect2f b);

    /*
     *  Basic non-maxima suppression based on IOU
     *  No sorting is done on box score
     *  threshold:  IOU threshold
     */
    void nms(std::vector<Detection>& detections, float thresh);

    /*
     *  Basic non-maxima suppression based on IOU
     *  Sorting is done on box score which makes it a bit slower
     *  threshold:  IOU threshold
     */
    void nms_sort(std::vector<Detection>& detections, float thresh);
}

#endif /* JETNET_NMS_H */
