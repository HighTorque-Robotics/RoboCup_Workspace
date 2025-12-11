#ifndef JETNET_DETECTION_H
#define JETNET_DETECTION_H

#include <opencv2/opencv.hpp>
#include <string>

namespace jetnet
{

struct Detection
{
    cv::Rect2f bbox;                        // x,y are top left and clipped to original image boundaries
    std::vector<float> probabilities;       // all probabilities for each class
};

struct BBox{
  cv::Point left_top;
  cv::Point right_bottom;
  int class_label_index;
  float class_label_prob;
};

}

#endif /* JETNET_DETECTION_H */
