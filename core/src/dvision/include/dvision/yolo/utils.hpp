
#ifndef TRTX_YOLOV5_UTILS_H_
#define TRTX_YOLOV5_UTILS_H_

#include <opencv2/opencv.hpp>
#include <boost/regex.hpp>
#include <dirent.h>
#include <fstream> 
#include "yololayer.h"
#include "common.hpp"

namespace dvision {

struct BBox{
  cv::Point left_top;
  cv::Point right_bottom;
  int class_label_index;
  float class_label_prob;
};

cv::Mat preprocess_img(cv::Mat& img, int input_w, int input_h);

void draw_detections(const std::vector<Yolo::Detection>& detections, std::vector<std::string> class_labels, cv::Mat& image);

void draw_detections(const std::vector<Yolo::Detection>& detections, cv::Mat& image);

bool read_text_file(std::vector<std::string>& names, std::string filename);

int read_files_in_dir(const char *p_dir_name, std::vector<std::string> &file_names);

}
#endif  // TRTX_YOLOV5_UTILS_H_

