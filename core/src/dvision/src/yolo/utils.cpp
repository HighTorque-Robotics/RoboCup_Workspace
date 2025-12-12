#include "dvision/yolo/utils.hpp"
 
 namespace dvision {

cv::Mat preprocess_img(cv::Mat& img, int input_w, int input_h) {
    int w, h, x, y;
    float r_w = input_w / (img.cols*1.0);
    float r_h = input_h / (img.rows*1.0);
    if (r_h > r_w) {
        w = input_w;
        h = r_w * img.rows;
        x = 0;
        y = (input_h - h) / 2;
    } else {
        w = r_h * img.cols;
        h = input_h;
        x = (input_w - w) / 2;
        y = 0;
    }
    cv::Mat re(h, w, CV_8UC3);
    cv::resize(img, re, re.size(), 0, 0, cv::INTER_LINEAR);
    cv::Mat out(input_h, input_w, CV_8UC3, cv::Scalar(128, 128, 128));
    re.copyTo(out(cv::Rect(x, y, re.cols, re.rows)));
    return out;
}
void draw_detections(const std::vector<Yolo::Detection>& detections, std::vector<std::string> class_labels, cv::Mat& img)
{
    for (size_t j = 0; j < detections.size(); j++) {
        cv::Rect r = get_rect(img, detections[j].bbox);
        cv::rectangle(img, r, cv::Scalar(0x27, 0xC1, 0x36), 2);
        cv::putText(img, std::to_string(detections[j].conf)+"%"+class_labels[(int)detections[j].class_id], cv::Point(r.x, r.y - 1), cv::FONT_HERSHEY_PLAIN, 1.2, cv::Scalar(0xFF, 0xFF, 0xFF), 2);
    }    
    //+class_labels[(int)detections[j].class_id]
}


void draw_detections(const std::vector<Yolo::Detection>& detections, cv::Mat& img)
{
    for (size_t j = 0; j < detections.size(); j++) {
        cv::Rect r = get_rect(img, detections[j].bbox);
        cv::rectangle(img, r, cv::Scalar(0x27, 0xC1, 0x36), 2);
        cv::putText(img, std::to_string(detections[j].conf), cv::Point(r.x, r.y - 1), cv::FONT_HERSHEY_PLAIN, 1.2, cv::Scalar(0xFF, 0xFF, 0xFF), 2);
    }
}


bool read_text_file(std::vector<std::string>& names, std::string filename)
{
    // Expand user path
    if (filename.find("~") != std::string::npos) {
      filename = boost::regex_replace(filename, boost::regex("~"), std::string(std::getenv("HOME")));
    }

    std::ifstream infile(filename);

    if (!infile) {
        std::cerr << "Unable to read binary file: " << filename << std::endl;
        return false;
    }

    std::string name;
    while (std::getline(infile, name)) {
        names.push_back(name);
    }

    return true;
}

int read_files_in_dir(const char *p_dir_name, std::vector<std::string> &file_names) {
    DIR *p_dir = opendir(p_dir_name);
    if (p_dir == nullptr) {
        return -1;
    }

    struct dirent* p_file = nullptr;
    while ((p_file = readdir(p_dir)) != nullptr) {
        if (strcmp(p_file->d_name, ".") != 0 &&
            strcmp(p_file->d_name, "..") != 0) {
            //std::string cur_file_name(p_dir_name);
            //cur_file_name += "/";
            //cur_file_name += p_file->d_name;
            std::string cur_file_name(p_file->d_name);
            file_names.push_back(cur_file_name);
        }
    }

    closedir(p_dir);
    return 0;
}

 }//dvision namespace
