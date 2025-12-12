
#include "dvision/yolo/yolov5.hpp"

#define USE_FP16  // set USE_INT8 or USE_FP16 or USE_FP32
#define DEVICE 0  // GPU id
#define NMS_THRESH 0.4
#define CONF_THRESH 0.5
#define BATCH_SIZE 1

// stuff we know about the network and the input/output blobs
using namespace dvision;

int main(int argc, char** argv){
    cudaSetDevice(DEVICE);

    std::string engine_name = "/home/nvidia/new-yolov5-test/build/yolov5s.engine";
    std::string img_dir;
    img_dir="/home/nvidia/yolo-test/src/yolo/samples";
    yolov5 v5;
    v5.init(engine_name,CONF_THRESH,NMS_THRESH,BATCH_SIZE);
    std::vector<std::string> file_names;
    if (read_files_in_dir(img_dir.c_str(), file_names) < 0) {
        std::cerr << "read_files_in_dir failed." << std::endl;
    }
    std::vector<cv::Mat> images;
    int fcount=0;
    for (int f = 0; f < (int)file_names.size(); f++) {
        cv::Mat img = cv::imread(img_dir + "/" + file_names[f]);
        if (img.empty()) continue;
        if (images.size()<BATCH_SIZE){
            images.push_back(img);
            fcount++;
            if(images.size()<BATCH_SIZE)
            continue;
        }
            
        auto batch_res=v5.detect(images);
        for (int b = 0; b < fcount; b++) {
            auto& res = batch_res[b];
            std::cout << res.size() << std::endl;
            cv::Mat img = cv::imread(img_dir + "/" + file_names[f - fcount + 1 + b]);
            for (size_t j = 0; j < res.size(); j++) {
                cv::Rect r = get_rect(img, res[j].bbox);
                cv::rectangle(img, r, cv::Scalar(0x27, 0xC1, 0x36), 2);
                cv::putText(img, std::to_string(res[j].conf), cv::Point(r.x, r.y - 1), cv::FONT_HERSHEY_PLAIN, 1.2, cv::Scalar(0xFF, 0xFF, 0xFF), 2);
            }
            cv::imwrite("_" + file_names[f - fcount + 1 + b], img);
        }
        images.clear();
        fcount = 0;
    }
    
    std::cout<<"ok"<<std::endl;

    return 0;
}
