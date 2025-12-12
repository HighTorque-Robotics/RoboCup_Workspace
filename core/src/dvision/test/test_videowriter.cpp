#include "dvision/v4l2_camera.hpp"
#include <ros/ros.h>
#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;
int main(int argc, char** argv) {
    ros::init(argc, argv, "videowriter");
    ros::NodeHandle nh;
    dvision::Camera c;
    auto frame = c.capture();
    Mat& src = frame.getBGR();

    bool isColor = (src.type() == CV_8UC3);
    double fps = 25.0;
    string filename = "./live.avi";

    VideoWriter writer;
    int codec = CV_FOURCC('X', '2', '6', '4');

    writer.open(filename, codec, fps, src.size(), isColor);
    if(!writer.isOpened()) {
        cerr << "Can't open output video file for write\n";
        return -1;
    }

    while(true) {
        auto frame = c.capture();
        writer.write(frame.getBGR());
    }

}