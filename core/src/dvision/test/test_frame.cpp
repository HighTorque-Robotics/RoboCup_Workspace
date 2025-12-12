#include "dvision/v4l2_camera.hpp"
#include <iostream>
#include "dvision/distortionModel.hpp"

using namespace dvision;
using namespace cv;
using namespace std;

int main(int argc, char **argv) {
    ros::init(argc, argv, "capture");
    ros::NodeHandle nh;
    parameters.init(&nh);
    DistortionModel dist;
    dist.init();

    dvision::Camera c;
    Frame::initEncoder();

    while (ros::ok())
    {
        auto frame = c.capture();
        int len;
        auto buf = frame.encode(len);

        Frame f;
        f.decode(buf.get());
        f.show();
        frame.show();
        waitKey(1);
    }
}

