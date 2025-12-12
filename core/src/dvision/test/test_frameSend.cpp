#include "dvision/v4l2_camera.hpp"
#include "dconfig/dconstant.hpp"
#include "dtransmit/dtransmit.hpp"
#include "dvision/distortionModel.hpp"

using namespace dvision;
using namespace cv;
using namespace std;

int main(int argc, char **argv) {
    ros::init(argc, argv, "frameSend");
    ros::NodeHandle nh;

    dvision::Camera c;
    Frame::initEncoder();
    dtransmit::DTransmit sender("192.168.255.255");

    while (ros::ok())
    {
        auto frame = c.capture();
        int len;
        auto buf = frame.encode(len);
        sender.sendRaw(dconstant::network::robotGuiBase + 1, buf.get(), len);
    }
}
