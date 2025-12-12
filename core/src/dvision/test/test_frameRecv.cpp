#include "dvision/v4l2_camera.hpp"
#include "dtransmit/dtransmit.hpp"
#include "dconfig/dconstant.hpp"

using namespace dvision;
using namespace cv;
using namespace std;

int main(int argc, char **argv) {
    ros::init(argc, argv, "frameRecv");
    ros::NodeHandle nh;
    dtransmit::DTransmit recv("192.168.255.255");

    Frame::initEncoder();
    recv.addRawRecv(dconstant::network::robotGuiBase + 1, [&](void* buffer, std::size_t size) {
        Frame f;
        try {
            f.decode(buffer);
            f.show();
            waitKey(1);
        } catch (std::exception& e) {
            cerr << e.what() << endl;
        }
    });
    recv.startService();

    ros::spin();
}

