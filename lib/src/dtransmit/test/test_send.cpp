#include <ros/ros.h>
#include <std_msgs/String.h>
#include "dtransmit/dtransmit.hpp"

// TODO(MWX): test on different machine, over wifi
using namespace dtransmit;
using namespace std;

int main()
{
    DTransmit d;

    int NUM = 2;

    for(int i = 0 ; i < 1000; ++i) {
        std_msgs::String msg;
        std::stringstream ss;
        ss << "Hello dtransmit " << i;
        msg.data = ss.str();

        ROS_INFO("Sending: [%s]", msg.data.c_str());

        for(int j = 0; j < NUM; ++j) {
            d.sendRos<std_msgs::String>(2000 + j, msg);
        }
        usleep(1000);
    }
}