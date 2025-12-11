#include <ros/ros.h>
#include <std_msgs/String.h>
#include "dtransmit/dtransmit.hpp"

using namespace dtransmit;
using namespace std;

int main()
{
    DTransmit d;
    int NUM = 2;

    int cnt = 0;
    for(int i = 0; i < NUM; ++i) {
        d.addRosRecv<std_msgs::String>(2000 + i, [=, &cnt](std_msgs::String &msg) {
            ROS_INFO("%d heard: [%s]", 2000 + i, msg.data.c_str());
            ++cnt;
        });
    }

    d.startService();

    while(cnt < 1000) {
        usleep(1000);
    }
}