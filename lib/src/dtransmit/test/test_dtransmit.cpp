#include <ros/ros.h>
#include <std_msgs/String.h>
#include "dtransmit/dtransmit.hpp"

// TODO(MWX): test on different machine, over wifi
using namespace dtransmit;
using namespace std;
int main()
{
    DTransmit d;
    int portBase = 26334;

    struct Foo {
        int x = 1;
        double y = 2;
        string s = "hello";
    } foo;
    int NUM = 10;

    for(int i = 0; i < NUM; ++i) {
        d.addRosRecv<std_msgs::String>(portBase + i, [=](std_msgs::String &msg) {
            //ROS_INFO("%d heard: [%s]", portBase + i, msg.data.c_str());
        });
    }

    bool recv = false;
    d.addRawRecv(2333, [&](const void* buffer, std::size_t size) {
        if(size != sizeof(foo)) {
            throw std::runtime_error("Received raw size not correct");
        }
        Foo* f = (Foo*) buffer;
        cout << buffer << endl;
        printf("recv: %d %lf %s", f->x, f->y, f->s.c_str());
        recv = true;
    });

    d.startService();

    for(int i = 0 ; i < 10 ; ++i) {
        std_msgs::String msg;
        std::stringstream ss;
        ss << "Hello dtransmit " << i;
        msg.data = ss.str();
//        ROS_INFO("Sending: [%s]", msg.data.c_str());
        for(int j = 0; j < NUM; ++j) {
            d.sendRos<std_msgs::String>(portBase + j, msg);
        }
    }

    // test send raw
    d.sendRaw(2333, (void*)&foo, sizeof(foo));

    // wait until all msgs are receiveed
    while(!recv) {
        sleep(1);
    }

}
