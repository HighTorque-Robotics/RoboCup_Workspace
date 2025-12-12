// Created on: May 20, 2017
//     Author: Wenxing Mei <mwx36mwx@gmail.com>

#include <signal.h>

// #include "dvision/dvision.hpp"

#ifdef USE_ZED_CAMERA
#include "dvision/zed_dvision.hpp"
#endif

void signalHandler(int sig) {
  ROS_WARN("Trying to exit!");
  ros::shutdown();
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "dvision_2");
  ros::NodeHandle n("~");

  // catch external interrupt initiated by the user and exit program
  signal(SIGINT, signalHandler);

  // Start DVision node
  ROS_INFO("dvision node");
  dvision::DVision v;
  v.Start();
  v.Join();
}
