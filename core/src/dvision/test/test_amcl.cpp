#include <ros/ros.h>
#include <gtest/gtest.h>
#include "dvision/amcl/amcl.hpp"
#include "dvision/amcl/map.hpp"

using namespace dvision;
using namespace std;

TEST(amcl, main)
{
//    ros::NodeHandle nh;
//    while (ros::ok()) {
//    }

    Map m;
    m.Init();
}

int
main(int argc, char** argv)
{
    ros::init(argc, argv, "testAmcl");
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
