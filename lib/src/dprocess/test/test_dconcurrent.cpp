#include "dprocess/dconcurrent.hpp"
#include <gtest/gtest.h>
#include <iostream>
#include <ros/ros.h>

using namespace dprocess;
using namespace std;

TEST(dconcurrent, main)
{
    DConcurrent c;

    c.push([]() { usleep(1000); });

    c.push([]() { usleep(1000); });

    // runs in 119 ms
    for (int i = 0; i < 100; ++i) {
        c.spinOnce();
        c.join();
    }

    // runs in 221 ms
    //  for(int i = 0; i < 100; ++i) {
    //    usleep(1000);
    //    usleep(1000);
    //  }
}

int
main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}