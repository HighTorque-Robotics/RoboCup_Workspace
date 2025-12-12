/**
 * @Author: Yusu Pan <yuthon>
 * @Date:   2017-06-04T10:29:30+08:00
 * @Email:  xxdsox@gmail.com
 * @Project: humanoid
 * @Filename: test_localization.cpp
 * @Last modified by:   yuthon
 * @Last modified time: 2017-06-04T10:29:52+08:00
 * @Copyright: ZJUDancer
 */

#include "dvision/dvision.hpp"
#include <gtest/gtest.h>

using namespace dvision;

TEST(Localization, main)
{

    ros::NodeHandle nh;
    dvision::DVision v(&nh);

    // while (ros::osk()) {
    v.tick();
    // }
}

int
main(int argc, char** argv)
{
    ros::init(argc, argv, "localization");
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
