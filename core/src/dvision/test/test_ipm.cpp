#include "dvision/ipm.hpp"
#include <gtest/gtest.h>
#include <iostream>

using namespace std;
using namespace dvision;
using namespace cv;
TEST(ipm, main) {

    std::vector<double> extrinsic_para = {
//        0,      0,50,       0,       0,       0,       0,       0,    10,      0,       0,       0,    1,   1,    0,    0
        5.4525,
        0.7083,
        49.5929,

        -0.0060,
        -0.3331,
        0.0072,

        2.4806,
        -0.6095,
        6.0255,

        -0.0002,
        0.0261,
        -0.0313,

        0.9839,
        1.1817,
        -0.0056,
        0.0633,
    };

    double fx = 360.591090231311;
    double fy = 360.4918824799427;
    double cx = 624.7131585594641;
    double cy = 496.0890520277582;

    dvision::IPM ipm;
    ipm.Init(extrinsic_para, fx, fy, cx, cy);

    ipm.updateDeg(60, 0);
    Point2d foo = ipm.project(10, 10);
    cout << foo << endl;
    EXPECT_NEAR(foo.x, 560.264, 0.01);
    EXPECT_NEAR(foo.y, 489.367, 0.01);

    Point2d bar = ipm.inverseProject(560, 491);
    cout << bar << endl;
    EXPECT_NEAR(bar.x, 9.78758, 0.01);
    EXPECT_NEAR(bar.y, 10.0394, 0.01);
}

int
main(int argc, char** argv)
{
    ros::init(argc, argv, "localization");
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
