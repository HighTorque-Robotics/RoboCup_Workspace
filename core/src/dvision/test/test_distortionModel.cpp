#include "dvision/v4l2_camera.hpp"
#include "dvision/distortionModel.hpp"
#include <gtest/gtest.h>
#include "dvision/parameters.hpp"

using namespace dvision;
using namespace cv;
using namespace std;

void
signalHandler(int sig)
{
    ROS_WARN("Trying to exit!");
    std::terminate();
}

TEST(distortionModel, main)
{
    ros::NodeHandle nh("~");
    signal(SIGINT, signalHandler);
    parameters.init(&nh);
    DistortionModel dist;
    dist.init();

    CameraSettings s(&nh);
    dvision::Camera c(s);
    while (ros::ok()) {
        auto frame = c.capture();
        frame.show();

        Mat dst1, dst2, dst3;
        dist.undistortImage(frame.getBGR(), dst1);
        dist.undistortImage2(frame.getBGR(), dst2);
        dist.undistortImage3(frame.getBGR(), dst3);

        namedWindow("undist1", CV_WINDOW_NORMAL);
        imshow("undist1", dst1);

        namedWindow("undist2", CV_WINDOW_NORMAL);
        imshow("undist2", dst2);

        namedWindow("undist3", CV_WINDOW_NORMAL);
        imshow("undist3", dst3);

        waitKey(1);
    }
}

int
main(int argc, char** argv)
{
    ros::init(argc, argv, "capture");

    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

//Mat cameraMatrix = (Mat_<double>(3, 3) << 360.591090231311, 0, 310.7131585594641, 0, 360.4918824799427, 252.0890520277582, 0, 0, 1);
//
//Mat distCoeff = (Mat_<double>(1, 14) << 12.17446992931163,
//    9.674181625244897,
//    -9.979700211544175e-05,
//    -0.0001482121838188539,
//    0.6561453924261769,
//    12.50596489391226,
//    13.94603005487614,
//    2.886257323576563,
//    0.001526969658511264,
//    -0.0002591965834689962,
//    0.0001776014517591451,
//    7.26029339820618e-05,
//    0.0005823374570643553,
//    0.001257762144819584);
