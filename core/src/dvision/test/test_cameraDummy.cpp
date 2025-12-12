#include "dvision/v4l2_camera.hpp"
#include "dvision/image_folder.hpp"
#include "dvision/distortionModel.hpp"
#include <gtest/gtest.h>
#include "dvision/parameters.hpp"

using namespace dvision;
using namespace cv;
using namespace std;

TEST(cameraDummy, main)
{
    ros::NodeHandle nh;
    parameters.init(&nh);
    DistortionModel dist;
    dvision::CameraDummy c("/home/yyj/Pictures/calibrationimage/");
    while (ros::ok()) {
        auto frame = c.capture();
         cv::imshow("undistorted",frame.getBGR());
        // auto frame = c.capture();
        // frame.show();
        //
        // Mat dst1, dst2;
        // dist.undistortImage(frame.getRGB(), dst1);
        // dist.undistortImage2(frame.getRGB(), dst2);
        //
        // namedWindow("undist1", CV_WINDOW_NORMAL);
        // imshow("undist1", dst1);
        //
        // namedWindow("undist2", CV_WINDOW_NORMAL);
        // imshow("undist2", dst2);
        waitKey(300);
    }
}

int
main(int argc, char** argv)
{
    ros::init(argc, argv, "capture");

    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
