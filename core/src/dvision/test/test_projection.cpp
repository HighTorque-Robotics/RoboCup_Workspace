#include "dvision/projection.hpp"
#include "dvision/dvision.hpp"
#include "dvision/parameters.hpp"
#include <gtest/gtest.h>
#include <signal.h>

using namespace dvision;
using namespace cv;
using namespace std;

// TODO(MWX):
// 1. calibrate / optimize initial camera location and orientation
// 2. project field to image and see result


int main(int argc, char** argv)
{
    ros::init(argc, argv, "test_projection");
    ros::NodeHandle n("~");

    // dvision::DVision v;
    // v.nh_ = new ros::NodeHandle("~");
    Projection projection;
    parameters.init(&n);
    projection.init(&n);
    projection.updateExtrinsic(8,0);

    cv::Point2f realpoint;
    cv::Point imgpoint(311,138);
    cout<<"initialization done"<<endl;
    projection.getOnRealCoordinate(imgpoint,realpoint,32);
    cout<<realpoint<<endl;
    cout<<"*****ok*****"<<endl;
    return 0;
}
