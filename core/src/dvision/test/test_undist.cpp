#include "dvision/v4l2_camera.hpp"
#include "dvision/distortionModel.hpp"
#include <gtest/gtest.h>
#include <dirent.h>
#include "dvision/parameters.hpp"

using namespace dvision;
using namespace cv;
using namespace std;

vector<string> imglist;

vector<string>
getImageList(string path)
{
    vector<string> imagesName;
    DIR* dir;
    struct dirent* ent;
    if ((dir = opendir(path.c_str())) != NULL) {
        while ((ent = readdir(dir)) != NULL) {
            string tmpFileName = ent->d_name;
            if (tmpFileName.length() > 4) {
                auto nPos = tmpFileName.find(".png");
                if (nPos != string::npos) {
                    imagesName.push_back(path + '/' + tmpFileName);
                }
            }
        }
        closedir(dir);
    }
    return imagesName;
}
TEST(testUndist, main)
{
    ros::NodeHandle nh;
    parameters.init(&nh);
    DistortionModel dist;
    dist.init();

    int cnt = 0;
    for(auto name : imglist) {
        auto img = imread(name);
        Mat undist;
        dist.undistortImage(img, undist);

        imwrite(name + to_string(cnt++) + ".png", undist);
    }
}

int
main(int argc, char** argv)
{
    if (argc != 2) {
        cout << "Usage: " << argv[0] << " <pic path>" << endl;
        return 0;
    }

    string pathDirectory = argv[1];
    imglist = getImageList(pathDirectory);

    ros::init(argc, argv, "capture");

    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

