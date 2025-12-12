/* Copyright (C) ZJUDancer
 * 2018 - Yusu Pan <xxdsox@gmail.com>
 * 2017 - Wenxing Mei <mwx36mwx@gmail.com>
 */

/**
 * \file calibration.cpp
 * \author Yusu Pan, Wenxing Mei
 * \version 2018
 * \date 2018-02-14
 */

#include <algorithm>
#include <cstdio>
#include <dirent.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <string>

const int BOARDWIDTH = 11;
const int BOARDHEIGHT = 8;
float SQUARESIZE = 30;

using namespace std;
using namespace cv;

struct CalibSettings
{
    int getFlag()
    {
        int flag = 0;
        flag |= CV_CALIB_RATIONAL_MODEL;
        flag |= CV_CALIB_THIN_PRISM_MODEL;
        flag |= CALIB_TILTED_MODEL;
        return flag;
    }

    Size getBoardSize()
    {
        return Size(BOARDWIDTH, BOARDHEIGHT);
    }

    float getSquareSize()
    {
        return SQUARESIZE;
    }
};

CalibSettings s;

static double
computeReprojectionErrors(const vector<vector<Point3f>>& objectPoints,
                          const vector<vector<Point2f>>& imagePoints,
                          const vector<Mat>& rvecs,
                          const vector<Mat>& tvecs,
                          const Mat& cameraMatrix,
                          const Mat& distCoeffs,
                          vector<float>& perViewErrors)
{
    vector<Point2f> imagePoints2;
    int i, totalPoints = 0;
    double totalErr = 0, err;
    perViewErrors.resize(objectPoints.size());

    for (i = 0; i < (int)objectPoints.size(); ++i) {
        projectPoints(Mat(objectPoints[i]), rvecs[i], tvecs[i], cameraMatrix, distCoeffs, imagePoints2);
        err = norm(Mat(imagePoints[i]), Mat(imagePoints2), NORM_L2);

        int n = (int)objectPoints[i].size();
        perViewErrors[i] = (float)sqrt(err * err / n);
        std::cout << "picture: " << i << " err: " << err << " pixel(s)" << std::endl;
        totalErr += err * err;
        totalPoints += n;
    }
    std::cout << "total error:" << totalErr << " and " << (totalErr / totalPoints) << std::endl;
    return sqrt(totalErr / totalPoints);
}

static void
calcBoardCornerPositions(Size boardSize, float squareSize, vector<Point3f>& corners)
{
    corners.clear();
    for (int i = 0; i < boardSize.height; ++i)
        for (int j = 0; j < boardSize.width; ++j)
            corners.push_back(Point3d(j * squareSize, i * squareSize, 0));
}

static bool
runCalibration(Size& imageSize, Mat& cameraMatrix, Mat& distCoeffs, vector<vector<Point2f>> imagePoints, vector<Mat>& rvecs, vector<Mat>& tvecs, vector<float>& reprojErrs, double& totalAvgErr)
{
    cameraMatrix = Mat::eye(3, 3, CV_64F);
    if (s.getFlag() & CALIB_FIX_ASPECT_RATIO) {
        cameraMatrix.at<double>(0, 0) = 1.0;
    }
    distCoeffs = Mat::zeros(14, 1, CV_64F);
    vector<vector<Point3f>> objectPoints(1);
    calcBoardCornerPositions(s.getBoardSize(), s.getSquareSize(), objectPoints[0]);
    objectPoints.resize(imagePoints.size(), objectPoints[0]);
    // Find intrinsic and extrinsic camera parameters
    double rms = calibrateCamera(objectPoints, imagePoints, imageSize, cameraMatrix, distCoeffs, rvecs, tvecs, s.getFlag());
    cout << "Re-projection error reported by calibrateCamera: " << rms << endl;
    bool ok = checkRange(cameraMatrix) && checkRange(distCoeffs);
    totalAvgErr = computeReprojectionErrors(objectPoints, imagePoints, rvecs, tvecs, cameraMatrix, distCoeffs, reprojErrs);
    return ok;
}

bool
runCalibrationAndSave(Size imageSize, Mat& cameraMatrix, Mat& distCoeffs, vector<vector<Point2f>> imagePoints)
{
    cout << "Start calibration" << endl;
    vector<Mat> rvecs, tvecs;
    vector<float> reprojErrs;
    double totalAvgErr = 0;
    bool ok = runCalibration(imageSize, cameraMatrix, distCoeffs, imagePoints, rvecs, tvecs, reprojErrs, totalAvgErr);
    cout << (ok ? "Calibration succeeded" : "Calibration failed") << ". avg re projection error = " << totalAvgErr << endl;
    return ok;
}

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
                } else {
                    nPos = tmpFileName.find(".jpg");
                    if (nPos != string::npos)
                        imagesName.push_back(path + '/' + tmpFileName);
                }
            }
        }
        closedir(dir);
    }
    return imagesName;
}

int
main(int argc, char** argv)
{
    if (argc < 2) {
        cout << "Usage: " << argv[0] << " <pic path> [square size(mm)]" << endl;
        return 0;
    }
    if (argc == 3) {
        std::string size;
        size = argv[2];
        SQUARESIZE = std::stof(size);
    }

    string pathDirectory = argv[1];
    auto imagesName = getImageList(pathDirectory);

    vector<vector<Point2f>> imagePoints;
    Mat cameraMatrix, distCoeffs;
    Size imageSize;
    for (auto image_name : imagesName) {
        Mat view;
        view = imread(image_name.c_str());

        imageSize = view.size();
        vector<Point2f> pointBuf;
        bool found = findChessboardCorners(view, s.getBoardSize(), pointBuf, CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_FAST_CHECK | CALIB_CB_NORMALIZE_IMAGE);
        if (found) {
            Mat viewGray;
            cvtColor(view, viewGray, COLOR_BGR2GRAY);
            cornerSubPix(viewGray, pointBuf, Size(11, 11), Size(-1, -1), TermCriteria(TermCriteria::EPS + TermCriteria::COUNT, 30, 0.1));
            imagePoints.push_back(pointBuf);
            drawChessboardCorners(view, s.getBoardSize(), Mat(pointBuf), found);
            cout << image_name << endl;
            namedWindow("image", CV_WINDOW_NORMAL);
            imshow("image", view);
            cvWaitKey(0);
        } else {
            cout << image_name << " found corner failed! & removed!" << endl;
        }
    }

    runCalibrationAndSave(imageSize, cameraMatrix, distCoeffs, imagePoints);

    cout << "-------------cameraMatrix--------------" << endl;
    cout << cameraMatrix.size() << endl;
    cout << cameraMatrix << endl;
    printf("fx:                    %.13lf\nfy:                    %.13lf\ncx:                    %.13lf\ncy:                    %.13lf\n",
           cameraMatrix.at<double>(0, 0),
           cameraMatrix.at<double>(1, 1),
           cameraMatrix.at<double>(0, 2),
           cameraMatrix.at<double>(1, 2));

    cout << "---------------distCoeffs--------------" << endl;
    cout << distCoeffs.size() << endl;
    // cout << distCoeffs << endl;
    cout << "[";
    for (int i = 0; i < distCoeffs.rows; i++) {
        cout << distCoeffs.at<double>(i, 0);
        if (i != distCoeffs.rows - 1)
            cout << "," << endl;
    }
    cout << "]" << endl;

    for (auto image_name : imagesName) {
        Mat view = imread(image_name.c_str());
        Mat temp = view.clone();
        undistort(temp, view, cameraMatrix, distCoeffs);
        namedWindow("undist", CV_WINDOW_NORMAL);
        imshow("undist", view);
        waitKey(0);
    }

    return 0;
}
