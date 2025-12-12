#include "dvision/amcl/types.hpp"
#include "dvision/kalman.hpp"
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#define TWO_PI 6.2831853071795864769252866
#include <fstream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

double
generateGaussianNoise()
{
    static bool hasSpare = false;
    static double rand1, rand2;

    if (hasSpare) {
        hasSpare = false;
        return sqrt(rand1) * sin(rand2);
    }

    hasSpare = true;

    rand1 = rand() / ((double)RAND_MAX);
    if (rand1 < 1e-100)
        rand1 = 1e-100;
    rand1 = -2 * log(rand1);
    rand2 = (rand() / ((double)RAND_MAX)) * TWO_PI;

    return sqrt(rand1) * cos(rand2);
}

int
main(int argc, char** argv)
{
    ros::init(argc, argv, "kalman");
    ros::NodeHandle nh("~");

    double r;
    srand((unsigned)time(NULL));
    cv::Point2f p(200, 200), result1, result2;
    dvision::ObjectPosKalmanFilter kalman1(p), kalman2(p);
    dvision::Control u1, u2;
    std::ofstream now("/home/daiz/kalman.csv");

    ros::Rate rate(30);
    now << "z,x,x^,x^add_u" << std::endl;
    now.flush();
    u1.dx = 0;
    u1.dy = 0;
    u2.dx = 1;
    u2.dy = 1;
    while (ros::ok()) {
        int rr = rand() % 21 - 10;
        p.x += rr;
        p.y += rr;
        u2.dx = -rr;
        u2.dy = -rr;
        cv::Point2f pp = p;

        r = 10 * generateGaussianNoise(); // rand() % 21 - 10;
        r = round(r);
        pp.x += r;
        pp.y += r;
        kalman1.Update(pp, u1);
        result1 = kalman1.GetResult();
        kalman2.Update(pp, u2);
        result2 = kalman2.GetResult();
        now << pp.x << "," << p.x << "," << result1.x << "," << result2.x << std::endl;
        now.flush();
        // p.x -= 1;
        // p.y -= 1;
        rate.sleep();
    }
}
