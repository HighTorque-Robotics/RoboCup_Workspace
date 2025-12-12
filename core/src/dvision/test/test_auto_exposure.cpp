/* Copyright (C) ZJUDancer
 * 2019 - Chengtao Yao <3160102438@zju.edu.cn>
 */

/**
 * @file test_auto_exposure.cpp
 * @brief Auto exposure
 * @author Chengtao Yao
 * @version 2019
 * @date 2019-05-12
 */

#include "dvision/v4l2_camera.hpp"
#include <poll.h>
#include <sys/fcntl.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/stat.h>


#include <csignal>
#include <algorithm>
#include <cstdio>
#include <dirent.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <string>

void
signalHandler(int sig)
{
    ROS_WARN("Trying to exit!");
    std::terminate();
}

cv::Mat getHistGraph(const cv::Mat image, int channels[], double* rate){
    float range[] = {0, 256};
    const float* ranges[]={range};
    const int histSize[] = {256};
    
    int sum = 0;
    int high = 0;
    double maxValue = 0;

    cv::Mat hist;
    cv::calcHist(&image, 1, channels, cv::Mat(), hist, 1, histSize, ranges, true, false);
    cv::minMaxLoc(hist, 0, &maxValue, 0, 0);
    int rows = cvRound(maxValue);
    
    // calculate hist image
    cv::Mat histImage = cv::Mat::zeros(rows, 256, CV_8UC1);
    for(int i=0; i<256; i++){
        int temp = (int)(hist.at<float>(i, 0));
        if(i>=128){
            high += temp;
        }
        sum += temp;
        if(temp){
            histImage.col(i).rowRange(cv::Range(rows-temp, rows)) = 255;
        }
    }

    // calculate rate
    *rate = 1.0*high/sum;

    // return hist image
    cv::Mat resizeImage;
    cv::resize(histImage, resizeImage, cv::Size(256, 256));
    return resizeImage;
}


int main(int argc, char** argv){
    ros::init(argc, argv, "test_autoexposure");
    ros::NodeHandle nh("~");
    signal(SIGINT, signalHandler);

    dvision::CameraSettings s(&nh);
    dvision::V4L2Camera c(s);
    int exposure[4] = {20, 130, 140, 150};
    int channels[] = {2};
    
    int count = 0;
    // int temp = 130;
    // bool flag = true;
    while (ros::ok()) {

        auto frame = c.capture();
        auto& mat = frame.bgr();
        double rate = 0;
        
        cv::Mat grayImage;
        cv::Mat hsvImage;
        cv::cvtColor(mat, grayImage, cv::COLOR_BGR2GRAY);
        cv::cvtColor(mat, hsvImage, cv::COLOR_BGR2HSV);
        cv::Mat hist = getHistGraph(hsvImage, channels, &rate);
        cv::namedWindow("test-auto-exposure", CV_WINDOW_NORMAL);
        cv::imshow("test-auto-exposure", mat);
        cv::imshow("hist", hist);
        // cv::imshow("grayImage", grayImage);
        cv::imshow("hsvImage", hsvImage);
        cv::waitKey(1);


        if(count++>=50){
            v4l2_control exposure_now;
            int now_value;
            exposure_now = c.getValue(dvision::V4L2CID::exposure_absolute);
            now_value = exposure_now.value;
            std::cout << "Now Exposure is : "<< now_value << std::endl;
            if(rate>0.06){
                if(exposure[3]<now_value){
                    c.setControl(dvision::V4L2CID::exposure_absolute, exposure[3]);
                }else{
                    for(int i=0;i<3;i++){
                        if(exposure[i]<now_value && ( exposure[i+1]>=now_value ) ){
                            c.setControl(dvision::V4L2CID::exposure_absolute, exposure[i]);
                        } 
                    }
                }
            }else if(rate<0.02){
                if(exposure[0]>now_value){
                    c.setControl(dvision::V4L2CID::exposure_absolute, exposure[3]);
                }else{
                    for(int i=0;i<3;i++){
                        if(exposure[i]<=now_value && ( exposure[i+1]>now_value ) ){
                            c.setControl(dvision::V4L2CID::exposure_absolute, exposure[i+1]);
                        } 
                    }
                }
            }

            // temp = temp >150  ? 130 : temp + 5;
            // c.setControl(dvision::V4L2CID::exposure_absolute, temp);
            std::cout << "exposure is:" << now_value << "   rate is: " << rate << std::endl;
            count = 0;
        }
    }
}





