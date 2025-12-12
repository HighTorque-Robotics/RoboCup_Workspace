#pragma once
#ifndef YOLOV5
#define YOLOV5

#include <iostream>
#include <chrono>
#include <cmath>
#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include "cuda_utils.h"
#include "logging.h"
#include "common.hpp"
#include "utils.hpp"

namespace dvision {

#define USE_FP16  // set USE_INT8 or USE_FP16 or USE_FP32
#define DEVICE 0  // GPU id
#define BATCH_SIZE 1


class yolov5
{
private:
    static float data[BATCH_SIZE * 3 * Yolo::INPUT_H * Yolo::INPUT_W];
    float prob[BATCH_SIZE * (Yolo::MAX_OUTPUT_BBOX_COUNT * sizeof(Yolo::Detection) / sizeof(float) + 1)];
    int inputIndex;
    int outputIndex;
    void* buffers[2];
    float threshold;
    float nms_threshold;
    int batch_size;
    IRuntime* runtime = nullptr;
    ICudaEngine* engine = nullptr;
    IExecutionContext* context = nullptr;
    cudaStream_t stream;
    std::vector<std::vector<Yolo::Detection>> batch_res;
    
public:
    void init(std::string engine_name,float threshold, float nms_threshold, int batch_size);
    yolov5() = default;
    ~yolov5();
    void doInference(IExecutionContext& context, cudaStream_t& stream, void **buffers, float* input, float* output, int batchSize);
    std::vector<std::vector<Yolo::Detection>> detect(std::vector<cv::Mat> images);


// yolov5::yolov5(std::string engine_name, float threshold_, float nms_threshold_, int batch_size_);

// yolov5::~yolov5();

// void yolov5::doInference(IExecutionContext& context, cudaStream_t& stream, void **buffers, float* input, float* output, int batchSize) ;

// std::vector<std::vector<Yolo::Detection>> yolov5::detect(std::vector<cv::Mat> images);
};

}

#endif