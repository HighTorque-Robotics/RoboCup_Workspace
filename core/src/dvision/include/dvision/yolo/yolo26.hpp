#pragma once

#include <iostream>
#include <chrono>
#include <cmath>
#include <algorithm>
#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include "cuda_utils.h"
#include "logging.h"
#include "common.hpp"

namespace dvision {

class yolo26
{
public:
    static constexpr int INPUT_H = 640;
    static constexpr int INPUT_W = 640;
    static constexpr int CLASS_NUM = 8;
    static constexpr int NUM_ANCHORS = 8400;

    void init(std::string engine_name, float threshold, float nms_threshold, int batch_size);
    yolo26() = default;
    ~yolo26();
    void doInference(IExecutionContext& context, cudaStream_t& stream, void **buffers, float* input, float* output, int batchSize);
    std::vector<std::vector<Yolo::Detection>> detect(std::vector<cv::Mat> images);

private:
    float* data = nullptr;
    float* prob = nullptr;
    void* buffers[2];
    int inputIndex = 0;
    int outputIndex = 1;
    float threshold;
    float nms_threshold;
    int batch_size;
    IRuntime* runtime = nullptr;
    ICudaEngine* engine = nullptr;
    IExecutionContext* context = nullptr;
    cudaStream_t stream;

    std::vector<Yolo::Detection> decodeOutput(const float* output, int img_w, int img_h);
};

}
