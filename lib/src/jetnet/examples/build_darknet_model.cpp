/*
 *  Darknet model builder
 *  copyright EAVISE
 *  author: Maarten Vandersteegen
 *
 */
#include "jetnet.h"
#include "create_builder.h"
#include <opencv2/opencv.hpp>   // for commandline parser
#include <algorithm>
#include <boost/algorithm/string.hpp>

using namespace jetnet;

static bool parse_layer_names(std::string input, std::vector<std::string>& out)
{
    std::string name;
    bool res = false;

    if (input[0] =='~') {
        res = true;
        input.erase(0, 1);
    }

    boost::split(out, input, [](char c){return c == ',';});

    return res;
}

int main(int argc, char** argv)
{
    std::string keys =
        "{help h usage ? |      | print this message                              }"
        "{@modelname     |<none>| model name to build                             }"
        "{@weightsfile   |<none>| darknet weights file                            }"
        "{@planfile      |<none>| serializes GIE output file                      }"
        "{fp16           |      | optimize for FP16 precision (FP32 by default)   }"
        "{int8calfiles   |      | INT8 optimization calibration file list.        }"
        "{int8cache      |      | INT8 cache file. This file is network dependent }"
        "{int8batch      | 50   | Batch size for INT8 calibration procedure       }"
        "{width          | 416  | network input width in pixels                   }"
        "{height         | 416  | network input height in pixels                  }"
        "{classes        | 80   | number of network output classes                }"
        "{dla            |      | Enable building for execution on dla            }"
        "{dladevice      | 0    | DLA device id to build for                      }"
        "{maxbatch       | 1    | maximum batch size the network must handle      }"
        "{floatlayers    |      | CSV list of layer names which must run as floats }"
        "{halflayers     |      | CSV list of layer names which must run as half floats }";

    cv::CommandLineParser parser(argc, argv, keys);
    parser.about("Jetnet darknet model builder");

    if (parser.has("help")) {
        parser.printMessage();
        return -1;
    }

    auto model_name = parser.get<std::string>("@modelname");
    auto weights_file = parser.get<std::string>("@weightsfile");
    auto output_file = parser.get<std::string>("@planfile");
    auto float_16_opt = parser.has("fp16");
    auto int8_cal_file_list = parser.get<std::string>("int8calfiles");
    auto int8_cache_file = parser.get<std::string>("int8cache");
    auto int8_batch_size = parser.get<int>("int8batch");
    auto input_width = parser.get<int>("width");
    auto input_height = parser.get<int>("height");
    auto num_classes = parser.get<int>("classes");
    auto use_dla = parser.has("dla");
    auto dla_id = parser.get<int>("dladevice");
    auto max_batch_size = parser.get<int>("maxbatch");
    auto float_layers = parser.get<std::string>("floatlayers");
    auto half_layers = parser.get<std::string>("halflayers");

    if (!parser.check()) {
        parser.printErrors();
        return 0;
    }

    DarknetBuilderFactory builder_fact(input_width, input_height, num_classes);

    auto builder = builder_fact.create(model_name, weights_file);

    if (!builder) {
        std::cerr << "Failed to create model builder" << std::endl;
        return -1;
    }

    auto logger = std::make_shared<Logger>(nvinfer1::ILogger::Severity::kINFO);
    if (!builder->init(logger)) {
        std::cerr << "Failed to initialize model builder" << std::endl;
        return -1;
    }

    nvinfer1::DataType weights_datatype = nvinfer1::DataType::kFLOAT;

    if (float_16_opt) {
        if (!builder->platform_supports_fp16()) {
            std::cerr << "Platform does not support FP16 optimization" << std::endl;
            return -1;
        }
        std::cout << "Building for inference with FP16 kernels and paired image mode" << std::endl;
        weights_datatype = nvinfer1::DataType::kHALF;

        // in case batch > 1, this will improve speed
        builder->platform_set_fp16_mode();
    }

    std::shared_ptr<LetterboxInt8Calibrator> calibrator;

    if (!int8_cal_file_list.empty() || !int8_cache_file.empty()) {
        if (!builder->platform_supports_int8()) {
            std::cerr << "Platform does not support INT8 optimization" << std::endl;
            return -1;
        }
        std::cout << "Building for inference with INT8 kernels" << std::endl;

        // reading calibration set file paths
        std::vector<std::string> image_paths;
        if (!int8_cal_file_list.empty() && !read_text_file(image_paths, int8_cal_file_list)) {
            std::cerr << "Failed to read calibration file list: " << int8_cal_file_list << std::endl;
            return -1;
        }

        calibrator = std::make_shared<LetterboxInt8Calibrator>(image_paths, int8_cache_file, logger,
                                                               std::vector<unsigned int>{0, 1, 2},
                                                               nvinfer1::DimsCHW{3, input_height, input_width},
                                                               int8_batch_size);

        builder->platform_set_int8_mode(calibrator.get());
    }

    if (use_dla) {
        std::cout << "Building for execution on DLA" << dla_id << std::endl;
        builder->platform_use_dla(dla_id);
    }

    if (!float_layers.empty() || !half_layers.empty()) {
        std::cout << "Enabling type strictness" << std::endl;
        builder->enable_type_strictness();
    }

    std::cout << "Parsing the network..." << std::endl;
    if (builder->parse(weights_datatype) == nullptr) {
        std::cerr << "Failed to parse network" << std::endl;
        return -1;
    }

    /* retarget individual layers to float precision */
    if (!float_layers.empty()) {
        std::vector<std::string> layers;
        bool invert = parse_layer_names(float_layers, layers);
        builder->set_layer_precision(layers, nvinfer1::DataType::kFLOAT, invert);
    }

    /* retarget individual layers to half float precision */
    if (!half_layers.empty()) {
        std::vector<std::string> layers;
        bool invert = parse_layer_names(half_layers, layers);
        builder->set_layer_precision(layers, nvinfer1::DataType::kHALF, invert);
    }

    std::cout << "Building the network..." << std::endl;
    if (builder->build(max_batch_size) == nullptr) {
        std::cerr << "Failed to build network" << std::endl;
        return -1;
    }

    std::cout << "Serializing to file..." << std::endl;
    if (builder->serialize(output_file) == nullptr) {
        std::cerr << "Failed to serialize network" << std::endl;
        return -1;
    }

    std::cout << "Successfully built model" << std::endl;

    return 0;
}
