/*
 *  Yolo model runner
 *  copyright EAVISE
 *  author: Maarten Vandersteegen
 *
 */
#include <opencv2/opencv.hpp>   // for commandline parser
#include "jetnet.h"
#include "create_runner.h"

using namespace jetnet;

int main(int argc, char** argv)
{
    std::string keys =
        "{help h usage ? |      | print this message                        }"
        "{@type          |<none>| Network type (yolov2, yolov3)             }"
        "{@modelfile     |<none>| Built and serialized TensorRT model file  }"
        "{@nameslist     |<none>| Class names list file                     }"
        "{@inputimage    |<none>| Input RGB image                           }"
        "{profile        |      | Enable profiling                          }"
        "{t thresh       | 0.24 | Detection threshold                       }"
        "{nt nmsthresh   | 0.45 | Non-maxima suppression threshold          }"
        "{batch          | 1    | Batch size                                }"
        "{anchors        |      | Anchor prior file name                    }";

    cv::CommandLineParser parser(argc, argv, keys);
    parser.about("Jetnet YOLO runner");

    if (parser.has("help")) {
        parser.printMessage();
        return -1;
    }

    auto network_type = parser.get<std::string>("@type");
    auto input_model_file = parser.get<std::string>("@modelfile");
    auto input_names_file = parser.get<std::string>("@nameslist");
    auto input_image_file = parser.get<std::string>("@inputimage");
    auto enable_profiling = parser.has("profile");
    auto threshold = parser.get<float>("thresh");
    auto nms_threshold = parser.get<float>("nmsthresh");
    auto batch_size = parser.get<int>("batch");
    auto anchors_file = parser.get<std::string>("anchors");

    if (!parser.check()) {
        parser.printErrors();
        return 0;
    }

    std::vector<std::string> class_names;
    if (!read_text_file(class_names, input_names_file)) {
        std::cerr << "Failed to read names file" << std::endl;
        return -1;
    }

    // read anchors file
    std::vector<std::string> anchor_priors_str;
    std::vector<float> anchor_priors;

    if (!anchors_file.empty()) {
        if (!read_text_file(anchor_priors_str, anchors_file)) {
            std::cerr << "Failed to read anchor priors file" << std::endl;
            return -1;
        }

        for (auto str : anchor_priors_str)
            anchor_priors.push_back(std::stof(str));
    }

    YoloRunnerFactory runner_fact(class_names.size(), threshold, nms_threshold, batch_size,
                                  anchor_priors, enable_profiling);
    YoloRunnerFactory::PreType pre;
    YoloRunnerFactory::RunnerType runner;
    YoloRunnerFactory::PostType post;

    std::tie(pre, runner, post) = runner_fact.create(network_type);

    if (!pre || !runner || !post) {
        std::cerr << "Failed to create runner" << std::endl;
        return -1;
    }

    std::vector<cv::Mat> images;

    cv::Mat img = read_image(input_image_file);
    if (img.empty()) {
        std::cerr << "Failed to read image: " << input_image_file << std::endl;
        return -1;
    }

    // process the same image multiple times if batch size > 1
    for (int i=0; i<batch_size; ++i) {
        images.push_back(img);
    }

    if (!runner->init(input_model_file)) {
        std::cerr << "Failed to init runner" << std::endl;
        return -1;
    }

    // register images to the preprocessor
    pre->register_images(images);

    // run pre/infer/post pipeline for a number of times depending on the profiling setting
    size_t iterations = enable_profiling ? 10 : 1;
    for (size_t i=0; i<iterations; ++i) {
        if (!(*runner)()) {
            std::cerr << "Failed to run network" << std::endl;
            return -1;
        }
    }

    // get detections and visualise
    auto detections = post->get_detections();

    // image is read in RGB, convert to BGR for display with imshow and bbox rendering
    cv::Mat out;
    cv::cvtColor(images[0], out, cv::COLOR_RGB2BGR);
    draw_detections(detections[0], class_names, out);

    cv::imshow("result", out);
    cv::waitKey(0);

    // show profiling if enabled
    runner->print_profiling();

    std::cout << "Success!" << std::endl;
    return 0;
}
