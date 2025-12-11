#ifndef JETNET_CREATE_RUNNER_H
#define JETNET_CREATE_RUNNER_H

#include "jetnet.h"
#include <tuple>

class YoloRunnerFactory
{
    size_t m_num_classes;
    float m_threshold;
    float m_nms_threshold;
    size_t m_batch_size;
    std::vector<float> m_anchor_priors;
    bool m_enable_profiling;

public:

    typedef std::shared_ptr<jetnet::CvLetterBoxPreProcessor>    PreType;
    typedef std::shared_ptr<jetnet::YoloPostProcessor>          PostType;
    typedef std::shared_ptr<jetnet::ModelRunner<jetnet::CvLetterBoxPreProcessor, jetnet::YoloPostProcessor>> RunnerType;

    YoloRunnerFactory(size_t num_classes, float threshold, float nms_threshold, size_t batch_size,
                      std::vector<float> anchor_priors, bool enable_profiling=false) :
        m_num_classes(num_classes),
        m_threshold(threshold),
        m_nms_threshold(nms_threshold),
        m_batch_size(batch_size),
        m_anchor_priors(anchor_priors),
        m_enable_profiling(enable_profiling)
    {
    }

    std::tuple<PreType, RunnerType, PostType> create(std::string model_name)
    {
        if (model_name == "yolov2")
            return create_yolov2();
        else if (model_name == "yolov3")
            return create_yolov3();
        else if (model_name == "yolov3-tiny")
            return create_yolov3_tiny();

        std::cerr << "Error: unknown model type " << model_name << std::endl;
        return std::make_tuple(nullptr, nullptr, nullptr);
    }

    std::tuple<PreType, RunnerType, PostType> create_yolov2()
    {
        const std::string input_blob_name = "data";
        const std::string output_blob_name = "probs";

        auto logger = std::make_shared<jetnet::Logger>(nvinfer1::ILogger::Severity::kINFO);
        auto plugin_fact = std::make_shared<jetnet::YoloPluginFactory>(logger);

        if (!m_anchor_priors.empty()) {
            std::cout << "Using custom anchor priors" << std::endl;
        } else {
            std::cout << "Using default anchor priors" << std::endl;
            m_anchor_priors = std::vector<float>{0.57273, 0.677385,
                                                 1.87446, 2.06253,
                                                 3.33843, 5.47434,
                                                 7.88282, 3.52778,
                                                 9.77052, 9.16828};
        }

        std::vector<unsigned int> channel_map{0, 1, 2};     //read_image read RGB order, network expects RGB order
        auto pre = std::make_shared<jetnet::CvLetterBoxPreProcessor>(input_blob_name, channel_map, logger);

        std::vector<jetnet::YoloPostProcessor::OutputSpec> output_specs = {
            jetnet::YoloPostProcessor::OutputSpec { output_blob_name, m_anchor_priors, m_num_classes }
        };

        auto post = std::make_shared<jetnet::YoloPostProcessor>(input_blob_name,
                        jetnet::YoloPostProcessor::Type::Yolov2,
                        output_specs,
                        m_threshold,
                        logger,
                        [=](std::vector<jetnet::Detection>& detections) { jetnet::nms_sort(detections, m_nms_threshold); });

        auto runner = std::make_shared<jetnet::ModelRunner<jetnet::CvLetterBoxPreProcessor, jetnet::YoloPostProcessor>>(plugin_fact, pre, post, logger, m_batch_size, m_enable_profiling);

        return std::make_tuple(pre, runner, post);
    }

    std::tuple<PreType, RunnerType, PostType> create_yolov3()
    {
        const std::string input_blob_name = "data";
        const std::string output_blob1_name = "probs1";
        const std::string output_blob2_name = "probs2";
        const std::string output_blob3_name = "probs3";

        std::vector<float> anchor_priors1;
        std::vector<float> anchor_priors2;
        std::vector<float> anchor_priors3;

        if (!m_anchor_priors.empty()) {
            std::cout << "Using custom anchor priors" << std::endl;
            anchor_priors3 = std::vector<float>(m_anchor_priors.begin(), m_anchor_priors.begin() + 6);
            anchor_priors2 = std::vector<float>(m_anchor_priors.begin() + 6, m_anchor_priors.begin() + 12);
            anchor_priors1 = std::vector<float>(m_anchor_priors.begin() + 12, m_anchor_priors.end());
        } else {
            std::cout << "Using default anchor priors" << std::endl;
            anchor_priors3 = std::vector<float>{10, 13, 16, 30, 33, 23};
            anchor_priors2 = std::vector<float>{30, 61, 62, 45, 59, 119};
            anchor_priors1 = std::vector<float>{116,90, 156,198,373,326};
        }

        auto logger = std::make_shared<jetnet::Logger>(nvinfer1::ILogger::Severity::kINFO);
        auto plugin_fact = std::make_shared<jetnet::YoloPluginFactory>(logger);

        std::vector<unsigned int> channel_map{0, 1, 2};     //read_image read RGB order, network expects RGB order
        auto pre = std::make_shared<jetnet::CvLetterBoxPreProcessor>(input_blob_name, channel_map, logger);

        std::vector<jetnet::YoloPostProcessor::OutputSpec> output_specs = {
            jetnet::YoloPostProcessor::OutputSpec { output_blob1_name, anchor_priors1, m_num_classes },
            jetnet::YoloPostProcessor::OutputSpec { output_blob2_name, anchor_priors2, m_num_classes },
            jetnet::YoloPostProcessor::OutputSpec { output_blob3_name, anchor_priors3, m_num_classes }
        };

        auto post = std::make_shared<jetnet::YoloPostProcessor>(input_blob_name,
                        jetnet::YoloPostProcessor::Type::Yolov3,
                        output_specs,
                        m_threshold,
                        logger,
                        [=](std::vector<jetnet::Detection>& detections) { jetnet::nms_sort(detections, m_nms_threshold); });

        auto runner = std::make_shared<jetnet::ModelRunner<jetnet::CvLetterBoxPreProcessor, jetnet::YoloPostProcessor>>(plugin_fact, pre, post, logger, m_batch_size, m_enable_profiling);

        return std::make_tuple(pre, runner, post);
    }

    std::tuple<PreType, RunnerType, PostType> create_yolov3_tiny()
    {
        const std::string input_blob_name = "data";
        const std::string output_blob1_name = "probs1";
        const std::string output_blob2_name = "probs2";

        std::vector<float> anchor_priors1;
        std::vector<float> anchor_priors2;

        if (!m_anchor_priors.empty()) {
            std::cout << "Using custom anchor priors" << std::endl;
            anchor_priors2 = std::vector<float>(m_anchor_priors.begin(), m_anchor_priors.begin() + 6);
            anchor_priors1 = std::vector<float>(m_anchor_priors.begin() + 6, m_anchor_priors.end());
        } else {
            std::cout << "Using default anchor priors" << std::endl;
            anchor_priors2 = std::vector<float>{10, 14, 23, 27, 37, 58};
            anchor_priors1 = std::vector<float>{81, 82, 135,169,344,319};
        }

        auto logger = std::make_shared<jetnet::Logger>(nvinfer1::ILogger::Severity::kINFO);
        auto plugin_fact = std::make_shared<jetnet::YoloPluginFactory>(logger);

        std::vector<unsigned int> channel_map{0, 1, 2};     //read_image read RGB order, network expects RGB order
        auto pre = std::make_shared<jetnet::CvLetterBoxPreProcessor>(input_blob_name, channel_map, logger);

        std::vector<jetnet::YoloPostProcessor::OutputSpec> output_specs = {
            jetnet::YoloPostProcessor::OutputSpec { output_blob1_name, anchor_priors1, m_num_classes },
            jetnet::YoloPostProcessor::OutputSpec { output_blob2_name, anchor_priors2, m_num_classes },
        };

        auto post = std::make_shared<jetnet::YoloPostProcessor>(input_blob_name,
                        jetnet::YoloPostProcessor::Type::Yolov3,
                        output_specs,
                        m_threshold,
                        logger,
                        [=](std::vector<jetnet::Detection>& detections) { jetnet::nms_sort(detections, m_nms_threshold); });

        auto runner = std::make_shared<jetnet::ModelRunner<jetnet::CvLetterBoxPreProcessor, jetnet::YoloPostProcessor>>(plugin_fact, pre, post, logger, m_batch_size, m_enable_profiling);

        return std::make_tuple(pre, runner, post);
    }
};

#endif /* JETNET_CREATE_RUNNER_H */
