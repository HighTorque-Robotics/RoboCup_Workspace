#ifndef JETNET_MODEL_RUNNER_H
#define JETNET_MODEL_RUNNER_H

#include "logger.h"
#include "gpu_blob.h"
#include "profiler.h"
#include <opencv2/opencv.hpp>
#include <NvInfer.h>
#include <memory>
#include <vector>
#include <map>

namespace jetnet
{

template<typename TPre, typename TPost>
class ModelRunner
{
public:

    /*
     *  plugin_factory:     plugin factory to be able to deserialize plugins
     *  pre:                pre-processor object reference
     *  post:               post-processor object reference
     *  logger:             logger object
     *  batch_size:         batch size, must be smaller or equal to the max batch size the network supports
     *  enable_profiling:   If true, network, pre - and postprocessors are profiled
     */
    ModelRunner(std::shared_ptr<nvinfer1::IPluginFactory> plugin_factory,
                std::shared_ptr<TPre> pre,
                std::shared_ptr<TPost> post,
                std::shared_ptr<Logger> logger,
                size_t batch_size, bool enable_profiling = false);

    ~ModelRunner();

    /*
     *  Init
     *  model_file: filename of the built model
     */
    bool init(std::string model_file);

    /*
     *  Run the pre/infer/post pipeline for the current batch
     */
    bool operator()();

    //TODO: add more flexibility
    void print_profiling();

private:
    /* methods */
    nvinfer1::ICudaEngine* deserialize(const void* data, size_t length);
    nvinfer1::ICudaEngine* deserialize(std::string filename);
    nvinfer1::IExecutionContext* get_context();
    void create_io_blobs();
    bool infer();

    /* variables initialized by the ctor */
    std::shared_ptr<nvinfer1::IPluginFactory> m_plugin_factory;
    std::shared_ptr<TPre> m_pre;
    std::shared_ptr<TPost> m_post;
    std::shared_ptr<Logger> m_logger;
    size_t m_batch_size;
    bool m_enable_profiling;

    /* variables */
    SimpleProfiler m_model_profiler;
    SimpleProfiler m_host_profiler;
    nvinfer1::ICudaEngine* m_cuda_engine = nullptr;
    nvinfer1::IRuntime* m_runtime = nullptr;
    nvinfer1::IExecutionContext* m_context = nullptr;
    cudaStream_t m_cuda_stream = nullptr;

    std::vector<void*> m_cuda_buffers;
    std::map<int, GpuBlob> m_input_blobs;
    std::map<int, GpuBlob> m_output_blobs;
};

}

#include "model_runner_impl.h"

#endif /* JETNET_MODEL_RUNNER_H */
