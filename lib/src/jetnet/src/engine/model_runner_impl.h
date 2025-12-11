#include "file_io.h"
#include "custom_assert.h"
#include <cuda_runtime.h>
#include <chrono>

#ifndef JETNET_MODEL_RUNNER_IMPL_H
#define JETNET_MODEL_RUNNER_IMPL_H

namespace jetnet
{

    template <typename TPre, typename TPost>
    ModelRunner<TPre, TPost>::ModelRunner(std::shared_ptr<nvinfer1::IPluginFactory> plugin_factory,
                                          std::shared_ptr<TPre> pre,
                                          std::shared_ptr<TPost> post,
                                          std::shared_ptr<Logger> logger,
                                          size_t batch_size, bool enable_profiling) : m_plugin_factory(plugin_factory),
                                                                                      m_pre(pre),
                                                                                      m_post(post),
                                                                                      m_logger(logger),
                                                                                      m_batch_size(batch_size),
                                                                                      m_enable_profiling(enable_profiling),
                                                                                      m_model_profiler("Model"),
                                                                                      m_host_profiler("Host")
    {
    }

    template <typename TPre, typename TPost>
    ModelRunner<TPre, TPost>::~ModelRunner()
    {
        if (m_cuda_stream)
            CUDA_CHECK(cudaStreamDestroy(m_cuda_stream));
        //TODO: validate that everything is cleaned with asan
    }

    template <typename TPre, typename TPost>
    bool ModelRunner<TPre, TPost>::init(std::string model_file)
    {
        m_runtime = nvinfer1::createInferRuntime(*m_logger);

        if (!m_runtime)
        {
            m_logger->log(nvinfer1::ILogger::Severity::kERROR, "Failed to create infer runtime");
            return false;
        }

        if (deserialize(model_file) == nullptr)
        {
            m_logger->log(nvinfer1::ILogger::Severity::kERROR, "Failed to deserialize network");
            return false;
        }

        if (m_batch_size > (size_t)m_cuda_engine->getMaxBatchSize())
        {
            m_logger->log(nvinfer1::ILogger::Severity::kERROR, "Batch size is " + std::to_string(m_batch_size) +
                                                                   ", max batch size this network supports is: " + std::to_string(m_cuda_engine->getMaxBatchSize()));
            return false;
        }

        if (!m_pre->init(m_cuda_engine))
        {
            m_logger->log(nvinfer1::ILogger::Severity::kERROR, "Init of pre-processing failed");
            return false;
        }

        if (!m_post->init(m_cuda_engine))
        {
            m_logger->log(nvinfer1::ILogger::Severity::kERROR, "Init of post-processing failed");
            return false;
        }

        if (get_context() == nullptr)
        {
            m_logger->log(nvinfer1::ILogger::Severity::kERROR, "Failed to get execution context");
            return false;
        }

        create_io_blobs();

        // create CUDA stream
        CUDA_CHECK(cudaStreamCreate(&m_cuda_stream));

        return true;
    }

    template <typename TPre, typename TPost>
    bool ModelRunner<TPre, TPost>::operator()()
    {
        typename TPost::Arg arg;
        std::chrono::high_resolution_clock::time_point start;

        if (m_enable_profiling)
            start = std::chrono::high_resolution_clock::now();

        if (!(*m_pre)(m_input_blobs, arg))
        {
            m_logger->log(nvinfer1::ILogger::Severity::kERROR, "Preprocess failed");
            return false;
        }

        if (m_enable_profiling)
        {
            auto now = std::chrono::high_resolution_clock::now();
            m_host_profiler.reportLayerTime("pre-processing", std::chrono::duration<float, std::milli>(now - start).count());
            start = now;
        }

        if (!infer())
        {
            m_logger->log(nvinfer1::ILogger::Severity::kERROR, "Infer failed");
            return false;
        }

        if (m_enable_profiling)
        {
            auto now = std::chrono::high_resolution_clock::now();
            m_host_profiler.reportLayerTime("inference", std::chrono::duration<float, std::milli>(now - start).count());
            start = now;
        }

        if (!(*m_post)(m_output_blobs, arg))
        {
            m_logger->log(nvinfer1::ILogger::Severity::kERROR, "Postprocess failed");
            return false;
        }

        if (m_enable_profiling)
            m_host_profiler.reportLayerTime("post-processing", std::chrono::duration<float, std::milli>(std::chrono::high_resolution_clock::now() - start).count());

        return true;
    }

    template <typename TPre, typename TPost>
    void ModelRunner<TPre, TPost>::print_profiling()
    {
        if (m_enable_profiling)
        {
            std::cout << m_host_profiler;
            std::cout << std::endl;
            std::cout << m_model_profiler;
        }
    }

    template <typename TPre, typename TPost>
    nvinfer1::ICudaEngine *ModelRunner<TPre, TPost>::deserialize(const void *data, size_t length)
    {
        m_cuda_engine = m_runtime->deserializeCudaEngine(data, length, m_plugin_factory.get());
        return m_cuda_engine;
    }

    template <typename TPre, typename TPost>
    nvinfer1::ICudaEngine *ModelRunner<TPre, TPost>::deserialize(std::string filename)
    {
        std::cout << "???????????????????????????????????" << std::endl
                  << filename << std::endl
                  << "???????????????????????????????????" << std::endl;
        std::vector<char> data = read_binary_file(filename);
        if (data.empty())
        {
            m_logger->log(nvinfer1::ILogger::Severity::kERROR, "Failed to read binary file " + filename);
            return nullptr;
        }

        deserialize(data.data(), data.size());
        return m_cuda_engine;
    }

    template <typename TPre, typename TPost>
    nvinfer1::IExecutionContext *ModelRunner<TPre, TPost>::get_context()
    {
        m_context = m_cuda_engine->createExecutionContext();
        if (m_enable_profiling)
            m_context->setProfiler(&m_model_profiler);
        return m_context;
    }

    template <typename TPre, typename TPost>
    void ModelRunner<TPre, TPost>::create_io_blobs()
    {
        int i, j;

        for (i = 0; i < m_cuda_engine->getNbBindings(); i++)
        {

            nvinfer1::Dims dim = m_cuda_engine->getBindingDimensions(i);
            size_t size = 1;
            for (j = 0; j < dim.nbDims; j++)
                size *= dim.d[j];

            size *= m_batch_size;
            GpuBlob gpu_blob(size);
            // get binding index sorted buffer pointer for infer() call
            m_cuda_buffers.push_back(gpu_blob.get());

            if (m_cuda_engine->bindingIsInput(i))
            {
                m_input_blobs.insert(std::pair<int, GpuBlob>(i, std::move(gpu_blob)));
            }
            else
            {
                m_output_blobs.insert(std::pair<int, GpuBlob>(i, std::move(gpu_blob)));
            }
        }
    }

    template <typename TPre, typename TPost>
    bool ModelRunner<TPre, TPost>::infer()
    {
        // Start execution
        if (!m_context->enqueue(m_batch_size, m_cuda_buffers.data(), m_cuda_stream, nullptr))
            return false;

        // Wait for execution to finish
        CUDA_CHECK(cudaStreamSynchronize(m_cuda_stream));

        return true;
    }

}

#endif /* JETNET_MODEL_RUNNER_IMPL_H */
