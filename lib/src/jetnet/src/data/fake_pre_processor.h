#ifndef JETNET_FAKE_PRE_PROCESSOR_H
#define JETNET_FAKE_PRE_PROCESSOR_H

#include "gpu_blob.h"
#include "logger.h"
#include <opencv2/opencv.hpp>
#include <NvInfer.h>
#include <string>
#include <vector>

namespace jetnet
{

class FakePreProcessor
{
public:
    /*
     *  Read the input tensors from binary tensor files. A binary file contains a chunk of float numbers
     *  that can be directly copied to the network input. A tensor file should have a name that embeds
     *  the image size of the image. Format = <prefix>_<width>x<height>.<extension>
     */
    FakePreProcessor(std::string input_blob_name, std::shared_ptr<Logger> logger);

    bool init(const nvinfer1::ICudaEngine* engine);
    void register_tensor_files(std::vector<std::string> file_names);
    bool operator()(std::map<int, GpuBlob>& input_blobs, std::vector<cv::Size>& image_sizes);

private:
    std::string m_input_blob_name;
    std::string m_tensor_file_name;
    size_t m_size;
    int m_input_blob_index;
    std::shared_ptr<Logger> m_logger;
    std::vector<std::string> m_file_names;
};

}

#endif /* JETNET_FAKE_PRE_PROCESSOR_H */
