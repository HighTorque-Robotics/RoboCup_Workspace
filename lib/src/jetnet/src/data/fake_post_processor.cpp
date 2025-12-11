#include "fake_post_processor.h"
#include "file_io.h"

using namespace jetnet;
using namespace nvinfer1;


FakePostProcessor::FakePostProcessor(std::vector<std::string> output_blob_names,
                                     std::vector<std::string> tensor_file_names) :
    m_output_blob_names(output_blob_names),
    m_tensor_file_names(tensor_file_names)
{
}

bool FakePostProcessor::init(const ICudaEngine* engine)
{
    if (m_output_blob_names.size() != m_tensor_file_names.size()) {
        std::cerr << "Fake: number of output blob names must equal number of tensor file names" << std::endl;
        return false;
    }

    for (auto& name : m_output_blob_names) {
        m_output_blob_indices.push_back(engine->getBindingIndex(name.c_str()));
    }

    return true;
}

bool FakePostProcessor::operator()(const std::vector<cv::Mat>& images, const std::map<int, GpuBlob>& output_blobs)
{
    // for now only support loading a tensor with batch size 1
    if (images.size() != 1) {
        std::cerr << "Fake: Batch size must be 1" << std::endl;
        return false;
    }

    for (size_t i=0; i<m_output_blob_indices.size(); ++i) {

        std::vector<float> data;
        output_blobs.at(m_output_blob_indices[i]).download(data);

        if (!save_tensor_text(data.data(), data.size(), m_tensor_file_names[i])) {
            std::cerr << "Fake: Failed to save " << m_tensor_file_names[i] << std::endl;
            return false;
        }
    }
    
    return true;
}
