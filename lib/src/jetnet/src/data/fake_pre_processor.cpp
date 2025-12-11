#include "fake_pre_processor.h"
#include "file_io.h"

using namespace jetnet;
using namespace nvinfer1;

FakePreProcessor::FakePreProcessor(std::string input_blob_name, std::shared_ptr<Logger> logger) :
    m_input_blob_name(input_blob_name),
    m_logger(logger)
{
}

bool FakePreProcessor::init(const nvinfer1::ICudaEngine* engine)
{
    Dims network_input_dims;

    m_input_blob_index = engine->getBindingIndex(m_input_blob_name.c_str());
    network_input_dims = engine->getBindingDimensions(m_input_blob_index);
    m_size = network_input_dims.d[0];
    m_size *= network_input_dims.d[1];
    m_size *= network_input_dims.d[2];

    return true;
}

void FakePreProcessor::register_tensor_files(std::vector<std::string> file_names)
{
    m_file_names = file_names;
}

bool FakePreProcessor::operator()(std::map<int, GpuBlob>& input_blobs, std::vector<cv::Size>& image_sizes)
{
    std::vector<char> batch;

    for (auto file_name : m_file_names) {

        // find image_size in filename
        const char *wp = strrchr(file_name.c_str(), '_');
        const char *hp = strrchr(file_name.c_str(), 'x');
        int width = atoi(wp + 1);
        int height = atoi(hp + 1);
        image_sizes.push_back(cv::Size(width, height));

        // read binary file content and insert it into a big batch tensor
        std::vector<char> data = read_binary_file(file_name);

        if (data.empty()) {
            m_logger->log(ILogger::Severity::kERROR, "Fake: Failed to read tensor file " + file_name);
            return false;
        }

        if (data.size() / sizeof(float) != m_size) {
            m_logger->log(ILogger::Severity::kERROR, "Fake: Expected tensor file to contain " + std::to_string(m_size) +
                          " values, bot got " + std::to_string(data.size() / sizeof(float)) + " values");
            return false;
        }

        batch.insert(batch.end(), data.begin(), data.end());
    }

    input_blobs.at(m_input_blob_index).upload(reinterpret_cast<float*>(&batch[0]), batch.size());

    return true;
}
