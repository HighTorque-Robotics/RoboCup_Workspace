#include "letterbox_int8_calibrator.h"
#include "custom_assert.h"
#include "file_io.h"
#include <opencv2/opencv.hpp>

using namespace jetnet;
using namespace nvinfer1;

LetterboxInt8Calibrator::LetterboxInt8Calibrator(std::vector<std::string> file_names,
                                                 std::string cache_file,
                                                 std::shared_ptr<Logger> logger,
                                                 std::vector<unsigned int> channel_map,
                                                 nvinfer1::DimsCHW net_in_dims,
                                                 size_t batch_size) :
    Int8Calibrator(cache_file, logger),
    m_file_names(file_names),
    m_net_in_dims(net_in_dims),
    m_batch_size(batch_size),
    m_initialised(false),
    m_file_idx(0),
    m_pre("", channel_map, logger)
{
}

int LetterboxInt8Calibrator::getBatchSize() const
{
    m_logger->log(ILogger::Severity::kINFO, "Calibrator: called getBatchSize");
    return m_batch_size;
}

bool LetterboxInt8Calibrator::getBatch(void* bindings[], const char* names[], int nbBindings)
{
    (void)names;
    std::vector<cv::Size> image_sizes;
    std::vector<cv::Mat> images;

    if (!m_initialised) {
        ASSERT( m_pre.init(m_net_in_dims, m_batch_size) );
        m_initialised = true;
    }

    // check if there is still enough data to fill a batch
    if (m_file_idx + m_batch_size > m_file_names.size()) {
        m_logger->log(ILogger::Severity::kINFO, "Calibrator: read all images or not enough images to fill "
                                                "another complete batch. Signalling end of calibration set");
        return false;
    }

    // read m_batch_size images
    size_t batch;
    for (batch=0; batch<m_batch_size; ++batch, ++m_file_idx) {
        cv::Mat image = read_image(m_file_names[m_file_idx], m_net_in_dims.c());
        ASSERT(!image.empty());
        images.push_back(image);
    }

    // preprocess the batch
    m_pre.register_images(images);
    ASSERT( m_pre(m_blobs, image_sizes) );

    // copy blob GPU pointer to bindings array
    ASSERT(nbBindings == 1);
    bindings[0] = m_blobs.at(0).get();

    m_logger->log(ILogger::Severity::kINFO, "Calibrator: called getBatch: read " + std::to_string(m_file_idx)
                                            + "/" + std::to_string(m_file_names.size()) + " images");

    return true;
}
