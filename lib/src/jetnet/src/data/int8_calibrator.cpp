#include "file_io.h"
#include "int8_calibrator.h"
#include "custom_assert.h"

using namespace jetnet;
using namespace nvinfer1;

Int8Calibrator::Int8Calibrator(std::string cache_file, std::shared_ptr<Logger> logger) :
    m_cache_file(cache_file),
    m_logger(logger)
{
}

const void* Int8Calibrator::readCalibrationCache(size_t& length)
{
    length = 0;

    if (m_cache_file.empty()) {
        m_logger->log(ILogger::Severity::kINFO, "No cache file provided, starting the "
                                                "calibration procedure");
        return nullptr;
    }

    m_cache_data = read_binary_file(m_cache_file);

    if (m_cache_data.empty()) {
        m_logger->log(ILogger::Severity::kINFO, "Calibration cache file " + m_cache_file +
                                                 " not found, (re)starting calibration procedure");
        return nullptr;
    }

    m_logger->log(ILogger::Severity::kINFO, "Calibration cache file " + m_cache_file +
                                                " found, skipping calibration procedure");
    length = m_cache_data.size();

    return &m_cache_data[0];
}

void Int8Calibrator::writeCalibrationCache(const void* ptr, size_t length)
{
    if (!m_cache_file.empty()) {
        m_logger->log(ILogger::Severity::kINFO, "Saving calibration cache file " + m_cache_file);
        ASSERT( write_binary_file(ptr, length, m_cache_file) );
    } else {
        m_logger->log(ILogger::Severity::kERROR, "Trying to save calibration cache file but an empty"
                                                 " cache file name was provided");
    }
}
