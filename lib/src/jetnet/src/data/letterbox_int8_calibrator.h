#ifndef JETNET_LETTERBOX_INT8_CALIBRATOR_H
#define JETNET_LETTERBOX_INT8_CALIBRATOR_H

#include "int8_calibrator.h"
#include "cv_letterbox_pre_processor.h"

namespace jetnet
{

class LetterboxInt8Calibrator : public Int8Calibrator
{
public:

    LetterboxInt8Calibrator(std::vector<std::string> file_names,
                            std::string cache_file,
                            std::shared_ptr<Logger> logger,
                            std::vector<unsigned int> channel_map,
                            nvinfer1::DimsCHW net_in_dims,
                            size_t batch_size);

    /*
     *  Must return the batch size of the calibration set
     */
    int getBatchSize() const override;

    /*
     *  Must copy one batch of calibration data to the GPU
     *  bindings:   output array with GPU pointers, one for each input blob
     *  names:      name of each input blob
     *  nbBindings: number of input blobs
     */
    bool getBatch(void* bindings[], const char* names[], int nbBindings) override;

private:
    std::vector<std::string> m_file_names;
    nvinfer1::DimsCHW m_net_in_dims;
    size_t m_batch_size;

    bool m_initialised;
    size_t m_file_idx;

    CvLetterBoxPreProcessor m_pre;
    std::map<int, GpuBlob> m_blobs;
};

}

#endif /* JETNET_LETTERBOX_INT8_CALIBRATOR_H */
