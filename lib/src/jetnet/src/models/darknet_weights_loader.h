#ifndef JETNET_DARKNET_WEIGHTS_LOADER_H
#define JETNET_DARKNET_WEIGHTS_LOADER_H

#include <vector>
#include <string>
#include <fstream>
#include <NvInfer.h>
#include "fp16.h"

namespace jetnet
{

class DarknetWeightsLoader
{
public:
    struct FileInfo
    {
        int major;
        int minor;
        int revision;
        size_t seen;
    };

    /*
     *  Datatype that will be used to convert to after loading
     */
    const nvinfer1::DataType datatype;

    DarknetWeightsLoader();
    DarknetWeightsLoader(nvinfer1::DataType dt);
    ~DarknetWeightsLoader();

    /*
     *  Open a darknet weights file
     */
    bool open(std::string weights_file);

    /*
     *  Get version and training info
     */
    FileInfo get_file_info();

    /*
     *  Get weights in float vector from file
     */
    std::vector<float> get_floats(size_t len);

    /*
     *  Get TensorRT weights from a float vector, according to the set datatype
     */
    nvinfer1::Weights get(std::vector<float> weights);

    /*
     *  Get TensorRT weights from file, according to the set datatype
     */
    nvinfer1::Weights get(size_t len);

private:
    struct __attribute__ ((__packed__)) FileRevision
    {
        int major;
        int minor;
        int revision;
    };

    std::ifstream m_file;
    FileInfo m_file_info;

    std::vector<std::vector<float>> m_float_store;
    std::vector<std::vector<__half>> m_half_store;
};

}

#endif /* JETNET_DARKNET_WEIGHTS_LOADER_H */
