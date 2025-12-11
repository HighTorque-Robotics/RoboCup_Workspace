#include "darknet_weights_loader.h"

using namespace jetnet;
using namespace nvinfer1;

DarknetWeightsLoader::DarknetWeightsLoader() :
    datatype(DataType::kFLOAT)
{
}
DarknetWeightsLoader::DarknetWeightsLoader(DataType dt) :
    datatype(dt)
{
}

DarknetWeightsLoader::~DarknetWeightsLoader()
{
    m_file.close();
}

bool DarknetWeightsLoader::open(std::string weights_file)
{
    FileRevision header_version;
    size_t seen;

    m_file.open(weights_file, std::ifstream::binary);
    if (!m_file)
        return false;

    /* read file revision from header */
    m_file.read(reinterpret_cast<char *>(&header_version), sizeof header_version);
    if (!m_file)
        return false;

    /* read number of seen bytes depending on revision number */
    if (header_version.major >= 1 || header_version.minor >= 2) {
        m_file.read(reinterpret_cast<char *>(&seen), sizeof seen);
        if (!m_file)
            return false;
    } else {
        int iseen;
        m_file.read(reinterpret_cast<char *>(&iseen), sizeof iseen);
        if (!m_file)
            return false;
        seen = iseen;
    }

    m_file_info.major = header_version.major;
    m_file_info.minor = header_version.minor;
    m_file_info.revision = header_version.revision;
    m_file_info.seen = seen;

    return true;
}

DarknetWeightsLoader::FileInfo DarknetWeightsLoader::get_file_info()
{
    return m_file_info;
}

std::vector<float> DarknetWeightsLoader::get_floats(size_t len)
{
    std::vector<float> weights(len);
    m_file.read(reinterpret_cast<char *>(weights.data()), len * sizeof(float));

    // return empty vector on failure
    if (!m_file)
        return std::vector<float>();

    return weights;
}

Weights DarknetWeightsLoader::get(std::vector<float> weights)
{
    Weights res{datatype, nullptr, 0};

    if (weights.empty())
        return res;

    if (datatype == DataType::kHALF) {
        std::vector<__half> weights_half(weights.size());

        // convert weights to 16-bit float weights
        for (size_t i=0; i<weights.size(); ++i) {
            weights_half[i] = __float2half(weights[i]);
        }

        m_half_store.push_back(weights_half);           // add to the store to manage weights lifetime
        res.values = m_half_store.back().data();        // returned weights refer to the store from now on
        res.count = m_half_store.back().size();
    } else {
        m_float_store.push_back(weights);               // add to the store to manage weights lifetime
        res.values = m_float_store.back().data();       // returned weights refer to the store from now on
        res.count = m_float_store.back().size();
    }

    return res;
}

Weights DarknetWeightsLoader::get(size_t len)
{
    std::vector<float> weights = get_floats(len);
    return get(weights);
}
