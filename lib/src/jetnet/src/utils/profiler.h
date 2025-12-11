#ifndef JETNET_PROFILER_H

#define JETNET_PROFILER_H

#include <NvInfer.h>
#include <vector>
#include <string>
#include <iostream>
#include <map>

namespace jetnet
{

class SimpleProfiler : public nvinfer1::IProfiler
{
public:
    struct Record
    {
        float time{0};
        int count{0};
    };

    /*
     *  Construct a profiler
     *  name:           Name of the profiler
     *  src_profilers:  Optionally initialize profiler with data of one or more other profilers
     *                  This is usefull for aggregating results of different profilers
     *                  Aggregation will sum all runtime periods and all invokations for each reported
     *                  layer of all given profilers
     */
    SimpleProfiler(std::string name,
        const std::vector<SimpleProfiler>& src_profilers = std::vector<SimpleProfiler>());

    void reportLayerTime(const char* layerName, float ms) override;

    friend std::ostream& operator<<(std::ostream& out, const SimpleProfiler& value);

private:
    std::string m_name;
    std::map<std::string, Record> m_profile;
};

}

#endif /* JETNET_PROFILER_H */
