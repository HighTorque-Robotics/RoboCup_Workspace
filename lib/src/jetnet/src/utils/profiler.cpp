#include "profiler.h"
#include <iomanip>

using namespace nvinfer1;

namespace jetnet
{

SimpleProfiler::SimpleProfiler(std::string name,
                               const std::vector<SimpleProfiler>& src_profilers) :
    m_name(name)
{
    for (const auto& src_profiler : src_profilers) {
        for (const auto& rec : src_profiler.m_profile) {
            auto it = m_profile.find(rec.first);

            if (it == m_profile.end()) {
                m_profile.insert(rec);
            } else {
                it->second.time += rec.second.time;
                it->second.count += rec.second.count;
            }
        }
    }
}

void SimpleProfiler::reportLayerTime(const char* layerName, float ms)
{
    m_profile[layerName].count++;
    m_profile[layerName].time += ms;
}

std::ostream& operator<<(std::ostream& out, const SimpleProfiler& value)
{
    out << "========== " << value.m_name << " profile ==========" << std::endl;
    float totalTime = 0;
    std::string layerNameStr = "Operation";

    int maxLayerNameLength = static_cast<int>(layerNameStr.size());
    for (const auto& elem : value.m_profile)
    {
        totalTime += elem.second.time;
        maxLayerNameLength = std::max(maxLayerNameLength, static_cast<int>(elem.first.size()));
    }

    auto old_settings = out.flags();
    auto old_precision = out.precision();
    // Output header
    {
        out << std::setw(maxLayerNameLength) << layerNameStr << " ";
        out << std::setw(12) << "Runtime, "
            << "%"
            << " ";
        out << std::setw(12) << "Invocations"
            << " ";
        out << std::setw(12) << "Runtime, ms" << std::endl;
    }
    for (const auto& elem : value.m_profile)
    {
        out << std::setw(maxLayerNameLength) << elem.first << " ";
        out << std::setw(12) << std::fixed << std::setprecision(1) << (elem.second.time * 100.0F / totalTime) << "%"
            << " ";
        out << std::setw(12) << elem.second.count << " ";
        out << std::setw(12) << std::fixed << std::setprecision(2) << elem.second.time << std::endl;
    }
    out.flags(old_settings);
    out.precision(old_precision);
    out << "========== " << value.m_name << " total runtime = " << totalTime << " ms ==========" << std::endl;

    return out;
}

}
