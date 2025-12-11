#include "yolo_plugin_factory.h"
#include <cstring>

using namespace jetnet;
using namespace nvinfer1;

YoloPluginFactory::YoloPluginFactory(std::shared_ptr<Logger> logger) :
    m_logger(logger)
{
}

IPlugin* YoloPluginFactory::createPlugin(const char* layerName, const void* serialData, size_t serialLength)
{
    ::plugin::RegionParameters params;

    m_logger->log(ILogger::Severity::kINFO, "Plugin factory creating: " + std::string(layerName));
    if (strstr(layerName, "PReLU")) {
        m_prelu_plugins.push_back(std::unique_ptr<::plugin::INvPlugin, decltype(nvPluginDeleter)>(
                                  ::plugin::createPReLUPlugin(serialData, serialLength), nvPluginDeleter));
        return m_prelu_plugins.back().get();

    } else if (strstr(layerName, "YOLOReorg")) {
        m_reorg_plugins.push_back(std::unique_ptr<::plugin::INvPlugin, decltype(nvPluginDeleter)>(
                                  ::plugin::createYOLOReorgPlugin(serialData, serialLength), nvPluginDeleter));
        return m_reorg_plugins.back().get();

    } else if (strstr(layerName, "YOLORegion")) {
        // also parse params for get_region_params method
        // NOTE: this is a dirty hack and might break future compatibility
        const int* data = reinterpret_cast<const int*>(serialData);
        params.num = data[3];
        params.classes = data[4];
        params.coords = data[5];
        params.smTree = nullptr;
        m_region_params.push_back(params);

        m_region_plugins.push_back(std::unique_ptr<::plugin::INvPlugin, decltype(nvPluginDeleter)>(
                                   ::plugin::createYOLORegionPlugin(serialData, serialLength), nvPluginDeleter));
        return m_region_plugins.back().get();

    } else if (strstr(layerName, "upsample")) {
        m_upsample_plugins.push_back(std::unique_ptr<UpsamplePlugin>(new UpsamplePlugin(serialData, serialLength)));
        return m_upsample_plugins.back().get();
    }

    m_logger->log(ILogger::Severity::kERROR, "Do not know how to create plugin " + std::string(layerName));
    return nullptr;
}

bool YoloPluginFactory::get_region_params(size_t index, ::plugin::RegionParameters& params)
{
    if (index >= m_region_params.size())
        return false;

    params = m_region_params[index];
    return true;
}
