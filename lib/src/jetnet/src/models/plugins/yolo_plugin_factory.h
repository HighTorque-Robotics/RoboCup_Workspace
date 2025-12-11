#ifndef JETNET_YOLO_PLUGIN_FACTORY_H
#define JETNET_YOLO_PLUGIN_FACTORY_H

#include "logger.h"
#include "upsample_plugin.h"
#include <NvInfer.h>
#include <NvInferPlugin.h>
#include <memory>
#include <vector>

namespace jetnet
{

/*
 *  Plugin factory class for all yolo models
 */
class YoloPluginFactory : public nvinfer1::IPluginFactory
{
public:
    YoloPluginFactory(std::shared_ptr<Logger> logger);

    /*
     *  (Re)create plugin based on layer name
     *  PReLU plugin: if the layername contains PReLU
     *  Reorg plugin: if the layername contains YOLOReorg
     *  Region plugin: if the layername contains YOLORegion
     *
     *  Used by deserialize
     */
    nvinfer1::IPlugin* createPlugin(const char* layerName, const void* serialData, size_t serialLength) override;

    /*
     *  Get region layer parameters since deserialized network has (as far as I know) no means to retrieve
     *  number of anchors, classes and coords
     *  Only call this function after createPlugin has been ran for at leased one region plugin
     *  index:  index of the region plugin (0 for first created region plugin, 1 for the second, etc...)
     *  params: retrieved region params
     *  returns true on success, false if no region layer was deserialized (yet) or if the index is wrong
     */
    bool get_region_params(size_t index, nvinfer1::plugin::RegionParameters& params);

private:
    std::shared_ptr<Logger> m_logger;
    void(*nvPluginDeleter)(nvinfer1::plugin::INvPlugin*) { [](nvinfer1::plugin::INvPlugin* ptr) {ptr->destroy();} };
    std::vector<std::unique_ptr<nvinfer1::plugin::INvPlugin, decltype(nvPluginDeleter)>> m_prelu_plugins;
    std::vector<std::unique_ptr<nvinfer1::plugin::INvPlugin, decltype(nvPluginDeleter)>> m_reorg_plugins;
    std::vector<std::unique_ptr<nvinfer1::plugin::INvPlugin, decltype(nvPluginDeleter)>> m_region_plugins;
    std::vector<nvinfer1::plugin::RegionParameters> m_region_params;
    std::vector<std::unique_ptr<UpsamplePlugin>> m_upsample_plugins;
};

}

#endif /* JETNET_YOLO_PLUGIN_FACTORY_H */
