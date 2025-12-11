#include "yolov2_builder.h"
#include "custom_assert.h"
#include "leaky_relu_plugin.h"
#include "leaky_relu_native.h"
#include "relu.h"
#include "logger.h"

using namespace jetnet;
using namespace nvinfer1;

template class Yolov2Builder<LeakyReluPlugin>;
template class Yolov2Builder<LeakyReluNative>;
template class Yolov2Builder<Relu>;

template<typename TActivation>
INetworkDefinition* Yolov2Builder<TActivation>::parse(DataType dt)
{
    m_logger->log(ILogger::Severity::kINFO, "Opening weights file '" + m_weightsfile + "'");
    m_weights = std::unique_ptr<DarknetWeightsLoader>(new DarknetWeightsLoader(dt));

    if (!m_weights->open(m_weightsfile)) {
        m_logger->log(ILogger::Severity::kERROR, "Failed Reading weights file '" + m_weightsfile + "'");
        return nullptr;
    }

    auto file_info = m_weights->get_file_info();
    m_logger->log(ILogger::Severity::kINFO, "Weights file info: V" + std::to_string(file_info.major) + "." +
                  std::to_string(file_info.minor) + "." + std::to_string(file_info.revision) + ", seen = " +
                  std::to_string(file_info.seen));

    // Note: assume the input blob is always provided with 32-bit floats
    ITensor* data = m_network->addInput(m_input_blob_name.c_str(), DataType::kFLOAT, m_input_dimensions);
    ASSERT(data);

    // Start of the network
    ILayer* conv0 = m_convs[0]("conv0", m_network, *m_weights, *data, 32, DimsHW{3, 3});
    ASSERT(conv0);  // 0

    IPoolingLayer* pool0 = m_network->addPooling(*conv0->getOutput(0), PoolingType::kMAX, DimsHW{2, 2});
    ASSERT(pool0);  // 1
    pool0->setStride(DimsHW{2, 2});
    pool0->setName("pool0");

    ILayer* conv1 = m_convs[1]("conv1", m_network, *m_weights, *pool0->getOutput(0), 64, DimsHW{3, 3});
    ASSERT(conv1);  // 2

    IPoolingLayer* pool1 = m_network->addPooling(*conv1->getOutput(0), PoolingType::kMAX, DimsHW{2, 2});
    ASSERT(pool1);  // 3
    pool1->setStride(DimsHW{2, 2});
    pool1->setName("pool1");

    ILayer* conv2 = m_convs[2]("conv2", m_network, *m_weights, *pool1->getOutput(0), 128, DimsHW{3, 3});
    ASSERT(conv2);  // 4

    ILayer* conv3 = m_convs[3]("conv3", m_network, *m_weights, *conv2->getOutput(0), 64, DimsHW{1, 1}, DimsHW{0, 0});
    ASSERT(conv3);  // 5

    ILayer* conv4 = m_convs[4]("conv4", m_network, *m_weights, *conv3->getOutput(0), 128, DimsHW{3, 3});
    ASSERT(conv4);  // 6

    IPoolingLayer* pool2 = m_network->addPooling(*conv4->getOutput(0), PoolingType::kMAX, DimsHW{2, 2});
    ASSERT(pool2);  // 7
    pool2->setStride(DimsHW{2, 2});
    pool2->setName("pool2");

    ILayer* conv5 = m_convs[5]("conv5", m_network, *m_weights, *pool2->getOutput(0), 256, DimsHW{3, 3});
    ASSERT(conv5);  // 8

    ILayer* conv6 = m_convs[6]("conv6", m_network, *m_weights, *conv5->getOutput(0), 128, DimsHW{1, 1}, DimsHW{0, 0});
    ASSERT(conv6);  // 9

    ILayer* conv7 = m_convs[7]("conv7", m_network, *m_weights, *conv6->getOutput(0), 256, DimsHW{3, 3});
    ASSERT(conv7);  // 10

    IPoolingLayer* pool3 = m_network->addPooling(*conv7->getOutput(0), PoolingType::kMAX, DimsHW{2, 2});
    ASSERT(pool3);  // 11
    pool3->setStride(DimsHW{2, 2});
    pool3->setName("pool3");

    ILayer* conv8 = m_convs[8]("conv8", m_network, *m_weights, *pool3->getOutput(0), 512, DimsHW{3, 3});
    ASSERT(conv8);  // 12

    ILayer* conv9 = m_convs[9]("conv9", m_network, *m_weights, *conv8->getOutput(0), 256, DimsHW{1, 1}, DimsHW{0, 0});
    ASSERT(conv9);  // 13

    ILayer* conv10 = m_convs[10]("conv10", m_network, *m_weights, *conv9->getOutput(0), 512, DimsHW{3, 3});
    ASSERT(conv10); // 14

    ILayer* conv11 = m_convs[11]("conv11", m_network, *m_weights, *conv10->getOutput(0), 256, DimsHW{1, 1}, DimsHW{0, 0});
    ASSERT(conv11); // 15

    ILayer* conv12 = m_convs[12]("conv12", m_network, *m_weights, *conv11->getOutput(0), 512, DimsHW{3, 3});
    ASSERT(conv12); // 16

    IPoolingLayer* pool4 = m_network->addPooling(*conv12->getOutput(0), PoolingType::kMAX, DimsHW{2, 2});
    ASSERT(pool4);  // 17
    pool4->setStride(DimsHW{2, 2});
    pool4->setName("pool4");

    ILayer* conv13 = m_convs[13]("conv13", m_network, *m_weights, *pool4->getOutput(0), 1024, DimsHW{3, 3});
    ASSERT(conv13); // 18

    ILayer* conv14 = m_convs[14]("conv14", m_network, *m_weights, *conv13->getOutput(0), 512, DimsHW{1, 1}, DimsHW{0, 0});
    ASSERT(conv14); // 19

    ILayer* conv15 = m_convs[15]("conv15", m_network, *m_weights, *conv14->getOutput(0), 1024, DimsHW{3, 3});
    ASSERT(conv15); // 20

    ILayer* conv16 = m_convs[16]("conv16", m_network, *m_weights, *conv15->getOutput(0), 512, DimsHW{1, 1}, DimsHW{0, 0});
    ASSERT(conv16); // 21

    ILayer* conv17 = m_convs[17]("conv17", m_network, *m_weights, *conv16->getOutput(0), 1024, DimsHW{3, 3});
    ASSERT(conv17); // 22

    ILayer* conv18 = m_convs[18]("conv18", m_network, *m_weights, *conv17->getOutput(0), 1024, DimsHW{3, 3});
    ASSERT(conv18); // 23

    ILayer* conv19 = m_convs[19]("conv19", m_network, *m_weights, *conv18->getOutput(0), 1024, DimsHW{3, 3});
    ASSERT(conv19); // 24

    // Parallel branch (input from conv12)
    ILayer* conv20 = m_convs[20]("conv20", m_network, *m_weights, *conv12->getOutput(0), 64, DimsHW{1, 1}, DimsHW{0, 0});
    ASSERT(conv20); // 26

    m_reorg_plugin = std::unique_ptr<::plugin::INvPlugin, decltype(nvPluginDeleter)>(plugin::createYOLOReorgPlugin(2),
                                                                                     nvPluginDeleter);
    ASSERT(m_reorg_plugin);

    ITensor *conv20_tensor = conv20->getOutput(0);
    ILayer* reorg = m_network->addPlugin(&conv20_tensor, 1, *m_reorg_plugin);
    ASSERT(reorg);  // 27
    reorg->setName("YOLOReorg");

    // Concatenate output of main branch and parallel branch
    ITensor* concat_tensors[] = {reorg->getOutput(0), conv19->getOutput(0)};
    ILayer* concat = m_network->addConcatenation(concat_tensors, 2);
    ASSERT(concat); // 28
    concat->setName("concat");

    const int conv21_num_filters = 1024;
    ILayer* conv21 = m_convs[21]("conv21", m_network, *m_weights, *concat->getOutput(0), conv21_num_filters, DimsHW{3, 3});
    ASSERT(conv21); // 29

    // last conv layer is convolution only (no batch norm, no activation)
    DimsHW conv22_kernel_size{1, 1};
    const int conv22_num_filters = m_num_anchors * (1 + m_num_coords + m_num_classes);
    Weights conv22_biases = m_weights->get(conv22_num_filters);
    Weights conv22_weights = m_weights->get(conv22_num_filters * conv21_num_filters * conv22_kernel_size.h() * conv22_kernel_size.w());
    ILayer* conv22 = m_network->addConvolution(*conv21->getOutput(0), conv22_num_filters, conv22_kernel_size, conv22_weights,
                                               conv22_biases);
    ASSERT(conv22); // 30
    conv22->setName("conv22");

    // Region layer
    plugin::RegionParameters region_params;
    region_params.num = m_num_anchors;
    region_params.coords = m_num_coords;
    region_params.classes = m_num_classes;
    region_params.smTree = nullptr;
    m_region_plugin = std::unique_ptr<::plugin::INvPlugin, decltype(nvPluginDeleter)>(
                                                    plugin::createYOLORegionPlugin(region_params), nvPluginDeleter);
    ASSERT(m_region_plugin);
    ITensor* conv22_tensor = conv22->getOutput(0);
    ILayer* region = m_network->addPlugin(&conv22_tensor, 1, *m_region_plugin);
    ASSERT(region); // 31
    region->setName("YOLORegion");

    // Set the network output
    region->getOutput(0)->setName(m_output_blob_name.c_str());
    m_network->markOutput(*region->getOutput(0));

    return m_network;
}
