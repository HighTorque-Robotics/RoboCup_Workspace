#ifndef JETNET_MODEL_BUILDER_H
#define JETNET_MODEL_BUILDER_H

#include "logger.h"
#include <NvInfer.h>
#include <string>
#include <memory>
#include <vector>

namespace jetnet
{

class ModelBuilder
{
public:
    /*
     *  Init stuff
     *  logger: all internal errors and info messages are send to this logger
     *  Returns true on success, false on failure
     */
    bool init(std::shared_ptr<Logger> logger);

    /*
     *  Check if this platform supports mixed precision 16-bit float
     */
    bool platform_supports_fp16();

    /*
     *  Check if this platform supports mixed precision 8-bit ints
     */
    bool platform_supports_int8();

    /*
     *  Set the builder in 16-bit float mode (16-bit kernels are allowed to be used,
     *  this is however not guaranteed)
     *  (see TensorRT developers guide)
     */
    void platform_set_fp16_mode();

    /*
     *  Set the builder in 8-bit int mode
     *  calibrator: calibrator class for calibrating the int8 weights
     */
    void platform_set_int8_mode(nvinfer1::IInt8Calibrator* calibrator);

    /*
     *	Build network for DLA i.s.o. GPU. Calling this method will also
     *	enable GPU fallback in case a layer cannot be executed on DLA (plugins)
     *	NOTE: only supported for Xavier platform
     *	device: DLA device id (0 or 1 for Xavier platform)
     */
    void platform_use_dla(int device);

    /*
     *  If layer types are set to specific precision, use that even if it results
     *  in lower time performance
     */
    void enable_type_strictness();

    /*
     *  Execute layers that start with a specific name with precision 'type'
     *  If however, that specific layer is faster in a higher precision than requested
     *  or the requested precision does not exist, this request will be ignored.
     *  Used 'enable_type_strictness' to override this behavior and use the requested
     *  precisions regardless of there performance.
     *  layers: list of layer names (or there first letters)
     *  type:   precision to set these layers too
     *  invert: if true, set all layers to this precision except for the layers requested
     *          (inverted action)
     */
    void set_layer_precision(std::vector<std::string> layers,
                             nvinfer1::DataType type, bool invert=false);

    /*
     *  Generic parser method that creates the network definition.
     *  Derive from this class to implement your parser. Use the protected
     *  m_network attribute to define your network.
     *  dt: data type for the weights. This param determines the mixed precision
     *      mode for the final network
     *  Must return the network definition on success, nullptr on failure
     */
    virtual nvinfer1::INetworkDefinition* parse(nvinfer1::DataType dt) = 0;

    /*
     *  Build an execution engine. Call this method after parsing
     *  maxBatchSize:   maximum batch size the network should be able to handle
     *  Returns nullptr on failure, the cuda engine on success
     */
    nvinfer1::ICudaEngine* build(int maxBatchSize);

    /*
     *  Create a graphical inference engine stream that can be send to a file.
     *  Call this method after building
     *  Returns nullptr on failure, the model stream on success
     */
    nvinfer1::IHostMemory* serialize();

    /*
     *  Create a graphical inference engine stream and send it to a file
     *  Call this method after building
     *  Returns nullptr on failure, the model stream on success
     */
    nvinfer1::IHostMemory* serialize(std::string filename);

protected:
    std::shared_ptr<Logger> m_logger = nullptr;
    nvinfer1::IBuilder* m_builder = nullptr;
    nvinfer1::ICudaEngine* m_cuda_engine = nullptr;
    nvinfer1::INetworkDefinition* m_network = nullptr;
};

}

#endif /* JETNET_MODEL_BUILDER_H */
