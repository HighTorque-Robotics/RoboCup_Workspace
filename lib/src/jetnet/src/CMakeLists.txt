set (HEADERS
    jetnet.h
    data/cv_letterbox_pre_processor.h
    data/detection.h
    data/fake_post_processor.h
    data/fake_pre_processor.h
    data/int8_calibrator.h
    data/letterbox_int8_calibrator.h
    data/nms.h
    data/yolo_post_processor.h
    engine/model_builder.h
    engine/model_runner.h
    engine/model_runner_impl.h
    engine/gpu_blob.h
    models/darknet_weights_loader.h
    models/yolov2_builder.h
    models/yolov3_builder.h
    models/yolov3_tiny_builder.h
    models/layers/relu.h
    models/layers/leaky_relu_native.h
    models/layers/leaky_relu_plugin.h
    models/layers/conv2d_batch_leaky.h
    models/layers/conv2d_batch_leaky_impl.h
    models/plugins/blas_cuda.h
    models/plugins/upsample_plugin.h
    models/plugins/yolo_plugin_factory.h
    utils/file_io.h
    utils/fp16.h
    utils/logger.h
    utils/profiler.h
    utils/visual.h
    utils/custom_assert.h
)

set (SOURCES
    data/cv_letterbox_pre_processor.cpp
    data/fake_post_processor.cpp
    data/fake_pre_processor.cpp
    data/int8_calibrator.cpp
    data/letterbox_int8_calibrator.cpp
    data/nms.cpp
    data/yolo_post_processor.cpp
    engine/model_builder.cpp
    engine/gpu_blob.cpp
    models/darknet_weights_loader.cpp
    models/yolov2_builder.cpp
    models/yolov3_builder.cpp
    models/yolov3_tiny_builder.cpp
    models/layers/relu.cpp
    models/layers/leaky_relu_native.cpp
    models/layers/leaky_relu_plugin.cpp
    models/plugins/upsample_plugin.cpp
    models/plugins/blas_cuda.cu
    models/plugins/yolo_plugin_factory.cpp
    utils/file_io.cpp
    utils/fp16.cpp
    utils/logger.cpp
    utils/profiler.cpp
    utils/visual.cpp
    utils/custom_assert.cpp
)

set (TARGETS_EXPORT_NAME "${PROJECT_NAME}")

# install directory paths
set (include_dest "include/jetnet")
set (package_dest "share/jetnet")
set (lib_dest "lib")

# project config file name
# this is the file that will be searched for by find_package when this library is used
# in other projects
set (project_config "${CMAKE_CURRENT_BINARY_DIR}/generated/${PROJECT_NAME}-config.cmake")

set (SRC ${SOURCES} ${HEADERS})

set (TARGET_INCLUDE_DIRS

    # system includes
    ${CUDA_INCLUDE_DIRS};
    ${OpenCV_INCLUDE_DIRS};

    # includes for building only
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>;
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/data>;
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/engine>;
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/models>;
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/models/layers>;
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/models/plugins>;
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/utils>;

    # includes used when library is installed and used by other project
    $<INSTALL_INTERFACE:${include_dest}>;
    CACHE INTERNAL ""
)

set (TARGET_LIBS
    m;
    nvinfer_plugin;
    nvinfer;
    ${CUDA_LIBRARIES};
    ${CUDA_CUBLAS_LIBRARIES};
    ${OpenCV_LIBS};
    CACHE INTERNAL ""
)

add_library (jetnet_static STATIC ${SRC})
add_library (jetnet SHARED ${SRC})

target_include_directories (jetnet_static PUBLIC ${TARGET_INCLUDE_DIRS})
target_include_directories (jetnet PUBLIC ${TARGET_INCLUDE_DIRS})

set_target_properties( jetnet_static PROPERTIES CUDA_SEPARABLE_COMPILATION ON)
set_target_properties( jetnet PROPERTIES CUDA_SEPARABLE_COMPILATION ON)

target_link_libraries (jetnet ${TARGET_LIBS})

include (CMakePackageConfigHelpers)

# needs ${TARGETS_EXPORT_NAME} variable
configure_package_config_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/../cmake/config.cmake.in"
    "${project_config}"
    INSTALL_DESTINATION "${package_dest}"
)

# install library targets
install (TARGETS jetnet jetnet_static
    EXPORT "${TARGETS_EXPORT_NAME}"
    DESTINATION "${lib_dest}"
)

# install header files
install (FILES ${HEADERS}
    DESTINATION "${include_dest}"
)

# install <name>.cmake files
install (EXPORT "${TARGETS_EXPORT_NAME}"
    DESTINATION "${package_dest}"
)

# install <name>-config.cmake files
install (FILES ${project_config}
    DESTINATION "${package_dest}"
)
