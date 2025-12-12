cmake_minimum_required(VERSION 2.8.12)
project(dvision)

set(ZJUDANCER_GUI $ENV{ZJUDANCER_GUI})
set(ZJUDANCER_GPU $ENV{ZJUDANCER_GPU})
if (ZJUDANCER_GPU EQUAL 1)
    add_definitions(-DUSE_CUDA)
    MESSAGE("Use CUDA acceleration.")
else ()
    MESSAGE("Ignore CUDA acceleration.")
endif ()

# temperoray flag for camera testing only
set(ZJUDANCER_USE_GSTCAMERA 0)
if (ZJUDANCER_USE_GSTCAMERA EQUAL 1)
    add_definitions(-DUSE_GST_CAMERA)
    MESSAGE("Use camera poewred by GStreamer.")
endif()

# temporary flag for ZED camera testing only
set(ZJUDANCER_USE_ZED_CAMERA 1)
if (ZJUDANCER_USE_ZED_CAMERA EQUAL 1)
    add_definitions(-DUSE_ZED_CAMERA)
    message("Using camera powered by ZED SDK.")
endif()

# temperoray flag for neo_info only
set(ZJUDANCER_USE_NEO_INFO 1)
if (ZJUDANCER_USE_NEO_INFO EQUAL 1)
    add_definitions(-DUSE_NEO_INFO)
    MESSAGE("Use neo_info.")
endif()

# add_definitions(-std=c++11 -Wall -g) # -g -Wextra -Wno-unused-result) #-Wfloat-equal -Weffc++ -pedantic
add_definitions(-std=c++11 -Wall -O2) # -g -Wextra -Wno-unused-result) #-Wfloat-equal -Weffc++ -pedantic
set(CMAKE_CXX_STANDARD 11)

set(DANCER_LIB_PATH /home/nvidia/RoboCup_Workspace/lib/devel/share/)

set(dmsgs_DIR ${DANCER_LIB_PATH}/dmsgs/cmake)
set(dancer_geometry_DIR ${DANCER_LIB_PATH}/dancer_geometry/cmake)
set(dprocess_DIR ${DANCER_LIB_PATH}/dprocess/cmake)
set(dtransmit_DIR ${DANCER_LIB_PATH}/dtransmit/cmake)
set(dcommon_DIR ${DANCER_LIB_PATH}/dcommon/cmake)
set(aruco_DIR ${DANCER_LIB_PATH}/aruco/cmake)
set(fmt_DIR ${DANCER_LIB_PATH}/fmt/cmake)
set(cv_bridge_DIR ${DANCER_LIB_PATH}/cv_bridge/cmake)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}/cmake_modules/")

find_package(catkin REQUIRED
    COMPONENTS
    roscpp
    dmsgs
    dprocess
    dtransmit
    dconfig
    dcommon
    dancer_geometry
    cv_bridge
    aruco
    fmt
    #jetnet
    image_transport
    )

# OpenCV
# message("CATLIB ${catkin_LIBRARIES}")
find_package(OpenCV 4.5.4 REQUIRED)
message("Using OpenCV: ${OpenCV_INCLUDE_DIRS}")
# message("Using OpenCV: ${OpenCV_LIBS}")
# message("aruco:${aruco_LIBS}")

# Eigen3
find_package(Eigen3 REQUIRED)

# Boost
find_package(Boost REQUIRED)
message("Boost FOUND: >>>>>>>>>>>>>>>>>>>>> ${Boost_INCLUDE_DIRS} ${Boost_LIBRARIES} ${Boost_LIBRARY_DIRS}")

# GStreamer
if (ZJUDANCER_USE_GSTCAMERA EQUAL 1)
    include_directories(
        /usr/include/gstreamer-1.0
        /usr/include/glib-2.0
        /usr/include/libxml2
        /usr/lib/aarch64-linux-gnu/gstreamer-1.0/include
        /usr/lib/x86_64-linux-gnu/gstreamer-1.0/include/
        /usr/lib/aarch64-linux-gnu/glib-2.0/include/
        /usr/lib/x86_64-linux-gnu/glib-2.0/include/)
endif()


# tensorrt
include_directories(/usr/include/aarch64-linux-gnu/)
link_directories(/usr/lib/aarch64-linux-gnu/)

# Qt is used to load images (installed by ubuntu-desktop)
if (ZJUDANCER_USE_GSTCAMERA EQUAL 1)
    find_package(Qt4 REQUIRED)
    include(${QT_USE_FILE})
    add_definitions(${QT_DEFINITIONS})
endif()

# CUDA
include_directories(/usr/local/cuda/include)
link_directories(/usr/local/cuda/lib64)
if (ZJUDANCER_GPU EQUAL 1)
    find_package(CUDA)
    message("-- CUDA version: ${CUDA_VERSION}")
    # set(
    #     CUDA_NVCC_FLAGS
    #     ${CUDA_NVCC_FLAGS};
    #     -O3
    #     -gencode arch=compute_50,code=sm_50
    #     -gencode arch=compute_60,code=sm_60
    #     -gencode arch=compute_61,code=sm_61
    #     -gencode arch=compute_62,code=sm_62
    # )
    # if(CUDA_VERSION_MAJOR GREATER 9)
    #     message("-- CUDA ${CUDA_VERSION_MAJOR} detected, enabling SM_72")
    #
    #     set(
    #         CUDA_NVCC_FLAGS
    #         ${CUDA_NVCC_FLAGS};
    #         -gencode arch=compute_72,code=sm_72
    #     )
    # endif()
endif ()

catkin_package(
    INCLUDE_DIRS include ${EIGEN3_INCLUDE_DIR} ${catkin_INCLUDE_DIRS}
    LIBRARIES dvisionlib ${OpenCV_LIBS}
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    # ${ARUCO_INCLUDE_DIRS}
)

if (ZJUDANCER_GPU EQUAL 1)
    file(GLOB dvisionlib_cuda_sources src/cuda/*.cu)
    file(GLOB dvisionlib_cuda_includes include/dvision/cuda/*.h)
    cuda_add_library(dvisionlib_cuda SHARED ${dvisionlib_cuda_sources} include/dvision/yolo/yololayer.cu)
endif ()

# ZED SDK
# set(CMAKE_PREFIX_PATH "/usr/local/zed;/opt/ros/noetic" CACHE PATH "Path to custom packages")
set(CMAKE_MODULE_PATH "/usr/local/zed" ${CMAKE_MODULE_PATH})
find_package(ZED 3 REQUIRED)
find_package(CUDA ${ZED_CUDA_VERSION} REQUIRED)

include_directories(${CUDA_INCLUDE_DIRS})
include_directories(${ZED_INCLUDE_DIRS})

link_directories(${ZED_LIBRARY_DIR})
link_directories(${CUDA_LIBRARY_DIRS})

SET(ZED_LIBS ${ZED_LIBRARIES} ${CUDA_CUDA_LIBRARY} ${CUDA_CUDART_LIBRARY})

add_library(dvisionlib
    # src/dvision.cpp
    src/zed_dvision.cpp
    src/parameters.cpp
    src/camera/misc/frame.cpp
    src/camera/misc/gst_utility.cpp
    src/camera/misc/gst_camera_impl.cpp
    src/camera/source/v4l2_camera.cpp
    src/camera/source/zed_camera.cpp
    src/camera/source/gst_camera.cpp
    src/camera/source/image_folder.cpp
    src/camera/source/video_file.cpp
    src/camera/projection/distortionModel.cpp
    src/camera/projection/ipm.cpp
    src/camera/projection/projection.cpp
    src/detector/object_detector.cpp
    src/detector/ball_detector.cpp
    src/detector/circle_detector.cpp
    src/detector/corner_detector.cpp
    src/detector/goal_detector.cpp
    src/detector/obstacle_detector.cpp
    src/detector/line_classifier.cpp
    src/localization/amcl/amcl.cpp
    src/localization/amcl/pose.cpp
    src/localization/amcl/particle.cpp
    src/localization/amcl/map.cpp
    src/localization/amcl/kdtree.cpp
    src/tools/kalman.cpp
    src/tools/ball_tracker.cpp
    src/yolo/common.cpp
    src/yolo/yolov5.cpp
    src/yolo/utils.cpp

)

target_link_libraries(
    dvisionlib
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
    nvinfer
    cudart
    ${ZED_LIBS}
)
if (ZJUDANCER_GPU EQUAL 1)
    target_link_libraries(
        dvisionlib
        dvisionlib_cuda
    )
endif ()

if (ZJUDANCER_USE_GSTCAMERA EQUAL 1)
    target_link_libraries(
        dvisionlib
        Qt4::QtGui
        gstreamer-1.0 gstapp-1.0
    )
endif ()

add_executable(dvision_node src/main.cpp)
target_link_libraries(dvision_node dvisionlib ${catkin_LIBRARIES})

add_executable(capture src/camera/misc/capture.cpp)
target_link_libraries(capture dvisionlib)

add_executable(zed_capture src/camera/misc/zed_capture.cpp)
target_link_libraries(zed_capture dvisionlib)

add_executable(detect src/camera/misc/detect.cpp)
target_link_libraries(detect dvisionlib)

add_executable(zed_detect src/camera/misc/zed_detect.cpp)
target_link_libraries(zed_detect dvisionlib)

add_executable(yolotest test/yolo_test.cpp)
target_link_libraries(yolotest dvisionlib)

add_executable(zed_ext_calib src/camera/calibration/zed_ext_calib.cpp)
target_link_libraries(zed_ext_calib dvisionlib)

add_executable(test_projection test/test_projection.cpp)
target_link_libraries(test_projection dvisionlib ${catkin_LIBRARIES})

if (ZJUDANCER_GUI EQUAL 1)
    add_executable(calib src/camera/calibration/calibration.cpp include/dvision/icamera.hpp)
    target_link_libraries(calib ${OpenCV_LIBS})

    add_executable(fishcalib src/camera/calibration/fisheyecalib.cpp)
    target_link_libraries(fishcalib ${OpenCV_LIBS})
endif ()
message("CATLIB ${dvisionlib}")
#add_executable(videowriter test/test_videowriter.cpp)
#target_link_libraries(videowriter dvisionlib ${Opencv_LIBS})

#add_executable(test_frame test/test_frame.cpp)
#target_link_libraries(test_frame dvisionlib ${OpenCV_LIBS})
#
#add_executable(test_frameSend test/test_frameSend.cpp)
#target_link_libraries(test_frameSend dvisionlib ${OpenCV_LIBS})
#
#add_executable(test_frameRecv test/test_frameRecv.cpp)
#target_link_libraries(test_frameRecv dvisionlib ${OpenCV_LIBS})
### TEST

#catkin_add_gtest(test_amcl test/test_amcl.cpp)
#target_link_libraries(test_amcl dvisionlib)
#target_link_libraries(test_amcl ${catkin_LIBRARIES})
#
#catkin_add_gtest(test_cameraDummy test/test_cameraDummy.cpp)
#target_link_libraries(test_cameraDummy dvisionlib)
#target_link_libraries(test_cameraDummy ${catkin_LIBRARIES})
#
#catkin_add_gtest(test_distortionModel test/test_distortionModel.cpp)
#target_link_libraries(test_distortionModel dvisionlib)
#target_link_libraries(test_distortionModel ${catkin_LIBRARIES})
#
#catkin_add_gtest(test_projection test/test_projection.cpp)
#target_link_libraries(test_projection dvisionlib)
#target_link_libraries(test_projection ${catkin_LIBRARIES})
#
#catkin_add_gtest(test_localization test/test_localization.cpp)
#target_link_libraries(test_localization dvisionlib)
#target_link_libraries(test_localization ${catkin_LIBRARIES})
#
#catkin_add_gtest(test_ringbuffer test/test_ringbuffer.cpp)
##target_link_libraries(test_ringbuffer dvisionlib)
#target_link_libraries(test_ringbuffer ${catkin_LIBRARIES})
#
#add_executable(kalman test/test_kalman.cpp)
#target_link_libraries(kalman dvisionlib)