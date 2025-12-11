cmake_minimum_required(VERSION 2.8.3)
project(ros_h264_streamer)

find_package(catkin REQUIRED COMPONENTS roscpp image_transport cv_bridge)
include_directories(${catkin_INCLUDE_DIRS})
find_package(OpenCV 4.5.4 REQUIRED)
find_package(Boost REQUIRED COMPONENTS program_options)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    DEPENDS roscpp image_transport
)

set(ros_h264_streamer_src
  src/h264_encoder.cpp
  src/h264_decoder.cpp
#  src/h264_streamer.cpp
#  src/h264_receiver.cpp
)

add_library(ros_h264_streamer ${ros_h264_streamer_src})
target_link_libraries(ros_h264_streamer ${catkin_LIBRARIES} ${OpenCV_LIBS} x264 avcodec avutil swscale)

## Test encoder/decoder requires opencv, we don't pull this dependency for win32
#add_executable(testEncoderDecoder test/testEncoderDecoder.cpp)
#target_link_libraries(testEncoderDecoder ros_h264_streamer ${catkin_LIBRARIES} ${OpenCV_LIBS} )
##Obviously streamer relies way too much on ROS
#add_executable(testStreamer test/testStreamer.cpp)
#target_link_libraries(testStreamer ros_h264_streamer ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OpenCV_LIBS} )
##Idem for mkv_recorder
#add_executable(mkv_recorder src/utils/mkv_recorder.cpp src/utils/output/matroska_ebml.c src/utils/output/matroska.c)
#target_link_libraries(mkv_recorder ros_h264_streamer ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OpenCV_LIBS} )
#
#add_executable(testReceiver test/testReceiver.cpp)
#target_link_libraries(testReceiver ros_h264_streamer ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OpenCV_LIBS} )
