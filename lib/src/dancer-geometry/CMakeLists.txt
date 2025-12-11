cmake_minimum_required(VERSION 2.8.3)
project(dancer_geometry)

add_compile_options(-std=c++11)
add_compile_options(-W -Wall)
add_compile_options(-O2)
#add_compile_options(-g)

find_package(OpenCV  4.5.4 REQUIRED)
message("Using OpenCV: ${OpenCV_INCLUDE_DIRS}")

find_package(catkin REQUIRED geometry_msgs)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME} ${OpenCV_LIBS}
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${OpenCv_INCLUDE_DIR}
)

## Declare a C++ library
add_library(${PROJECT_NAME} SHARED
    src/angle.cpp
    src/point.cpp
    src/line_segment.cpp
    src/utils.cpp
)

target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
)


#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
if (CATKIN_ENABLE_TESTING)
    catkin_add_gtest(${PROJECT_NAME}_utest
        test/utest.cpp)
    target_link_libraries(${PROJECT_NAME}_utest
        ${PROJECT_NAME}
        ${catkin_LIBRARIES})
endif ()
