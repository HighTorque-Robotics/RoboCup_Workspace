cmake_minimum_required(VERSION 2.8.3)
project(dprocess)
 add_definitions(-std=c++11)

find_package(catkin REQUIRED roscpp)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES dprocess
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(dprocess
  src/dconcurrent.cpp include/dprocess/dconcurrent.hpp)

catkin_add_gtest(test_dconcurrent test/test_dconcurrent.cpp)
target_link_libraries(test_dconcurrent dprocess)
target_link_libraries(test_dconcurrent ${catkin_LIBRARIES})

catkin_add_gtest(test_dprocess test/test_dprocess.cpp)
target_link_libraries(test_dprocess dprocess)
target_link_libraries(test_dprocess ${catkin_LIBRARIES})