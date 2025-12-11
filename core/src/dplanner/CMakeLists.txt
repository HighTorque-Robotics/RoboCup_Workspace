cmake_minimum_required(VERSION 3.0.2)
project(dplanner)

# add_definitions(-std=c++11 -Wall -g)
add_definitions(-std=c++17 -Wall -g)

set(dmsgs_DIR ../../../lib/devel/share/dmsgs/cmake)
# set(dmsgs_DIR /home/nvidia/robocup_ws/lib/devel/share/dmsgs/cmake)
# set(sim2real_msg_DIR /home/nvidia/Desktop/totally_new/sim2real_master/devel/share/sim2real_msg/cmake)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  dmsgs
  # sim2real_msg
)
find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES DWAPlanner
  # CATKIN_DEPENDS roscpp rospy std_msgs dmsgs sim2real_msg
  CATKIN_DEPENDS roscpp rospy std_msgs dmsgs
  DEPENDS EIGEN3
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS} 

)

add_library(DWAPlanner
  src/DWAPlanner.cpp
)

target_link_libraries(DWAPlanner
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES}
)

add_executable(motion_hub_communicator src/motion_hub_communicator.cpp)

add_dependencies(motion_hub_communicator ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(motion_hub_communicator
  DWAPlanner
  ${catkin_LIBRARIES}
)
