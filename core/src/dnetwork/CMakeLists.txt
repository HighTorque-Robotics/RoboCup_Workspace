cmake_minimum_required(VERSION 2.8.3)
project(dnetwork)

add_definitions(-std=c++11 -Wall -O2)

set(dmsgs_DIR /home/nvidia/robocup_ws/lib/devel/share/dmsgs/cmake)
set(dtransmit_DIR /home/nvidia/robocup_ws/lib/devel/share/dtransmit/cmake)
set(dprocess_DIR /home/nvidia/robocup_ws/lib/devel/share/dprocess/cmake)

find_package(catkin
  REQUIRED
  roscpp
  dmsgs
  dconfig
  dtransmit
  dprocess
  )

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES dnetwork
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node
  src/main.cpp
  src/team.cpp
  src/gamecontroller.cpp
  )

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)
