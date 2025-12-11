cmake_minimum_required(VERSION 2.8.3)
project(dbehavior)

set(dmsgs_DIR /home/nvidia/robocup_ws/lib/devel/share/dmsgs/cmake)

find_package(catkin REQUIRED
  COMPONENTS 
  rospy
  dmsgs
)

catkin_package(
    #INCLUDE_DIRS include
#  LIBRARIES dbehavior
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

