cmake_minimum_required(VERSION 2.8.3)
project(dtransmit)
add_definitions(-std=c++11 -Wall -O2 -Wextra -Wno-unused-result) #-Wfloat-equal -Weffc++ -pedantic

find_package(catkin REQUIRED roscpp)

find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
  INCLUDE_DIRS include ${Boost_INCLUDE_DIRS}
  LIBRARIES dtransmit ${Boost_LIBRARIES}
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

add_library(dtransmit
  src/dtransmit.cpp
  )
target_link_libraries(dtransmit ${catkin_LIBRARIES} ${Boost_LIBRARIES})


## Add gtest based cpp test target and link libraries
if (CATKIN_ENABLE_TESTING)
    catkin_add_gtest(${PROJECT_NAME}_utest
        test/utest.cpp)
    target_link_libraries(${PROJECT_NAME}_utest
        ${PROJECT_NAME}
        ${catkin_LIBRARIES})

    add_executable(test_dtransmit test/test_dtransmit.cpp)
    target_link_libraries(test_dtransmit dtransmit)

    add_executable(test_recv test/test_recv.cpp)
    target_link_libraries(test_recv dtransmit)

    add_executable(test_send test/test_send.cpp)
    target_link_libraries(test_send dtransmit)
endif ()
