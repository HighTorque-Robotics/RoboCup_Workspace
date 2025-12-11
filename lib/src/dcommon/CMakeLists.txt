cmake_minimum_required(VERSION 2.8.3)
project(dcommon)

add_compile_options(-std=c++11)
add_compile_options(-W -Wall)
add_compile_options(-O2)
#add_compile_options(-g)

find_package(catkin REQUIRED
    COMPONENTS
    roscpp
    )

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
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
)

## Declare a C++ library
add_library(${PROJECT_NAME} SHARED
    src/util.cpp
    src/profiler.cpp
    )

target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
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
