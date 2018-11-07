cmake_minimum_required(VERSION 3.5)
project(imu_nav)

# enable C++11 standard (current directory scope)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build." FORCE)
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release"
    "MinSizeRel" "RelWithDebInfo")
endif()

set(CMAKE_CXX_FLAGS "-Wall -Wextra")
# set(CMAKE_CXX_FLAGS_DEBUG "-g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

find_package(yaml-cpp REQUIRED)

find_package(Eigen3 REQUIRED)

find_package(catkin REQUIRED COMPONENTS 
  roscpp
  std_msgs
  serial # requires ros-kinetic-serial pacakge
)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES rs2_ros
#  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)


add_executable(calibration 
  src/calibration.cpp
)

add_executable(dead_reckoning_demo 
  src/dead_reckoning_demo.cpp
)

include_directories(
    include
    ${YAML_CPP_INCLUDE_DIR}
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
) # to find ros/ros.h


target_link_libraries(calibration
  PRIVATE
    ${catkin_LIBRARIES}
    ${YAML_CPP_LIBRARIES}
)

target_link_libraries(dead_reckoning_demo
  PRIVATE
    ${catkin_LIBRARIES}
    ${YAML_CPP_LIBRARIES}
)