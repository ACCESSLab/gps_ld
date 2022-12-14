cmake_minimum_required(VERSION 3.0.2)
project(gps_ld)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)
set(CMAKE_BUILD_TYPE "Release")

## Find catkin macros and libraries
find_package(OpenCV 4 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
  cv_bridge
  image_transport
  # aggies_lib
)

find_package(PkgConfig REQUIRED)

###################################
## catkin specific configuration ##
###################################
catkin_package(
   INCLUDE_DIRS include
  #  LIBRARIES GPS_LD
   CATKIN_DEPENDS roscpp rospy sensor_msgs std_msgs
  #  DEPENDS system_lib
)

###########
## Build ##
###########
include_directories(
  # include
  include/gps_ld
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(
  gps_ld
  src/gps_ld.cpp
  src/lane_detector.cpp
  src/lanemarking_extractor.cpp
  src/helpers.cpp
)

target_link_libraries(
  gps_ld 
  ${catkin_LIBRARIES} 
  ${OpenCV_LIBRARIES} 
)