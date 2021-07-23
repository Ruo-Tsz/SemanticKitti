cmake_minimum_required(VERSION 2.8.3)
project(kitti_segment_filtering)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
)

catkin_package(
   CATKIN_DEPENDS 
    roscpp
    rospy
    sensor_msgs
   INCLUDE_DIRS include
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

## test read kittiSemantic bag
add_executable(
   segment_cloud_filter
   src/segment_cloud_filter.cpp
   src/visualization/color_table.cpp
 )

 target_link_libraries(
   segment_cloud_filter
   ${catkin_LIBRARIES}
 )

## encode as original table color
add_executable(
   colorKittiLabel
   src/visualization/colorKittiLabel.cpp
   src/visualization/color_table.cpp
 )

 target_link_libraries(
   colorKittiLabel
   ${catkin_LIBRARIES}
 )