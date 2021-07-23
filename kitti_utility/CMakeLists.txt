cmake_minimum_required(VERSION 2.8.3)
project(kitti_utility)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## catkin_python_setup()
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
	roscpp
	rospy
	sensor_msgs
	pcl_ros
	pcl_conversions
	rosbag
)

catkin_package(
   CATKIN_DEPENDS 
	   pcl_ros
	   roscpp
	   sensor_msgs
	   pcl_conversions
	   rosbag
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
   colorKittiLabel
   src/colorKittiLabel.cpp
   src/color_table.cpp
 )

 target_link_libraries(
   colorKittiLabel
   ${catkin_LIBRARIES}
 )

