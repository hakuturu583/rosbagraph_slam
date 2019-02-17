cmake_minimum_required(VERSION 2.8.3)
project(rosbagraph_slam)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  pcl_ros
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf2
  tf2_ros
)

find_package(Ceres REQUIRED)

catkin_package(
  INCLUDE_DIRS include/rosbagraph_slam
  LIBRARIES rosbagraph_slam
  CATKIN_DEPENDS pcl_ros roscpp rospy sensor_msgs std_msgs tf2 tf2_ros
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(rosbagraph_slam_node 
  src/submap.cpp
  src/rosbagraph_slam_node.cpp 
  src/rosbagraph_slam.cpp 
  src/feature_matching.cpp
  src/ray_ground_filter.cpp
  src/rosbag_reader.cpp)
target_link_libraries(rosbagraph_slam_node ${catkin_LIBRARIES})