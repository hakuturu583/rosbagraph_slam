// headers in ROS
#include <ros/ros.h>

//headers in rosbgraph_slam
#include <rosbagraph_slam/rosbagraph_slam.h>

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "rosbagraphslam_node");
  ros::spin();
  return 0;
}