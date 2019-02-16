#ifndef ROSBAGRAPHSLAM_H_INCLUDED
#define ROSBAGRAPHSLAM_H_INCLUDED

//headers in ROS
#include <ros/ros.h>

//headers in this package
#include "rosbag_reader.h"

//headers in Boost
#include <boost/shared_ptr.hpp>

class RosbagraphSlam
{
public:
    RosbagraphSlam(ros::NodeHandle nh,ros::NodeHandle pnh);
    ~RosbagraphSlam();
private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    std::string rosbag_filepath_;
    std::string pointcloud_topic_;
    boost::shared_ptr<RosbagReader> reader_ptr_;
};

#endif  //ROSBAGRAPHSLAM_H_INCLUDED