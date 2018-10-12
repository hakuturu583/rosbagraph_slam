#ifndef ROSBAG_READER_H_INCLUDED
#define ROSBAG_READER_H_INCLUDED

//headers in ROS
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

//headers for ROS message
#include <nmea_msgs/Sentence.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>

//headers in boost
#include <boost/filesystem.hpp>
#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH

class rosbag_reader
{
public:
    rosbag_reader(std::string target_rosbag_path);
    ~rosbag_reader();
    bool read(std::string nmea_topic, std::string imu_topic, std::string pointcloud_topic);
private:
    const std::string target_rosbag_path_;
    bool rosbag_file_exist_();
};

#endif  //ROSBAG_READER_H_INCLUDED