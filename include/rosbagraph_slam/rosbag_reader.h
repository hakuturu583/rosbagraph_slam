#ifndef ROSBAG_READER_H_INCLUDED
#define ROSBAG_READER_H_INCLUDED

/*
 * Copyright 2019 Masaya Kataoka. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 *
 * v1.0 Masaya Kataoka
 */

//headers in ROS
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

//headers for ROS message
#include <sensor_msgs/PointCloud2.h>

//headers in boost
#include <boost/filesystem.hpp>
#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH

class RosbagReader
{
public:
    RosbagReader(std::string target_rosbag_path);
    ~RosbagReader();
    std::vector<sensor_msgs::PointCloud2::ConstPtr> read(std::string pointcloud_topic);
private:
    const std::string target_rosbag_path_;
    bool rosbag_file_exist_();
};

#endif  //ROSBAG_READER_H_INCLUDED