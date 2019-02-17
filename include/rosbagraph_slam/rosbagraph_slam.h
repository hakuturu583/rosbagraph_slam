#ifndef ROSBAGRAPHSLAM_H_INCLUDED
#define ROSBAGRAPHSLAM_H_INCLUDED

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

// headers in ROS
#include <ros/ros.h>

// headers in this package
#include "rosbag_reader.h"

// headers in Boost
#include <boost/shared_ptr.hpp>

class RosbagraphSlam {
public:
  RosbagraphSlam(ros::NodeHandle nh, ros::NodeHandle pnh);
  ~RosbagraphSlam();

private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  std::string rosbag_filepath_;
  std::string pointcloud_topic_;
  boost::shared_ptr<RosbagReader> reader_ptr_;
};

#endif // ROSBAGRAPHSLAM_H_INCLUDED