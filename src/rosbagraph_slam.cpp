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

// headers in rosbgraph_slam
#include <rosbagraph_slam/rosbagraph_slam.h>

RosbagraphSlam::RosbagraphSlam(ros::NodeHandle nh, ros::NodeHandle pnh) {
  nh_ = nh;
  pnh_ = pnh;
  pnh_.param<std::string>("rosbag_filepath", rosbag_filepath_, "");
  pnh_.param<std::string>("pointcloud_topic", pointcloud_topic_, "/points_raw");
  reader_ptr_ = boost::make_shared<RosbagReader>(rosbag_filepath_);
  std::vector<sensor_msgs::PointCloud2::ConstPtr> point_clouds =
      reader_ptr_->read(pointcloud_topic_);
  ROS_INFO_STREAM("read rosbag file finished, " << point_clouds.size()
                                                << " point clouds were found.");
}

RosbagraphSlam::~RosbagraphSlam() {}