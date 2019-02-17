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
#include <rosbagraph_slam/rosbag_reader.h>

RosbagReader::RosbagReader(std::string target_rosbag_path)
    : target_rosbag_path_(target_rosbag_path) {
  if (!rosbag_file_exist_()) {
    std::exit(-1);
  }
}

RosbagReader::~RosbagReader() {}

bool RosbagReader::rosbag_file_exist_() {
  namespace fs = boost::filesystem;
  fs::path path(target_rosbag_path_);
  boost::system::error_code error;
  const bool result = fs::exists(path, error);
  if (!result || error) {
    ROS_ERROR_STREAM("rosbag path is invalid.");
    return false;
  }
  return true;
}

std::vector<sensor_msgs::PointCloud2::ConstPtr>
RosbagReader::read(std::string pointcloud_topic) {
  std::vector<sensor_msgs::PointCloud2::ConstPtr> ret;
  rosbag::Bag bag;
  bag.open(target_rosbag_path_, rosbag::bagmode::Read);
  std::vector<std::string> topics;
  topics.push_back(pointcloud_topic);
  rosbag::View view(bag, rosbag::TopicQuery(topics));
  foreach (rosbag::MessageInstance const m, view) {
    sensor_msgs::PointCloud2::ConstPtr pc =
        m.instantiate<sensor_msgs::PointCloud2>();
    ret.push_back(pc);
  }
  return ret;
}