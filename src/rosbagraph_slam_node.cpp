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

// headers in rosbgraph_slam
#include <rosbagraph_slam/rosbagraph_slam.h>

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "rosbagraphslam_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  RosbagraphSlam slam(nh, pnh);
  return 0;
}