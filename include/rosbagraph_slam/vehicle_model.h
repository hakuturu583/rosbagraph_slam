#ifndef VEHICLE_MODEL_H_INCLUDED
#define VEHICLE_MODEL_H_INCLUDED

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

// headers in Boost
#include <boost/optional.hpp>

// headers in ROS
#include <geometry_msgs/TwistStamped.h>

// headers in STL
#include <algorithm>

class VehicleModel {
public:
  VehicleModel();
  ~VehicleModel();
  void addTwist(geometry_msgs::TwistStamped twist);
  boost::optional<geometry_msgs::TwistStamped>
  estimateTwist(ros::Time target_stamp);

private:
  std::vector<geometry_msgs::TwistStamped> buf_;
  bool isOlderTimestamp(geometry_msgs::TwistStamped a,
                        geometry_msgs::TwistStamped b) {
    return a.header.stamp < b.header.stamp;
  };
  void sortBuffer();
};
#endif // VEHICLE_MODEL_H_INCLUDED