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
#include <rosbagraph_slam/vehicle_model.h>

VehicleModel::VehicleModel() {}

VehicleModel::~VehicleModel() {}

boost::optional<geometry_msgs::TwistStamped>
VehicleModel::estimateTwist(ros::Time target_stamp) {
  sortBuffer();
  if (buf_.size() == 0) {
    return boost::none;
  }
  if (buf_.begin()->header.stamp > target_stamp) {
    return boost::none;
  }
  if (buf_.end()->header.stamp < target_stamp) {
    if (buf_.size() == 1) {
      geometry_msgs::TwistStamped twist;
      twist.header.stamp = target_stamp;
      twist.twist = buf_.end()->twist;
      return twist;
    } else {
      geometry_msgs::TwistStamped twist;
      twist.header.stamp = target_stamp;
      twist.twist.linear.x =
          (buf_.end()->twist.linear.x - ((buf_.end() - 1)->twist.linear.x)) /
              (buf_.end()->header.stamp - ((buf_.end() - 1)->header.stamp))
                  .toSec() *
              (target_stamp - buf_.end()->header.stamp).toSec() +
          buf_.end()->twist.linear.x;
      twist.twist.linear.y =
          (buf_.end()->twist.linear.y - ((buf_.end() - 1)->twist.linear.y)) /
              (buf_.end()->header.stamp - ((buf_.end() - 1)->header.stamp))
                  .toSec() *
              (target_stamp - buf_.end()->header.stamp).toSec() +
          buf_.end()->twist.linear.y;
      twist.twist.linear.z =
          (buf_.end()->twist.linear.z - ((buf_.end() - 1)->twist.linear.z)) /
              (buf_.end()->header.stamp - ((buf_.end() - 1)->header.stamp))
                  .toSec() *
              (target_stamp - buf_.end()->header.stamp).toSec() +
          buf_.end()->twist.linear.z;
      twist.twist.angular.x =
          (buf_.end()->twist.angular.x - ((buf_.end() - 1)->twist.angular.x)) /
              (buf_.end()->header.stamp - ((buf_.end() - 1)->header.stamp))
                  .toSec() *
              (target_stamp - buf_.end()->header.stamp).toSec() +
          buf_.end()->twist.angular.x;
      twist.twist.angular.y =
          (buf_.end()->twist.angular.y - ((buf_.end() - 1)->twist.angular.y)) /
              (buf_.end()->header.stamp - ((buf_.end() - 1)->header.stamp))
                  .toSec() *
              (target_stamp - buf_.end()->header.stamp).toSec() +
          buf_.end()->twist.angular.y;
      twist.twist.angular.z =
          (buf_.end()->twist.angular.z - ((buf_.end() - 1)->twist.angular.z)) /
              (buf_.end()->header.stamp - ((buf_.end() - 1)->header.stamp))
                  .toSec() *
              (target_stamp - buf_.end()->header.stamp).toSec() +
          buf_.end()->twist.angular.z;
      return twist;
    }
    if (buf_.end()->header.stamp > target_stamp) {
      if (buf_.size() == 1) {
        geometry_msgs::TwistStamped twist;
        twist.header.stamp = target_stamp;
        twist.twist = buf_.end()->twist;
        return twist;
      } else {
        geometry_msgs::TwistStamped twist;
        twist.header.stamp = target_stamp;
        for (auto itr = buf_.begin(); itr != buf_.end() - 1; itr++) {
          if (itr->header.stamp < target_stamp &&
              (itr + 1)->header.stamp > target_stamp) {
            geometry_msgs::TwistStamped twist;
            twist.header.stamp = target_stamp;
            twist.twist.linear.x =
                ((itr->twist.linear.x *
                      ((itr + 1)->header.stamp - target_stamp).toSec() +
                  (itr + 1)->twist.linear.x *
                      (target_stamp - itr->header.stamp).toSec())) /
                ((itr + 1)->header.stamp - (itr)->header.stamp).toSec();
            twist.twist.linear.y =
                (itr->twist.linear.y *
                     ((itr + 1)->header.stamp - target_stamp).toSec() +
                 (itr + 1)->twist.linear.y *
                     (target_stamp - itr->header.stamp).toSec()) /
                ((itr + 1)->header.stamp - (itr)->header.stamp).toSec();
            twist.twist.linear.z =
                (itr->twist.linear.z *
                     ((itr + 1)->header.stamp - target_stamp).toSec() +
                 (itr + 1)->twist.linear.z *
                     (target_stamp - itr->header.stamp).toSec()) /
                ((itr + 1)->header.stamp - (itr)->header.stamp).toSec();
            twist.twist.angular.x =
                ((itr->twist.angular.x *
                      ((itr + 1)->header.stamp - target_stamp).toSec() +
                  (itr + 1)->twist.angular.x *
                      (target_stamp - itr->header.stamp).toSec())) /
                ((itr + 1)->header.stamp - (itr)->header.stamp).toSec();
            twist.twist.angular.y =
                (itr->twist.angular.y *
                     ((itr + 1)->header.stamp - target_stamp).toSec() +
                 (itr + 1)->twist.angular.y *
                     (target_stamp - itr->header.stamp).toSec()) /
                ((itr + 1)->header.stamp - (itr)->header.stamp).toSec();
            twist.twist.angular.z =
                (itr->twist.angular.z *
                     ((itr + 1)->header.stamp - target_stamp).toSec() +
                 (itr + 1)->twist.angular.z *
                     (target_stamp - itr->header.stamp).toSec()) /
                ((itr + 1)->header.stamp - (itr)->header.stamp).toSec();
            return twist;
          }
        }
      }
    }
  }
  return boost::none;
}

void VehicleModel::addTwist(geometry_msgs::TwistStamped twist) {
  buf_.push_back(twist);
  sortBuffer();
  return;
}

void VehicleModel::sortBuffer() {
  std::sort(buf_.begin(), buf_.end(),
            std::bind(&VehicleModel::isOlderTimestamp, this,
                      std::placeholders::_1, std::placeholders::_2));
  return;
}