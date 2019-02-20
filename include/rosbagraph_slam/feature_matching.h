#ifndef FEATURE_MATCHING_H_INCLUDED
#define FEATURE_MATCHING_H_INCLUDED
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

#include "define.h"

// headers in PCL
#include <pcl/features/normal_3d_omp.h>
#include <pcl/keypoints/harris_3d.h>
#include <pcl/search/kdtree.h>

class FeatureMatching {
public:
  FeatureMatching(double kdtree_radius);
  ~FeatureMatching();
  void matching(pcl::PointCloud<PointType>::Ptr cloud1,
                pcl::PointCloud<PointType>::Ptr cloud2);

private:
  double computeCloudResolution(pcl::PointCloud<PointType>::Ptr &cloud);
  pcl::PointCloud<PointType>::Ptr
  getKeypoints(pcl::PointCloud<PointType>::Ptr cloud);
  pcl::PointCloud<pcl::Normal>::Ptr
  surfaceNormals(pcl::PointCloud<PointType>::Ptr cloud);
  double kdtree_radius_;
};

#endif // FEATURE_MATCHING_H_INCLUDED