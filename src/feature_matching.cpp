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

#include <rosbagraph_slam/feature_matching.h>

FeatureMatching::FeatureMatching(double kdtree_radius) {
  kdtree_radius_ = kdtree_radius;
}

FeatureMatching::~FeatureMatching() {}

void FeatureMatching::matching(pcl::PointCloud<PointType>::Ptr cloud1,
                               pcl::PointCloud<PointType>::Ptr cloud2) {
  float resolution = static_cast<float>(computeCloudResolution(cloud1));
  //
  //  Compute Normals
  //
  pcl::NormalEstimationOMP<PointType, NormalType> norm_est;
  norm_est.setKSearch(10);
  norm_est.setInputCloud(cloud1);
  //
  //  Extract Keypoints
  //
  pcl::PointCloud<PointType>::Ptr keypoints1 = getKeypoints(cloud1);
  pcl::PointCloud<PointType>::Ptr keypoints2 = getKeypoints(cloud2);
}

pcl::PointCloud<PointType>::Ptr
FeatureMatching::getKeypoints(pcl::PointCloud<PointType>::Ptr cloud) {
  pcl::PointCloud<PointType>::Ptr keypoints(new pcl::PointCloud<PointType>());
  pcl::HarrisKeypoint3D<PointType, PointType> detector;
  detector.setNonMaxSupression(true);
  detector.setRadius(computeCloudResolution(cloud));
  detector.setInputCloud(cloud);
  detector.compute(*keypoints);
  return keypoints;
}

pcl::PointCloud<pcl::Normal>::Ptr
FeatureMatching::surfaceNormals(pcl::PointCloud<PointType>::Ptr cloud) {
  pcl::NormalEstimation<PointType, pcl::Normal> ne;
  ne.setInputCloud(cloud);
  pcl::search::KdTree<PointType>::Ptr tree(
      new pcl::search::KdTree<PointType>());
  ne.setSearchMethod(tree);
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(
      new pcl::PointCloud<pcl::Normal>);
  ne.setRadiusSearch(kdtree_radius_);
  ne.compute(*cloud_normals);
  return cloud_normals;
}

double FeatureMatching::computeCloudResolution(
    pcl::PointCloud<PointType>::Ptr &cloud) {
  double res = 0.0;
  int n_points = 0;
  int nres;
  std::vector<int> indices(2);
  std::vector<float> sqr_distances(2);
  pcl::search::KdTree<PointType> tree;
  tree.setInputCloud(cloud);
  for (size_t i = 0; i < cloud->size(); ++i) {
    if (!std::isfinite((*cloud)[i].x)) {
      continue;
    }
    // Considering the second neighbor since the first is the point itself.
    nres = tree.nearestKSearch(i, 2, indices, sqr_distances);
    if (nres == 2) {
      res += sqrt(sqr_distances[1]);
      ++n_points;
    }
  }
  if (n_points != 0) {
    res /= n_points;
  }
  return res;
}