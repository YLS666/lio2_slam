#pragma once

#define PCL_NO_PRECOMPILE

#include "cloud_utils/point_type.hpp"

#include <ikd-Tree/ikd_Tree.h>
#include <pcl/point_cloud.h>

class KDTreeMap {
 public:
  KDTreeMap() : ikd_tree_(0.3f, 0.6f, 0.2f) { map_cloud_.reset(new pcl::PointCloud<PointType>()); }

  void addCloud(const pcl::PointCloud<PointType>::Ptr& cloud);

  bool nearestSearch(const PointType& pt, PointType& nearest_pt, float& dist);

  pcl::PointCloud<PointType>::Ptr getMap();

 private:
  pcl::PointCloud<PointType>::Ptr map_cloud_;
  KD_TREE<PointType> ikd_tree_;
  bool initialized_ = false;
};