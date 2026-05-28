#include "frontend/kdtree_map.hpp"
#include <vector>

void KDTreeMap::addCloud(const pcl::PointCloud<PointType>::Ptr& cloud) {
  *map_cloud_ += *cloud;
  std::cout << "map_cloud_ size: " << map_cloud_->size() << std::endl;

  KD_TREE<PointType>::PointVector points;
  points.reserve(cloud->size());

  for (const auto& pt : cloud->points) {
    points.push_back(pt);
  }

  // 第一次:
  if (!initialized_) {
    ikd_tree_.Build(points);
    initialized_ = true;
    return;
  }

  // 后续:
  ikd_tree_.Add_Points(points, true);
}

bool KDTreeMap::nearestSearch(const PointType& pt, PointType& nearest_pt, float& dist) {
  KD_TREE<PointType>::PointVector nearest_points;
  std::vector<float> points_dist;

  ikd_tree_.Nearest_Search(pt, 1, nearest_points, points_dist, 10.0f);
  if (nearest_points.empty()) {
    return false;
  }
  nearest_pt = nearest_points[0];
  dist = points_dist[0];

  return true;
}

pcl::PointCloud<PointType>::Ptr KDTreeMap::getMap() { return map_cloud_; }