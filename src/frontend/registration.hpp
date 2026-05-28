#pragma once

#include "frontend/state.hpp"
#include "frontend/voxel_map.hpp"

class Registration {
 public:
  Registration();

  bool align(const pcl::PointCloud<PointType>::Ptr& cloud, VoxelMap* map, State& state);

 private:
  Eigen::Matrix3d skew(const Eigen::Vector3d& v);
};