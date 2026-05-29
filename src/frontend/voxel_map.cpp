#include "frontend/voxel_map.hpp"
#include <Eigen/src/Core/Matrix.h>

#include <cmath>
#include <limits>

VoxelMap::VoxelMap(float voxel_size, float local_radius) : voxel_size_(voxel_size), local_radius_(local_radius) {}

size_t VoxelMap::size() const { return voxel_map_.size(); }

VoxelKey VoxelMap::pointToVoxel(const PointType& pt) const {
  return {static_cast<int>(std::floor(pt.x / voxel_size_)), static_cast<int>(std::floor(pt.y / voxel_size_)),
          static_cast<int>(std::floor(pt.z / voxel_size_))};
}

void VoxelMap::addCloud(const pcl::PointCloud<PointType>::Ptr& cloud, const Eigen::Vector3d& center) {
  std::lock_guard<std::mutex> lock(mutex_);

  for (const auto& pt : cloud->points) {
    VoxelKey key = pointToVoxel(pt);

    auto iter = voxel_map_.find(key);
    if (iter == voxel_map_.end()) {
      voxel_map_.insert({key, pt});
    }
  }

  removeFarVoxels(center);
}

bool VoxelMap::nearestSearch(const PointType& pt, PointType& nearest_pt, float& nearest_dist) {
  std::lock_guard<std::mutex> lock(mutex_);

  VoxelKey center = pointToVoxel(pt);

  nearest_dist = std::numeric_limits<float>::max();

  bool found = false;

  for (int dx = -1; dx <= 1; ++dx) {
    for (int dy = -1; dy <= 1; ++dy) {
      for (int dz = -1; dz <= 1; ++dz) {
        VoxelKey key{center.x + dx, center.y + dy, center.z + dz};

        auto iter = voxel_map_.find(key);

        if (iter == voxel_map_.end()) {
          continue;
        }

        const auto& candidate = iter->second;

        float rx = pt.x - candidate.x;
        float ry = pt.y - candidate.y;
        float rz = pt.z - candidate.z;

        float dist = rx * rx + ry * ry + rz * rz;

        if (dist < nearest_dist) {
          nearest_dist = dist;
          nearest_pt = candidate;
          found = true;
        }
      }
    }
  }

  nearest_dist = std::sqrt(nearest_dist);

  return found;
}

bool VoxelMap::hasNearbyPoint(const PointType& pt, float radius) {
  PointType nearest_pt;

  float dist;

  if (!nearestSearch(pt, nearest_pt, dist)) {
    return false;
  }

  return dist < radius;
}

void VoxelMap::removeFarVoxels(const Eigen::Vector3d& center) {
  const float radius2 = local_radius_ * local_radius_;

  for (auto iter = voxel_map_.begin(); iter != voxel_map_.end();) {
    float dx = (iter->first.x + 0.5f) * voxel_size_ - center.x();
    float dy = (iter->first.y + 0.5f) * voxel_size_ - center.y();
    float dz = (iter->first.z + 0.5f) * voxel_size_ - center.z();

    float dist2 = dx * dx + dy * dy + dz * dz;
    if (dist2 > radius2) {
      iter = voxel_map_.erase(iter);
    } else {
      ++iter;
    }
  }
}
