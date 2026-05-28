#include "frontend/voxel_map.hpp"

#include <cmath>
#include <limits>

VoxelMap::VoxelMap(float voxel_size) : voxel_size_(voxel_size) {}

size_t VoxelMap::size() const { return voxel_map_.size(); }

VoxelKey VoxelMap::posToKey(float x, float y, float z) const {
  VoxelKey key;

  key.x = static_cast<int>(std::floor(x / voxel_size_));
  key.y = static_cast<int>(std::floor(y / voxel_size_));
  key.z = static_cast<int>(std::floor(z / voxel_size_));

  return key;
}

void VoxelMap::addCloud(const pcl::PointCloud<PointType>::Ptr& cloud) {
  for (const auto& pt : cloud->points) {
    VoxelKey key = posToKey(pt.x, pt.y, pt.z);

    auto iter = voxel_map_.find(key);

    if (iter == voxel_map_.end()) {
      voxel_map_.insert({key, pt});
    }
  }
}

bool VoxelMap::nearestSearch(const PointType& pt, PointType& nearest_pt, float& nearest_dist) {
  VoxelKey center = posToKey(pt.x, pt.y, pt.z);

  nearest_dist = std::numeric_limits<float>::max();

  bool found = false;

  for (int dx = -1; dx <= 1; ++dx) {
    for (int dy = -1; dy <= 1; ++dy) {
      for (int dz = -1; dz <= 1; ++dz) {
        VoxelKey key;
        key.x = center.x + dx;
        key.y = center.y + dy;
        key.z = center.z + dz;

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

pcl::PointCloud<PointType>::Ptr VoxelMap::getCloud() {
  pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>());

  cloud->reserve(voxel_map_.size());

  for (const auto& kv : voxel_map_) {
    cloud->push_back(kv.second);
  }

  return cloud;
}

bool VoxelMap::hasNearbyPoint(const PointType& pt, float radius) {
  PointType nearest_pt;

  float dist;

  if (!nearestSearch(pt, nearest_pt, dist)) {
    return false;
  }

  return dist < radius;
}