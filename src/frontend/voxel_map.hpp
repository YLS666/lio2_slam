#pragma once

#include <mutex>
#include <unordered_map>
#include <vector>

#include <Eigen/Core>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

using PointType = pcl::PointXYZI;

struct VoxelKey {
  int x;
  int y;
  int z;

  bool operator==(const VoxelKey& other) const { return x == other.x && y == other.y && z == other.z; }
};

struct VoxelHash {
  size_t operator()(const VoxelKey& k) const {
    size_t hx = std::hash<int>()(k.x);
    size_t hy = std::hash<int>()(k.y);
    size_t hz = std::hash<int>()(k.z);

    return hx ^ (hy << 1) ^ (hz << 2);
  }
};

class VoxelMap {
 public:
  explicit VoxelMap(float voxel_size = 0.2f, float local_radius = 20.0f);

  size_t size() const;

  void addCloud(const pcl::PointCloud<PointType>::Ptr& cloud, const Eigen::Vector3d& center);

  bool nearestSearch(const PointType& pt, PointType& nearest_pt, float& nearest_dist);

  bool hasNearbyPoint(const PointType& pt, float radius);

 private:
  VoxelKey posToKey(float x, float y, float z) const;

  VoxelKey pointToVoxel(const PointType& pt) const;

  void removeFarVoxels(const Eigen::Vector3d& center);

 private:
  float voxel_size_;
  float local_radius_;

  std::unordered_map<VoxelKey, PointType, VoxelHash> voxel_map_;

  mutable std::mutex mutex_;
};