#pragma once

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
  explicit VoxelMap(float voxel_size = 0.2f);

  size_t size() const;

  void addCloud(const pcl::PointCloud<PointType>::Ptr& cloud);

  bool nearestSearch(const PointType& pt, PointType& nearest_pt, float& nearest_dist);

  pcl::PointCloud<PointType>::Ptr getCloud();

  bool hasNearbyPoint(const PointType& pt, float radius);

 private:
  VoxelKey posToKey(float x, float y, float z) const;

 private:
  float voxel_size_;

  std::unordered_map<VoxelKey, PointType, VoxelHash> voxel_map_;
};