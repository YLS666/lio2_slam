#include "frontend/voxel_map.hpp"
#include <Eigen/src/Core/Matrix.h>

#include <cmath>
#include <limits>
#include <vector>

const std::vector<VoxelKey> VoxelMap::kNeighborOffset7{
    {0, 0, 0},   // 中心
    {1, 0, 0},   // 右
    {-1, 0, 0},  // 左
    {0, 1, 0},   // 上
    {0, -1, 0},  // 下
    {0, 0, 1},   // 前
    {0, 0, -1},  // 后
};

const std::vector<VoxelKey> VoxelMap::kNeighborOffset27 = []() {
  std::vector<VoxelKey> offsets;
  for (int dx = -1; dx <= 1; ++dx) {
    for (int dy = -1; dy <= 1; ++dy) {
      for (int dz = -1; dz <= 1; ++dz) {
        offsets.push_back({dx, dy, dz});
      }
    }
  }
  return offsets;
}();

VoxelMap::VoxelMap(float voxel_size, float block_size, int local_block_radius)
    : voxel_size_(voxel_size), block_size_(block_size), local_block_radius_(local_block_radius) {}

size_t VoxelMap::size() const { return voxel_map_.size(); }

VoxelKey VoxelMap::pointToVoxel(const PointType& pt) const {
  return {static_cast<int>(std::floor(pt.x / voxel_size_)), static_cast<int>(std::floor(pt.y / voxel_size_)),
          static_cast<int>(std::floor(pt.z / voxel_size_))};
}

BlockKey VoxelMap::voxelToBlock(const VoxelKey& vkey) const {
  // 一个区块包含 floor(block_size_ / voxel_size_)个体素
  int vperblock = static_cast<int>(std::floor(block_size_ / voxel_size_));
  return {vkey.x / vperblock, vkey.y / vperblock, vkey.z / vperblock};
}

void VoxelMap::addCloud(const pcl::PointCloud<PointType>::Ptr& cloud) {
  for (const auto& pt : cloud->points) {
    VoxelKey key = pointToVoxel(pt);
    // insert 如果已存在则不覆盖（保持第一个点）
    voxel_map_.insert({key, pt});
  }
}

bool VoxelMap::nearestSearch(const PointType& pt, PointType& nearest_pt, float& nearest_dist, NearbyType nearby) const {
  VoxelKey center = pointToVoxel(pt);
  nearest_dist = std::numeric_limits<float>::max();
  bool found = false;

  const auto& offsets = (nearby == NearbyType::NEARBY6) ? kNeighborOffset7 : kNeighborOffset27;

  for (const auto& offset : offsets) {
    VoxelKey key{center.x + offset.x, center.y + offset.y, center.z + offset.z};

    auto iter = voxel_map_.find(key);
    if (iter == voxel_map_.end()) {
      continue;
    }

    const auto& candidate = iter->second;

    // 使用平方距离可以避免sqrt
    float rx = pt.x - candidate.x;
    float ry = pt.y - candidate.y;
    float rz = pt.z - candidate.z;

    float dist2 = rx * rx + ry * ry + rz * rz;

    if (dist2 < nearest_dist) {
      nearest_dist = dist2;
      nearest_pt = candidate;
      found = true;
    }
  }

  if (found) {
    nearest_dist = std::sqrt(nearest_dist);
  }

  return found;
}

bool VoxelMap::hasNearbyPoint(const PointType& pt, float radius, NearbyType nearby) const {
  VoxelKey center = pointToVoxel(pt);
  float radius2 = radius * radius;

  const auto& offsets = (nearby == NearbyType::NEARBY6) ? kNeighborOffset7 : kNeighborOffset27;

  for (const auto& offset : offsets) {
    VoxelKey key{center.x + offset.x, center.y + offset.y, center.z + offset.z};

    auto iter = voxel_map_.find(key);
    if (iter == voxel_map_.end()) {
      continue;
    }

    const auto& candidate = iter->second;

    // 使用平方距离可以避免sqrt
    float rx = pt.x - candidate.x;
    float ry = pt.y - candidate.y;
    float rz = pt.z - candidate.z;

    float dist2 = rx * rx + ry * ry + rz * rz;

    if (dist2 < radius2) {
      return true;
    }
  }

  return false;
}

void VoxelMap::setLocalCenter(const Eigen::Vector3d& center) {
  // 将中心点转换为区块坐标
  int vperblock = static_cast<int>(std::floor(block_size_ / voxel_size_));
  VoxelKey center_voxel{static_cast<int>(std::floor(center.x() / voxel_size_)),
                        static_cast<int>(std::floor(center.y() / voxel_size_)),
                        static_cast<int>(std::floor(center.z() / voxel_size_))};
  BlockKey center_block{center_voxel.x / vperblock, center_voxel.y / vperblock, center_voxel.z / vperblock};

  // 如果区块中心没变，跳过
  if (center_block == last_block_center_) {
    return;
  }
  last_block_center_ = center_block;

  // 标记当前活跃区块
  tbb::concurrent_unordered_map<BlockKey, int, BlockHash> new_active;

  for (int dx = -local_block_radius_; dx <= local_block_radius_; ++dx) {
    for (int dy = -local_block_radius_; dy <= local_block_radius_; ++dy) {
      for (int dz = -local_block_radius_; dz <= local_block_radius_; ++dz) {
        BlockKey bk{center_block.bx + dx, center_block.by + dy, center_block.bz + dz};
        new_active.insert({bk, 1});
      }
    }
  }

  // 【优化】遍历所有真实存在的体素，删除不在新活跃集合中的体素
  // 复杂度: O(active_voxels) 而不是 O(blocks * vperblock^3)
  for (auto iter = voxel_map_.begin(); iter != voxel_map_.end();) {
    BlockKey bk = voxelToBlock(iter->first);
    if (new_active.find(bk) == new_active.end()) {
      iter = voxel_map_.unsafe_erase(iter);
    } else {
      ++iter;
    }
  }

  active_blocks_ = std::move(new_active);
}

pcl::PointCloud<PointType>::Ptr VoxelMap::getCloud() const {
  pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>());
  cloud->reserve(voxel_map_.size());

  for (const auto& kv : voxel_map_) {
    cloud->push_back(kv.second);
  }

  return cloud;
}
