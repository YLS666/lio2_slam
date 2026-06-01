#pragma once

#include <tbb/concurrent_unordered_map.h>
#include <tbb/tbb.h>

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
    size_t seed = 0;
    seed ^= std::hash<int>()(k.x) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    seed ^= std::hash<int>()(k.y) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    seed ^= std::hash<int>()(k.z) + 0x9e3779b9 + (seed << 6) + (seed >> 2);

    return seed;
  }
};

// 搜索模式
enum class NearbyType {
  CENTER,   // 中心点
  NEARBY6,  // 中心点 + 上下左右前后 6个邻域
  NEARBY26  // 检查全部27个体素
};

// 区块坐标
struct BlockKey {
  int bx;
  int by;
  int bz;

  bool operator==(const BlockKey& other) const { return bx == other.bx && by == other.by && bz == other.bz; }
};

// 区块坐标
struct BlockHash {
  size_t operator()(const BlockKey& k) const {
    size_t seed = 0;
    seed ^= std::hash<int>()(k.bx) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    seed ^= std::hash<int>()(k.by) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    seed ^= std::hash<int>()(k.bz) + 0x9e3779b9 + (seed << 6) + (seed >> 2);

    return seed;
  }
};

class VoxelMap {
 public:
  explicit VoxelMap(float voxel_size = 0.2f,
                    float block_size = 20.0f,     // 区块大小，每个区块20m
                    int local_block_radius = 2);  // 保留周围2圈区块

  size_t size() const;

  void addCloud(const pcl::PointCloud<PointType>::Ptr& cloud, const Eigen::Vector3d& center);

  bool nearestSearch(const PointType& pt, PointType& nearest_pt, float& nearest_dist,
                     NearbyType nearby = NearbyType::NEARBY6) const;

  bool hasNearbyPoint(const PointType& pt, float radius, NearbyType nearby = NearbyType::NEARBY6) const;

  // 设置新的局部中心点，触发区块加载/卸载
  void setLocalCenter(const Eigen::Vector3d& center);

 private:
  VoxelKey pointToVoxel(const PointType& pt) const;
  BlockKey voxelToBlock(const VoxelKey& vkey) const;

  // 区块管理
  void loadBlock(const BlockKey& block_key);
  void unloadBlock(const BlockKey& block_key);
  void updateBlock(const BlockKey& block_key);

 private:
  float voxel_size_;
  float block_size_;
  int local_block_radius_;

  // 无锁并发哈希表
  tbb::concurrent_unordered_map<VoxelKey, PointType, VoxelHash> voxel_map_;
  tbb::concurrent_unordered_map<BlockKey, int, BlockHash> active_blocks_;

  // 上次的区块中心
  BlockKey last_block_center_{0, 0, 0};

  // 邻居体素偏移（预计算）
  static const std::vector<VoxelKey> kNeighborOffset7;
  static const std::vector<VoxelKey> kNeighborOffset27;
};
