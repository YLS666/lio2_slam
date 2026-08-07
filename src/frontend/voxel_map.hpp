#pragma once

#include <cmath>
#include <vector>

#include <tbb/concurrent_unordered_map.h>

#include "cloud_utils/point_type.hpp"
#include "utils/eigen_types.hpp"

// voxel key
struct VoxelKey {
  int x;
  int y;
  int z;

  bool operator==(const VoxelKey& other) const { return x == other.x && y == other.y && z == other.z; }
};

struct VoxelHash {
  size_t operator()(const VoxelKey& k) const {
    size_t h = static_cast<size_t>(k.x) * 73856093;

    h ^= static_cast<size_t>(k.y) * 19349663;

    h ^= static_cast<size_t>(k.z) * 83492791;

    return h;
  }
};

// block key
struct BlockKey {
  int bx;
  int by;
  int bz;

  bool operator==(const BlockKey& other) const { return bx == other.bx && by == other.by && bz == other.bz; }
};

struct BlockHash {
  size_t operator()(const BlockKey& k) const {
    size_t seed = 0;

    seed ^= std::hash<int>()(k.bx) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    seed ^= std::hash<int>()(k.by) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    seed ^= std::hash<int>()(k.bz) + 0x9e3779b9 + (seed << 6) + (seed >> 2);

    return seed;
  }
};

// NDT Gaussian voxel
struct NDTCell {
  static constexpr int MAX_POINTS = 50;
  // 点数量
  int points_num = 0;

  // 增量统计
  V3d sum = V3d::Zero();
  M3d sum_sq = M3d::Zero();

  // Gaussian参数
  V3d mean = V3d::Zero();
  M3d covariance = M3d::Identity();

  // 信息矩阵 Σ^-1
  M3d info = M3d::Identity();

  bool ndt_estimated = false;
};

enum class NearbyType {

  CENTER,

  NEARBY6,

  NEARBY26

};

class VoxelMap {
 public:
  explicit VoxelMap(float voxel_size = 0.5f, float block_size = 20.0f, int local_block_radius = 2);

  size_t size() const;

  /**
   * @brief 增量更新NDT地图
   */
  void addCloud(const CloudPtr& cloud);

  /**
   * @brief 查询附近NDT voxel
   */
  bool getCell(const PointType& pt, NDTCell& cell, NearbyType nearby = NearbyType::NEARBY6) const;

  /**
   * @brief 检查查询点附近是否存在已估计的NDT体素
   * @param pt 查询点
   * @param radius 搜索半径（体素中心到查询点的距离阈值）
   * @param nearby 搜索范围（CENTER: 仅自身; NEARBY6: 7邻域; NEARBY26: 27邻域）
   * @return true 存在可用的NDT体素
   */
  bool hasNearbyCell(const PointType& pt, float radius, NearbyType nearby = NearbyType::CENTER) const;

  /**
   * @brief 根据当前位姿裁剪局部地图
   */
  void setLocalCenter(const V3d& center);

  /**
   * @brief 用于显示
   */
  CloudPtr getCloud() const;

  void clearAll() {
    ndt_map_.clear();
    active_blocks_.clear();
  }

 private:
  VoxelKey pointToVoxel(const PointType& pt) const;

  BlockKey voxelToBlock(const VoxelKey& key) const;

  void updateCell(NDTCell& cell);

 private:
  float voxel_size_;
  float block_size_;
  int local_block_radius_;

  tbb::concurrent_unordered_map<VoxelKey, NDTCell, VoxelHash> ndt_map_;
  tbb::concurrent_unordered_map<BlockKey, int, BlockHash> active_blocks_;

  BlockKey last_block_center_{0, 0, 0};

  static const std::vector<VoxelKey> kCenterOnly;
  static const std::vector<VoxelKey> kNeighborOffset7;
  static const std::vector<VoxelKey> kNeighborOffset27;
};