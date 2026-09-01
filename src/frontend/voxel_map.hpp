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

// 打包体素键: 3×int → uint64
// 21bit/维, 偏移编码保证符号可还原(供 setLocalCenter 还原 block 用)
// 范围 ±2^20 个体素, 0.5m 体素下约 ±524km, 远超实际场景
inline uint64_t packVoxelKey(int x, int y, int z) {
  constexpr int64_t OFF = 1 << 20;
  return (static_cast<uint64_t>(x + OFF) << 42) | (static_cast<uint64_t>(y + OFF) << 21) |
         static_cast<uint64_t>(z + OFF);
}

inline VoxelKey unpackVoxelKey(uint64_t key) {
  constexpr int64_t OFF = 1 << 20;
  return {static_cast<int>((key >> 42) & 0x1FFFFF) - static_cast<int>(OFF),
          static_cast<int>((key >> 21) & 0x1FFFFF) - static_cast<int>(OFF),
          static_cast<int>((key >> 0) & 0x1FFFFF) - static_cast<int>(OFF)};
}

// 扁平开地址哈希表 (替代 tbb::concurrent_unordered_map)
// 快在哪: 连续内存(keys_/cells_ 两个 vector), find 顺序探测无链表指针追逐;
// 写全在 map_mutex_ 下串行, 读不与写并发, 所以无需 "concurrent"
class FlatVoxelMap {
 public:
  static constexpr uint64_t kEmpty = 0xFFFFFFFFFFFFFFFFULL;  // 空槽(打包键 bit63 恒 0)

  void clear() {
    std::fill(keys_.begin(), keys_.end(), kEmpty);
    count_ = 0;
  }
  size_t size() const { return count_; }
  void reserve(size_t n) {
    if (keys_.size() >= n * 2) {
      return;
    }
    rehash(std::max<size_t>(16, n * 2));
  }

  const NDTCell* find(uint64_t key) const {
    if (keys_.empty()) {
      return nullptr;
    }
    const size_t mask = keys_.size() - 1;
    size_t i = mix(key) & mask;
    while (keys_[i] != kEmpty) {
      if (keys_[i] == key) {
        return &cells_[i];
      }
      i = (i + 1) & mask;
    }
    return nullptr;
  }

  NDTCell& operator[](uint64_t key) {
    if (count_ * 2 >= keys_.size()) {
      rehash(keys_.empty() ? 16 : keys_.size() * 2);
    }
    const size_t mask = keys_.size() - 1;
    size_t i = mix(key) & mask;
    while (keys_[i] != kEmpty && keys_[i] != key) {
      i = (i + 1) & mask;
    }
    if (keys_[i] == kEmpty) {
      keys_[i] = key;
      cells_[i] = NDTCell();
      ++count_;
    }
    return cells_[i];
  }

  template <typename F>
  void forEach(const F& f) const {
    for (size_t i = 0; i < keys_.size(); ++i) {
      if (keys_[i] != kEmpty) {
        f(keys_[i], cells_[i]);
      }
    }
  }

 private:
  static uint64_t mix(uint64_t x) {  // splitmix64 最终化, 打散连续体素键的低位结构
    x ^= x >> 30;
    x *= 0xbf58476d1ce4e5b9ULL;
    x ^= x >> 27;
    x *= 0x94d049bb133111ebULL;
    x ^= x >> 31;
    return x;
  }

  void rehash(size_t want_cap) {
    size_t cap = 1;
    while (cap < want_cap) {
      cap <<= 1;
    }

    std::vector<uint64_t> old_keys;
    std::vector<NDTCell> old_cells;
    old_keys.swap(keys_);
    old_cells.swap(cells_);

    keys_.assign(cap, kEmpty);
    cells_.assign(cap, NDTCell());
    count_ = 0;

    const size_t mask = cap - 1;
    for (size_t i = 0; i < old_keys.size(); ++i) {
      if (old_keys[i] == kEmpty) {
        continue;
      }
      size_t j = mix(old_keys[i]) & mask;
      while (keys_[j] != kEmpty) {
        j = (j + 1) & mask;
      }
      keys_[j] = old_keys[i];
      cells_[j] = old_cells[i];
      ++count_;
    }
  }

  std::vector<uint64_t> keys_;
  std::vector<NDTCell> cells_;
  size_t count_ = 0;
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
  const NDTCell* getCell(const PointType& pt, NearbyType nearby = NearbyType::NEARBY6) const;

  /**
   * @brief 检查查询点附近是否存在已估计的NDT体素
   * @param pt 查询点
   * @param nearby 搜索范围（CENTER: 仅自身; NEARBY6: 7邻域; NEARBY26: 27邻域）
   * @return true 存在可用的NDT体素
   */
  bool hasNearbyCell(const PointType& pt, NearbyType nearby = NearbyType::CENTER) const;

  /**
   * @brief 根据当前位姿裁剪局部地图
   */
  void setLocalCenter(const V3d& center);

  /**
   * @brief 用于显示
   */
  CloudPtr getCloud() const;

  void clearAll() {
    std::lock_guard<std::mutex> lock(map_mutex_);
    ndt_map_.clear();
    active_blocks_.clear();
  }

  /** @brief 设置并行线程数 (0=不限制, >0=限制) */
  void setNumThreads(int n) { num_threads_ = n; }

 private:
  VoxelKey pointToVoxel(const PointType& pt) const;

  BlockKey voxelToBlock(const VoxelKey& key) const;

  void updateCell(NDTCell& cell);

 private:
  float voxel_size_;
  float block_size_;
  int local_block_radius_;

  FlatVoxelMap ndt_map_;
  tbb::concurrent_unordered_map<BlockKey, int, BlockHash> active_blocks_;

  BlockKey last_block_center_{0, 0, 0};

  static const std::vector<VoxelKey> kCenterOnly;
  static const std::vector<VoxelKey> kNeighborOffset7;
  static const std::vector<VoxelKey> kNeighborOffset27;

  int num_threads_ = 0;           // 0=不限制
  mutable std::mutex map_mutex_;  // 保护 ndt_map_ 的写/遍历 (addCloud/getCloud/setLocalCenter)
};