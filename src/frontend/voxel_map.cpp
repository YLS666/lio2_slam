#include "frontend/voxel_map.hpp"
#include <Eigen/Eigenvalues>
#include "utils/parallel.hpp"

// neighbor
const std::vector<VoxelKey> VoxelMap::kNeighborOffset7{{0, 0, 0},  {1, 0, 0}, {-1, 0, 0}, {0, 1, 0},
                                                       {0, -1, 0}, {0, 0, 1}, {0, 0, -1}};

const std::vector<VoxelKey> VoxelMap::kNeighborOffset27 = []() {
  std::vector<VoxelKey> v;

  for (int x = -1; x <= 1; x++) {
    for (int y = -1; y <= 1; y++) {
      for (int z = -1; z <= 1; z++) {
        v.push_back({x, y, z});
      }
    }
  }

  return v;
}();

const std::vector<VoxelKey> VoxelMap::kCenterOnly{{0, 0, 0}};

VoxelMap::VoxelMap(float voxel_size, float block_size, int local_block_radius)
    : voxel_size_(voxel_size), block_size_(block_size), local_block_radius_(local_block_radius) {}

size_t VoxelMap::size() const { return ndt_map_.size(); }

VoxelKey VoxelMap::pointToVoxel(const PointType& pt) const {
  return {static_cast<int>(std::floor(pt.x / voxel_size_)), static_cast<int>(std::floor(pt.y / voxel_size_)),
          static_cast<int>(std::floor(pt.z / voxel_size_))};
}

BlockKey VoxelMap::voxelToBlock(const VoxelKey& key) const {
  int vpb = static_cast<int>(block_size_ / voxel_size_);
  return {key.x / vpb, key.y / vpb, key.z / vpb};
}

void VoxelMap::addCloud(const CloudPtr& cloud) {
  std::lock_guard<std::mutex> lock(map_mutex_);
  if (!cloud || cloud->empty()) {
    return;
  }

  struct CellAccum {
    int points_num = 0;
    V3d sum = V3d::Zero();
    M3d sum_sq = M3d::Zero();
  };
  using LocalMap = std::unordered_map<uint64_t, CellAccum>;  // 键已打包, 默认 hash 即可

  tbb::enumerable_thread_specific<LocalMap> local_maps;

  tbb::task_arena arena(lio::effectiveThreads(num_threads_));
  arena.execute([&] {
    tbb::parallel_for(tbb::blocked_range<size_t>(0, cloud->size(), 4096), [&](const tbb::blocked_range<size_t>& range) {
      LocalMap& lm = local_maps.local();
      for (size_t i = range.begin(); i != range.end(); ++i) {
        const auto& pt = (*cloud)[i];
        VoxelKey vk = pointToVoxel(pt);
        uint64_t key = packVoxelKey(vk.x, vk.y, vk.z);
        CellAccum& a = lm[key];
        V3d p(pt.x, pt.y, pt.z);
        a.points_num++;
        a.sum += p;
        a.sum_sq += p * p.transpose();
      }
    });
  });

  for (auto& lm : local_maps) {
    for (auto& kv : lm) {
      auto& cell = ndt_map_[kv.first];
      cell.points_num += kv.second.points_num;
      cell.sum += kv.second.sum;
      cell.sum_sq += kv.second.sum_sq;
      updateCell(cell);
    }
  }
}

void VoxelMap::updateCell(NDTCell& cell) {
  // 已饱和的体素不再更新
  if (cell.ndt_estimated && cell.points_num >= NDTCell::MAX_POINTS) {
    return;
  }

  if (cell.points_num < 5) {
    cell.ndt_estimated = false;
    return;
  }

  double n = static_cast<double>(cell.points_num);

  cell.mean = cell.sum / n;
  cell.covariance = cell.sum_sq / n - cell.mean * cell.mean.transpose();

  // 只用极轻的绝对正则化防止奇异
  cell.covariance += M3d::Identity() * 1e-3;

  // SVD 后做相对条件数约束，保留各向异性
  Eigen::JacobiSVD<M3d> svd(cell.covariance, Eigen::ComputeFullU | Eigen::ComputeFullV);
  V3d lambda = svd.singularValues();

  // 相对约束：最小特征值不低于最大特征值的 1/1000
  // 这保留了地面的 Z<<XY 结构，同时防止完全退化
  if (lambda[1] < lambda[0] * 1e-3) {
    lambda[1] = lambda[0] * 1e-3;
  }
  if (lambda[2] < lambda[0] * 1e-3) {
    lambda[2] = lambda[0] * 1e-3;
  }

  M3d lambda_inv = V3d(1.0 / lambda[0], 1.0 / lambda[1], 1.0 / lambda[2]).asDiagonal();
  cell.info = svd.matrixV() * lambda_inv * svd.matrixU().transpose();
  cell.ndt_estimated = true;
}

const NDTCell* VoxelMap::getCell(const PointType& pt, NearbyType nearby) const {
  VoxelKey center = pointToVoxel(pt);

  const auto& offsets = (nearby == NearbyType::NEARBY26) ? kNeighborOffset27
                        : (nearby == NearbyType::CENTER) ? kCenterOnly
                                                         : kNeighborOffset7;

  for (const auto& off : offsets) {
    VoxelKey key{center.x + off.x, center.y + off.y, center.z + off.z};
    const NDTCell* cell = ndt_map_.find(packVoxelKey(key.x, key.y, key.z));
    if (!cell) {
      continue;
    }
    if (!cell->ndt_estimated) {
      continue;
    }
    return cell;
  }
  return nullptr;
}

bool VoxelMap::hasNearbyCell(const PointType& pt, NearbyType nearby) const {
  VoxelKey center = pointToVoxel(pt);

  const auto& offsets = (nearby == NearbyType::NEARBY26) ? kNeighborOffset27
                        : (nearby == NearbyType::CENTER) ? kCenterOnly
                                                         : kNeighborOffset7;

  for (const auto& off : offsets) {
    VoxelKey key{center.x + off.x, center.y + off.y, center.z + off.z};
    const NDTCell* cell = ndt_map_.find(packVoxelKey(key.x, key.y, key.z));
    if (!cell) {
      continue;
    }
    if (!cell->ndt_estimated) {
      continue;
    }
    return true;
  }
  return false;
}

void VoxelMap::setLocalCenter(const V3d& center) {
  std::lock_guard<std::mutex> lock(map_mutex_);
  int vpb = static_cast<int>(block_size_ / voxel_size_);

  VoxelKey cv{static_cast<int>(std::floor(center.x() / voxel_size_)),
              static_cast<int>(std::floor(center.y() / voxel_size_)),
              static_cast<int>(std::floor(center.z() / voxel_size_))};

  BlockKey cb{cv.x / vpb, cv.y / vpb, cv.z / vpb};

  if (cb == last_block_center_) {
    return;
  }

  last_block_center_ = cb;

  tbb::concurrent_unordered_map<BlockKey, int, BlockHash> active;
  for (int x = -local_block_radius_; x <= local_block_radius_; x++) {
    for (int y = -local_block_radius_; y <= local_block_radius_; y++) {
      for (int z = -local_block_radius_; z <= local_block_radius_; z++) {
        active.insert({{cb.bx + x, cb.by + y, cb.bz + z}, 1});
      }
    }
  }

  FlatVoxelMap kept;
  kept.reserve(ndt_map_.size());
  ndt_map_.forEach([&](uint64_t key, const NDTCell& cell) {
    VoxelKey vk = unpackVoxelKey(key);
    BlockKey bk{vk.x / vpb, vk.y / vpb, vk.z / vpb};
    if (active.find(bk) != active.end()) {
      kept[key] = cell;
    }
  });
  ndt_map_ = std::move(kept);

  active_blocks_ = std::move(active);
}

CloudPtr VoxelMap::getCloud() const {
  std::lock_guard<std::mutex> lock(map_mutex_);
  CloudPtr cloud(new PointCloudType());
  cloud->reserve(ndt_map_.size());
  ndt_map_.forEach([&](uint64_t /*key*/, const NDTCell& cell) {
    if (!cell.ndt_estimated) {
      return;
    }
    PointType pt;
    pt.x = static_cast<float>(cell.mean.x());
    pt.y = static_cast<float>(cell.mean.y());
    pt.z = static_cast<float>(cell.mean.z());
    cloud->push_back(pt);
  });
  return cloud;
}
