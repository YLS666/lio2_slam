
#include "loop_closure/scan_context.hpp"

ScanContext::ScanContext() {
  // 初始化描述子为0
  for (auto& ring : desc_) {
    ring.fill(0.0f);
  }

  ring_key_.resize(RING_NUM);
  ring_key_.setZero();
}

void ScanContext::make(const CloudPtr& cloud) {
  // 重置
  for (auto& ring : desc_) {
    ring.fill(0.0f);
  }

  // 遍历点云，填充描述子
  for (const auto& pt : cloud->points) {
    double r = std::sqrt(pt.x * pt.x + pt.y * pt.y);
    if (r > MAX_RADIUS) {
      continue;
    }
    if (r < 0.1) {
      continue;
    }

    // 径向索引 (环)
    int ring_idx = static_cast<int>(r / MAX_RADIUS * RING_NUM);
    ring_idx = std::min(ring_idx, RING_NUM - 1);

    // 角度索引 (扇区)
    double theta = std::atan2(pt.y, pt.x);
    if (theta < 0) {
      theta += 2.0 * M_PI;
    }
    int sector_idx = static_cast<int>(theta / (2.0 * M_PI) * SECTOR_NUM);
    sector_idx = std::min(sector_idx, SECTOR_NUM - 1);

    // 取最大高度 (z) 作为该格子值
    float z = pt.z;
    if (z > desc_[ring_idx][sector_idx]) {
      desc_[ring_idx][sector_idx] = z;
    }
  }

  // 计算 RingKey (每环的最大值)
  for (int r = 0; r < RING_NUM; ++r) {
    float max_val = 0.0f;
    for (int s = 0; s < SECTOR_NUM; ++s) {
      max_val = std::max(max_val, desc_[r][s]);
    }
    ring_key_[r] = max_val;
  }
}

double ScanContext::distance(const Descriptor& a, const Descriptor& b) {
  // 归一化距离 (0 = 完全相同, 1 = 完全不同)
  double total_dist = 0.0;
  double total_count = 0.0;

  for (int r = 0; r < RING_NUM; ++r) {
    for (int s = 0; s < SECTOR_NUM; ++s) {
      double diff = a[r][s] - b[r][s];
      total_dist += diff * diff;
      total_count += 1.0;
    }
  }

  return std::sqrt(total_dist / total_count);
}