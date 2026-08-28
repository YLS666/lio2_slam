#pragma once

#include <pcl/filters/voxel_grid.h>
#include <pcl/impl/point_types.hpp>
#include "utils/parallel.hpp"
#define PCL_NO_PRECOMPILE

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/register_point_struct.h>

struct FullPointType {
  PCL_ADD_POINT4D;
  uint16_t intensity = 0;
  uint16_t ring = 0;
  double timestamp = 0;

  inline FullPointType() {}
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

POINT_CLOUD_REGISTER_POINT_STRUCT(FullPointType,
                                  (float, x, x)(float, y, y)(float, z, z)(uint16_t, intensity, intensity)(
                                      uint16_t, ring, ring)(double, timestamp, timestamp))

using PointType = pcl::PointXYZI;
using PointCloudType = pcl::PointCloud<PointType>;
using CloudPtr = PointCloudType::Ptr;

using FullCloudPointType = pcl::PointCloud<FullPointType>;
using FullCloudPtr = FullCloudPointType::Ptr;

inline CloudPtr dsCloud(const CloudPtr& cloud, float voxel_size, int num_threads = 0) {
  CloudPtr ds(new PointCloudType());
  if (!cloud || cloud->empty()) {
    return ds;
  }

  const size_t N = cloud->size();
  const float inv = 1.0f / voxel_size;

  // 1) 并行计算每个点的体素 key（与 featureSample 同款：无哈希、无堆分配）
  std::vector<uint64_t> keys(N);
  std::vector<uint32_t> idx(N);
  tbb::task_arena arena(lio::effectiveThreads(num_threads));
  arena.execute([&] {
    tbb::parallel_for(tbb::blocked_range<size_t>(0, N, 4096), [&](const tbb::blocked_range<size_t>& range) {
      for (size_t i = range.begin(); i != range.end(); ++i) {
        const auto& pt = (*cloud)[i];
        int64_t vx = static_cast<int64_t>(std::floor(pt.x * inv));
        int64_t vy = static_cast<int64_t>(std::floor(pt.y * inv));
        int64_t vz = static_cast<int64_t>(std::floor(pt.z * inv));
        keys[i] = (static_cast<uint64_t>(vx & 0x1FFFFF) << 42) | (static_cast<uint64_t>(vy & 0x1FFFFF) << 21) |
                  static_cast<uint64_t>(vz & 0x1FFFFF);
        idx[i] = static_cast<uint32_t>(i);
      }
    });
  });

  // 2) 并行排序
  tbb::parallel_sort(idx.begin(), idx.end(), [&](uint32_t a, uint32_t b) {
    if (keys[a] != keys[b]) {
      return keys[a] < keys[b];
    }
    return a < b;
  });

  // 3) 串行扫描排序结果，对相同 key 求质心（复现 pcl::VoxelGrid 的质心语义）
  ds->reserve(N);
  for (size_t s = 0; s < N;) {
    const uint64_t key = keys[idx[s]];
    double sx = 0.0, sy = 0.0, sz = 0.0;
    size_t e = s;
    while (e < N && keys[idx[e]] == key) {
      const auto& pt = (*cloud)[idx[e]];
      sx += pt.x;
      sy += pt.y;
      sz += pt.z;
      ++e;
    }
    PointType out = (*cloud)[idx[s]];  // intensity 等字段取该体素内 index 最小的点
    out.x = static_cast<float>(sx / static_cast<float>(e - s));
    out.y = static_cast<float>(sy / static_cast<float>(e - s));
    out.z = static_cast<float>(sz / static_cast<float>(e - s));
    ds->push_back(out);
    s = e;
  }
  return ds;
}
