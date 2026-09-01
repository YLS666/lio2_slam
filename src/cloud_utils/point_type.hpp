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

enum class VoxelReduce { FIRST, CENTROID };

/** @brief 通用体素降采样：FIRST=每体素保留索引最小点；CENTROID=每体素质心 */
inline CloudPtr voxelDownsample(const CloudPtr& cloud, float voxel_size, VoxelReduce mode, int num_threads = 0) {
  CloudPtr ds(new PointCloudType());
  if (!cloud || cloud->empty()) {
    return ds;
  }

  const size_t N = cloud->size();
  const float inv = 1.0f / voxel_size;

  std::vector<uint64_t> keys(N);
  std::vector<uint32_t> idx(N);
  tbb::task_arena arena(lio::effectiveThreads(num_threads));
  arena.execute([&] {
    tbb::parallel_for(tbb::blocked_range<size_t>(0, N, 4096), [&](const tbb::blocked_range<size_t>& r) {
      for (size_t i = r.begin(); i != r.end(); ++i) {
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

  tbb::parallel_sort(idx.begin(), idx.end(),
                     [&](uint32_t a, uint32_t b) { return keys[a] != keys[b] ? keys[a] < keys[b] : a < b; });

  ds->reserve(N);
  for (size_t s = 0; s < N;) {
    const uint64_t key = keys[idx[s]];
    size_t e = s;
    if (mode == VoxelReduce::CENTROID) {
      double sx = 0, sy = 0, sz = 0;
      while (e < N && keys[idx[e]] == key) {
        const auto& pt = (*cloud)[idx[e]];
        sx += pt.x;
        sy += pt.y;
        sz += pt.z;
        ++e;
      }
      PointType out = (*cloud)[idx[s]];
      out.x = static_cast<float>(sx / static_cast<float>(e - s));
      out.y = static_cast<float>(sy / static_cast<float>(e - s));
      out.z = static_cast<float>(sz / static_cast<float>(e - s));
      ds->push_back(out);
    } else {  // FIRST
      ds->push_back((*cloud)[idx[s]]);
      while (e < N && keys[idx[e]] == key) {
        ++e;
      }
    }
    s = e;
  }
  return ds;
}

/** 兼容旧接口：质心降采样 */
inline CloudPtr dsCloud(const CloudPtr& cloud, float voxel_size, int num_threads = 0) {
  return voxelDownsample(cloud, voxel_size, VoxelReduce::CENTROID, num_threads);
}
