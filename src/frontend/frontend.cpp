#include "frontend/frontend.hpp"

#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <chrono>
#include <iostream>
#include <mutex>

Frontend::Frontend() { map_ = std::make_unique<VoxelMap>(0.2f); }

void Frontend::process(const pcl::PointCloud<PointType>::Ptr& cloud) {
  std::lock_guard<std::mutex> lock(frontend_mutex_);

  // 1. voxel downsample
  pcl::PointCloud<PointType>::Ptr ds_cloud(new pcl::PointCloud<PointType>());
  static pcl::VoxelGrid<PointType> voxel;
  voxel.setInputCloud(cloud);
  voxel.setLeafSize(0.1f, 0.1f, 0.1f);
  voxel.filter(*ds_cloud);
  std::cout << "ds_cloud size: " << ds_cloud->size() << std::endl;

  // init
  if (!initialized_) {
    map_->addCloud(ds_cloud, state_.p);
    initialized_ = true;
    std::cout << "frontend init" << std::endl;
    return;
  }

  // 2. feature extraction
  auto feature_cloud = featureSample(ds_cloud);
  std::cout << "feature_cloud size: " << feature_cloud->size() << std::endl;

  // 3. registration
  auto start = std::chrono::steady_clock::now();

  registration_.align(feature_cloud, map_.get(), state_);

  auto end = std::chrono::steady_clock::now();
  std::chrono::duration<double> elapsed = end - start;
  std::cout << "registration time: " << elapsed.count() * 1000.0 << " ms" << std::endl;

  // 4. transform to world
  pcl::PointCloud<PointType>::Ptr world_cloud(new pcl::PointCloud<PointType>());
  Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
  T.block<3, 3>(0, 0) = state_.q.toRotationMatrix().cast<float>();
  T.block<3, 1>(0, 3) = state_.p.cast<float>();
  pcl::transformPointCloud(*feature_cloud, *world_cloud, T);

  // 5. add ONLY new voxels
  pcl::PointCloud<PointType>::Ptr new_cloud(new pcl::PointCloud<PointType>());
  new_cloud->reserve(world_cloud->size());
  constexpr float MAP_VOXEL = 0.2f;

  for (const auto& pt : world_cloud->points) {
    if (!map_->hasNearbyPoint(pt, MAP_VOXEL)) {
      new_cloud->push_back(pt);
    }
  }

  // 6. update map
  map_->addCloud(new_cloud, state_.p);
  std::cout << "map add: " << new_cloud->size() << std::endl;
  std::cout << "map size: " << map_->size() << std::endl;
  std::cout << "pose: " << state_.p.transpose() << std::endl;
}

pcl::PointCloud<PointType>::Ptr Frontend::featureSample(const pcl::PointCloud<PointType>::Ptr& cloud) {
  constexpr float VOXEL_SIZE = 0.4f;

  std::unordered_map<VoxelKey, PointType, VoxelHash> voxel_map;

  for (const auto& pt : cloud->points) {
    VoxelKey key;

    key.x = static_cast<int>(std::floor(pt.x / VOXEL_SIZE));
    key.y = static_cast<int>(std::floor(pt.y / VOXEL_SIZE));
    key.z = static_cast<int>(std::floor(pt.z / VOXEL_SIZE));

    // 每个 voxel 只保留一个点
    if (voxel_map.find(key) == voxel_map.end()) {
      voxel_map[key] = pt;
    }
  }

  pcl::PointCloud<PointType>::Ptr out(new pcl::PointCloud<PointType>());

  out->reserve(voxel_map.size());

  for (const auto& kv : voxel_map) {
    out->push_back(kv.second);
  }

  return out;
}