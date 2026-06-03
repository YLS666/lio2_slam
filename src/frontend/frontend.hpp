#pragma once

#include <pcl/point_cloud.h>
#include <mutex>
#include "cloud_utils/point_type.hpp"
#include "frontend/registration.hpp"
#include "frontend/state.hpp"
#include "frontend/voxel_map.hpp"

class Frontend {
 public:
  Frontend();
  ~Frontend() = default;

  void process(const pcl::PointCloud<PointType>::Ptr& cloud);

  pcl::PointCloud<PointType>::Ptr featureSample(const pcl::PointCloud<PointType>::Ptr& cloud);

  void saveMap(const std::string& filename) const;

  State getState() const { return state_; }

 private:
  std::unique_ptr<VoxelMap> map_;
  Registration registration_;
  State state_;

  bool initialized_ = false;
  std::mutex frontend_mutex_;
};