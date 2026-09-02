#pragma once

#include <functional>
#include "config_def.hpp"
#include "utils/ros_types.hpp"

class BagIO {
 public:
  explicit BagIO(AllConfig& config);

  void run(std::function<void(const Imu&)> imu_callback,
           std::function<void(const PointCloud2SharedPtr&)> cloud_callback);

 private:
  std::string bag_path_;
  std::string imu_topic_;
  std::string lidar_topic_;
};