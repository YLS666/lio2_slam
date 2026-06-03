#pragma once

#include <functional>
#include <string>

#include <pcl/point_cloud.h>

#include "cloud_utils/point_type.hpp"
#include "config_def.hpp"
#include "sensor_msgs/msg/imu.hpp"

class BagIO {
 public:
  explicit BagIO(AllConfig& config);

  void run(std::function<void(const sensor_msgs::msg::Imu&)> imu_callback,

           std::function<void(const pcl::PointCloud<FullPointType>::Ptr&)> cloud_callback);

 private:
  std::string bag_path_;
  std::string imu_topic_;
  std::string lidar_topic_;
};