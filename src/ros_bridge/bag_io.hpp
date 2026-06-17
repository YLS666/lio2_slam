#pragma once

#include <functional>
#include "config_def.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

class BagIO {
 public:
  explicit BagIO(AllConfig& config);

  void run(std::function<void(const sensor_msgs::msg::Imu&)> imu_callback,

           std::function<void(const sensor_msgs::msg::PointCloud2::SharedPtr&)> cloud_callback);

 private:
  std::string bag_path_;
  std::string imu_topic_;
  std::string lidar_topic_;
};