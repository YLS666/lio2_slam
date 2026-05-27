#pragma once

#include <functional>
#include <string>

#include <pcl/point_cloud.h>

#include "cloud_utils/point_type.hpp"
#include "sensor_msgs/msg/imu.hpp"

class BagIO {
 public:
  explicit BagIO(const std::string& bag_path);

  void run(std::function<void(const sensor_msgs::msg::Imu&)> imu_callback,

           std::function<void(const pcl::PointCloud<PointXYZIT>::Ptr&)> cloud_callback);

 private:
  std::string bag_path_;
};