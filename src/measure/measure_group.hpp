#pragma once

#include <deque>

#include <pcl/point_cloud.h>

#include "cloud_utils/point_type.hpp"
#include "imu_utils/imu_processor.hpp"
#include "sensor_msgs/msg/imu.hpp"

struct MeasureGroup {
  double lidar_begin_time = 0.0;
  double lidar_end_time = 0.0;

  pcl::PointCloud<FullPointType>::Ptr lidar;
  std::deque<sensor_msgs::msg::Imu> imu_datas;
  std::vector<ImuState> imu_states;
};