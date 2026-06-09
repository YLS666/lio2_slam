#pragma once

#include <deque>

#include "cloud_utils/point_type.hpp"
#include "imu_utils/imu_processor.hpp"

struct MeasureGroup {
  double lidar_begin_time = 0.0;
  double lidar_end_time = 0.0;

  FullCloudPtr lidar;
  std::deque<sensor_msgs::msg::Imu> imu_datas;
  std::vector<ImuState> imu_states;
};