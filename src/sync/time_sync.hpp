#pragma once

#include <deque>

#include <sensor_msgs/msg/imu.hpp>

#include <pcl/point_cloud.h>

#include "cloud_utils/point_type.hpp"
#include "imu_utils/imu_processor.hpp"
#include "measure/measure_group.hpp"

class TimeSync {
 public:
  explicit TimeSync(ImuProcessor* imu_processor);

  void pushImu(const sensor_msgs::msg::Imu& imu);

  void pushCloud(pcl::PointCloud<PointXYZIT>::Ptr cloud);

  bool syncMeasure(MeasureGroup& measures);

 private:
  std::deque<sensor_msgs::msg::Imu> imu_buffer_;

  std::deque<pcl::PointCloud<PointXYZIT>::Ptr> cloud_buffer_;

  ImuProcessor* imu_processor_;
};