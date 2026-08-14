#pragma once

#include "cloud_utils/point_type.hpp"
#include "imu_utils/imu_processor.hpp"
#include "measure/measure_group.hpp"

class TimeSync {
 public:
  explicit TimeSync(ImuProcessor* imu_processor);

  void pushImu(const Imu& imu);

  void pushCloud(FullCloudPtr cloud);

  bool syncMeasure(MeasureGroup& measures);

 private:
  std::deque<Imu> imu_buffer_;

  std::deque<FullCloudPtr> cloud_buffer_;

  ImuProcessor* imu_processor_;
};