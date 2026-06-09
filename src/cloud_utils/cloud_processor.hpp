#pragma once

#include <pcl/point_cloud.h>
#include "cloud_utils/point_type.hpp"
#include "config_def.hpp"
#include "imu_utils/imu_processor.hpp"
#include "measure/measure_group.hpp"
#include "utils/eigen_types.hpp"

struct PoseCache {
  M3d R;
  V3d t;
};

class CloudProcessor {
 public:
  explicit CloudProcessor(AllConfig& config);

  CloudPtr process(const MeasureGroup& measures, ImuProcessor* imu_processor);

  void pre_process(const FullCloudPtr& cloud, FullCloudPtr& out_cloud);

 private:
  Qd q_il_ = Qd::Identity();
  V3d t_il_ = V3d::Zero();
  std::vector<PoseCache> pose_table_;
};