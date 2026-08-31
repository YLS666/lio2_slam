#pragma once

#include <pcl/point_cloud.h>
#include "cloud_utils/point_type.hpp"
#include "config_def.hpp"
#include "imu_utils/imu_processor.hpp"
#include "sync/measure_group.hpp"
#include "utils/eigen_types.hpp"
#include "utils/ros_types.hpp"

struct PoseCache {
  M3d R;
  V3d t;
};

class CloudProcessor {
 public:
  explicit CloudProcessor(AllConfig& config);

  CloudPtr process(const MeasureGroup& measures, ImuProcessor* imu_processor);

  void pre_process(const PointCloud2SharedPtr& cloud, FullCloudPtr& out_cloud);

 private:
  Qd q_li_ = Qd::Identity();
  V3d t_li_ = V3d::Zero();
  std::vector<PoseCache> pose_table_;
  int num_threads_ = 0;  // 0=不限制
};