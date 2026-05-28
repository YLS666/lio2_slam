#pragma once

#include <Eigen/Dense>

#include <pcl/point_cloud.h>
#include "cloud_utils/point_type.hpp"
#include "imu_utils/imu_processor.hpp"
#include "measure/measure_group.hpp"

struct PoseCache {
  Eigen::Matrix3d R;
  Eigen::Vector3d t;
};

class CloudProcessor {
 public:
  pcl::PointCloud<PointType>::Ptr process(const MeasureGroup& measures, ImuProcessor* imu_processor);

  void setExtrinsic(const Eigen::Quaterniond& q, const Eigen::Vector3d& t);

  void pre_process(const pcl::PointCloud<FullPointType>::Ptr& cloud, pcl::PointCloud<FullPointType>::Ptr& out_cloud);

 private:
  Eigen::Quaterniond q_il_ = Eigen::Quaterniond::Identity();
  Eigen::Vector3d t_il_ = Eigen::Vector3d::Zero();
  std::vector<PoseCache> pose_table_;
};