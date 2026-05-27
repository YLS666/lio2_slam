#pragma once
#define PCL_NO_PRECOMPILE
#include "cloud_utils/point_type.hpp"

#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <Eigen/Dense>

class ScanToMap {
 public:
  ScanToMap();

  void process(pcl::PointCloud<PointXYZIT>::Ptr cloud);

  Eigen::Quaterniond getQuaternion() const { return q_w_curr_; }

  Eigen::Vector3d getPosition() const { return t_w_curr_; }

  pcl::PointCloud<PointXYZIT>::Ptr getMap() { return map_cloud_; }

 private:
  pcl::PointCloud<PointXYZIT>::Ptr map_cloud_;

  pcl::KdTreeFLANN<PointXYZIT>::Ptr kdtree_;

  Eigen::Quaterniond q_w_curr_;

  Eigen::Vector3d t_w_curr_;

 private:
  bool fitPlane(const std::vector<Eigen::Vector3d>& points, Eigen::Vector4d& plane);

  void optimizePose(pcl::PointCloud<PointXYZIT>::Ptr cloud);

  void addCloudToMap(pcl::PointCloud<PointXYZIT>::Ptr cloud);
};