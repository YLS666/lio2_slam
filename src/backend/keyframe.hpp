#pragma once

#include "cloud_utils/point_type.hpp"
#include "utils/eigen_types.hpp"

/**
 * @brief KeyFrame 结构体
 */
struct KeyFrame {
  int id = -1;
  double timestamp = -1.0;

  // 位姿: p_world, q_world
  V3d p = V3d::Zero();
  Qd q = Qd::Identity();

  // 特征点云 (降采样后, 用于回环匹配)
  CloudPtr cloud;

  // 协方差矩阵 (来自 ESKF, 用于图优化的信息矩阵)
  Eigen::Matrix<double, 6, 6> info_mat = Eigen::Matrix<double, 6, 6>::Identity();

  // 帧间约束 (与前一帧)
  V3d relative_p = V3d::Zero();
  Qd relative_q = Qd::Identity();
};