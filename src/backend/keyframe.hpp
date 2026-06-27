#pragma once

#include "cloud_utils/point_type.hpp"
#include "utils/eigen_types.hpp"

/**
 * @brief KeyFrame 结构体
 */
struct KeyFrame {
  int id = -1;
  double timestamp = -1.0;

  V3d p = V3d::Zero();
  Qd q = Qd::Identity();

  CloudPtr cloud;

  Eigen::Matrix<double, 6, 6> info_mat = Eigen::Matrix<double, 6, 6>::Identity();

  V3d relative_p = V3d::Zero();
  Qd relative_q = Qd::Identity();

  // 标记是否已合并到地图
  mutable bool merged = false;
};