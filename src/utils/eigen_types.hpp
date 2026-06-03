#pragma once
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <vector>

/**
 * @brief 计算向量的反对称矩阵
 */
inline Eigen::Matrix3d skewSymmetric(const Eigen::Vector3d& v) {
  Eigen::Matrix3d m;
  m << 0.0, -v.z(), v.y(), v.z(), 0.0, -v.x(), -v.y(), v.x(), 0.0;
  return m;
}

/**
 * @brief 根据角速度计算增量旋转四元数
 */
inline Eigen::Quaterniond deltaQ(const Eigen::Vector3d& omega) {
  double theta = omega.norm();

  Eigen::Quaterniond dq;

  if (theta < 1e-10) {
    dq.w() = 1.0;
    dq.x() = 0.5 * omega.x();
    dq.y() = 0.5 * omega.y();
    dq.z() = 0.5 * omega.z();
  } else {
    Eigen::Vector3d axis = omega / theta;

    dq = Eigen::Quaterniond(Eigen::AngleAxisd(theta, axis));
  }

  return dq.normalized();
}

/**
 * @brief vector数组 转 Matrix3d 旋转矩阵
 */
inline Eigen::Matrix3d vecToMat(std::vector<double>& mat_vec) {
  if (mat_vec.size() != 9) {
    throw std::invalid_argument("Input vector must have 9 elements");
  }
  return Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(mat_vec.data());
}
