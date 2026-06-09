#pragma once
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <sophus/se2.hpp>
#include <sophus/se3.hpp>
#include <vector>

using V2d = Eigen::Vector2d;
using V3d = Eigen::Vector3d;
using V4d = Eigen::Vector4d;
using M2d = Eigen::Matrix2d;
using M3d = Eigen::Matrix3d;
using M4d = Eigen::Matrix4d;

using VXf = Eigen::VectorXf;
using V2f = Eigen::Vector2f;
using V3f = Eigen::Vector3f;
using V4f = Eigen::Vector4f;
using M2f = Eigen::Matrix2f;
using M3f = Eigen::Matrix3f;
using M4f = Eigen::Matrix4f;

using V2i = Eigen::Vector2i;
using V3i = Eigen::Vector3i;
using V4i = Eigen::Vector4i;
using M2i = Eigen::Matrix2i;
using M3i = Eigen::Matrix3i;
using M4i = Eigen::Matrix4i;

using Qd = Eigen::Quaterniond;

using SE2 = Sophus::SE2d;
using SE2f = Sophus::SE2f;
using SO2 = Sophus::SO2d;
using SE3 = Sophus::SE3d;
using SE3f = Sophus::SE3f;
using SO3 = Sophus::SO3d;

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
