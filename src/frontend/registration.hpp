#pragma once

#include "cloud_utils/point_type.hpp"
#include "frontend/state.hpp"
#include "frontend/voxel_map.hpp"
#include "utils/eigen_types.hpp"

class Registration {
 public:
  Registration();

  /**
   * @brief 配准主函数 (点对点 + Huber 鲁棒核 + TBB 并行)
   *
   * @param cloud  当前帧特征点云
   * @param map    局部地图
   * @param state  位姿 (输入初值，输出优化后结果)
   * @return true  配准成功
   */
  bool align(const CloudPtr& cloud, VoxelMap* map, State& state);

  /**
   * @brief 获取配准结果协方差 (6x6, 用于 ESKF 和关键帧)
   */
  Eigen::Matrix<double, 6, 6> getCovariance() const { return covariance_; }

  /**
   * @brief 获取内点数量 (权重 > 0.5 的点)
   */
  int getInlierCount() const { return inlier_count_; }

  /**
   * @brief 获取总匹配点数
   */
  int getMatchCount() const { return match_count_; }

  /**
   * @brief 设置 Huber 核参数 k
   * @param k 阈值 (米), 默认 0.3
   *          残差 > k 的点权重降低
   */
  void setHuberK(double k) { huber_k_ = k; }

  /**
   * @brief 启用/禁用 Huber 鲁棒核
   */
  void enableHuber(bool enable) { use_huber_ = enable; }

 private:
  M3d skew(const Eigen::Vector3d& v);

  Eigen::Matrix<double, 6, 6> covariance_;  // 配准结果的协方差矩阵
  int inlier_count_ = 0;                    // 配准结果的内点数
  int match_count_ = 0;                     // 配准结果的匹配点数
  double huber_k_ = 0.3;                    // Huber 鲁棒核的参数，默认 0.3m
  bool use_huber_ = true;                   // 是否使用 Huber 鲁棒核

  /**
   * @brief Huber 权重函数
   *
   *   w(r) = 1,          if |r| ≤ k
   *   w(r) = k / |r|,    if |r| > k
   *
   * 当残差很大时权重很小，有效抑制动态物体/坑洼地面
   */
  double huberWeight(double residual, double k) const {
    if (residual <= k) {
      return 1.0;
    }
    // Huber 核函数: 对大残差降权
    // 当残差为 2*k 时, 权重为 0.5
    // 当残差为 4*k 时, 权重为 0.25
    return k / residual;
  }
};