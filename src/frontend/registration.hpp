#pragma once

#include "cloud_utils/point_type.hpp"
#include "frontend/voxel_map.hpp"

class NDTRegistration {
 public:
  explicit NDTRegistration(int max_iteration = 5);

  /**
   * @brief 设置当前帧源点云
   */
  void setSource(const CloudPtr& cloud) { source_ = cloud; }

  /**
   * @brief 对给定位姿计算 NDT 残差的信息形式 HᵀV⁻¹H 和 HᵀV⁻¹r
   *
   * 用于 IESKF::updateUsingCustomObserve 的回调。
   * 每次 IEKF 迭代调用一次，用最新名义位姿重线性化。
   *
   * Jacobian J (3×18) 非零块:
   *   J = [I₃, 0, A, 0, 0, 0]  其中 A = -R·skew(q_lidar)
   *
   * @param map         NDT 体素地图 (只读访问, 支持 TBB 并发读取)
   * @param input_pose  当前 IEKF 名义位姿
   * @param HTVH        [输出] HᵀV⁻¹H (18×18)
   * @param HTVr        [输出] HᵀV⁻¹r (18×1)
   * @return            有效匹配点数
   */
  int computeResidualAndJacobians(const VoxelMap* map, const SE3& input_pose, Eigen::Matrix<double, 18, 18>& HTVH,
                                  Eigen::Matrix<double, 18, 1>& HTVr);

  int matchCount() const { return match_count_; }

  Eigen::Matrix<double, 6, 6> getCovariance() const { return covariance_; }

  void setHuber(bool enable, double k) {
    use_huber_ = enable;
    huber_k_ = k;
  }

  /** @brief 设置信息矩阵缩放系数 */
  void setInfoRatio(double ratio) { info_ratio_ = ratio; }

  /** @brief 设置马氏距离离群阈值  */
  void setOutlierThreshold(double th) { res_outlier_th_ = th; }

  /** @brief 设置并行线程数 (0=不限制, >0=限制) */
  void setNumThreads(int n) { num_threads_ = n; }

 private:
  double huberWeight(double error) const;

  int max_iteration_;
  bool use_huber_ = false;
  double huber_k_ = 0.3;
  double info_ratio_ = 0.01;     // HᵀV⁻¹H 缩放系数
  double res_outlier_th_ = 5.0;  // 马氏距离离群阈值

  int match_count_ = 0;
  Eigen::Matrix<double, 6, 6> covariance_ = Eigen::Matrix<double, 6, 6>::Identity();

  CloudPtr source_ = nullptr;

  int num_threads_ = 0;  // 0=不限制
};
