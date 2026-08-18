#pragma once

#include <functional>
#include "frontend/state.hpp"
#include "utils/eigen_types.hpp"

/**
 * @brief 18-DOF 迭代误差状态卡尔曼滤波器 (IESKF)
 *
 * 误差状态顺序: [δp(3), δv(3), δR(3), δbg(3), δba(3), δg(3)]
 *
 * 与旧 ESKF(9-DOF) 的关键区别:
 *  1. bg(陀螺仪bias)、ba(加速度计bias)、g(重力向量) 全部在线估计
 *  2. 重力向量不再固定为 (0,0,-9.8)，而是作为状态量被滤波器修正
 *  3. NDT 观测以回调形式集成到 IEKF 迭代中，每次迭代用最新状态重算 Jacobian
 *     → IMU 先验(协方差 P) 和 NDT 观测(HTVH/HTVr) 在每次迭代中同时起作用
 */
class IESKF {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  struct Options {
    int num_iterations_ = 3;  // IEKF 最大迭代次数
    double quit_eps_ = 1e-3;  // dx 范数小于此值则提前收敛

    double gyr_noise_ = 1.7e-4;  // 陀螺仪白噪声方差 (rad²/s²)
    double acc_noise_ = 2.0e-3;  // 加速度计白噪声方差 (m²/s⁴)
    double bg_noise_ = 1e-6;     // 陀螺仪 bias 随机游走方差
    double ba_noise_ = 1e-4;     // 加速度计 bias 随机游走方差

    bool update_bg_ = true;  // 是否在线估计陀螺仪 bias
    bool update_ba_ = true;  // 是否在线估计加速度计 bias
  };

  IESKF();
  explicit IESKF(const Options& options);

  /** @brief 设置初始全量状态 */
  void setState(const Qd& q, const V3d& p, const V3d& v, const V3d& bg, const V3d& ba, const V3d& g,
                double timestamp = 0.0);

  /** @brief 设置协方差矩阵 */
  void setCovariance(const Eigen::Matrix<double, 18, 18>& P) { cov_ = P; }

  /** @brief 设置 IMU 噪声参数 (兼容旧接口) */
  void setImuNoise(double gyr_noise, double acc_noise, double bg_noise, double ba_noise);

  /**
   * @brief IMU 前向传播
   * @param gyr  陀螺仪读数 (已减去已校准 bias, 单位 rad/s)
   * @param acc  加速度计读数 (已减去已校准 bias, 单位 m/s²)
   * @param dt   时间间隔 (s)
   * @note 滤波器内部会再减去在线估计的 bg/ba, 因此传入的 gyr/acc 应为
   *       ImuProcessor 去完固定 bias 后的值
   */
  bool predict(const V3d& gyr, const V3d& acc, double dt);

  /**
   * @brief NDT 观测回调类型
   *
   * 回调函数负责:
   *   1. 用 input_pose 将当前帧点云变换到世界坐标
   *   2. 对每个点查询 NDT 地图, 计算残差 e = qs - μ
   *   3. 累加 H^T·V^{-1}·H 和 H^T·V^{-1}·r (18×18 / 18×1)
   *
   * @param input_pose  当前 IESKF 的名义位姿 (每次迭代更新)
   * @param HTVH        [输出] H^T·V^{-1}·H 信息矩阵, 仅 [δp,δR] 块非零
   * @param HTVr        [输出] H^T·V^{-1}·r 信息残差, 仅 [δp,δR] 块非零
   * @return            有效匹配点数 (用于判断配准是否成功)
   */
  using NdtObsFunc = std::function<int(const SE3& input_pose, Eigen::Matrix<double, 18, 18>& HTVH,
                                       Eigen::Matrix<double, 18, 1>& HTVr)>;

  /**
   * @brief IEKF 迭代更新 —— NDT 观测融合
   *
   * 每轮迭代:
   *   1. 调用 obs 回调 — 用当前名义位姿计算 NDT 的 HTVH 和 HTVr
   *   2. 协方差投影 — 补偿迭代间旋转变化 (J·cov·J^T)
   *   3. 信息形式 Kalman 更新 — Qk = (Pk^{-1} + HTVH)^{-1}, dx = Qk·HTVr
   *   4. 误差注入 — 将 dx 合入名义状态
   *
   * @return true 更新成功, false 有效点数不足
   */
  bool updateUsingCustomObserve(NdtObsFunc obs, int min_effective = 50);

  /** @brief 获取名义状态 */
  State getNominalState() const;

  /** @brief 获取当前的 SE3 位姿 */
  SE3 getNominalSE3() const { return SE3(R_, p_); }

  /** @brief 获取协方差矩阵 (18×18) */
  Eigen::Matrix<double, 18, 18> getCovariance() const { return cov_; }

  /** @brief 提取 6-DOF 位姿协方差 (用于后端信息矩阵) */
  Eigen::Matrix<double, 6, 6> getPoseCovariance() const;

  /** @brief 获取重力向量 */
  V3d getGravity() const { return g_; }

  /** @brief 设置重力范数约束值 (修复 g 范数漂移) */
  void setGravityNorm(double g) { g_norm_ = g; }

  /** @brief 获取在线估计的 bias */
  V3d getBg() const { return bg_; }
  V3d getBa() const { return ba_; }

  double getGyrNoise() const { return options_.gyr_noise_; }
  double getAccNoise() const { return options_.acc_noise_; }

 private:
  void buildNoise();

  /** @brief 误差状态注入名义状态 */
  void update();

  // 名义状态
  double current_time_ = 0.0;
  SO3 R_;
  V3d p_ = V3d::Zero();
  V3d v_ = V3d::Zero();
  V3d bg_ = V3d::Zero();
  V3d ba_ = V3d::Zero();
  V3d g_{0, 0, -9.80665};
  double g_norm_ = 9.80665;  // 重力范数约束值

  // 误差状态
  Eigen::Matrix<double, 18, 1> dx_ = Eigen::Matrix<double, 18, 1>::Zero();
  // 协方差
  Eigen::Matrix<double, 18, 18> cov_ = Eigen::Matrix<double, 18, 18>::Identity();
  // 离散噪声协方差
  Eigen::Matrix<double, 18, 18> Q_ = Eigen::Matrix<double, 18, 18>::Zero();

  Options options_;
  bool initialized_ = false;
};
