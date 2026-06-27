#pragma once

#include <deque>

#include <sensor_msgs/msg/imu.hpp>
#include "config_def.hpp"
#include "utils/eigen_types.hpp"

struct ImuState {
  double timestamp = 0.0;

  SE3 T;
  V3d v = V3d::Zero();
  V3d bg = V3d::Zero();
  V3d ba = V3d::Zero();
};

class ImuProcessor {
 public:
  explicit ImuProcessor(AllConfig& config);

  bool processImu(const sensor_msgs::msg::Imu& imu);

  ImuState interpolate(double t) const;

  bool isInitialized() const { return initialized_; }

  const std::deque<ImuState>& getStates() const { return states_; }

  // online update interface
  void updateBias(const V3d& bg, const V3d& ba);

  /**
   * @brief 将整条 IMU 姿态链对齐到 ESKF 估计值，保持相对运动不变
   */
  void resetStates(const SE3& T_reset, const V3d& v_reset);

 private:
  void initializeImu(double t, const V3d& gyr, const V3d& acc);

 private:
  bool initialized_ = false;

  int init_count_ = 0;

  static constexpr int kInitSize = 200;
  double g_norm_ = 9.80665;

  int init_attempt_ = 0;                      // 当前初始化尝试次数
  static constexpr int kMaxInitAttempts = 5;  // 最大尝试次数（约 5*200 = 1000 帧）

  static constexpr double kMaxStaticGyrVar = 0.5;   // 陀螺仪静态噪声最大方差
  static constexpr double kMaxStaticAccVar = 0.05;  // 加速度计静态噪声最大方差

  double acc_scale_ = 1.0;  // 加速度计放缩系数
  V3d gravity_dir_;         // 归一化重力方向

  std::deque<V3d> init_accs_;
  std::deque<V3d> init_gyrs_;
  std::deque<ImuState> states_;

  V3d gravity_;
  V3d bg_;
  V3d ba_;
};