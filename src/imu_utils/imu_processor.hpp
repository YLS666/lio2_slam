#pragma once

#include <deque>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <sensor_msgs/msg/imu.hpp>
#include <sophus/se3.hpp>
#include "config_def.hpp"

struct ImuState {
  double timestamp = 0.0;

  Sophus::SE3d T;
  Eigen::Vector3d v = Eigen::Vector3d::Zero();
  Eigen::Vector3d bg = Eigen::Vector3d::Zero();
  Eigen::Vector3d ba = Eigen::Vector3d::Zero();
};

class ImuProcessor {
 public:
  explicit ImuProcessor(AllConfig& config);

  bool processImu(const sensor_msgs::msg::Imu& imu);

  ImuState interpolate(double t) const;

  bool isInitialized() const;

  const std::deque<ImuState>& getStates() const;

  // online update interface
  void updateBias(const Eigen::Vector3d& bg, const Eigen::Vector3d& ba);

 private:
  void initializeImu(double t, const Eigen::Vector3d& gyr, const Eigen::Vector3d& acc);

 private:
  bool initialized_ = false;

  int init_count_ = 0;

  static constexpr int kInitSize = 200;
  double g_norm_ = 9.80665;

  int init_attempt_ = 0;                      // 当前初始化尝试次数
  static constexpr int kMaxInitAttempts = 5;  // 最大尝试次数（约 5*200 = 1000 帧）

  static constexpr double kMaxStaticGyrVar = 0.5;   // 陀螺仪静态噪声最大方差
  static constexpr double kMaxStaticAccVar = 0.05;  // 加速度计静态噪声最大方差

  double acc_scale_ = 1.0;       // 加速度计放缩系数
  Eigen::Vector3d gravity_dir_;  // 归一化重力方向

  std::deque<Eigen::Vector3d> init_accs_;
  std::deque<Eigen::Vector3d> init_gyrs_;
  std::deque<ImuState> states_;

  Eigen::Vector3d gravity_;
  Eigen::Vector3d bg_;
  Eigen::Vector3d ba_;
};