#pragma once

#include <deque>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <sensor_msgs/msg/imu.hpp>

struct ImuState {
  double timestamp = 0.0;

  Eigen::Quaterniond q = Eigen::Quaterniond::Identity();
  Eigen::Vector3d p = Eigen::Vector3d::Zero();
  Eigen::Vector3d v = Eigen::Vector3d::Zero();
  Eigen::Vector3d bg = Eigen::Vector3d::Zero();
  Eigen::Vector3d ba = Eigen::Vector3d::Zero();
};

class ImuProcessor {
 public:
  ImuProcessor();

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

  std::deque<Eigen::Vector3d> init_accs_;
  std::deque<Eigen::Vector3d> init_gyrs_;
  std::deque<ImuState> states_;

  Eigen::Vector3d gravity_;
  Eigen::Vector3d bg_;
  Eigen::Vector3d ba_;
};