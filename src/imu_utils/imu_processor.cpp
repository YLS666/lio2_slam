#include "imu_processor.hpp"

#include <algorithm>
#include <iomanip>
#include <iostream>
#include "utils/eigen_types.hpp"

ImuProcessor::ImuProcessor() {
  gravity_ = Eigen::Vector3d(0, 0, -g_norm_);
  bg_.setZero();
  ba_.setZero();
}

bool ImuProcessor::isInitialized() const { return initialized_; }

const std::deque<ImuState>& ImuProcessor::getStates() const { return states_; }

void ImuProcessor::updateBias(const Eigen::Vector3d& bg, const Eigen::Vector3d& ba) {
  bg_ = bg;
  ba_ = ba;
}

void ImuProcessor::initializeImu(double t, const Eigen::Vector3d& gyr, const Eigen::Vector3d& acc) {
  init_gyrs_.push_back(gyr);
  init_accs_.push_back(acc);

  init_count_++;

  if (init_count_ < kInitSize) {
    return;
  }

  Eigen::Vector3d mean_gyr = Eigen::Vector3d::Zero();
  Eigen::Vector3d mean_acc = Eigen::Vector3d::Zero();

  for (const auto& g : init_gyrs_) {
    mean_gyr += g;
  }

  for (const auto& a : init_accs_) {
    mean_acc += a;
  }

  mean_gyr /= init_gyrs_.size();
  mean_acc /= init_accs_.size();

  // gyro bias
  bg_ = mean_gyr;

  // accel bias
  // cannot fully observe during static
  // initialize as zero
  ba_.setZero();

  // gravity direction
  Eigen::Vector3d gravity_dir = -mean_acc.normalized();
  Eigen::Vector3d world_gravity(0, 0, -1);
  Eigen::Quaterniond q0 = Eigen::Quaterniond::FromTwoVectors(gravity_dir, world_gravity);

  ImuState init_state;
  init_state.timestamp = t;
  init_state.T = Sophus::SE3d(q0, Eigen::Vector3d::Zero());
  init_state.v.setZero();
  init_state.bg = bg_;
  init_state.ba = ba_;

  states_.push_back(init_state);

  initialized_ = true;

  std::cout << std::endl;
  std::cout << "========== IMU INIT ==========" << std::endl;
  std::cout << "imu samples : " << init_count_ << std::endl;
  std::cout << "bg = " << bg_.transpose() << std::endl;
  std::cout << "ba = " << ba_.transpose() << std::endl;
  std::cout << "gravity = " << gravity_.transpose() << std::endl;
  std::cout << "mean acc = " << mean_acc.transpose() << std::endl;
  std::cout << "init quaternion : " << std::endl << q0.coeffs().transpose() << std::endl;
  std::cout << "IMU init success." << std::endl << std::endl;
}

bool ImuProcessor::processImu(const sensor_msgs::msg::Imu& imu) {
  double t = imu.header.stamp.sec + imu.header.stamp.nanosec * 1e-9;  // 1778046511.092436

  Eigen::Vector3d gyr(imu.angular_velocity.x, imu.angular_velocity.y, imu.angular_velocity.z);
  Eigen::Vector3d acc(imu.linear_acceleration.x, imu.linear_acceleration.y, imu.linear_acceleration.z);

  // g -> m/s²
  acc *= g_norm_;

  // 初始化
  if (!initialized_) {
    initializeImu(t, gyr, acc);

    return false;
  }

  // 时间合法性检查
  const ImuState& last = states_.back();
  double dt = t - last.timestamp;  // 计算当前帧imu数据与上一帧imu数据的时间差

  if (dt <= 0.0 || dt > 0.1) {
    std::cout << "imu数据时间差不合法，丢弃当前帧数据！ dt = " << dt << std::endl;
    return false;
  }

  // 去除ba和bg
  gyr -= bg_;
  acc -= ba_;

  // SO3更新
  Eigen::Vector3d omega = gyr * dt;       // 计算当前帧imu数据的角速度
  Eigen::Quaterniond dq = deltaQ(omega);  // 计算当前帧imu数据的旋转矩阵

  ImuState state;
  state.timestamp = t;
  // 姿态更新：当前帧的姿态 = 上一帧的姿态 * 当前帧的增量旋转
  Eigen::Quaterniond q = last.T.unit_quaternion() * dq;
  q.normalize();
  // 加速度更新：当前帧的加速度 = 当前帧的旋转矩阵 * 当前帧的加速度 + 重力(一般为-g)
  Eigen::Vector3d acc_world = q * acc + gravity_;
  // 位姿更新：当前帧的位姿 = 上一帧的位姿 + 上一帧的线速度 * 时间差 + 0.5 * 加速度 * 时间差平方
  Eigen::Vector3d p = last.T.translation() + last.v * dt + 0.5 * acc_world * dt * dt;
  state.T = Sophus::SE3d(q, p);
  // 速度更新：当前帧的速度 = 上一帧的速度 + 当前帧的加速度 * 时间差
  state.v = last.v + acc_world * dt;
  // 保存偏置
  state.bg = bg_;
  state.ba = ba_;

  // std::cout << "=========== IMU PROCESS ==========" << std::endl;
  // std::cout << std::fixed << std::setprecision(6);
  // std::cout << "timestamp : " << state.timestamp << std::endl;
  // std::cout << "acc : " << acc.transpose() << std::endl;
  // std::cout << "acc_world : " << acc_world.transpose() << std::endl;
  // std::cout << "gravity : " << gravity_.transpose() << std::endl;
  // std::cout << "quaternion : " << state.T.unit_quaternion().coeffs().transpose() << std::endl;
  // std::cout << "position : " << state.T.translation().transpose() << std::endl;
  // std::cout << "velocity : " << state.v.transpose() << std::endl;
  // std::cout << "bg = " << bg_.transpose() << std::endl;
  // std::cout << "ba = " << ba_.transpose() << std::endl << std::endl;

  states_.push_back(state);

  // 数据清理
  while (!states_.empty()) {
    double dt = state.timestamp - states_.front().timestamp;

    // 保留最近5秒
    if (dt > 5.0) {
      states_.pop_front();
    } else {
      break;
    }
  }
  return true;
}

ImuState ImuProcessor::interpolate(double t) const {
  // 检查是否为空
  if (states_.empty()) {
    std::cout << "imu数据为空，无法插值！" << std::endl;
    return ImuState();
  }

  // 检查时间是否合法
  if (t <= states_.front().timestamp) {
    std::cout << "imu数据时间回朔，无法插值！ t = " << t << std::endl;
    return states_.front();
  }
  if (t >= states_.back().timestamp) {
    std::cout << "imu数据时间超出范围，无法插值！ t = " << t << std::endl;
    return states_.back();
  }

  // 插值
  // 找到第一个大于t的元素
  // 如果找不到，返回最后一个元素
  auto it = std::lower_bound(states_.begin(), states_.end(), t,
                             [](const ImuState& s, double time) { return s.timestamp < time; });

  // 如果找到的元素是第一个元素，返回第一个元素
  if (it == states_.begin()) {
    return *it;
  }

  auto prev = std::prev(it);  // 上一个元素

  const auto& s1 = *prev;
  const auto& s2 = *it;

  double ratio = (t - s1.timestamp) / (s2.timestamp - s1.timestamp);  // 插值比例

  ImuState state;

  state.timestamp = t;

  Eigen::Quaterniond q = s1.T.unit_quaternion().slerp(ratio, s2.T.unit_quaternion());
  Eigen::Vector3d p = (1.0 - ratio) * s1.T.translation() + ratio * s2.T.translation();

  state.T = Sophus::SE3d(q, p);
  state.v = (1.0 - ratio) * s1.v + ratio * s2.v;
  state.bg = (1.0 - ratio) * s1.bg + ratio * s2.bg;
  state.ba = (1.0 - ratio) * s1.ba + ratio * s2.ba;

  return state;
}