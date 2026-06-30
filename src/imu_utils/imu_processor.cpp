#include "imu_processor.hpp"
#include <glog/logging.h>
#include <iostream>

ImuProcessor::ImuProcessor(AllConfig& config) {
  g_norm_ = config.g_norm;  // 9.80665 m/s²
  gravity_ = V3d(0, 0, -g_norm_);
  bg_.setZero();
  ba_.setZero();
}

void ImuProcessor::updateBias(const V3d& bg, const V3d& ba) {
  bg_ = bg;
  ba_ = ba;
}

void ImuProcessor::initializeImu(double t, const V3d& gyr, const V3d& acc) {
  init_gyrs_.push_back(gyr);
  init_accs_.push_back(acc);

  init_count_++;

  if (init_count_ < kInitSize) {
    return;
  }

  // 1. 计算均值
  V3d mean_gyr = V3d::Zero();
  V3d mean_acc = V3d::Zero();

  for (const auto& g : init_gyrs_) {
    mean_gyr += g;
  }
  for (const auto& a : init_accs_) {
    mean_acc += a;
  }
  mean_gyr /= static_cast<double>(init_gyrs_.size());
  mean_acc /= static_cast<double>(init_accs_.size());

  // 2. 计算协方差
  V3d cov_gyr = V3d::Zero();
  V3d cov_acc = V3d::Zero();

  for (size_t i = 0; i < init_gyrs_.size(); ++i) {
    V3d dg = init_gyrs_[i] - mean_gyr;
    V3d da = init_accs_[i] - mean_acc;

    cov_gyr += dg.cwiseProduct(dg);
    cov_acc += da.cwiseProduct(da);
  }
  cov_gyr /= static_cast<double>(init_gyrs_.size());
  cov_acc /= static_cast<double>(init_accs_.size());

  LOG(INFO) << "cov_gyr = " << cov_gyr.transpose() << " (norm=" << cov_gyr.norm() << ")";
  LOG(INFO) << "cov_acc = " << cov_acc.transpose() << " (norm=" << cov_acc.norm() << ")";

  // 3. 检查是否满足静止条件（噪声足够小）+ 重试机制
  if (cov_gyr.norm() > kMaxStaticGyrVar || cov_acc.norm() > kMaxStaticAccVar) {
    init_attempt_++;
    LOG(WARNING) << "IMU 噪声过大（可能是运动中）,正在重试... "
                 << "尝试次数 " << init_attempt_ << " / " << kMaxInitAttempts;

    if (init_attempt_ >= kMaxInitAttempts) {
      // 达到最大重试次数：使用降级策略,用默认值初始化
      LOG(WARNING) << "IMU 初始化达到最大尝试次数,使用降级默认值";

      // 零偏使用当前均值（虽然可能有误差,但比0好）
      bg_ = mean_gyr;
      ba_.setZero();

      // 重力：使用 mean_acc 方向,长度归一化为 9.80665
      gravity_dir_ = -mean_acc.normalized();
      acc_scale_ = g_norm_ / mean_acc.norm();
      Qd q0 = Qd::FromTwoVectors(gravity_dir_, V3d(0, 0, -1));

      ImuState init_state;
      init_state.timestamp = t;
      init_state.T = SE3(q0, V3d::Zero());
      init_state.v.setZero();
      init_state.bg = bg_;
      init_state.ba = ba_;

      states_.push_back(init_state);
      initialized_ = true;

      LOG(INFO) << "========== IMU INIT (FALLBACK) ==========";
      LOG(INFO) << "bg = " << bg_.transpose();
      LOG(INFO) << "ba = " << ba_.transpose();
      LOG(INFO) << "acc_scale_ = " << acc_scale_;
      LOG(INFO) << "IMU init success (fallback).";
    } else {
      // 未达到最大次数：清空数据,重新采样
      init_gyrs_.clear();
      init_accs_.clear();
      init_count_ = 0;
    }
    return;
  }

  // 4. 初始化成功：设置零偏
  bg_ = mean_gyr;
  ba_.setZero();  // 静止时加速度计偏置不可观,设为0

  // 5. 重力缩放
  // 使用 mean_acc 精确校准重力方向和大小
  gravity_dir_ = -mean_acc.normalized();
  acc_scale_ = g_norm_ / mean_acc.norm();  // 缩放系数

  // 使用精确的重力方向计算初始姿态
  Qd q0 = Qd::FromTwoVectors(gravity_dir_, V3d(0, 0, -1));

  // 6. 创建初始状态
  ImuState init_state;
  init_state.timestamp = t;
  init_state.T = SE3(q0, V3d::Zero());
  init_state.v.setZero();
  init_state.bg = bg_;
  init_state.ba = ba_;

  states_.push_back(init_state);
  initialized_ = true;
  last_gyr_ = mean_gyr;
  last_acc_ = mean_acc;
  has_last_imu_ = true;

  LOG(INFO);
  LOG(INFO) << "========== IMU INIT ==========";
  LOG(INFO) << "imu samples : " << init_count_;
  LOG(INFO) << "cov_gyr     = " << cov_gyr.transpose() << " (norm=" << cov_gyr.norm() << ")";
  LOG(INFO) << "cov_acc     = " << cov_acc.transpose() << " (norm=" << cov_acc.norm() << ")";
  LOG(INFO) << "bg          = " << bg_.transpose();
  LOG(INFO) << "ba          = " << ba_.transpose();
  LOG(INFO) << "mean acc    = " << mean_acc.transpose() << " (norm=" << mean_acc.norm() << ")";
  LOG(INFO) << "mean gyr    = " << mean_gyr.transpose() << " (norm=" << mean_gyr.norm() << ")";
  LOG(INFO) << "acc_scale   = " << acc_scale_;
  LOG(INFO) << "gravity_dir = " << gravity_dir_.transpose();
  LOG(INFO) << "grav        = " << (gravity_dir_ / gravity_dir_.norm() * g_norm_).transpose();
  LOG(INFO) << "init quat   = " << q0.coeffs().transpose();
  LOG(INFO) << "IMU init success.";
}

bool ImuProcessor::processImu(const sensor_msgs::msg::Imu& imu) {
  double t = imu.header.stamp.sec + imu.header.stamp.nanosec * 1e-9;  // s

  V3d gyr(imu.angular_velocity.x, imu.angular_velocity.y, imu.angular_velocity.z);
  V3d acc(imu.linear_acceleration.x, imu.linear_acceleration.y, imu.linear_acceleration.z);

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
    LOG(WARNING) << "imu数据时间差不合法,丢弃当前帧数据！ dt = " << dt;
    return false;
  }

  // 去除ba和bg
  gyr -= bg_;
  acc -= ba_;

  if (!has_last_imu_) {
    last_gyr_ = gyr;
    last_acc_ = acc;
    has_last_imu_ = true;
  }

  // 中值
  V3d gyr_mid = 0.5 * (last_gyr_ + gyr);
  V3d acc_mid = 0.5 * (last_acc_ + acc);
  // 姿态积分
  Qd dq = deltaQ(gyr_mid * dt);
  Qd q = last.T.unit_quaternion() * dq;
  q.normalize();
  // 加速度更新：当前帧的加速度 = 当前帧的旋转矩阵 * 当前帧的加速度 + 重力(一般为-g)
  Eigen::Quaterniond q_mid = last.T.unit_quaternion().slerp(0.5, q);
  V3d acc_world = q_mid * acc_mid + gravity_;
  // 位姿更新：当前帧的位姿 = 上一帧的位姿 + 上一帧的线速度 * 时间差 + 0.5 * 加速度 * 时间差平方
  V3d p = last.T.translation() + last.v * dt + 0.5 * acc_world * dt * dt;
  ImuState state;
  state.timestamp = t;
  state.T = SE3(q, p);
  // 速度更新：当前帧的速度 = 上一帧的速度 + 当前帧的加速度 * 时间差
  state.v = last.v + acc_world * dt;
  // 保存偏置
  state.bg = bg_;
  state.ba = ba_;

  states_.push_back(state);

  last_gyr_ = gyr;
  last_acc_ = acc;

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
    LOG(WARNING) << "imu数据为空,无法插值！";
    return ImuState();
  }

  // 检查时间是否合法
  if (t <= states_.front().timestamp) {
    LOG(WARNING) << "imu数据时间回朔,无法插值！ t = " << t;
    return states_.front();
  }
  if (t >= states_.back().timestamp) {
    LOG(WARNING) << "imu数据时间超出范围,无法插值！ t = " << t;
    return states_.back();
  }

  // 插值
  // 找到第一个大于t的元素
  // 如果找不到,返回最后一个元素
  auto it = std::lower_bound(states_.begin(), states_.end(), t,
                             [](const ImuState& s, double time) { return s.timestamp < time; });

  // 如果找到的元素是第一个元素,返回第一个元素
  if (it == states_.begin()) {
    return *it;
  }

  auto prev = std::prev(it);  // 上一个元素

  const auto& s1 = *prev;
  const auto& s2 = *it;

  double ratio = (t - s1.timestamp) / (s2.timestamp - s1.timestamp);  // 插值比例

  ImuState state;

  state.timestamp = t;

  Qd q = s1.T.unit_quaternion().slerp(ratio, s2.T.unit_quaternion());
  V3d p = (1.0 - ratio) * s1.T.translation() + ratio * s2.T.translation();

  state.T = SE3(q, p);
  state.v = (1.0 - ratio) * s1.v + ratio * s2.v;
  state.bg = (1.0 - ratio) * s1.bg + ratio * s2.bg;
  state.ba = (1.0 - ratio) * s1.ba + ratio * s2.ba;

  return state;
}

void ImuProcessor::resetStates(const SE3& T_reset, const V3d& v_reset) {
  if (!initialized_ || states_.empty()) {
    return;
  }
  // 计算最后一个 state 与 ESKF 估计的偏差
  SE3 T_correction = T_reset * states_.back().T.inverse();
  V3d v_correction = v_reset - states_.back().v;

  // 对整条姿态链施加相同偏移，保持相对运动不变
  for (auto& s : states_) {
    s.T = T_correction * s.T;
    s.v += v_correction;
  }
}
