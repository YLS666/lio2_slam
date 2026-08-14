#include "estimator/ieskf.hpp"
#include <glog/logging.h>
#include "utils/eigen_types.hpp"

IESKF::IESKF() { buildNoise(); }

IESKF::IESKF(const Options& options) : options_(options) { buildNoise(); }

void IESKF::buildNoise() {
  // 离散化噪声协方差
  // 误差状态顺序: [δp, δv, δR, δbg, δba, δg]
  //
  // δp: 无直接噪声驱动 (由 δv 间接驱动)
  // δv: 加速度计白噪声方差 (m²/s²)
  // δR: 陀螺仪白噪声方差 (rad²/s²)
  // δbg: 陀螺仪 bias 随机游走方差 (仅 update_bg_=true 时有值)
  // δba: 加速度计 bias 随机游走方差 (仅 update_ba_=true 时有值)
  // δg: 重力估计不确定性 (小值，允许滤波器在线修正重力方向)

  // 参考实现将 options 值直接作为方差使用，不做平方
  double gv = options_.gyr_noise_;
  double av = options_.acc_noise_;
  // bg/ba 不更新时不加噪声，避免 cov_ 对角线出现 1e20 导致数值发散
  double bgv = options_.update_bg_ ? options_.bg_noise_ : 1e-10;
  double bav = options_.update_ba_ ? options_.ba_noise_ : 1e-10;
  // 重力向量缓慢变化，给一个很小的噪声防止协方差塌缩
  double ggv = 1e-6;

  Q_.setZero();
  // clang-format off
  Q_.diagonal() << 0, 0, 0,                          // δp
                   av, av, av,                        // δv
                   gv, gv, gv,                        // δR
                   bgv, bgv, bgv,                     // δbg
                   bav, bav, bav,                     // δba
                   ggv, ggv, ggv;                     // δg  ← 关键: 重力向量的协方差不下沉
  // clang-format on
}

void IESKF::setImuNoise(double gyr_noise, double acc_noise, double bg_noise, double ba_noise) {
  options_.gyr_noise_ = gyr_noise;
  options_.acc_noise_ = acc_noise;
  options_.bg_noise_ = bg_noise;
  options_.ba_noise_ = ba_noise;
  buildNoise();
}

void IESKF::setState(const Qd& q, const V3d& p, const V3d& v, const V3d& bg, const V3d& ba, const V3d& g,
                     double timestamp) {
  R_ = SO3(q.normalized());
  p_ = p;
  v_ = v;
  bg_ = bg;
  ba_ = ba;
  g_ = g;
  current_time_ = timestamp;
  dx_.setZero();
  initialized_ = true;
}

bool IESKF::predict(const V3d& gyr, const V3d& acc, double dt) {
  if (!initialized_) {
    return false;
  }
  if (dt <= 0.0 || dt > 0.1) {
    return false;
  }

  // 1. 名义状态传播 (IMU 运动学)
  V3d acc_corrected = acc - ba_;  // 加速度减去在线估计的 bias
  V3d gyr_corrected = gyr - bg_;  // 角速度减去在线估计的 bias

  // 位置: p' = p + v*dt + 0.5*(R*acc + g)*dt²
  V3d p_new = p_ + v_ * dt + 0.5 * (R_ * acc_corrected + g_) * dt * dt;
  // 速度: v' = v + (R*acc + g)*dt
  V3d v_new = v_ + (R_ * acc_corrected + g_) * dt;
  // 姿态: R' = R * exp((gyr-bg)*dt)
  SO3 R_new = R_ * SO3::exp(gyr_corrected * dt);

  // 2. 协方差传播 P = F·P·Fᵀ + Q
  // 误差状态: [δp, δv, δR, δbg, δba, δg]
  // 连续时间 A 矩阵 (非零块):
  //   d(δp)/dt   = δv
  //   d(δv)/dt   = -R·[acc-ba]×·δR - R·δba + δg
  //   d(δR)/dt   = -[gyr-bg]×·δR - δbg
  //   d(δbg)/dt  = 0 (随机游走由噪声驱动)
  //   d(δba)/dt  = 0 (随机游走由噪声驱动)
  //   d(δg)/dt   = 0
  //
  // 离散化 F ≈ I + A·dt (一阶近似, 旋转块用闭式解):
  Mat18d F = Mat18d::Identity();

  // δv → δp: I·dt
  F.block<3, 3>(0, 3) = M3d::Identity() * dt;
  // δR → δv: -R·[acc-ba]×·dt
  F.block<3, 3>(3, 6) = -R_.matrix() * SO3::hat(acc_corrected) * dt;
  // δba → δv: -R·dt (加速度 bias 影响速度)
  F.block<3, 3>(3, 12) = -R_.matrix() * dt;
  // δg → δv: I·dt (重力误差影响速度)
  F.block<3, 3>(3, 15) = M3d::Identity() * dt;
  // δR → δR: exp(-[gyr-bg]×·dt)  (闭式解, 比 I-[ω]×·dt 更精确)
  F.block<3, 3>(6, 6) = SO3::exp(-gyr_corrected * dt).matrix();
  // δbg → δR: -I·dt
  F.block<3, 3>(6, 9) = -M3d::Identity() * dt;

  // 协方差传播
  cov_ = F * cov_ * F.transpose() + Q_;
  cov_ = (cov_ + cov_.transpose()) / 2.0;  // 对称化

  // 3. 更新名义状态
  R_ = R_new;
  p_ = p_new;
  v_ = v_new;
  current_time_ += dt;

  dx_.setZero();
  return true;
}

bool IESKF::updateUsingCustomObserve(NdtObsFunc obs, int min_effective) {
  // 保存迭代开始时的旋转，用于协方差投影
  SO3 start_R = R_;

  Eigen::Matrix<double, 18, 1> HTVr;
  Eigen::Matrix<double, 18, 18> HTVH;
  Mat18d Pk, Qk;

  for (int iter = 0; iter < options_.num_iterations_; ++iter) {
    // 1. 调用 NDT 观测回调 —— 用当前名义位姿计算 HᵀV⁻¹H 和 HᵀV⁻¹r
    int effective = obs(getNominalSE3(), HTVH, HTVr);

    if (effective < min_effective) {
      LOG(WARNING) << "[IESKF] iter=" << iter << " effective=" << effective << " < " << min_effective
                   << ", 有效点数不足，跳过本轮更新";
      return false;
    }

    // 2. 协方差投影 —— 补偿迭代间旋转变化
    // 误差状态是在 start_R 处定义的, 但 R 在迭代中变化了
    // 需要将 cov 从当前 R 的切空间投影回 start_R 的切空间
    Mat18d J = Mat18d::Identity();
    J.block<3, 3>(6, 6) = M3d::Identity() - 0.5 * SO3::hat((R_.inverse() * start_R).log());
    Pk = J * cov_ * J.transpose();

    // 3. 信息形式 Kalman 更新
    // P_post = (P^{-1} + HᵀV^{-1}H)^{-1}
    // dx     = P_post · HᵀV^{-1}r
    Qk = (Pk.inverse() + HTVH).inverse();
    dx_ = Qk * HTVr;

    LOG(INFO) << "[IESKF] iter=" << iter << " effective=" << effective << " dx_norm=" << dx_.norm()
              << " dp=" << dx_.head<3>().transpose() << " dR=" << dx_.segment<3>(6).transpose();

    // 4. 误差状态注入名义状态
    update();

    // 提前收敛判断
    if (dx_.norm() < options_.quit_eps_) {
      LOG(INFO) << "[IESKF] 第 " << iter << " 轮迭代收敛, dx=" << dx_.norm();
      break;
    }
  }

  // 5. 协方差更新
  cov_ = (Mat18d::Identity() - Qk * HTVH) * Pk;
  cov_ = (cov_ + cov_.transpose()) / 2.0;

  // 6. 协方差投影回当前 R 的切空间
  {
    Mat18d J = Mat18d::Identity();
    V3d dtheta = (R_.inverse() * start_R).log();
    J.block<3, 3>(6, 6) = M3d::Identity() - 0.5 * SO3::hat(dtheta);
    cov_ = J * cov_ * J.transpose();
    cov_ = (cov_ + cov_.transpose()) / 2.0;
  }

  dx_.setZero();
  return true;
}

void IESKF::update() {
  // 误差状态注入名义状态
  // dx = [δp, δv, δR, δbg, δba, δg]
  p_ += dx_.segment<3>(0);
  v_ += dx_.segment<3>(3);
  R_ = R_ * SO3::exp(dx_.segment<3>(6));

  if (options_.update_bg_) {
    bg_ += dx_.segment<3>(9);
  }
  if (options_.update_ba_) {
    ba_ += dx_.segment<3>(12);
  }
  g_ += dx_.segment<3>(15);
}

State IESKF::getNominalState() const {
  State s;
  s.q = R_.unit_quaternion();
  s.p = p_;
  s.v = v_;
  s.bg = bg_;
  s.ba = ba_;
  s.g = g_;
  s.timestamp = current_time_;
  return s;
}

Eigen::Matrix<double, 6, 6> IESKF::getPoseCovariance() const {
  // 从 18×18 协方差中提取 6-DOF 位姿块
  // 误差状态顺序: [δp, δv, δR, δbg, δba, δg]
  // 位姿 = [δp(0:3), δR(6:9)]
  Eigen::Matrix<double, 6, 6> pose_cov;
  pose_cov.block<3, 3>(0, 0) = cov_.block<3, 3>(0, 0);  // δp-δp
  pose_cov.block<3, 3>(0, 3) = cov_.block<3, 3>(0, 6);  // δp-δR
  pose_cov.block<3, 3>(3, 0) = cov_.block<3, 3>(6, 0);  // δR-δp
  pose_cov.block<3, 3>(3, 3) = cov_.block<3, 3>(6, 6);  // δR-δR
  return pose_cov;
}
