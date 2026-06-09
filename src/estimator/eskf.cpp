#include "estimator/eskf.hpp"

ESKF::ESKF() { reset(); }

void ESKF::reset() {
  q_.setIdentity();
  p_.setZero();
  v_.setZero();
  dx_.setZero();
  P_.setIdentity();
  P_ = Eigen::Matrix<double, 9, 9>::Identity() * 100.0;  // 默认协方差：较大的初始化不确定值

  gyr_noise_density_ = 1.7e-4;  // 0.01 rad/s
  acc_noise_density_ = 2.0e-3;  // 0.02 m/s^2
  gyr_bias_noise_ = 1e10;       // 设置极大 = 不更新
  acc_bias_noise_ = 1e10;
  ang_meas_std_ = 0.05;  // 2.86度
  pos_meas_std_ = 0.1;   // 10cm
  initialized_ = false;
}

void ESKF::setState(const Qd& q, const V3d& p, const V3d& v) {
  q_ = q.normalized();
  p_ = p;
  v_ = v;
  dx_.setZero();
  initialized_ = true;
}

void ESKF::predict(const V3d& gyr, const V3d& acc, double dt, const V3d& gravity) {
  if (!initialized_) {
    return;
  }
  if (dt <= 0.0 || dt > 0.1) {
    return;
  }

  // 1.名义状态传播
  Qd dq = deltaQ(gyr * dt);  // 姿态: q_{k+1} = q_k ⊗ exp(gyr * dt)
  Qd q_new = (q_ * dq).normalized();
  M3d R = q_.toRotationMatrix();                         // 旋转矩阵
  V3d acc_world = R * acc + gravity;                     // 世界坐标系下的加速度
  V3d p_new = p_ + v_ * dt + 0.5 * acc_world * dt * dt;  // 位置
  V3d v_new = v_ + acc_world * dt;                       // 速度

  // 2.协方差传播
  Eigen::Matrix<double, 9, 9> F = Eigen::Matrix<double, 9, 9>::Identity();  // 状态转移矩阵
  M3d acc_skew = skewSymmetric(acc);                                        // 加速度的 skew-symmetric矩阵
  F.block<3, 3>(3, 6) = M3d::Identity() * dt;                               // 位置
  F.block<3, 3>(6, 0) = -R * acc_skew * dt;                                 // 速度

  Eigen::Matrix<double, 9, 6> G = Eigen::Matrix<double, 9, 6>::Zero();  // 噪声输入矩阵
  G.block<3, 3>(0, 0) = M3d::Identity() * dt;                           // 位置
  G.block<3, 3>(6, 3) = R * dt;                                         // 速度

  Eigen::Matrix<double, 6, 6> Q = Eigen::Matrix<double, 6, 6>::Zero();                   // 离散噪声协方差
  Q.block<3, 3>(0, 0) = M3d::Identity() * gyr_noise_density_ * gyr_noise_density_ * dt;  // 陀螺仪噪声
  Q.block<3, 3>(3, 3) = M3d::Identity() * acc_noise_density_ * acc_noise_density_ * dt;  // 加速度计噪声

  // 协方差传播：P = F*P*F^T + G*Q*G^T
  // 不更新bias噪声
  P_ = F * P_ * F.transpose() + G * Q * G.transpose();

  P_ = (P_ + P_.transpose()) / 2.0;  // 协方差矩阵对称化

  // 更新名义状态
  q_ = q_new;
  p_ = p_new;
  v_ = v_new;

  dx_.setZero();  // 误差状态清零
}

void ESKF::observePose(const Qd& q_obs, const V3d& p_obs) {
  if (!initialized_) {
    setState(q_obs, p_obs, V3d::Zero());
    return;
  }

  // 1.计算残差
  // 旋转残差
  M3d R_nom = q_.toRotationMatrix();
  M3d R_obs = q_obs.toRotationMatrix();
  M3d R_err = R_nom.transpose() * R_obs;
  V3d r_theta = SO3(R_err).log();

  // 位置残差
  V3d r_p = p_obs - p_;

  // 残差向量
  Eigen::Matrix<double, 6, 1> residual;
  residual.head<3>() = r_theta;
  residual.tail<3>() = r_p;

  // 2.观测矩阵
  Eigen::Matrix<double, 6, 9> H = Eigen::Matrix<double, 6, 9>::Zero();
  H.block<3, 3>(0, 0) = M3d::Identity();  // 旋转
  H.block<3, 3>(3, 3) = M3d::Identity();  // 位置

  // 3.观测噪声协方差
  Eigen::Matrix<double, 6, 6> R = Eigen::Matrix<double, 6, 6>::Identity();  // 观测噪声协方差
  R.block<3, 3>(0, 0) = M3d::Identity() * ang_meas_std_ * ang_meas_std_;
  R.block<3, 3>(3, 3) = M3d::Identity() * pos_meas_std_ * pos_meas_std_;

  // 4.卡尔曼增益
  Eigen::Matrix<double, 6, 6> S = H * P_ * H.transpose() + R;        // 观测噪声协方差
  Eigen::Matrix<double, 9, 6> K = P_ * H.transpose() * S.inverse();  // 卡尔曼增益

  // 5.更新误差状态
  dx_ = K * residual;

  // 6.更新协方差
  P_ = (Eigen::Matrix<double, 9, 9>::Identity() - K * H) * P_;
  P_ = (P_ + P_.transpose()) / 2.0;

  // 7.将误差状态注入名义状态
  injectErrorState();

  // 8.ESKF重置误差状态归零
  dx_.setZero();
}

void ESKF::injectErrorState() {
  V3d dtheta = dx_.head<3>();
  Qd dq = deltaQ(dtheta);
  q_ = (q_ * dq).normalized();

  p_ += dx_.segment<3>(3);
  v_ += dx_.segment<3>(6);
}
