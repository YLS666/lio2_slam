#pragma once

#include "frontend/state.hpp"
#include "utils/eigen_types.hpp"

class ESKF {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ESKF();

  /**
   *@brief: 重设滤波器
   */
  void reset();

  /**
   *@brief: 设置初始名义状态
   */
  void setState(const Qd& q, const V3d& p, const V3d& v, double timestamp = 0.0);

  /**
   *@brief: 设置初始化协方差和噪声
   */
  void setCovariance(const Eigen::Matrix<double, 9, 9>& P) { P_ = P; }
  void setImuNoise(double gyr_noise_density, double acc_noise_density, double gyr_bias_noise, double acc_bias_noise) {
    gyr_noise_density_ = gyr_noise_density;
    acc_noise_density_ = acc_noise_density;
    gyr_bias_noise_ = gyr_bias_noise;
    acc_bias_noise_ = acc_bias_noise;
  }
  void setMeasNoise(double ang_std, double pos_std) {
    ang_meas_std_ = ang_std;
    pos_meas_std_ = pos_std;  // 位置测量噪声标准差(m)
  }

  /**
   *@brief: IMU预积分(前向传播)
   * 名义状态更新:
   *   q_{k+1} = q_k ⊗ exp(gyr·dt)
   *   p_{k+1} = p_k + v_k·dt + 0.5·(R_k·acc + g)·dt²
   *   v_{k+1} = v_k + (R_k·acc + g)·dt
   *
   * 协方差传播:
   *   P_{k+1} = F·P_k·Fᵀ + G·Q·Gᵀ
   */
  void predict(const V3d& gyr, const V3d& acc, double dt, double g_norm = 9.80665);

  /**
   * @brief 配准观测更新
   *
   * 观测: 配准结果 [q_obs, p_obs]
   * 残差: r_θ = log(R_nomᵀ·R_obs) ∈ so(3)
   *       r_p = p_obs - p_nom
   *
   * 观测矩阵 H = [I₃, 0₃, 0₃; 0₃, I₃, 0₃] (R⁶ˣ⁹)
   *
   * 由于配准从零开始(不依赖IMU预测)，观测直接给出绝对位姿。
   * ESKF 的作用是平滑配准结果，结合IMU运动模型进行滤波。
   */
  void observePose(const Qd& q_obs, const V3d& p_obs);

  /**
   * @brief 用外部可靠位姿重置滤波器状态
   *        用于回环/后端优化后将 IMU 递推归零
   *
   * @param q  优化后的姿态
   * @param p  优化后的位置
   * @param v  优化后的速度
   * @param P_reset  重置后的协方差（可选，默认小值）
   */
  void resetWithPose(const Qd& q, const V3d& p, const V3d& v,
                     const Eigen::Matrix<double, 9, 9>& P_reset = Eigen::Matrix<double, 9, 9>::Identity());

  /**
   * @brief 获取校正后的名义状态
   */
  State getNominalState() const {
    State s;
    s.q = q_;
    s.p = p_;
    s.v = v_;
    s.timestamp = timestamp_;  // 保存时间戳，用于后端优化时的边缘化
    return s;
  }

  /**
   * @brief 获取误差状态
   */
  Eigen::Matrix<double, 9, 1> getErrorState() const { return dx_; }

  /**
   * @brief 获取协方差矩阵 (用于后端优化, 边缘化等)
   */
  Eigen::Matrix<double, 9, 9> getCovariance() const { return P_; }

  /**
   * @brief 获取信息矩阵 (协方差逆)
   */
  Eigen::Matrix<double, 9, 9> getInformation() const { return P_.inverse(); }

  /**
   * @brief 手动重置误差状态为0 (ESKF的"重置"步骤)
   *        通常在 observePose 后自动调用
   */
  void resetErrorState() { dx_.setZero(); }

  /**
   * @brief 获取当前时间下IMU预测的位姿
   *        用于去畸变时的姿态插值
   */
  Qd getPredictedQuat() const { return q_; }
  V3d getPredictedPos() const { return p_; }
  V3d getPredictedVel() const { return v_; }

 private:
  Qd q_;                    // 名义姿态
  V3d p_;                   // 位置
  V3d v_;                   // 速度
  double timestamp_ = 0.0;  // 时间戳

  Eigen::Matrix<double, 9, 9> P_;  // 协方差矩阵

  Eigen::Matrix<double, 9, 1> dx_;  // 误差状态

  double gyr_noise_density_;  // 陀螺仪白噪声密度
  double acc_noise_density_;  // 加速度计白噪声密度
  double gyr_bias_noise_;     // 陀螺仪bias随机游走
  double acc_bias_noise_;     // 加速度计bias随机游走

  double ang_meas_std_;  // 角度测量噪声标准差(rad)
  double pos_meas_std_;  // 位置测量噪声标准差(m)

  bool initialized_;  // 滤波器是否初始化

  /**
   * @brief 更新名义状态：注入误差状态
   */
  void injectErrorState();
};