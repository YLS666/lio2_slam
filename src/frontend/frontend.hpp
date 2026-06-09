#pragma once

#include <memory>
#include "backend/backend.hpp"
#include "cloud_utils/point_type.hpp"
#include "estimator/eskf.hpp"
#include "frontend/registration.hpp"
#include "frontend/state.hpp"
#include "frontend/voxel_map.hpp"
#include "loop_closure/loop_closure.hpp"

class Frontend {
 public:
  Frontend();
  /** @brief 设置基础地图参数 (转发给 VoxelMap) */
  void setMapParams(float voxel_size, float block_size, int block_radius) {
    map_ = std::make_unique<VoxelMap>(voxel_size, block_size, block_radius);
  }

  /** @brief 设置 ESKF 参数 */
  void setESKFParams(double gyr_noise, double acc_noise, double ang_meas_std, double pos_meas_std) {
    eskf_->setImuNoise(gyr_noise, acc_noise, 1e10, 1e10);  // bias噪声极大=不更新
    eskf_->setMeasNoise(ang_meas_std, pos_meas_std);
  }

  /** @brief 设置关键帧参数 */
  void setKeyframeParams(double dist_thresh, double angle_thresh) {
    backend_->setKeyframeDistance(dist_thresh);
    backend_->setKeyframeAngle(angle_thresh);
  }

  /** @brief 初始化 (设置初始位姿) */
  void init(const State& init_state);

  /** @brief 是否已初始化 */
  bool isInitialized() const { return initialized_; }

  /**
   * @brief IMU 前向传播 (每帧 IMU 数据调用)
   *
   * @param gyr  陀螺仪 (已减 bias)
   * @param acc  加速度计 (已减 bias)
   * @param dt   时间间隔
   */
  void predict(const V3d gyr, const V3d acc, double dt);

  /**
   * @brief 配准 + 观测更新 (每帧点云数据调用)
   *
   * @param cloud  去畸变后的点云
   * @return State 校正后的状态
   */
  State process(const CloudPtr& cloud);

  /**
   * @brief 特征点云采样 (用于关键帧)
   *
   * @param cloud  去畸变后的点云
   * @return CloudPtr 特征点云
   */
  CloudPtr featureSample(const CloudPtr& cloud) const;

  /** @brief 获取当前状态 */
  State getState() const { return state_; }

  /** @brief 获取 ESKF 的协方差 */
  Eigen::Matrix<double, 9, 9> getCovariance() const { return eskf_->getCovariance(); }

  /** @brief 保存地图 */
  void saveMap(const std::string& filename) const;

  /** @brief 获取后端关键帧 */
  const std::deque<KeyFrame>& getKeyframes() const { return backend_->getKeyFrames(); }

 private:
  // 核心组件
  std::unique_ptr<VoxelMap> map_;
  std::unique_ptr<Registration> reg_;
  std::unique_ptr<ESKF> eskf_;
  std::unique_ptr<Backend> backend_;
  std::unique_ptr<LoopClosure> loop_closure_;

  // 状态
  State state_;
  State raw_obs_state_;  // 配准原始观测 (用于ESKF)

  bool initialized_ = false;
  int frame_count_ = 0;

  int loop_closure_interval_ = 50;  // 每隔 50 帧触发一次回环检测

  // 配准后的特征点云 (用于关键帧)
  CloudPtr last_feature_cloud_;

  /**
   * @brief 每隔 N 帧触发一次回环检测
   */
  void tryLoopClosure();
};