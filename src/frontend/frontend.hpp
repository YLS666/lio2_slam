#pragma once

#include <memory>
#include "backend/backend.hpp"
#include "cloud_utils/point_type.hpp"
#include "config_def.hpp"
#include "estimator/ieskf.hpp"
#include "frontend/registration.hpp"
#include "frontend/state.hpp"
#include "frontend/voxel_map.hpp"
#include "utils/ros_types.hpp"
#include "visualization/pangolin_viewer.hpp"

class Frontend {
 public:
  explicit Frontend(AllConfig config);

  /** @brief 设置基础地图参数 */
  void setMapParams(float voxel_size, float block_size, int block_radius) {
    map_ = std::make_unique<VoxelMap>(voxel_size, block_size, block_radius);
  }

  /** @brief 设置 IESKF 参数 */
  void setESKFParams(double gyr_noise, double acc_noise, double bg_noise, double ba_noise) {
    ieskf_->setImuNoise(gyr_noise, acc_noise, bg_noise, ba_noise);
  }

  /** @brief 设置关键帧参数 */
  void setKeyframeParams(double dist_thresh, double angle_thresh) {
    backend_->setKeyframeDistance(dist_thresh);
    backend_->setKeyframeAngle(angle_thresh);
  }

  void setImuBiasNoise(double gyr_bias_noise, double acc_bias_noise) {
    ieskf_->setImuNoise(ieskf_->getGyrNoise(), ieskf_->getAccNoise(), gyr_bias_noise, acc_bias_noise);
  }

  /** @brief 设置最大关键帧数及保留点云的最大帧数 */
  void setMaxKeyFrames(int n) { backend_->setMaxKeyFrames(n); }
  void setMaxKfClouds(int n) { backend_->setMaxKfClouds(n); }

  /** @brief 初始化 (设置初始位姿) */
  void init(const State& init_state);

  /** @brief 是否已初始化 */
  bool isInitialized() const { return initialized_; }

  /**
   * @brief 短期 IMU 递推（每次点云帧处理前调用一次）
   *
   * 从 IESKF 当前状态（上一帧可靠位姿）出发，
   * 用 imu_states 中相对于点云时间的最近 N 帧做一次性递推，
   * 作为当前帧配准的初值。
   *
   * @param imu_datas   原始 IMU 消息队列
   * @param cloud_time  当前点云帧的时间戳
   * @param g_norm      重力范数 (IESKF 内部估计 g, 此参数仅用于 IMU 消息的 g→m/s² 转换)
   */
  void propagateFromTrustedPose(const std::deque<Imu>& imu_datas, double cloud_time, double g_norm);

  /**
   * @brief NDT + IESKF 配准 (每帧点云数据调用)
   *
   * 新流程 (对齐 slam_tools):
   *   1. reg_->setSource(feature_cloud)
   *   2. ieskf_->updateUsingCustomObserve(lambda) — IEKF 迭代中调用 NDT 回调
   *   3. ieskf_->getNominalState() 获取更新后的状态
   *
   * @param cloud       去畸变后的点云
   * @param kf_save_dir 保存关键帧点云的目录
   * @return State 校正后的全量状态 (含 bg, ba, g)
   */
  State process(const CloudPtr& cloud, const std::string& kf_save_dir = "");

  /** @brief 特征点云采样 (用于关键帧) */
  CloudPtr featureSample(const CloudPtr& cloud) const;

  /** @brief 获取当前状态 */
  State getState() const { return state_; }

  /** @brief 获取 6-DOF 位姿协方差 (从 IESKF 的 18×18 协方差中提取) */
  Eigen::Matrix<double, 6, 6> getPoseCovariance() const { return ieskf_->getPoseCovariance(); }

  /** @brief 获取 18-DOF 全状态协方差 */
  Eigen::Matrix<double, 18, 18> getCovariance() const { return ieskf_->getCovariance(); }

  /** @brief 保存地图 */
  void saveMap(const std::string& save_dir);

  /** @brief 获取后端关键帧 */
  const std::deque<KeyFrame>& getKeyframes() const { return backend_->getKeyFrames(); }

  bool lastRegSuccess() const { return last_reg_success_; }

  bool isDiverged() const { return diverged_; }

  /** @brief 初始化并启动 Pangolin 可视化 */
  void initViewer();

  /** @brief 停止可视化 */
  void stopViewer() {
    if (viewer_) {
      viewer_->stop();
    }
  }

 private:
  // 核心组件
  std::unique_ptr<VoxelMap> map_;
  std::unique_ptr<NDTRegistration> reg_;
  std::unique_ptr<IESKF> ieskf_;
  std::unique_ptr<Backend> backend_;

  // 可视化
  std::unique_ptr<PangolinViewer> viewer_;
  bool is_use_viewer_ = true;

  // 状态
  State state_;
  bool initialized_ = false;
  int frame_count_ = 0;

  // 配准后的特征点云 (用于关键帧)
  CloudPtr last_feature_cloud_;

  bool last_reg_success_ = false;
  bool diverged_ = false;

  /**
   * @brief 将已优化的关键帧点云合并到地图
   */
  void mergeOptimizedKeyframesToMap();
};
