#pragma once

#include <deque>

#include "backend/keyframe.hpp"
#include "cloud_utils/point_type.hpp"
#include "frontend/state.hpp"
#include "utils/eigen_types.hpp"

/**
 * @brief 全局位姿图优化
 *
 * @param loop_pairs 回环对: (frame_id_a, frame_id_b, relative_pose)
 */
struct LoopPair {
  int id_a;
  int id_b;
  V3d rel_p;
  Qd rel_q;
  double info_weight;  // 回环约束的信息权重
};

/**
 * 后端关键帧管理模块
 *
 * 功能:
 *   1. 关键帧管理: 基于距离/角度/时间间隔选取关键帧
 *   2. 分级内存管理: 释放已合并关键帧点云 / 淘汰超限旧关键帧
 */
class Backend {
 public:
  Backend();

  /** @brief 设置关键帧选取阈值 */
  void setKeyframeDistance(double d) { keyframe_distance_ = d; }
  void setKeyframeAngle(double a) { keyframe_angle_ = a; }
  void setKeyframeMinInterval(double t) { keyframe_min_interval_ = t; }

  /** @brief 设置滑动窗口大小 */
  void setWindowSize(int n) { window_size_ = n; }

  /** @brief 设置最大关键帧数（位姿骨架）和最大保留点云的关键帧数 */
  void setMaxKeyFrames(int n) { max_keyframes_ = n; }
  void setMaxKfClouds(int n) { max_kf_clouds_ = n; }

  /**
   * @brief 判断是否创建新关键帧, 如果是则添加到后端
   *
   * @param state    当前估计状态
   * @param cloud    当前帧点云(降采样后)
   * @param info_mat 信息矩阵(6x6)
   * @return true    创建了新关键帧
   */
  bool addKeyFrame(const State& state, const CloudPtr& cloud, const Eigen::Matrix<double, 6, 6>& info_mat,
                   int ndt_effective = 0, float ndt_rot_correction = 0.0f);

  /**
   * @brief 获取关键帧列表
   */
  const std::deque<KeyFrame>& getKeyFrames() const { return keyframes_; }

  /** @brief 获取关键帧数量 */
  int getKeyframeCount() const { return static_cast<int>(keyframes_.size()); }

  /** @brief 根据帧ID获取位姿 */
  bool getPose(int d, V3d& p, Qd& q) const;

  /**
   * @brief 标记指定关键帧已合并到地图
   *
   * @param ids  已合并的关键帧 ID 列表
   */
  void markKeyframesMerged(const std::vector<int>& ids);

  /**
   * @brief 获取滑动窗口中第一帧的位姿（用于边缘化约束）
   */
  bool getWindowFirstPose(V3d& p, Qd& q) const;

 private:
  std::deque<KeyFrame> keyframes_;  ///< 关键帧列表

  double keyframe_distance_ = 1.0;      ///< 关键帧距离阈值 m
  double keyframe_angle_ = 0.174;       ///< 关键帧角度阈值 rad
  double keyframe_min_interval_ = 0.5;  ///< 关键帧最小间隔阈值 s

  int window_size_ = 20;  ///< 滑动窗口大小

  int max_keyframes_ = 5000;  ///< 最大关键帧数（位姿骨架 ~300B/帧，内存友好）
  int max_kf_clouds_ = 300;   ///< 最多保留点云的关键帧数（~64KB/帧）
  int next_id_ = 0;           ///< 单调递增的关键帧 ID 计数器（淘汰后不复用，避免 PCD 文件名冲突）

  double last_keyframe_timestamp_ = -1.0;  ///< 上一个关键帧时间戳
};