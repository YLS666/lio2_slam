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
 * 后端优化模块
 *
 * 功能:
 *   1. 关键帧管理: 基于距离/角度/时间间隔选取关键帧
 *   2. 滑动窗口优化: 维护最近 N 个关键帧的位姿图
 *   3. 全局优化: 当回环检测触发时, 进行全局位姿图优化
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

  /**
   * @brief 判断是否创建新关键帧, 如果是则添加到后端
   *
   * @param state    当前估计状态
   * @param cloud    当前帧点云(降采样后)
   * @param info_mat 信息矩阵(6x6)
   * @return true    创建了新关键帧
   */
  bool addKeyFrame(const State& state, const CloudPtr& cloud, const Eigen::Matrix<double, 6, 6>& info_mat);

  /**
   * @brief 滑动窗口优化
   *
   * 维护最近 window_size_ 个关键帧:
   *   - 第1帧固定 (边缘化约束)
   *   - 其余帧的位姿优化
   *   - 使用帧间相对约束作为测量
   */
  void slideWindowOptimize();

  void globalOptimize(const std::vector<LoopPair>& loop_pairs);

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

  double keyframe_distance_ = 0.5;      ///< 关键帧距离阈值 m
  double keyframe_angle_ = 0.35;        ///< 关键帧角度阈值 rad
  double keyframe_min_interval_ = 0.5;  ///< 关键帧最小间隔阈值 s

  int window_size_ = 20;  ///< 滑动窗口大小

  double last_keyframe_timestamp_ = -1.0;  ///< 上一个关键帧时间戳
};