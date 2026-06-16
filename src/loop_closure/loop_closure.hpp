#pragma once

#include "backend/backend.hpp"
#include "backend/keyframe.hpp"
#include "loop_closure/scan_context.hpp"
#include "utils/eigen_types.hpp"

/**
 * 回环检测模块
 *
 * 流程:
 *   1. 新关键帧 → 生成 ScanContext 描述子
 *   2. 在历史描述子库中检索 top-K 候选项
 *   3. 对候选项进行几何验证 (ICP配准)
 *   4. 验证通过 → 输出回环约束 → 后端全局优化
 */

class LoopClosure {
 public:
  LoopClosure() = default;

  /**
   * @brief 添加关键帧到检索库
   */
  void addKeyframe(const KeyFrame& kf);

  /**
   * @brief 回环检测
   *
   * @param current_kf  当前关键帧
   * @param loop_pairs  输出的回环约束 (如果检测到)
   * @return true       检测到回环
   */
  bool detect(const KeyFrame& current_kf, std::vector<LoopPair>& loop_pairs);

  /**
   * @brief 设置参数
   */
  void setMinInterval(int interval) { min_interval_ = interval; }
  void setCandidateNum(int n) { candidate_num_ = n; }
  void setMinScore(double s) { min_score_ = s; }

  /**
   * @brief 设置关键帧列表指针 (用于几何验证中查找候选帧)
   */
  void setKeyframesPtr(const std::deque<KeyFrame>* kfs) { keyframes_ptr_ = kfs; }

 private:
  // 历史描述子库
  std::vector<ScanContext::Descriptor> descriptors_;  // 描述子库
  std::vector<int> frame_ids_;

  // 参数
  int min_interval_ = 30;          // 最小间隔 (关键帧数)
  int candidate_num_ = 5;          // 候选数量
  double min_score_ = 0.3;         // 配准得分阈值
  double icp_score_thresh_ = 0.1;  // ICP配准得分阈值

  int last_loop_frame_id_ = -1;  // 上一次回环的帧ID（避免重复检测）

  const std::deque<KeyFrame>* keyframes_ptr_ = nullptr;

  /**
   * @brief 几何验证: ICP 配准
   *
   * 对候选关键帧和当前关键帧的点云做ICP
   * 如果配准收敛且分数足够 → 回环验证通过
   */
  bool geometricVerification(const KeyFrame& kf1, const KeyFrame& kf2, V3d& rel_p, Qd& rel_q);
};