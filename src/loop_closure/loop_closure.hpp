#pragma once

#include <deque>
#include <unordered_map>
#include <vector>

#include "backend/backend.hpp"
#include "backend/keyframe.hpp"
#include "utils/eigen_types.hpp"

/**
 * 回环检测模块（离线后处理）
 *
 * 流程:
 *   1. 基于位姿距离筛选候选 (min_id_interval + min_distance + skip_id)
 *   2. 对候选做局部子图的多分辨率 NDT 配准 (TBB 并行)
 *   3. 得分超过阈值 → 输出回环约束 → 后端全局优化
 */
class LoopClosure {
 public:
  LoopClosure() = default;

  /**
   * @brief 批量回环检测
   *
   * @param keyframes   全部关键帧（id 与 deque 下标一致）
   * @param loop_pairs  输出的回环约束
   */
  void run(const std::deque<KeyFrame>& keyframes, std::vector<LoopPair>& loop_pairs);

  /**
   * @brief 设置参数
   */
  void setMinInterval(int v) { min_id_interval_ = v; }
  void setMinDistance(double v) { min_distance_ = v; }
  void setSkipId(int v) { skip_id_ = v; }
  void setNdtScoreTh(double v) { ndt_score_th_ = v; }
  void setSubmapRange(int v) { submap_idx_range_ = v; }

 private:
  /// 候选回环
  struct Candidate {
    int idx1 = 0;          ///< 候选点云1 (子图中心 / 目标)
    int idx2 = 0;          ///< 候选点云2 (源)
    SE3 Tij_;              ///< 候选点云相对位姿 (kf1 系下 kf2 的位姿)
    double ndt_score = 0;  ///< NDT 配准得分
  };

  /**
   * @brief 基于位姿距离提取回环候选
   */
  void detectCandidates(const std::vector<V3d>& translations, std::vector<Candidate>& candidates) const;

  /**
   * @brief 对单个候选做子图 NDT 配准
   */
  void computeForCandidate(const std::vector<const KeyFrame*>& kfs, const std::vector<M4f>& world_T,
                           const std::unordered_map<int, int>& id_to_index, Candidate& c) const;

  /**
   * @brief 以给定关键帧为中心构建世界系局部子图
   */
  CloudPtr buildSubmap(const std::vector<const KeyFrame*>& kfs, const std::vector<M4f>& world_T,
                       const std::unordered_map<int, int>& id_to_index, int center_id) const;

  // 参数
  int min_id_interval_ = 50;    ///< 候选两关键帧之间的最小 ID 间隔
  double min_distance_ = 30.0;  ///< 候选帧之间的最小 xy 距离阈值 (m)
  int skip_id_ = 5;             ///< 选中一个候选后隔开多少 ID 再选下一个
  double ndt_score_th_ = 3.0;   ///< 有效回环的 NDT 得分阈值

  int submap_idx_range_ = 40;  ///< 子图半径 (关键帧 ID)
  int submap_step_ = 4;        ///< 子图采样步长 (关键帧 ID)

  std::vector<double> resolutions_{10.0, 5.0, 4.0, 3.0};  ///< NDT 多分辨率匹配序列

  double ndt_trans_epsilon_ = 0.05;  ///< NDT 变换收敛阈值
  double ndt_step_size_ = 0.7;       ///< NDT 线搜索步长
  int ndt_max_iter_ = 40;            ///< NDT 最大迭代次数
};
