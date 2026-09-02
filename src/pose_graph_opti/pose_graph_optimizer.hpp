#pragma once

#include <deque>
#include <map>
#include <string>
#include <vector>
#include "backend/backend.hpp"
#include "cloud_utils/point_type.hpp"

/**
 * @brief 离线后处理：位姿图优化 + 回环检测 + 全局地图重建
 *
 * 与在线 frontend 完全解耦：
 *   - 在线阶段只负责提取关键帧、保存位姿文件与关键帧点云、构建临时地图；
 *   - 本模块在全部关键帧提取完成后离线运行：
 *       1. 从位姿文件 + pcd 文件加载关键帧
 *       2. 回环检测（距离提取 + NDT）
 *       3. 全局位姿图优化（g2o）
 *       4. 用优化后位姿重建最终地图
 */
class pose_graph_opti {
 public:
  explicit pose_graph_opti(std::string map_path);

  void run();

  /**
   * @brief 从文本位姿文件加载关键帧（不含点云）
   *
   * 每行格式:
   *   id timestamp px py pz qx qy qz qw i00 i01 ... i55
   * 其中 q 为 (x,y,z,w)，i 为 6x6 信息矩阵按行展开（共 36 个）。
   */
  bool loadKeyframePoses();

  /**
   * @brief 将关键帧位姿写回文本文件
   */
  void saveKeyframePoses();

  /**
   * @brief 从 <cloud_dir>/kf_<id>.pcd 加载关键帧点云
   */
  bool loadKeyframeClouds();

  /**
   * @brief 回环检测：遍历所有关键帧，收集回环约束
   */
  void detectLoopClosures();

  /**
   * @brief 全局位姿图优化，将优化后位姿写回 keyframes
   */
  void globalOptimize();

  /**
   * @brief 用优化后的关键帧位姿重建全局地图
   */
  void rebuildGlobalMap();

  /**
   * @brief 保存全局地图和分块地图和分块索引
   */
  void saveGlobalAndSplitMap();

 private:
  std::map<V2i, CloudPtr, less_vec<2>> map_data_;

  std::string map_path_;
  std::string keyframes_path_;
  std::string split_map_path_;
  std::string pose_file_;
  std::string cloud_dir_;
  std::string out_pose_file_;
  std::string out_map_file_;

  CloudPtr global_map_ = nullptr;
  std::deque<KeyFrame> keyframes_;
  std::vector<LoopPair> loop_pairs_;
};
