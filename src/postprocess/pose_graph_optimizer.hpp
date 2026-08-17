#pragma once

#include <deque>
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
 *       2. 回环检测（ScanContext + ICP）
 *       3. 全局位姿图优化（g2o）
 *       4. 用优化后位姿重建最终地图
 */
namespace postprocess {

/**
 * @brief 从文本位姿文件加载关键帧（不含点云）
 *
 * 每行格式:
 *   id timestamp px py pz qx qy qz qw i00 i01 ... i55
 * 其中 q 为 (x,y,z,w)，i 为 6x6 信息矩阵按行展开（共 36 个）。
 */
bool loadKeyframePoses(const std::string& pose_file, std::deque<KeyFrame>& keyframes);

/**
 * @brief 将关键帧位姿写回文本文件（格式同上）
 */
bool saveKeyframePoses(const std::string& pose_file, const std::deque<KeyFrame>& keyframes);

/**
 * @brief 从 <cloud_dir>/kf_<id>.pcd 加载关键帧点云
 */
bool loadKeyframeClouds(const std::string& cloud_dir, std::deque<KeyFrame>& keyframes);

/**
 * @brief 回环检测：遍历所有关键帧，收集回环约束
 */
void detectLoopClosures(const std::deque<KeyFrame>& keyframes, std::vector<LoopPair>& loop_pairs);

/**
 * @brief 全局位姿图优化，将优化后位姿写回 keyframes
 */
bool globalOptimize(std::deque<KeyFrame>& keyframes, const std::vector<LoopPair>& loop_pairs);

/**
 * @brief 用优化后的关键帧位姿重建全局地图
 */
CloudPtr rebuildGlobalMap(const std::deque<KeyFrame>& keyframes, float voxel_size = 0.1f);

}  // namespace postprocess
