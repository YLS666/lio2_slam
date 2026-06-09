#pragma once

#include <pcl/point_cloud.h>
#include <array>
#include "cloud_utils/point_type.hpp"
#include "utils/eigen_types.hpp"

/**
 * ScanContext 回环描述子
 *
 * 将点云投影到极坐标网格, 生成 2D 描述子
 * 用于快速回环候选检索
 *
 * 参数:
 *   PC_NUM_RINGS  = 20  (径向分区数)
 *   PC_NUM_SECTORS = 60 (角度分区数)
 *   PC_MAX_RADIUS  = 80 (最大半径, 米)
 *
 * 描述子大小: 20 x 60 = 1200 维向量
 */
class ScanContext {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  static constexpr int RING_NUM = 20;
  static constexpr int SECTOR_NUM = 60;
  static constexpr double MAX_RADIUS = 80.0;

  using Descriptor = std::array<std::array<float, SECTOR_NUM>, RING_NUM>;

  ScanContext();

  /**
   * @brief 从点云生成描述子
   */
  void make(const CloudPtr& cloud);

  /**
   * @brief 计算两个描述子之间的距离
   *        距离越小越相似
   */
  static double distance(const Descriptor& a, const Descriptor& b);

  /**
   * @brief 获取描述子
   */
  const Descriptor& getDescriptor() const { return desc_; }

  /**
   * @brief 获取描述子的向量形式 (用于检索)
   */
  VXf getRingKey() const { return ring_key_; }

 private:
  Descriptor desc_;

  VXf ring_key_;  // 每个环的最大高度
};