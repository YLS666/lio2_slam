#pragma once

#include <deque>

#include <pangolin/pangolin.h>

#include "cloud_utils/point_type.hpp"
#include "visualization/viewer_types.hpp"

namespace viewer {

/**
 * 绘制全局地图
 */
void DrawGlobalMap(const CloudPtr& cloud);

/**
 * 绘制局部地图
 */
void DrawLocalMap(const CloudPtr& cloud);

/**
 * 绘制当前点云
 */
void DrawCurrentCloud(const CloudPtr& cloud);

/**
 * 绘制轨迹
 */
void DrawTrajectory(const std::deque<TrajPoint>& trajectory);

}  // namespace viewer