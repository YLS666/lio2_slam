#pragma once

#include <deque>
#include <memory>

#include <pangolin/pangolin.h>

#include "cloud_utils/point_type.hpp"
#include "utils/eigen_types.hpp"

namespace viewer {

/**
 * @brief 轨迹点
 */
struct TrajPoint {
  TrajPoint() = default;

  TrajPoint(double t, const V3d& position, const Qd& rotation) : timestamp(t), p(position), q(rotation) {}

  double timestamp = 0.0;

  V3d p = V3d::Zero();

  Qd q = Qd::Identity();
};

/**
 * @brief Plot 管理结构
 */
struct PlotContext {
  PlotContext() = default;

  std::unique_ptr<pangolin::DataLog> log;

  pangolin::Plotter* vel = nullptr;

  pangolin::Plotter* gyr = nullptr;

  pangolin::Plotter* acc = nullptr;
};

/**
 * @brief Viewer 本地缓存
 *
 * run() 每帧从 PangolinViewer 拷贝一次，
 * 后续所有渲染均使用这里的数据，
 * 避免长时间占用互斥锁。
 */
struct ViewerCache {
  ViewerCache() {
    current_cloud.reset(new PointCloudType());
    local_map.reset(new PointCloudType());
    global_map.reset(new PointCloudType());
  }

  CloudPtr current_cloud;

  CloudPtr local_map;

  CloudPtr global_map;

  std::deque<TrajPoint> trajectory;
};

}  // namespace viewer