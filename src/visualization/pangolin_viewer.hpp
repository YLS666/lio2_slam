#pragma once

#include <pangolin/pangolin.h>
#include <Eigen/Dense>
#include <atomic>
#include <memory>
#include <mutex>
#include <thread>
#include "cloud_utils/point_type.hpp"
#include "utils/eigen_types.hpp"
#include "visualization/viewer_control.hpp"
#include "visualization/viewer_layout.hpp"
#include "visualization/viewer_plot.hpp"
#include "visualization/viewer_types.hpp"

using namespace viewer;

class PangolinViewer {
 public:
  PangolinViewer();

  ~PangolinViewer();

  void start();

  void stop();

  bool isRunning() const { return running_.load(); }

  // 数据更新接口
  void updateCurrentCloud(const CloudPtr& cloud, const Eigen::Matrix4f& T);

  void updateLocalMap(const CloudPtr& cloud);

  void setLocalMapSource(std::function<CloudPtr()> getter);

  void appendGlobalMap(const CloudPtr& cloud, const Eigen::Matrix4f& T);

  void clearGlobalMap();

  void ResetCamera();

  void FollowRobot();

  void updatePose(const V3d& p, const Qd& q, double timestamp);

  void updateMotionInfo(const V3d& vel, const V3d& gyr_raw, const V3d& acc_raw);

 private:
  // 主循环
  void run();

  // 工具
  static Eigen::Vector3f heightToColor(float z, float z_min, float z_max);

 private:
  // Thread
  std::unique_ptr<std::thread> thread_;

  std::atomic<bool> running_{false};

  std::atomic<bool> should_exit_{false};

  std::atomic<bool> initialized_{false};

  // Data
  std::mutex data_mutex_;

  CloudPtr current_cloud_;

  CloudPtr raw_current_cloud_;  // 只存指针, 不拷贝

  Eigen::Matrix4f current_T_ = Eigen::Matrix4f::Identity();

  CloudPtr local_map_;

  CloudPtr global_map_;

  std::function<CloudPtr()> local_map_getter_;  // 由 Frontend 注入, 返回降采样后的局部地图

  std::deque<std::pair<CloudPtr, Eigen::Matrix4f>> pending_global_clouds_;  // 主线程投递的待处理全局点云

  int local_map_refresh_counter_ = 0;

  std::deque<viewer::TrajPoint> trajectory_;

  float vel_magnitude_ = 0.0f;

  float gyr_magnitude_ = 0.0f;

  float acc_magnitude_ = 0.0f;

  bool current_cloud_dirty_ = false;

  bool local_map_dirty_ = false;

  bool global_map_dirty_ = false;

  // Render Modules
  std::unique_ptr<pangolin::OpenGlRenderState> camera_;

  pangolin::OpenGlMatrix default_view_;

  viewer::ViewerLayout layout_;

  viewer::ViewerPlot plot_;

  // Color
  float color_min_z_ = -2.0f;

  float color_max_z_ = 5.0f;

  // Control
  std::shared_ptr<ViewerControl> control_;

  V3d current_position_{0, 0, 0};
  Qd current_orientation_{1, 0, 0, 0};
};