#pragma once

#include <pangolin/pangolin.h>

namespace viewer {

class ViewerControl {
 public:
  ViewerControl();

  void Update();

 public:
  // 一次性事件
  bool ResetView();
  bool ClearTrajectory();

  // 状态
  bool FollowRobot() const;
  bool ShowLocalMap() const;
  bool ShowGlobalMap() const;
  bool ShowScan() const;

  void DisableFollow();

 private:
  pangolin::Var<bool> reset_view_;
  pangolin::Var<bool> clear_traj_;

  pangolin::Var<bool> follow_robot_;
  pangolin::Var<bool> show_local_map_;
  pangolin::Var<bool> show_global_map_;
  pangolin::Var<bool> show_scan_;
};

}  // namespace viewer