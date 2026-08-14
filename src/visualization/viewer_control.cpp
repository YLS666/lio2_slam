#include "viewer_control.hpp"

namespace viewer {

ViewerControl::ViewerControl()
    : reset_view_("ui.Reset View", false, false),
      clear_traj_("ui.Clear Traj", false, false),
      follow_robot_("ui.Follow Robot", false, true),
      show_local_map_("ui.Show Local Map", true, true),
      show_global_map_("ui.Show Global Map", true, true),
      show_scan_("ui.Show Scan", true, true) {}

void ViewerControl::Update() {}

bool ViewerControl::ResetView() {
  if (reset_view_) {
    reset_view_ = false;
    return true;
  }

  return false;
}

bool ViewerControl::ClearTrajectory() {
  if (clear_traj_) {
    clear_traj_ = false;
    return true;
  }

  return false;
}

bool ViewerControl::FollowRobot() const { return follow_robot_; }

void ViewerControl::DisableFollow() { follow_robot_ = false; }

bool ViewerControl::ShowLocalMap() const { return show_local_map_; }

bool ViewerControl::ShowGlobalMap() const { return show_global_map_; }

bool ViewerControl::ShowScan() const { return show_scan_; }

}  // namespace viewer