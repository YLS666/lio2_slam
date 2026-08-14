#include "visualization/viewer_layout.hpp"

namespace viewer {

void ViewerLayout::Init(pangolin::OpenGlRenderState& camera) {
  //---------------- 左侧 3D ----------------//

  scene_ = &pangolin::CreateDisplay();

  scene_->SetBounds(0.0, 1.0, 0.0, 0.75);

  scene_->SetHandler(new pangolin::Handler3D(camera));

  //---------------- 右侧 Plot ----------------//

  pangolin::Display("plot_vel").SetBounds(0.5, 0.7, 0.75, 1.0);

  pangolin::Display("plot_gyr").SetBounds(0.3, 0.5, 0.75, 1.0);

  pangolin::Display("plot_acc").SetBounds(0.1, 0.3, 0.75, 1.0);
}
}  // namespace viewer