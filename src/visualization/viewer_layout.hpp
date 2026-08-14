#pragma once

#include <pangolin/pangolin.h>

namespace viewer {

/**
 * @brief Viewer 界面布局
 *
 * +---------------------------------------------------------+
 * |                                                         |
 * |                     3D Viewer                   | Vel   |
 * |                                                 |-------|
 * |                                                 | Gyr   |
 * |                                                 |-------|
 * |                                                 | Acc   |
 * +---------------------------------------------------------+
 */
class ViewerLayout {
 public:
  ViewerLayout() = default;

  ~ViewerLayout() = default;

  /**
   * @brief 创建所有 Display
   */
  void Init(pangolin::OpenGlRenderState& camera);

  /**
   * @brief 返回 3D View
   */
  pangolin::View& Scene() { return *scene_; }

 private:
  pangolin::View* scene_ = nullptr;
};

}  // namespace viewer