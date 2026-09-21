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

  /** @brief 在渲染线程内释放 Handler3D */
  void Shutdown();

 private:
  pangolin::View* scene_ = nullptr;

  std::unique_ptr<pangolin::Handler3D> handler_;
};

}  // namespace viewer