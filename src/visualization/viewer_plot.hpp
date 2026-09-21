#pragma once

#include <memory>

#include <pangolin/pangolin.h>
#include <pangolin/plot/datalog.h>
#include <pangolin/plot/plotter.h>

namespace viewer {

/**
 * @brief 运动曲线显示
 *
 * Time
 * Velocity
 * Gyroscope
 * Acceleration
 */
class ViewerPlot {
 public:
  ViewerPlot() = default;
  ~ViewerPlot() = default;

  /**
   * @brief 创建三个 Plotter
   */
  void Init();

  /**
   * @brief 渲染各 plotter
   */
  void RenderVel();
  void RenderGyr();
  void RenderAcc();

  /**
   * @brief 添加一帧数据
   */
  void Push(float vel, float gyr, float acc);

  /** @brief 在渲染线程内释放 Plotter (GL 上下文仍有效时调用) */
  void Shutdown();

 private:
  static constexpr float kHistory = 60.0f;
  static constexpr float kVelMax = 5.0f;
  static constexpr float kGyrMax = 2.0f;
  static constexpr float kAccMax = 20.0f;

  float time_ = 0.0f;

  std::unique_ptr<pangolin::DataLog> log_;
  std::unique_ptr<pangolin::Plotter> vel_plot_;
  std::unique_ptr<pangolin::Plotter> gyr_plot_;
  std::unique_ptr<pangolin::Plotter> acc_plot_;
};

}  // namespace viewer