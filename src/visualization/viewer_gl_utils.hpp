#pragma once

#include <algorithm>

#include <Eigen/Core>

namespace viewer {

/**
 * @brief SLAM 坐标系 -> OpenGL 坐标系
 *
 * SLAM:
 *   x : 前
 *   y : 左
 *   z : 上
 *
 * OpenGL:
 *   x : 右
 *   y : 上
 *   z : 朝屏幕外
 *
 * 转换关系：
 *   x_gl = x
 *   y_gl = z
 *   z_gl = -y
 */
inline Eigen::Vector3f SlamToGL(float x, float y, float z) { return Eigen::Vector3f(x, z, -y); }

/**
 * @brief 高度颜色映射
 *
 * 蓝 → 青 → 绿 → 黄 → 红
 */
inline Eigen::Vector3f HeightToColor(float z, float z_min, float z_max) {
  float t = (z - z_min) / (z_max - z_min);

  t = std::max(0.0f, std::min(1.0f, t));

  float r;
  float g;
  float b;

  if (t < 0.25f) {
    float s = t / 0.25f;

    r = 0.0f;
    g = s;
    b = 1.0f;
  } else if (t < 0.5f) {
    float s = (t - 0.25f) / 0.25f;

    r = 0.0f;
    g = 1.0f;
    b = 1.0f - s;
  } else if (t < 0.75f) {
    float s = (t - 0.5f) / 0.25f;

    r = s;
    g = 1.0f;
    b = 0.0f;
  } else {
    float s = (t - 0.75f) / 0.25f;

    r = 1.0f;
    g = 1.0f - s;
    b = 0.0f;
  }

  return Eigen::Vector3f(r, g, b);
}

}  // namespace viewer