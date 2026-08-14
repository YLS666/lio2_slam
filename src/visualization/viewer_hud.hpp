#pragma once

#include <cstddef>

namespace viewer {

/**
 * @brief HUD 信息
 */
struct HudInfo {
  float velocity = 0.0f;

  float gyroscope = 0.0f;

  float acceleration = 0.0f;

  size_t trajectory_size = 0;

  size_t global_map_size = 0;
};

/**
 * @brief 绘制 HUD
 *
 * 左上角文字信息
 */
void DrawHUD(const HudInfo& info);

}  // namespace viewer