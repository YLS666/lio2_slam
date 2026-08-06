#include "visualization/viewer_hud.hpp"

#include <pangolin/display/default_font.h>
#include <pangolin/pangolin.h>

namespace viewer {

void DrawHUD(const HudInfo& info) {
  pangolin::GlFont& font = pangolin::default_font();

  constexpr float kStartX = 1500.0f;
  constexpr float kStartY = 20.0f;
  constexpr float kLineHeight = 18.0f;

  float x = kStartX;
  float y = kStartY;

  glDisable(GL_DEPTH_TEST);

  glColor3f(0.0f, 0.0f, 0.0f);

  font.Text("=== LIO2-SLAM ===").DrawWindow(x, y);

  y += kLineHeight * 2.0f;

  font.Text("Velocity : %.3f m/s", info.velocity).DrawWindow(x, y);

  y += kLineHeight;

  font.Text("Gyroscope : %.3f rad/s", info.gyroscope).DrawWindow(x, y);

  y += kLineHeight;

  font.Text("Acceleration : %.3f m/s^2", info.acceleration).DrawWindow(x, y);

  y += kLineHeight * 2.0f;

  font.Text("Trajectory : %zu", info.trajectory_size).DrawWindow(x, y);

  y += kLineHeight;

  font.Text("Global Map : %zu points", info.global_map_size).DrawWindow(x, y);

  glEnable(GL_DEPTH_TEST);
}

}  // namespace viewer