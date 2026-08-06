#include "visualization/viewer_render.hpp"

#include <Eigen/Dense>

#include "visualization/viewer_gl_utils.hpp"

namespace viewer {

void DrawGlobalMap(const CloudPtr& cloud) {
  if (!cloud || cloud->empty()) {
    return;
  }

  glPointSize(2.0f);

  glColor4f(0.4f, 0.6f, 1.0f, 0.5f);  // 淡蓝色，透明度 0.5

  glBegin(GL_POINTS);

  for (const auto& pt : cloud->points) {
    auto p = SlamToGL(pt.x, pt.y, pt.z);

    glVertex3f(p.x(), p.y(), p.z());
  }

  glEnd();
}

void DrawLocalMap(const CloudPtr& cloud) {
  if (!cloud || cloud->empty()) {
    return;
  }

  glPointSize(2.0f);

  glColor4f(0.2f, 1.0f, 0.3f, 0.6f);  // 绿色半透明 → 和全局蓝色区分

  glBegin(GL_POINTS);

  for (const auto& pt : cloud->points) {
    auto p = SlamToGL(pt.x, pt.y, pt.z);

    glVertex3f(p.x(), p.y(), p.z());
  }

  glEnd();
}

void DrawCurrentCloud(const CloudPtr& cloud) {
  if (!cloud || cloud->empty()) {
    return;
  }

  glPointSize(4.0f);

  glColor4f(0.9f, 0.4f, 0.1f, 0.5f);  // 橙色，透明度 0.5

  glBegin(GL_POINTS);

  for (const auto& pt : cloud->points) {
    auto p = SlamToGL(pt.x, pt.y, pt.z);
    glVertex3f(p.x(), p.y(), p.z());
  }

  glEnd();
}

void DrawTrajectory(const std::deque<TrajPoint>& trajectory) {
  if (trajectory.size() < 2) {
    return;
  }

  glLineWidth(2.5f);

  glBegin(GL_LINE_STRIP);

  for (size_t i = 0; i < trajectory.size(); ++i) {
    float alpha = 0.3f + 0.7f * static_cast<float>(i) / static_cast<float>(trajectory.size() - 1);

    glColor3f(alpha, 0.0f, 0.0f);

    auto p = SlamToGL(trajectory[i].p.x(), trajectory[i].p.y(), trajectory[i].p.z());

    glVertex3f(p.x(), p.y(), p.z());
  }

  glEnd();

  const auto& last = trajectory.back();

  glPushMatrix();

  auto pos = SlamToGL(last.p.x(), last.p.y(), last.p.z());

  glTranslatef(pos.x(), pos.y(), pos.z());

  Eigen::Quaternionf q_slam = last.q.cast<float>();

  Eigen::Matrix3f R_convert;

  R_convert << 1, 0, 0, 0, 0, 1, 0, -1, 0;

  Eigen::Matrix3f R_gl = R_convert * q_slam.toRotationMatrix();

  Eigen::Quaternionf q_gl(R_gl);

  glMultMatrixf(Eigen::Affine3f(q_gl).matrix().data());

  pangolin::glDrawAxis(0.5f);

  glPopMatrix();
}

}  // namespace viewer