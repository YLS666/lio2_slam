#pragma once

#include "utils/eigen_types.hpp"

struct State {
  Qd q;
  V3d p;
  V3d v;

  // 18-DOF IESKF 状态量
  V3d bg = V3d::Zero();             // 陀螺仪 bias (rad/s)
  V3d ba = V3d::Zero();             // 加速度计 bias (m/s²)
  V3d g = V3d(0.0, 0.0, -9.80665);  // 重力向量 (m/s²)

  double timestamp = 0.0;

  State() {
    q.setIdentity();
    p.setZero();
    v.setZero();
    bg.setZero();
    ba.setZero();
    g << 0.0, 0.0, -9.80665;
    timestamp = 0.0;
  }
};
