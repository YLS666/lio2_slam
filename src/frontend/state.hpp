#pragma once

#include "utils/eigen_types.hpp"

struct State {
  Qd q;
  V3d p;
  V3d v;
  double timestamp = 0.0;

  State() {
    q.setIdentity();
    p.setZero();
    v.setZero();
    timestamp = 0.0;
  }
};