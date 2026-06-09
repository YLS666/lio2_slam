#pragma once

#include "utils/eigen_types.hpp"

struct State {
  Qd q;
  V3d p;
  V3d v;

  State() {
    q.setIdentity();
    p.setZero();
    v.setZero();
  }
};