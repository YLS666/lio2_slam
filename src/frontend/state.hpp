#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

struct State {
  Eigen::Quaterniond q;

  Eigen::Vector3d p;

  Eigen::Vector3d v;

  State() {
    q.setIdentity();
    p.setZero();
    v.setZero();
  }
};