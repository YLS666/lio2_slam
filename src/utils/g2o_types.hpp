#pragma once

#include <g2o/core/base_binary_edge.h>
#include <g2o/core/base_vertex.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>
#include <g2o/types/slam3d/edge_se3.h>
#include <g2o/types/slam3d/vertex_se3.h>
#include "utils/eigen_types.hpp"

using BlockSolver = g2o::BlockSolver<g2o::BlockSolverTraits<6, 3>>;
using LinearSolver = g2o::LinearSolverEigen<BlockSolver::PoseMatrixType>;

namespace g2o_optimizer {
/**
 * 旋转在前的SO3+t类型pose，6自由度，存储时伪装为g2o::VertexSE3，供g2o_viewer查看
 */
class VertexPose : public g2o::BaseVertex<6, SE3> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  explicit VertexPose() = default;

  bool read(std::istream& /*is*/) override { return true; }

  bool write(std::ostream& /*os*/) const override { return true; }

  void setToOriginImpl() override {
    _estimate = SE3();  // 单位变换，旋转单位四元数、平移0
  }

  void oplusImpl(const double* update_) override {
    _estimate.so3() = _estimate.so3() * SO3::exp(Eigen::Map<const V3d>(&update_[0]));  // 旋转部分
    _estimate.translation() += Eigen::Map<const V3d>(&update_[3]);                     // 平移部分
    updateCache();
  }
};

/**
 * 6 自由度相对运动
 * 误差的平移在前，角度在后
 * 观测：T12
 */
class EdgeRelativeMotion : public g2o::BaseBinaryEdge<6, SE3, VertexPose, VertexPose> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  EdgeRelativeMotion() = default;
  EdgeRelativeMotion(VertexPose* v1, VertexPose* v2, const SE3& obs) {
    setVertex(0, v1);
    setVertex(1, v2);
    setMeasurement(obs);
  }

  EdgeRelativeMotion(VertexPose* v1, VertexPose* v2, const SE3& obs, const M6d& info) {
    setVertex(0, v1);
    setVertex(1, v2);
    setMeasurement(obs);
    setInformation(info);
  }

  void computeError() override {
    VertexPose* v1 = static_cast<VertexPose*>(_vertices[0]);
    VertexPose* v2 = static_cast<VertexPose*>(_vertices[1]);
    _error = (_measurement.inverse() * v1->estimate().inverse() * v2->estimate()).log();
  };

  bool read(std::istream& /*is*/) override { return true; }

  bool write(std::ostream& /*os*/) const override { return true; }
};

}  // namespace g2o_optimizer