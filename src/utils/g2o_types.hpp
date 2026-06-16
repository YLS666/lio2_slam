#pragma once

#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>
#include <g2o/types/slam3d/edge_se3.h>
#include <g2o/types/slam3d/vertex_se3.h>

using BlockSolver = g2o::BlockSolver<g2o::BlockSolverTraits<6, 3>>;
using LinearSolver = g2o::LinearSolverEigen<BlockSolver::PoseMatrixType>;