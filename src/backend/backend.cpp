#include "backend/backend.hpp"
#include <glog/logging.h>
#include <cmath>
#include <iostream>
#include "backend/keyframe.hpp"
#include "cloud_utils/point_type.hpp"
#include "frontend/state.hpp"
#include "utils/eigen_types.hpp"
#include "utils/g2o_types.hpp"

Backend::Backend() {}

bool Backend::addKeyFrame(const State& state, const CloudPtr& cloud, const Eigen::Matrix<double, 6, 6>& info_mat) {
  // 第一帧
  if (keyframes_.empty()) {
    KeyFrame kf;
    kf.id = 0;
    kf.timestamp = state.timestamp;
    kf.p = state.p;
    kf.q = state.q;
    kf.cloud = cloud;
    kf.info_mat = info_mat;
    kf.relative_p.setZero();
    kf.relative_q.setIdentity();
    keyframes_.push_back(kf);
    last_keyframe_timestamp_ = state.timestamp;
    return true;
  }

  // 检查是否满足关键帧条件
  const auto& last = keyframes_.back();
  double time_diff = state.timestamp - last.timestamp;  // 简化时间

  // 计算与上一关键帧的相对变换
  M3d R_last = last.q.toRotationMatrix();
  M3d R_cur = state.q.toRotationMatrix();

  // 相对旋转角度
  M3d R_rel = R_last.transpose() * R_cur;
  double angle = std::acos(std::min(1.0, std::max(-1.0, (R_rel.trace() - 1.0) / 2.0)));

  // 相对平移距离
  double dist = (state.p - last.p).norm();

  // 判断是否创建关键帧
  bool create_kf = false;

  if (dist > keyframe_distance_) {
    create_kf = true;
  } else if (angle > keyframe_angle_) {
    create_kf = true;
  } else if (time_diff > keyframe_min_interval_ && dist > keyframe_distance_ * 0.5) {
    // 时间间隔足够并且有一定的位移
    create_kf = true;
  }

  if (!create_kf) {
    return false;
  }

  // 创建新关键帧
  KeyFrame kf;
  kf.id = static_cast<int>(keyframes_.size());
  kf.timestamp = state.timestamp;
  kf.p = state.p;
  kf.q = state.q;
  kf.cloud = cloud;
  kf.info_mat = info_mat;

  // 计算与前一帧的相对位姿 (用于图优化约束)
  kf.relative_q = last.q.inverse() * state.q;
  kf.relative_p = R_last.transpose() * (state.p - last.p);

  keyframes_.push_back(kf);
  last_keyframe_timestamp_ = kf.timestamp;

  // 保留所有关键帧用于回环, 仅限制最大数量
  static constexpr int MAX_TOTAL_KFS = 200;
  if (static_cast<int>(keyframes_.size()) > MAX_TOTAL_KFS) {
    keyframes_.pop_front();
    // 重新编号
    for (size_t i = 0; i < keyframes_.size(); ++i) {
      keyframes_[i].id = static_cast<int>(i);
    }
  }

  return true;
}

void Backend::slideWindowOptimize() {
  if (keyframes_.size() < 3) {
    return;
  }

  // 提取滑动窗口内的关键帧 (最近 window_size_ 个)
  int start_idx = std::max(0, static_cast<int>(keyframes_.size()) - window_size_);
  int N = static_cast<int>(keyframes_.size()) - start_idx;

  if (N < 2) {
    return;
  }

  std::unique_ptr<LinearSolver> linearSolver = std::make_unique<LinearSolver>();
  linearSolver->setBlockOrdering(false);
  std::unique_ptr<BlockSolver> blockSolver = std::make_unique<BlockSolver>(std::move(linearSolver));
  auto* algorithm = new g2o::OptimizationAlgorithmLevenberg(std::move(blockSolver));

  g2o::SparseOptimizer optimizer;
  optimizer.setAlgorithm(algorithm);
  optimizer.setVerbose(false);

  // 添加顶点
  std::vector<g2o::VertexSE3*> vertices(N);

  for (int i = 0; i < N; ++i) {
    auto* v = new g2o::VertexSE3();
    v->setId(i);

    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    T.rotate(keyframes_[start_idx + i].q.toRotationMatrix());
    T.pretranslate(keyframes_[start_idx + i].p);
    v->setEstimate(T);

    if (i == 0) {
      v->setFixed(true);
    }
    optimizer.addVertex(v);
    vertices[i] = v;
  }

  // 添加帧间约束边（i->i+1)
  for (int i = 0; i < N - 1; ++i) {
    const auto& kf_next = keyframes_[start_idx + i + 1];
    auto* edge = new g2o::EdgeSE3();
    edge->setVertex(0, vertices[i]);
    edge->setVertex(1, vertices[i + 1]);

    Eigen::Isometry3d T_rel = Eigen::Isometry3d::Identity();
    T_rel.rotate(kf_next.relative_q.toRotationMatrix());
    T_rel.pretranslate(kf_next.relative_p);
    edge->setMeasurement(T_rel);

    Eigen::Matrix<double, 6, 6> info = kf_next.info_mat;
    double det = info.determinant();
    if (det < 1e-12 || det > 1e18 || std::isnan(det) || std::isinf(det)) {
      LOG(WARNING) << "关键帧 " << kf_next.id << " 信息矩阵异常(det=" << det << "), 使用单位矩阵";
      info = Eigen::Matrix<double, 6, 6>::Identity();
    }
    edge->setInformation(info);
    auto* rk = new g2o::RobustKernelHuber();
    rk->setDelta(0.1);
    edge->setRobustKernel(rk);
    optimizer.addEdge(edge);
  }

  // 优化
  optimizer.initializeOptimization();
  optimizer.optimize(20);

  // 写回
  for (int i = 0; i < N; ++i) {
    Eigen::Isometry3d T = vertices[i]->estimate();
    keyframes_[start_idx + i].q = Eigen::Quaterniond(T.rotation()).normalized();
    keyframes_[start_idx + i].p = T.translation();
  }

  for (int i = 0; i < N - 1; ++i) {
    const auto& kf_cur = keyframes_[start_idx + i];
    const auto& kf_next = keyframes_[start_idx + i + 1];
    // 重新计算相对位姿
    keyframes_[start_idx + i + 1].relative_q = kf_cur.q.inverse() * kf_next.q;
    M3d R_cur = kf_cur.q.toRotationMatrix();
    keyframes_[start_idx + i + 1].relative_p = R_cur.transpose() * (kf_next.p - kf_cur.p);
  }

  LOG(INFO) << "[SlideWindow] N=" << N << " start_idx=" << start_idx;
  for (int i = 0; i < N; ++i) {
    LOG(INFO) << "[SlideWindow] kf_id=" << keyframes_[start_idx + i].id
              << " p=" << keyframes_[start_idx + i].p.transpose();
  }

  LOG(INFO) << "滑动窗口优化完成, 帧数: " << N;
}

void Backend::globalOptimize(const std::vector<LoopPair>& loop_pairs) {
  if (keyframes_.empty()) {
    return;
  }

  int N = static_cast<int>(keyframes_.size());

  auto linearSolver = std::make_unique<LinearSolver>();
  linearSolver->setBlockOrdering(false);
  auto blockSolver = std::make_unique<BlockSolver>(std::move(linearSolver));
  auto* algorithm = new g2o::OptimizationAlgorithmLevenberg(std::move(blockSolver));

  g2o::SparseOptimizer optimizer;
  optimizer.setAlgorithm(algorithm);
  optimizer.setVerbose(true);

  // 顶点
  std::vector<g2o::VertexSE3*> vertices(N);
  for (int i = 0; i < N; ++i) {
    auto* v = new g2o::VertexSE3();
    v->setId(i);

    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    T.rotate(keyframes_[i].q.toRotationMatrix());
    T.pretranslate(keyframes_[i].p);
    v->setEstimate(T);

    if (i == 0) {
      v->setFixed(true);
    }
    optimizer.addVertex(v);
    vertices[i] = v;
  }

  // 帧间约束
  for (int i = 0; i < N - 1; ++i) {
    auto* edge = new g2o::EdgeSE3();
    edge->setVertex(0, vertices[i]);
    edge->setVertex(1, vertices[i + 1]);

    Eigen::Isometry3d T_rel = Eigen::Isometry3d::Identity();
    T_rel.rotate(keyframes_[i + 1].relative_q.toRotationMatrix());
    T_rel.pretranslate(keyframes_[i + 1].relative_p);
    edge->setMeasurement(T_rel);

    Eigen::Matrix<double, 6, 6> info = keyframes_[i + 1].info_mat;
    if (info.determinant() < 1e-12) {
      info = Eigen::Matrix<double, 6, 6>::Identity();
    }
    edge->setInformation(info);
    auto* rk = new g2o::RobustKernelHuber();
    rk->setDelta(0.1);
    edge->setRobustKernel(rk);
    optimizer.addEdge(edge);
  }

  // 回环约束
  for (const auto& lp : loop_pairs) {
    if (lp.id_a < 0 || lp.id_a >= N || lp.id_b < 0 || lp.id_b >= N) {
      continue;
    }

    auto* edge = new g2o::EdgeSE3();
    edge->setVertex(0, vertices[lp.id_a]);
    edge->setVertex(1, vertices[lp.id_b]);

    Eigen::Isometry3d T_rel = Eigen::Isometry3d::Identity();
    T_rel.rotate(lp.rel_q.toRotationMatrix());
    T_rel.pretranslate(lp.rel_p);
    edge->setMeasurement(T_rel);

    Eigen::Matrix<double, 6, 6> loop_info = Eigen::Matrix<double, 6, 6>::Identity() * lp.info_weight * 10.0;
    edge->setInformation(loop_info);
    optimizer.addEdge(edge);
  }

  // 优化
  optimizer.initializeOptimization();
  optimizer.optimize(50);

  // 写回
  for (int i = 0; i < N; ++i) {
    Eigen::Isometry3d T = vertices[i]->estimate();
    keyframes_[i].q = Eigen::Quaterniond(T.rotation()).normalized();
    keyframes_[i].p = T.translation();
  }

  // 重算所有帧间相对位姿
  for (int i = 0; i < N - 1; ++i) {
    keyframes_[i + 1].relative_q = keyframes_[i].q.inverse() * keyframes_[i + 1].q;
    M3d R_i = keyframes_[i].q.toRotationMatrix();
    keyframes_[i + 1].relative_p = R_i.transpose() * (keyframes_[i + 1].p - keyframes_[i].p);
  }

  LOG(INFO) << "全局优化完成, 关键帧数: " << N << ", 回环约束数: " << loop_pairs.size();
}

bool Backend::getPose(int id, V3d& p, Qd& q) const {
  for (const auto& kf : keyframes_) {
    if (kf.id == id) {
      p = kf.p;
      q = kf.q;
      return true;
    }
  }
  return false;
}

void Backend::markKeyframesMerged(const std::vector<int>& ids) {
  for (int id : ids) {
    for (auto& kf : keyframes_) {
      if (kf.id == id) {
        kf.merged = true;
        break;
      }
    }
  }
}

bool Backend::getWindowFirstPose(V3d& p, Qd& q) const {
  if (keyframes_.empty()) {
    return false;
  }
  int start_idx = std::max(0, static_cast<int>(keyframes_.size()) - window_size_);
  p = keyframes_[start_idx].p;
  q = keyframes_[start_idx].q;
  return true;
}
