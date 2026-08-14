#include "backend/backend.hpp"
#include <glog/logging.h>
#include <cmath>
#include <iostream>
#include "backend/keyframe.hpp"
#include "cloud_utils/point_type.hpp"
#include "frontend/state.hpp"
#include "utils/eigen_types.hpp"
#include "utils/g2o_types.hpp"
#include "utils/math_types.hpp"

Backend::Backend() {}

bool Backend::addKeyFrame(const State& state, const CloudPtr& cloud, const Eigen::Matrix<double, 6, 6>& info_mat,
                          int ndt_effective, float ndt_rot_correction) {
  // 第一帧
  if (keyframes_.empty()) {
    KeyFrame kf;
    kf.id = 0;
    kf.timestamp = state.timestamp;
    kf.p = state.p;
    kf.q = state.q;
    kf.cloud = cloud;
    kf.info_mat = info_mat;
    kf.ndt_effective = ndt_effective;
    kf.ndt_rot_correction = ndt_rot_correction;
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
  kf.ndt_effective = ndt_effective;
  kf.ndt_rot_correction = ndt_rot_correction;

  // 计算与前一帧的相对位姿 (用于图优化约束)
  kf.relative_q = last.q.inverse() * state.q;
  kf.relative_p = R_last.transpose() * (state.p - last.p);

  keyframes_.push_back(kf);
  last_keyframe_timestamp_ = kf.timestamp;

  // 分级内存管理
  // 第一级：释放已合并的老 KF 的点云（保留位姿骨架用于回环和地图重建）
  int cloud_count = 0;
  for (const auto& k : keyframes_) {
    if (k.cloud && !k.cloud->empty()) {
      cloud_count++;
    }
  }
  if (cloud_count > max_kf_clouds_) {
    int freed = 0;
    for (auto& k : keyframes_) {
      if (!k.merged) {
        continue;
      }  // 未合并的不释放
      if (!k.cloud || k.cloud->empty()) {
        continue;
      }
      k.cloud.reset();  // 释放点云，保留位姿
      freed++;
      if (cloud_count - freed <= max_kf_clouds_) {
        break;
      }
    }
    LOG_IF(INFO, freed > 0) << "[KFMgr] 释放 " << freed << " 个已合并KF的点云"
                            << ", 剩余云: " << (cloud_count - freed);
  }

  // 第二级：KF 总数超限时淘汰最老的
  if (static_cast<int>(keyframes_.size()) > max_keyframes_) {
    int removed = 0;
    while (static_cast<int>(keyframes_.size()) > max_keyframes_) {
      keyframes_.pop_front();
      removed++;
    }
    for (size_t i = 0; i < keyframes_.size(); ++i) {
      keyframes_[i].id = static_cast<int>(i);
    }
    LOG(INFO) << "[KFMgr] 淘汰 " << removed << " 个最老KF, 剩余: " << keyframes_.size();
  }

  return true;
}

bool Backend::slideWindowOptimize() {
  if (keyframes_.size() < 3) {
    return false;
  }

  int start_idx = std::max(0, static_cast<int>(keyframes_.size()) - window_size_);
  int N = static_cast<int>(keyframes_.size()) - start_idx;

  if (N < 2) {
    return false;
  }

  // 自洽性检查：逐条边验证「存储的相对测量值」与「从 KF 位姿反算的相对变换」是否一致
  // 如果一致，说明无需优化（初始解已是最优）；如果不一致，说明此前有 KF 因回滚等原因产生了不一致，但也用位姿本身为准
  bool all_consistent = true;
  for (int i = 0; i < N - 1; ++i) {
    const auto& kf_cur = keyframes_[start_idx + i];
    const auto& kf_next = keyframes_[start_idx + i + 1];

    // 从 KF 位姿反算相对变换
    SE3 T_cur(kf_cur.q, kf_cur.p);
    SE3 T_next(kf_next.q, kf_next.p);
    SE3 T_rel_computed = T_cur.inverse() * T_next;
    V3d rel_p_computed = T_rel_computed.translation();
    Qd rel_q_computed = T_rel_computed.unit_quaternion();

    // 与存储值比较
    double dp = (rel_p_computed - kf_next.relative_p).norm();
    double dtheta = 2.0 * std::acos(std::min(1.0, std::abs((kf_next.relative_q.inverse() * rel_q_computed).w())));

    if (dp > 0.01 || dtheta > 0.01) {
      // 不一致：用位姿本身重算 relative，维持后续一致性
      LOG(WARNING) << "[SlideWindow] 边 " << kf_cur.id << "→" << kf_next.id << " 不一致: rel_p 差值=" << dp
                   << "m, rel_q 差值=" << dtheta << "rad，以位姿为准重算";
      const_cast<KeyFrame&>(kf_next).relative_p = rel_p_computed;
      const_cast<KeyFrame&>(kf_next).relative_q = rel_q_computed;
      all_consistent = false;
    }
  }

  VLOG(1) << "[SlideWindow] N=" << N << " start_idx=" << start_idx << (all_consistent ? " 全部自洽" : " 已修复不一致");
  return true;
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
    double det = info.determinant();
    if (det < 1e-12 || det > 1e18 || float_check::isnan(det) || float_check::isinf(det)) {
      LOG(WARNING) << "关键帧 " << keyframes_[i + 1].id << " 信息矩阵异常(det=" << det << "), 使用单位矩阵";
      info = Eigen::Matrix<double, 6, 6>::Identity();
    } else {
      Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6>> eig_info(info);
      double max_ev = eig_info.eigenvalues().maxCoeff();
      constexpr double MAX_INFO_EIGEN = 1e6;
      if (max_ev > MAX_INFO_EIGEN) {
        info *= MAX_INFO_EIGEN / max_ev;
        VLOG(1) << "关键帧 " << keyframes_[i + 1].id << " 信息矩阵特征值过大(" << max_ev << "), 已缩放";
      }
    }
    edge->setInformation(info);
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
