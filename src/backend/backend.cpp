#include "backend/backend.hpp"
#include <cmath>
#include <iostream>
#include <vector>
#include "backend/keyframe.hpp"
#include "cloud_utils/point_type.hpp"
#include "frontend/state.hpp"
#include "utils/eigen_types.hpp"

Backend::Backend() {}

bool Backend::addKeyFrame(const State& state, const CloudPtr& cloud, const Eigen::Matrix<double, 6, 6>& info_mat) {
  // 第一帧
  if (keyframes_.empty()) {
    KeyFrame kf;
    kf.id = 0;
    kf.timestamp = static_cast<int>(keyframes_.size());
    kf.p = state.p;
    kf.q = state.q;
    kf.cloud = cloud;
    kf.info_mat = info_mat;
    keyframes_.push_back(kf);
    last_keyframe_timestamp_ = 0.0;
    return true;
  }

  // 检查是否满足关键帧条件
  const auto& last = keyframes_.back();
  double time_diff = static_cast<int>(keyframes_.size()) - last.timestamp;  // 简化时间

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
  kf.timestamp = static_cast<int>(keyframes_.size());
  kf.p = state.p;
  kf.q = state.q;
  kf.cloud = cloud;
  kf.info_mat = info_mat;

  // 计算与前一帧的相对位姿 (用于图优化约束)
  kf.relative_q = last.q.inverse() * state.q;
  kf.relative_p = R_last.transpose() * (state.p - last.p);

  keyframes_.push_back(kf);
  last_keyframe_timestamp_ = static_cast<int>(keyframes_.size());

  // 限制滑动窗口大小
  while (static_cast<int>(keyframes_.size()) > (window_size_ + 5)) {
    // 保留第一个固定帧, 移除最老的
    keyframes_.pop_front();
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

  std::vector<V3d> positions(N);
  std::vector<Qd> rotations(N);
  std::vector<int> fixed_ids;

  // 第1帧固定 (提供参考系)
  fixed_ids.push_back(0);

  for (int i = 0; i < N; ++i) {
    positions[i] = keyframes_[start_idx + i].p;
    rotations[i] = keyframes_[start_idx + i].q;
  }

  // 执行高斯牛顿优化
  gaussNewtonPoseGraph(positions, rotations, fixed_ids);

  // 写入回关键帧
  for (int i = 0; i < N; ++i) {
    keyframes_[start_idx + i].p = positions[i];
    keyframes_[start_idx + i].q = rotations[i].normalized();
  }
}

void Backend::gaussNewtonPoseGraph(std::vector<V3d>& positions, std::vector<Qd>& rotations,
                                   const std::vector<int>& fixed_ids) {
  int N = static_cast<int>(positions.size());
  if (N < 2) {
    return;
  }

  constexpr int MAX_ITER = 10;

  for (int iter = 0; iter < MAX_ITER; ++iter) {
    // 构建稀疏系统 (简化实现: 使用稠密矩阵)
    // 系统矩阵维度: 6*(N - fixed) x 6*(N - fixed)
    int fixed_count = static_cast<int>(fixed_ids.size());
    int opt_count = N - fixed_count;

    if (opt_count <= 0) {
      break;
    }

    // 构建映射: 原始索引 -> 优化变量索引
    std::vector<int> var_map(N - 1);
    int var_idx = 0;
    for (int i = 0; i < N; ++i) {
      bool is_fixed = false;
      for (int fid : fixed_ids) {
        if (i == fid) {
          is_fixed = true;
          break;
        }
      }
      if (!is_fixed) {
        var_map[i] = var_idx++;
      }
    }

    int dim = 6 * opt_count;
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(dim, dim);
    Eigen::VectorXd b = Eigen::VectorXd::Zero(dim);

    // 遍历所有连续帧间约束
    for (int i = 0; i < N - 1; ++i) {
      int j = i + 1;
      // 相对位姿: p_ij, q_ij
      M3d R_i = rotations[i].toRotationMatrix();
      M3d R_j = rotations[j].toRotationMatrix();

      // 从初始相对位姿计算 (固定不变)
      V3d p_ij = R_i.transpose() * (positions[j] - positions[i]);

      // 残差: 当前相对位姿与期望值的差
      V3d r_p = positions[j] - positions[i] - R_i * p_ij;
      M3d R_rel = rotations[i].inverse().toRotationMatrix() * R_j;
      V3d r_theta;

      // @TODO: g2o优化
    }

    break;
  }
}

void Backend::globalOptimize(const std::vector<LoopPair>& loop_pairs) {
  if (keyframes_.empty()) {
    return;
  }

  int N = static_cast<int>(keyframes_.size());

  // 提取所有位姿
  std::vector<V3d> positions(N);
  std::vector<Qd> rotations(N);

  for (int i = 0; i < N; ++i) {
    positions[i] = keyframes_[i].p;
    rotations[i] = keyframes_[i].q;
  }

  // 第1帧固定
  std::vector<int> fixed_ids = {0};

  // @TODO : 执行高斯牛顿 (包括回环约束)
  // 直接更新关键帧位姿
  for (int i = 0; i < N; i++) {
    keyframes_[i].p = positions[i];
    keyframes_[i].q = rotations[i].normalized();
  }

  std::cout << "全局优化完成，关键帧数：" << N << ",回环约束数：" << loop_pairs.size() << std::endl;
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
