#include "backend/backend.hpp"
#include <glog/logging.h>
#include <cmath>
#include <iostream>
#include "backend/keyframe.hpp"
#include "cloud_utils/point_type.hpp"
#include "frontend/state.hpp"
#include "utils/eigen_types.hpp"

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
  }
  // else if (time_diff > keyframe_min_interval_ && dist > keyframe_distance_ * 0.5) {
  //   // 时间间隔足够并且有一定的位移
  //   create_kf = true;
  // }

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
