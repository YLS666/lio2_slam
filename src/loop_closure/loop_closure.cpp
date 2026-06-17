#include "loop_closure/loop_closure.hpp"
#include <glog/logging.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/icp.h>
#include <algorithm>
#include <cmath>
#include <iostream>
#include "backend/backend.hpp"
#include "backend/keyframe.hpp"
#include "cloud_utils/point_type.hpp"
#include "loop_closure/scan_context.hpp"

void LoopClosure::addKeyframe(const KeyFrame& kf) {
  if (!kf.cloud || kf.cloud->empty()) {
    return;
  }

  // 生成描述子
  ScanContext sc;
  sc.make(kf.cloud);

  descriptors_.push_back(sc.getDescriptor());
  frame_ids_.push_back(kf.id);
}

bool LoopClosure::detect(const KeyFrame& current_kf, std::vector<LoopPair>& loop_pairs) {
  if (descriptors_.empty() || current_kf.id < min_interval_) {
    return false;
  }

  // 生成当前关键帧的描述子
  ScanContext current_sc;
  current_sc.make(current_kf.cloud);
  const auto& current_desc = current_sc.getDescriptor();

  // 搜索历史描述子, 找最近邻
  std::vector<std::pair<double, int>> scores;

  for (size_t i = 0; i < descriptors_.size(); ++i) {
    int frame_id = frame_ids_[i];
    // 跳过太近的帧
    if (current_kf.id - frame_id < min_interval_) {
      continue;
    }
    // 跳过已回环的帧
    if (frame_id == last_loop_frame_id_) {
      continue;
    }

    double dist = ScanContext::distance(current_desc, descriptors_[i]);
    scores.push_back({dist, frame_id});
  }

  if (scores.empty()) {
    return false;
  }

  // 按距离排序 (取最小的 top-N)
  std::sort(scores.begin(), scores.end());

  bool loop_found = false;
  for (int c = 0; c < std::min(candidate_num_, static_cast<int>(scores.size())); ++c) {
    if (scores[c].first > min_score_) {
      continue;
    }

    int candidate_id = scores[c].second;

    const KeyFrame* candidate_kf = nullptr;
    if (keyframes_ptr_) {
      for (const auto& kf : *keyframes_ptr_) {
        if (kf.id == candidate_id) {
          candidate_kf = &kf;
          break;
        }
      }
    }

    // 没找到候选帧，跳过
    if (!candidate_kf) {
      LOG(WARNING) << "回环检测: 未找到候选关键帧 id=" << candidate_id;
      continue;
    }

    // 获取候选关键帧
    V3d rel_p;
    Qd rel_q;

    bool verified = geometricVerification(*candidate_kf, current_kf, rel_p, rel_q);

    if (verified) {
      LoopPair pair;
      pair.id_a = candidate_id;
      pair.id_b = current_kf.id;
      pair.rel_p = rel_p;
      pair.rel_q = rel_q;
      pair.info_weight = 1.0;

      loop_pairs.push_back(pair);
      last_loop_frame_id_ = candidate_id;
      loop_found = true;

      LOG(INFO) << "回环检测成功，帧" << candidate_id << "<->帧" << current_kf.id << "，距离：" << scores[c].first;
    }
  }

  return loop_found;
}

bool LoopClosure::geometricVerification(const KeyFrame& kf1, const KeyFrame& kf2, V3d& rel_p, Qd& rel_q) {
  if (!kf1.cloud || kf1.cloud->empty() || !kf2.cloud || kf2.cloud->empty()) {
    return false;
  }

  // 使用ICP 进行几何验证
  pcl::IterativeClosestPoint<PointType, PointType> icp;
  icp.setMaxCorrespondenceDistance(2.0);  // 2米内搜索
  icp.setMaximumIterations(50);
  icp.setTransformationEpsilon(1e-6);
  icp.setEuclideanFitnessEpsilon(0.01);

  // 源：kf2(当前帧) ， 目标：kf1(历史帧)
  icp.setInputSource(kf2.cloud);
  icp.setInputTarget(kf1.cloud);

  PointCloudType aligned;

  icp.align(aligned);

  if (!icp.hasConverged() || icp.getFitnessScore() > icp_score_thresh_) {
    return false;
  }

  // 获取相对变换
  M4f T = icp.getFinalTransformation().matrix();
  rel_p = T.block<3, 1>(0, 3).cast<double>();
  rel_q = Qd(T.block<3, 3>(0, 0).cast<double>());

  return true;
}