#include "loop_closure/loop_closure.hpp"

#include <glog/logging.h>
#include <tbb/tbb.h>

#include <pcl/common/transforms.h>
#include <pcl/registration/ndt.h>

#include <cmath>
#include <cstdlib>
#include <unordered_map>

#include "cloud_utils/point_type.hpp"
#include "utils/eigen_types.hpp"

void LoopClosure::run(const std::deque<KeyFrame>& keyframes, std::vector<LoopPair>& loop_pairs) {
  loop_pairs.clear();

  const size_t n = keyframes.size();
  if (n < 2) {
    return;
  }

  // 1. 构建索引结构 (一次性, 合并计算)
  std::vector<const KeyFrame*> kfs;
  std::vector<V3d> translations;
  std::vector<M4f> world_T;
  std::unordered_map<int, int> id_to_index;
  kfs.reserve(n);
  translations.reserve(n);
  world_T.reserve(n);
  id_to_index.reserve(n);

  for (size_t i = 0; i < n; ++i) {
    const KeyFrame& kf = keyframes[i];
    kfs.push_back(&kf);
    translations.push_back(kf.p);

    M4f T = M4f::Identity();
    T.block<3, 3>(0, 0) = kf.q.toRotationMatrix().cast<float>();
    T.block<3, 1>(0, 3) = kf.p.cast<float>();
    world_T.push_back(T);

    id_to_index[kf.id] = static_cast<int>(i);
  }

  // 2. 位姿距离筛选候选
  std::vector<Candidate> candidates;
  detectCandidates(translations, candidates);
  if (candidates.empty()) {
    LOG(INFO) << "no loop candidates";
    return;
  }

  // 3. TBB 并行 NDT 配准 (每个候选独立 NDT 对象/点云, 线程安全)
  tbb::parallel_for(tbb::blocked_range<size_t>(0, candidates.size()), [&](const tbb::blocked_range<size_t>& range) {
    for (size_t i = range.begin(); i != range.end(); ++i) {
      computeForCandidate(kfs, world_T, id_to_index, candidates[i]);
    }
  });

  // 4. 过滤并输出回环约束
  int succ = 0;
  for (const auto& c : candidates) {
    if (c.ndt_score > ndt_score_th_) {
      LoopPair pair;
      pair.id_a = c.idx1;
      pair.id_b = c.idx2;
      pair.rel_p = c.Tij_.translation();
      pair.rel_q = c.Tij_.so3().unit_quaternion();
      pair.info_weight = 1.0;
      loop_pairs.push_back(pair);
      succ++;
    }
  }
  LOG(INFO) << "success: " << succ << "/" << candidates.size();
}

void LoopClosure::detectCandidates(const std::vector<V3d>& translations, std::vector<Candidate>& candidates) const {
  int check_first = -1;
  int check_second = -1;
  const int n = static_cast<int>(translations.size());

  for (int i = 0; i < n; ++i) {
    if (check_first != -1 && std::abs(i - check_first) <= skip_id_) {
      continue;
    }

    for (int j = i; j < n; ++j) {
      if (check_second != -1 && std::abs(j - check_second) <= skip_id_) {
        continue;
      }

      if (std::abs(i - j) < min_id_interval_) {
        continue;
      }

      double t2d = (translations[i] - translations[j]).head<2>().norm();
      if (t2d < min_distance_) {
        Candidate c;
        c.idx1 = i;
        c.idx2 = j;
        candidates.push_back(c);
        check_first = i;
        check_second = j;
      }
    }
  }
  LOG(INFO) << "detected candidates: " << candidates.size();
}

void LoopClosure::computeForCandidate(const std::vector<const KeyFrame*>& kfs, const std::vector<M4f>& world_T,
                                      const std::unordered_map<int, int>& id_to_index, Candidate& c) const {
  const KeyFrame* kf1 = kfs[c.idx1];
  const KeyFrame* kf2 = kfs[c.idx2];

  // 目标: kf1 邻域世界系子图; 源: kf2 单帧局部系
  CloudPtr target_orig = buildSubmap(kfs, world_T, id_to_index, c.idx1);
  CloudPtr source_orig = kf2->cloud;

  if (!target_orig || target_orig->empty() || !source_orig || source_orig->empty()) {
    c.ndt_score = 0;
    return;
  }

  pcl::NormalDistributionsTransform<PointType, PointType> ndt;
  ndt.setTransformationEpsilon(ndt_trans_epsilon_);
  ndt.setStepSize(ndt_step_size_);
  ndt.setMaximumIterations(ndt_max_iter_);

  M4f Tw2 = world_T[c.idx2];
  CloudPtr output(new PointCloudType());
  for (double r : resolutions_) {
    ndt.setResolution(static_cast<float>(r));
    // 每次都从原始点云重采样
    CloudPtr target = dsCloud(target_orig, static_cast<float>(r * 0.1), 1);
    CloudPtr source = dsCloud(source_orig, static_cast<float>(r * 0.1), 1);
    ndt.setInputTarget(target);
    ndt.setInputSource(source);

    ndt.align(*output, Tw2);
    Tw2 = ndt.getFinalTransformation();
  }

  M4d T = Tw2.cast<double>();
  Qd q(T.block<3, 3>(0, 0));
  q.normalize();
  V3d t = T.block<3, 1>(0, 3);

  c.Tij_ = SE3(kf1->q, kf1->p).inverse() * SE3(q, t);
  c.ndt_score = ndt.getTransformationProbability();

  LOG(INFO) << "first " << c.idx1 << " second " << c.idx2 << " score " << c.ndt_score;
}

CloudPtr LoopClosure::buildSubmap(const std::vector<const KeyFrame*>& kfs, const std::vector<M4f>& world_T,
                                  const std::unordered_map<int, int>& id_to_index, int center_id) const {
  CloudPtr submap(new PointCloudType());
  for (int offset = -submap_idx_range_; offset < submap_idx_range_; offset += submap_step_) {
    int id = center_id + offset;
    auto iter = id_to_index.find(id);
    if (iter == id_to_index.end()) {
      continue;
    }

    const KeyFrame* kf = kfs[iter->second];
    if (!kf->cloud || kf->cloud->empty()) {
      continue;
    }

    CloudPtr world(new PointCloudType());
    pcl::transformPointCloud(*kf->cloud, *world, world_T[iter->second]);
    *submap += *world;
  }
  return submap;
}
