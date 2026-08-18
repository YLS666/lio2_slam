#include "postprocess/pose_graph_optimizer.hpp"
#include <glog/logging.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <fstream>
#include <iomanip>
#include <sstream>
#include "loop_closure/loop_closure.hpp"
#include "utils/eigen_types.hpp"
#include "utils/g2o_types.hpp"
#include "utils/math_types.hpp"

namespace postprocess {

namespace {
constexpr int kPoseLineFields = 45;  // 2 + 3 + 4 + 36
}  // namespace

bool loadKeyframePoses(const std::string& pose_file, std::deque<KeyFrame>& keyframes) {
  std::ifstream fin(pose_file);
  if (!fin.is_open()) {
    LOG(ERROR) << "无法打开位姿文件: " << pose_file;
    return false;
  }

  keyframes.clear();
  std::string line;
  while (std::getline(fin, line)) {
    if (line.empty() || line[0] == '#') {
      continue;
    }

    std::istringstream iss(line);
    double v[kPoseLineFields];
    bool ok = true;
    for (int i = 0; i < kPoseLineFields; ++i) {
      if (!(iss >> v[i])) {
        ok = false;
        break;
      }
    }
    if (!ok) {
      LOG(WARNING) << "位姿文件行格式错误: " << line;
      continue;
    }

    KeyFrame kf;
    kf.id = static_cast<int>(v[0]);
    kf.timestamp = v[1];
    kf.p = V3d(v[2], v[3], v[4]);
    kf.q = Qd(v[8], v[5], v[6], v[7]).normalized();  // 文件存 qx qy qz qw
    for (int r = 0; r < 6; ++r) {
      for (int c = 0; c < 6; ++c) {
        kf.info_mat(r, c) = v[9 + r * 6 + c];
      }
    }
    keyframes.push_back(kf);
  }

  LOG(INFO) << "加载关键帧位姿: " << keyframes.size() << " 帧";
  return !keyframes.empty();
}

bool saveKeyframePoses(const std::string& pose_file, const std::deque<KeyFrame>& keyframes) {
  std::ofstream fout(pose_file);
  if (!fout.is_open()) {
    LOG(ERROR) << "无法写入位姿文件: " << pose_file;
    return false;
  }

  fout << std::fixed << std::setprecision(9);
  fout << "# id timestamp px py pz qx qy qz qw i00 i01 ... i55\n";
  for (const auto& kf : keyframes) {
    const Qd& q = kf.q;
    fout << kf.id << " " << kf.timestamp << " " << kf.p.x() << " " << kf.p.y() << " " << kf.p.z() << " " << q.x() << " "
         << q.y() << " " << q.z() << " " << q.w();
    for (int r = 0; r < 6; ++r) {
      for (int c = 0; c < 6; ++c) {
        fout << " " << kf.info_mat(r, c);
      }
    }
    fout << "\n";
  }
  fout.close();
  LOG(INFO) << "保存关键帧位姿: " << keyframes.size() << " 帧 → " << pose_file;
  return true;
}

bool loadKeyframeClouds(const std::string& cloud_dir, std::deque<KeyFrame>& keyframes) {
  std::string base = cloud_dir;
  if (!base.empty() && base.back() != '/') {
    base += '/';
  }

  int loaded = 0;
  for (auto& kf : keyframes) {
    std::string path = base + "kf_" + std::to_string(kf.id) + ".pcd";
    CloudPtr cloud(new PointCloudType());
    if (pcl::io::loadPCDFile<PointType>(path, *cloud) == -1) {
      LOG(WARNING) << "加载点云失败: " << path;
      continue;
    }
    kf.cloud = cloud;
    loaded++;
  }
  LOG(INFO) << "加载关键帧点云: " << loaded << "/" << keyframes.size();
  return loaded > 0;
}

void detectLoopClosures(const std::deque<KeyFrame>& keyframes, std::vector<LoopPair>& loop_pairs) {
  loop_pairs.clear();

  LoopClosure lc;
  lc.setKeyframesPtr(&keyframes);

  // 先全部加入描述子库
  for (const auto& kf : keyframes) {
    lc.addKeyframe(kf);
  }

  // 再对每个关键帧检测回环
  for (const auto& kf : keyframes) {
    if (!kf.cloud || kf.cloud->empty()) {
      continue;
    }
    std::vector<LoopPair> pairs;
    if (lc.detect(kf, pairs)) {
      loop_pairs.insert(loop_pairs.end(), pairs.begin(), pairs.end());
    }
  }

  LOG(INFO) << "回环检测完成, 共 " << loop_pairs.size() << " 个回环约束";
}

bool globalOptimize(std::deque<KeyFrame>& keyframes, const std::vector<LoopPair>& loop_pairs) {
  if (keyframes.empty()) {
    return false;
  }

  int N = static_cast<int>(keyframes.size());

  // 重建帧间相对位姿（以绝对位姿为准，保证自洽）
  for (int i = 0; i < N - 1; ++i) {
    keyframes[i + 1].relative_q = keyframes[i].q.inverse() * keyframes[i + 1].q;
    M3d R_i = keyframes[i].q.toRotationMatrix();
    keyframes[i + 1].relative_p = R_i.transpose() * (keyframes[i + 1].p - keyframes[i].p);
  }

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
    T.rotate(keyframes[i].q.toRotationMatrix());
    T.pretranslate(keyframes[i].p);
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
    T_rel.rotate(keyframes[i + 1].relative_q.toRotationMatrix());
    T_rel.pretranslate(keyframes[i + 1].relative_p);
    edge->setMeasurement(T_rel);

    Eigen::Matrix<double, 6, 6> info = keyframes[i + 1].info_mat;
    double det = info.determinant();
    if (det < 1e-12 || det > 1e18 || float_check::isnan(det) || float_check::isinf(det)) {
      LOG(WARNING) << "关键帧 " << keyframes[i + 1].id << " 信息矩阵异常(det=" << det << "), 使用单位矩阵";
      info = Eigen::Matrix<double, 6, 6>::Identity();
    } else {
      Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6>> eig_info(info);
      double max_ev = eig_info.eigenvalues().maxCoeff();
      constexpr double MAX_INFO_EIGEN = 1e6;
      if (max_ev > MAX_INFO_EIGEN) {
        info *= MAX_INFO_EIGEN / max_ev;
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

  optimizer.initializeOptimization();
  optimizer.optimize(50);

  for (int i = 0; i < N; ++i) {
    Eigen::Isometry3d T = vertices[i]->estimate();
    keyframes[i].q = Eigen::Quaterniond(T.rotation()).normalized();
    keyframes[i].p = T.translation();
  }

  LOG(INFO) << "全局优化完成, 关键帧数: " << N << ", 回环约束数: " << loop_pairs.size();
  return true;
}

CloudPtr rebuildGlobalMap(const std::deque<KeyFrame>& keyframes, float voxel_size) {
  CloudPtr all(new PointCloudType());
  for (const auto& kf : keyframes) {
    if (!kf.cloud || kf.cloud->empty()) {
      continue;
    }

    CloudPtr world(new PointCloudType());
    M4f T = M4f::Identity();
    T.block<3, 3>(0, 0) = kf.q.toRotationMatrix().cast<float>();
    T.block<3, 1>(0, 3) = kf.p.cast<float>();
    pcl::transformPointCloud(*kf.cloud, *world, T);
    *all += *world;
  }

  if (voxel_size > 0) {
    return dsCloud(all, voxel_size);
  }
  return all;
}

}  // namespace postprocess
