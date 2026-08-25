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
  lc.run(keyframes, loop_pairs);

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
  std::vector<postprocess::VertexPose*> vertices(N);
  for (int i = 0; i < N; ++i) {
    auto* v = new postprocess::VertexPose();
    v->setId(i);
    v->setEstimate(SE3(keyframes[i].q, keyframes[i].p));
    if (i == 0) {
      v->setFixed(true);
    }
    optimizer.addVertex(v);
    vertices[i] = v;
  }

  // 帧间约束
  for (int i = 0; i < N - 1; ++i) {
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

    SE3 obs(keyframes[i + 1].relative_q, keyframes[i + 1].relative_p);
    auto* edge = new postprocess::EdgeRelativeMotion(vertices[i], vertices[i + 1], obs, info);
    optimizer.addEdge(edge);
  }

  // 回环约束
  constexpr double loop_rk_th = 0.2;
  for (const auto& lp : loop_pairs) {
    if (lp.id_a < 0 || lp.id_a >= N || lp.id_b < 0 || lp.id_b >= N) {
      continue;
    }
    if (!lp.rel_p.allFinite() || !lp.rel_q.coeffs().allFinite()) {
      LOG(WARNING) << "回环约束 " << lp.id_a << "<->" << lp.id_b << " 非有限, 跳过";
      continue;
    }

    SE3 obs(lp.rel_q, lp.rel_p);
    Eigen::Matrix<double, 6, 6> loop_info = Eigen::Matrix<double, 6, 6>::Identity() * lp.info_weight * 10.0;
    auto* edge = new postprocess::EdgeRelativeMotion(vertices[lp.id_a], vertices[lp.id_b], obs, loop_info);
    auto rk = new g2o::RobustKernelCauchy();
    rk->setDelta(loop_rk_th);
    edge->setRobustKernel(rk);
    optimizer.addEdge(edge);
  }

  optimizer.initializeOptimization();
  optimizer.optimize(50);

  for (int i = 0; i < N; ++i) {
    const SE3& T = vertices[i]->estimate();
    keyframes[i].q = T.so3().unit_quaternion().normalized();
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
