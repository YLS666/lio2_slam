#include "pose_graph_opti/pose_graph_optimizer.hpp"
#include <glog/logging.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include "loop_closure/loop_closure.hpp"
#include "utils/eigen_types.hpp"
#include "utils/g2o_types.hpp"
#include "utils/math_types.hpp"

pose_graph_opti::pose_graph_opti(std::string map_path) : map_path_(map_path) {
  keyframes_path_ = map_path_ + "/keyframes/";
  split_map_path_ = map_path_ + "/split_map/";
  pose_file_ = keyframes_path_ + "keyframe_poses.txt";
  cloud_dir_ = keyframes_path_;
  out_pose_file_ = keyframes_path_ + "optimized_poses.txt";
  out_map_file_ = split_map_path_ + "all_map.pcd";

  global_map_.reset(new PointCloudType());
}

void pose_graph_opti::run() {
  // 0. 清空split_map_path
  if (std::filesystem::exists(split_map_path_)) {
    std::filesystem::remove_all(split_map_path_);
  }
  std::filesystem::create_directories(split_map_path_);

  // 1. 加载关键帧位姿
  if (!loadKeyframePoses()) {
    LOG(ERROR) << "加载位姿文件失败";
    return;
  }

  // 2. 加载关键帧点云 (回环 ICP + 重建地图需要)
  if (!loadKeyframeClouds()) {
    LOG(ERROR) << "加载点云失败";
    return;
  }

  // 3. 回环检测
  detectLoopClosures();

  // 4. 全局位姿图优化 (g2o 内部迭代收敛, 即「再优化」)
  globalOptimize();

  // 5. 保存优化后的位姿
  saveKeyframePoses();

  // 6. 用优化后的位姿重建最终地图
  rebuildGlobalMap();

  // 7. 保存全局地图和分块地图和分块索引
  saveGlobalAndSplitMap();
}

bool pose_graph_opti::loadKeyframePoses() {
  std::ifstream fin(pose_file_);
  if (!fin.is_open()) {
    LOG(ERROR) << "无法打开位姿文件: " << pose_file_;
    return false;
  }

  keyframes_.clear();
  std::string line;
  while (std::getline(fin, line)) {
    if (line.empty() || line[0] == '#') {
      continue;
    }

    std::istringstream iss(line);
    constexpr int kPoseLineFields = 45;  // 2 + 3 + 4 + 36
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
    keyframes_.push_back(kf);
  }

  LOG(INFO) << "加载关键帧位姿: " << keyframes_.size() << " 帧";
  return !keyframes_.empty();
}

void pose_graph_opti::saveKeyframePoses() {
  std::ofstream fout(pose_file_);
  if (!fout.is_open()) {
    LOG(ERROR) << "无法写入位姿文件: " << pose_file_;
    return;
  }

  fout << std::fixed << std::setprecision(9);
  fout << "# id timestamp px py pz qx qy qz qw i00 i01 ... i55\n";
  for (const auto& kf : keyframes_) {
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
  LOG(INFO) << "保存关键帧位姿: " << keyframes_.size() << " 帧 → " << out_pose_file_;
  return;
}

bool pose_graph_opti::loadKeyframeClouds() {
  int loaded = 0;
  for (auto& kf : keyframes_) {
    std::string path = cloud_dir_ + "kf_" + std::to_string(kf.id) + ".pcd";
    CloudPtr cloud(new PointCloudType());
    if (pcl::io::loadPCDFile<PointType>(path, *cloud) == -1) {
      LOG(WARNING) << "加载点云失败: " << path;
      continue;
    }
    kf.cloud = cloud;
    loaded++;
  }
  LOG(INFO) << "加载关键帧点云: " << loaded << "/" << keyframes_.size();
  return loaded > 0;
}

void pose_graph_opti::detectLoopClosures() {
  loop_pairs_.clear();

  LoopClosure lc;
  lc.run(keyframes_, loop_pairs_);

  LOG(INFO) << "回环检测完成, 共 " << loop_pairs_.size() << " 个回环约束";
}

void pose_graph_opti::globalOptimize() {
  if (keyframes_.empty()) {
    return;
  }

  int N = static_cast<int>(keyframes_.size());

  // 重建帧间相对位姿（以绝对位姿为准，保证自洽）
  for (int i = 0; i < N - 1; ++i) {
    keyframes_[i + 1].relative_q = keyframes_[i].q.inverse() * keyframes_[i + 1].q;
    M3d R_i = keyframes_[i].q.toRotationMatrix();
    keyframes_[i + 1].relative_p = R_i.transpose() * (keyframes_[i + 1].p - keyframes_[i].p);
  }

  auto linearSolver = std::make_unique<LinearSolver>();
  linearSolver->setBlockOrdering(false);
  auto blockSolver = std::make_unique<BlockSolver>(std::move(linearSolver));
  auto* algorithm = new g2o::OptimizationAlgorithmLevenberg(std::move(blockSolver));

  g2o::SparseOptimizer optimizer;
  optimizer.setAlgorithm(algorithm);
  optimizer.setVerbose(true);

  // 顶点
  std::vector<g2o_optimizer::VertexPose*> vertices(N);
  for (int i = 0; i < N; ++i) {
    auto* v = new g2o_optimizer::VertexPose();
    v->setId(i);
    v->setEstimate(SE3(keyframes_[i].q, keyframes_[i].p));
    if (i == 0) {
      v->setFixed(true);
    }
    optimizer.addVertex(v);
    vertices[i] = v;
  }

  // 帧间约束
  for (int i = 0; i < N - 1; ++i) {
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
      }
    }

    SE3 obs(keyframes_[i + 1].relative_q, keyframes_[i + 1].relative_p);
    auto* edge = new g2o_optimizer::EdgeRelativeMotion(vertices[i], vertices[i + 1], obs, info);
    optimizer.addEdge(edge);
  }

  // 回环约束
  constexpr double loop_trans_std = 0.20;  // m，回环 NDT 平移精度
  constexpr double loop_rot_std = 0.03;    // rad，回环 NDT 旋转精度
  const double w_t = 1.0 / (loop_trans_std * loop_trans_std);
  const double w_r = 1.0 / (loop_rot_std * loop_rot_std);
  for (const auto& lp : loop_pairs_) {
    if (lp.id_a < 0 || lp.id_a >= N || lp.id_b < 0 || lp.id_b >= N) {
      continue;
    }
    if (!lp.rel_p.allFinite() || !lp.rel_q.coeffs().allFinite()) {
      LOG(WARNING) << "回环约束 " << lp.id_a << "<->" << lp.id_b << " 非有限, 跳过";
      continue;
    }

    SE3 obs(lp.rel_q, lp.rel_p);
    Eigen::Matrix<double, 6, 6> loop_info = Eigen::Matrix<double, 6, 6>::Zero();
    loop_info(0, 0) = loop_info(1, 1) = loop_info(2, 2) = w_t;  // 平移
    loop_info(3, 3) = loop_info(4, 4) = loop_info(5, 5) = w_r;  // 旋转
    loop_info *= lp.info_weight;

    auto* edge = new g2o_optimizer::EdgeRelativeMotion(vertices[lp.id_a], vertices[lp.id_b], obs, loop_info);
    auto rk = new g2o::RobustKernelCauchy();
    rk->setDelta(0.2);
    edge->setRobustKernel(rk);
    optimizer.addEdge(edge);
  }

  optimizer.initializeOptimization();
  optimizer.optimize(50);

  for (int i = 0; i < N; ++i) {
    const SE3& T = vertices[i]->estimate();
    keyframes_[i].q = T.so3().unit_quaternion().normalized();
    keyframes_[i].p = T.translation();
  }

  LOG(INFO) << "全局优化完成, 关键帧数: " << N << ", 回环约束数: " << loop_pairs_.size();
  return;
}

void pose_graph_opti::rebuildGlobalMap() {
  CloudPtr all(new PointCloudType());

  // 1. 所有 KeyFrame 转到世界坐标系
  for (const auto& kf : keyframes_) {
    if (!kf.cloud || kf.cloud->empty()) {
      continue;
    }

    CloudPtr world(new PointCloudType());

    M4f T = se3ToMatrix4f(kf.q, kf.p);
    pcl::transformPointCloud(*kf.cloud, *world, T);

    *all += *world;
  }

  // 2. 按 50m × 50m 进行空间分块
  for (const auto& pt : all->points) {
    const int gx = static_cast<int>(std::floor((pt.x - 25.0f) / 50.0f));
    const int gy = static_cast<int>(std::floor((pt.y - 25.0f) / 50.0f));
    const V2i key(gx, gy);

    auto iter = map_data_.find(key);
    if (iter == map_data_.end()) {
      CloudPtr cloud(new PointCloudType());
      cloud->points.emplace_back(pt);
      cloud->is_dense = false;
      cloud->height = 1;

      map_data_.emplace(key, cloud);
    } else {
      iter->second->points.emplace_back(pt);
    }
  }

  global_map_->reserve(10000000);
  for (const auto& iter : map_data_) {
    CloudPtr ds_cloud = dsCloud(iter.second, 0.1);
    *global_map_ += *ds_cloud;
  }

  global_map_->width = static_cast<uint32_t>(global_map_->size());
  global_map_->height = 1;
  global_map_->is_dense = false;

  LOG(INFO) << "final map:"
            << " input=" << all->size() << " output=" << global_map_->size();

  return;
}

void pose_graph_opti::saveGlobalAndSplitMap() {
  // 1.保存全局地图
  if (!global_map_ || global_map_->empty()) {
    LOG(WARNING) << "全局地图为空, 无法保存";
    return;
  }
  pcl::io::savePCDFileBinary(out_map_file_, *global_map_);

  // 2.保存分块地图与索引
  std::string index_file = split_map_path_ + "map_index.txt";
  std::ofstream fout(index_file);
  if (!fout.is_open()) {
    LOG(ERROR) << "无法写入分块索引文件: " << index_file;
    return;
  }

  for (const auto& iter : map_data_) {
    const V2i& key = iter.first;
    const CloudPtr& cloud = iter.second;
    std::string filename = split_map_path_ + "map_" + std::to_string(key.x()) + "_" + std::to_string(key.y()) + ".pcd";
    pcl::io::savePCDFileBinary(filename, *cloud);
    fout << key.x() << " " << key.y() << " " << "\n";
  }

  fout.close();
  return;
}
