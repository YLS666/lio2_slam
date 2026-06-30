#include "frontend/frontend.hpp"
#include <Eigen/src/Core/Matrix.h>
#include <glog/logging.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>

Frontend::Frontend() : last_feature_cloud_(new pcl::PointCloud<PointType>()) {
  map_ = std::make_unique<VoxelMap>(0.2f, 20.0f, 2);
  reg_ = std::make_unique<Registration>();
  eskf_ = std::make_unique<ESKF>();
  backend_ = std::make_unique<Backend>();
  loop_closure_ = std::make_unique<LoopClosure>();
  loop_closure_->setKeyframesPtr(&backend_->getKeyFrames());
}

void Frontend::init(const State& init_state) {
  state_ = init_state;
  eskf_->setState(init_state.q, init_state.p, init_state.v, init_state.timestamp);
}

void Frontend::predict(const V3d gyr, const V3d acc, double dt, double g_norm) {
  if (!initialized_) {
    return;
  }

  // ESKF 前向传播
  eskf_->predict(gyr, acc, dt, g_norm);

  // 更新状态 (用于 map 的局部中心)
  state_ = eskf_->getNominalState();

  state_.timestamp += dt;
}

State Frontend::process(const CloudPtr& cloud) {
  // 初始化
  if (!initialized_) {
    CloudPtr init_cloud(new PointCloudType);
    static pcl::VoxelGrid<PointType> voxel_init;
    voxel_init.setInputCloud(cloud);
    voxel_init.setLeafSize(0.2f, 0.2f, 0.2f);
    voxel_init.filter(*init_cloud);
    map_->addCloud(init_cloud);
    map_->setLocalCenter(state_.p);
    initialized_ = true;
    LOG(INFO) << "初始化完成，地图点数: " << map_->size();
    return state_;
  }

  frame_count_++;

  // 1: 降采样特征
  CloudPtr ds_cloud(new PointCloudType);
  static pcl::VoxelGrid<PointType> voxel;
  voxel.setInputCloud(cloud);
  voxel.setLeafSize(0.1f, 0.1f, 0.1f);
  voxel.filter(*ds_cloud);

  auto feature_cloud = featureSample(ds_cloud);

  // 2: 配准 (使用 ESKF 预测作为初值)
  State reg_init = state_;  // ESKF 预测值
  bool reg_success = reg_->align(feature_cloud, map_.get(), reg_init);

  // 3: 配准失败处理
  if (!reg_success) {
    LOG(ERROR) << "配准失败，系统发散! 停止递推，准备保存地图退出...";
    diverged_ = true;
    last_reg_success_ = false;
    return state_;
  }
  last_reg_success_ = true;

  // 4: ESKF 观测更新
  eskf_->observePose(reg_init.q, reg_init.p);
  double state_ts = state_.timestamp;
  state_ = eskf_->getNominalState();
  if (state_.timestamp == 0.0) {
    state_.timestamp = state_ts;  // 重置时间戳
  }

  // 5: 构建信息矩阵
  Eigen::Matrix<double, 6, 6> info_mat = Eigen::Matrix<double, 6, 6>::Identity();
  Eigen::Matrix<double, 6, 6> cov = reg_->getCovariance();
  if (cov.norm() > 1e-10 && cov.norm() < 1e10) {
    Eigen::Matrix<double, 6, 6> cov_reg = cov;
    cov_reg.diagonal() += Eigen::Matrix<double, 6, 1>::Constant(1e-3);
    cov_reg = (cov_reg + cov_reg.transpose()) / 2.0;
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6>> eig(cov_reg);
    auto eigenvalues = eig.eigenvalues();
    auto eigenvectors = eig.eigenvectors();
    constexpr double MIN_EIGEN = 1e-8;
    Eigen::Matrix<double, 6, 1> inv_eigen;
    for (int i = 0; i < 6; ++i) {
      inv_eigen(i) = (eigenvalues(i) < MIN_EIGEN) ? (1.0 / MIN_EIGEN) : (1.0 / eigenvalues(i));
    }
    info_mat = eigenvectors * inv_eigen.asDiagonal() * eigenvectors.transpose();
    info_mat = (info_mat + info_mat.transpose()) / 2.0;
  }

  // 6: 后端关键帧管理 (先保存，不插入地图)
  bool is_keyframe = backend_->addKeyFrame(state_, feature_cloud, info_mat);

  if (is_keyframe) {
    // 6.1: 滑动窗口优化
    backend_->slideWindowOptimize();

    // 获取优化后的最新关键帧位姿（用于 ESKF 重置）
    const auto& kfs = backend_->getKeyFrames();
    if (!kfs.empty()) {
      const auto& last_kf = kfs.back();
      double pos_diff = (last_kf.p - state_.p).norm();
      double angle_diff = 2.0 * std::acos(std::min(1.0, std::abs((state_.q.inverse() * last_kf.q).w())));

      if (pos_diff < 0.5 && angle_diff < 0.2) {
        // 用滑窗优化后的位姿更新当前 state_
        state_.p = last_kf.p;
        state_.q = last_kf.q;

        // 重置 ESKF：以优化后位姿为新起点
        // 下次 propagateFromTrustedPose 会从此起点重新递推
        resetESKFWithOptimizedPose(state_);

        LOG(INFO) << "滑窗优化更新位姿，ESKF 已重置: dp=" << pos_diff << "m, dθ=" << angle_diff << "rad";
      } else {
        LOG(WARNING) << "滑窗优化结果异常，拒绝更新: dp=" << pos_diff << "m, dθ=" << angle_diff << "rad";
      }
    }

    // 6.2: 回环检测
    tryLoopClosure();

    // 7: 合并已优化的关键帧到地图（延迟插入）
    mergeOptimizedKeyframesToMap();
  }

  // 8: 更新地图局部中心
  map_->setLocalCenter(state_.p);

  // 保存特征云 (用于回环)
  last_feature_cloud_ = feature_cloud;

  return state_;
}

CloudPtr Frontend::featureSample(const CloudPtr& cloud) const {
  constexpr float VOXEL_SIZE = 0.4f;

  std::unordered_map<VoxelKey, PointType, VoxelHash> voxel_map;

  for (const auto& pt : cloud->points) {
    VoxelKey key{static_cast<int>(std::floor(pt.x / VOXEL_SIZE)), static_cast<int>(std::floor(pt.y / VOXEL_SIZE)),
                 static_cast<int>(std::floor(pt.z / VOXEL_SIZE))};

    // 每个 voxel 只保留一个点
    if (voxel_map.find(key) == voxel_map.end()) {
      voxel_map[key] = pt;
    }
  }

  CloudPtr out(new PointCloudType);
  out->reserve(voxel_map.size());

  for (const auto& kv : voxel_map) {
    out->push_back(kv.second);
  }

  return out;
}

void Frontend::tryLoopClosure() {
  const auto& kfs = backend_->getKeyFrames();
  if (kfs.size() < 30) {
    return;
  }

  const auto& current_kf = kfs.back();

  // 添加到回环检测库
  loop_closure_->addKeyframe(current_kf);

  // 检测回环
  if (frame_count_ % loop_closure_interval_ == 0) {
    std::vector<LoopPair> loop_pairs;
    if (loop_closure_->detect(current_kf, loop_pairs)) {
      LOG(INFO) << "检测到 " << loop_pairs.size() << " 个回环!";

      // 全局优化
      backend_->globalOptimize(loop_pairs);

      // 更新当前状态为优化后的最新关键帧位姿
      const auto& updated_kfs = backend_->getKeyFrames();
      if (!updated_kfs.empty()) {
        state_.p = updated_kfs.back().p;
        state_.q = updated_kfs.back().q;

        // 重置 ESKF（下次 propagateFromTrustedPose 会从此起点重新递推）
        resetESKFWithOptimizedPose(state_);
      }

      // 回环后重建地图（所有关键帧位姿都变了）
      rebuildMapFromKeyframes();

      LOG(INFO) << "回环优化完成，地图已重建";
    }
  }
}

void Frontend::saveMap(const std::string& filename) const {
  auto cloud = map_->getCloud();
  if (cloud->empty()) {
    LOG(WARNING) << "地图为空，无法保存";
    return;
  }
  pcl::io::savePCDFileBinary(filename, *cloud);
  LOG(INFO) << "地图保存完成：" << filename << ",  点数：" << cloud->size();
}

void Frontend::resetESKFWithOptimizedPose(const State& state) {
  // 将 ESKF 的名义状态重置为优化后的可靠位姿
  eskf_->setState(state.q, state.p, state.v, state.timestamp);

  // 重置协方差：给旋转和平移小幅不确定性，速度较大不确定性
  Eigen::Matrix<double, 9, 9> P_new = Eigen::Matrix<double, 9, 9>::Identity();
  P_new.block<3, 3>(0, 0) *= 0.01;  // 旋转: 0.01 rad² (~5.7°)
  P_new.block<3, 3>(3, 3) *= 0.05;  // 平移: 0.05 m² (~22cm)
  P_new.block<3, 3>(6, 6) *= 0.5;   // 速度: 0.5 (m/s)²
  eskf_->setCovariance(P_new);

  LOG(INFO) << "[ESKF Reset] 用优化后位姿重置: p=" << state.p.transpose() << " q=" << state.q.coeffs().transpose();
}

void Frontend::mergeOptimizedKeyframesToMap() {
  const auto& kfs = backend_->getKeyFrames();
  if (kfs.empty()) {
    return;
  }

  // 收集所有已优化但尚未合并的关键帧
  std::vector<int> merged_ids;
  for (size_t i = 0; i < kfs.size(); ++i) {
    const auto& kf = kfs[i];
    if (kf.merged) {
      continue;  // 已合并的跳过
    }

    if (!kf.cloud || kf.cloud->empty()) {
      continue;
    }

    // 使用优化后的位姿将点云投影到世界坐标系
    CloudPtr world_cloud(new PointCloudType());
    M4f T = M4f::Identity();
    T.block<3, 3>(0, 0) = kf.q.toRotationMatrix().cast<float>();
    T.block<3, 1>(0, 3) = kf.p.cast<float>();
    pcl::transformPointCloud(*kf.cloud, *world_cloud, T);

    // 过滤已存在的体素
    CloudPtr new_cloud(new PointCloudType());
    new_cloud->reserve(world_cloud->size());
    for (const auto& pt : world_cloud->points) {
      if (!map_->hasNearbyPoint(pt, 0.2f, NearbyType::CENTER)) {
        new_cloud->push_back(pt);
      }
    }

    map_->addCloud(new_cloud);
    merged_ids.push_back(kf.id);

    LOG(INFO) << "[MapMerge] KF#" << kf.id << " 添加 " << new_cloud->size() << " 点, 地图总数: " << map_->size();
  }

  // 标记已合并
  if (!merged_ids.empty()) {
    backend_->markKeyframesMerged(merged_ids);
  }
}

void Frontend::rebuildMapFromKeyframes() {
  // 回环后：清空地图，用所有优化后的关键帧重建
  map_->clearAll();

  const auto& kfs = backend_->getKeyFrames();
  for (const auto& kf : kfs) {
    if (!kf.cloud || kf.cloud->empty()) {
      continue;
    }

    CloudPtr world_cloud(new PointCloudType());
    M4f T = M4f::Identity();
    T.block<3, 3>(0, 0) = kf.q.toRotationMatrix().cast<float>();
    T.block<3, 1>(0, 3) = kf.p.cast<float>();
    pcl::transformPointCloud(*kf.cloud, *world_cloud, T);

    // 降采样避免过密
    static pcl::VoxelGrid<PointType> voxel_rebuild;
    voxel_rebuild.setInputCloud(world_cloud);
    voxel_rebuild.setLeafSize(0.1f, 0.1f, 0.1f);
    CloudPtr ds(new PointCloudType());
    voxel_rebuild.filter(*ds);

    map_->addCloud(ds);
  }

  // 重建后标记所有关键帧为已合并，后续新帧可以继续正常合并
  for (auto& kf : kfs) {
    kf.merged = true;
  }

  LOG(INFO) << "[MapRebuild] 回环后地图重建完成, 点数: " << map_->size();
}

void Frontend::propagateFromTrustedPose(const std::vector<ImuState>& imu_states,
                                        const std::deque<sensor_msgs::msg::Imu>& imu_datas, double cloud_time,
                                        double g_norm) {
  if (!initialized_ || imu_states.size() < 2) {
    return;
  }

  // 起点始终是 state_（上一帧配准/后端给出的可靠位姿）
  // 不从上次 predict 结果继续，而是每次重新 setState
  eskf_->setState(state_.q, state_.p, state_.v, state_.timestamp);

  // 找到相对于 cloud_time 的最近 IMU 帧
  // 从 state_ 的时间戳到 cloud_time 之间的 IMU 数据
  double start_time = state_.timestamp;

  int propagate_count = 0;
  constexpr int MAX_PROPAGATE_STATES = 50;  // 最多处理50个状态对
  for (size_t i = 0; i < imu_states.size() - 1; ++i) {
    const auto& s1 = imu_states[i];
    const auto& s2 = imu_states[i + 1];

    // 只处理 start_time 之后的 IMU 数据
    if (s1.timestamp < start_time) {
      continue;
    }
    if (s1.timestamp > cloud_time) {
      break;
    }

    if (++propagate_count > MAX_PROPAGATE_STATES) {
      LOG(WARNING) << "[IMU Propagate] 处理状态过多(" << propagate_count << ")，可能时间戳异常，强制截断";
      break;  // 跳过后续 IMU 数据
    }

    double dt = s2.timestamp - s1.timestamp;
    if (dt <= 0.0 || dt > 0.1) {
      continue;  // 时间戳异常才跳过
    }

    // 从 s1→s2 的相对运动提取角速度
    SE3 T_rel = s1.T.inverse() * s2.T;
    V3d gyr = T_rel.so3().log() / dt;

    // 从原始 IMU 消息中按时间戳匹配提取加速度
    V3d acc_body = V3d::Zero();
    for (const auto& imu_msg : imu_datas) {
      double msg_time = imu_msg.header.stamp.sec + imu_msg.header.stamp.nanosec * 1e-9;
      if (std::abs(msg_time - s1.timestamp) < 0.005) {  // 5ms 内匹配
        acc_body << imu_msg.linear_acceleration.x, imu_msg.linear_acceleration.y, imu_msg.linear_acceleration.z;
        acc_body *= g_norm;  // g → m/s²
        acc_body -= s1.ba;   // 减去加速度计 bias
        break;
      }
    }

    eskf_->predict(gyr, acc_body, dt, g_norm);
  }

  // 更新当前 state_ 为递推结果
  state_ = eskf_->getNominalState();
  state_.timestamp = cloud_time;

  VLOG(1) << "[IMU Propagate] 短期递推 " << imu_states.size() << " 帧, pred_p=" << state_.p.transpose();
}
