#include "frontend/frontend.hpp"
#include <glog/logging.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <chrono>
#include <filesystem>
#include "cloud_utils/point_type.hpp"

Frontend::Frontend(AllConfig config)
    : is_use_viewer_(config.is_use_imu), last_feature_cloud_(new pcl::PointCloud<PointType>()) {
  // 体素地图
  map_ = std::make_unique<VoxelMap>(0.5f, 20.0f, 4);

  // NDT 配准 (设为 IEKF 回调模式)
  reg_ = std::make_unique<NDTRegistration>(5);
  reg_->setHuber(false, 0.0);      // NDT 不需要 Huber
  reg_->setInfoRatio(0.01);        // 信息矩阵缩放 100 倍
  reg_->setOutlierThreshold(5.0);  // 马氏距离阈值

  // 18-DOF IESKF (替代旧 9-DOF ESKF)
  IESKF::Options ieskf_opt;
  ieskf_opt.num_iterations_ = 3;  // IEKF 迭代次数
  ieskf_opt.gyr_noise_ = 1.7e-4;  // 陀螺仪白噪声
  ieskf_opt.acc_noise_ = 2.0e-3;  // 加速度计白噪声
  ieskf_opt.bg_noise_ = 1e-6;     // bg 随机游走
  ieskf_opt.ba_noise_ = 1e-7;     // ba 随机游走
  ieskf_opt.update_bg_ = true;    // bg 在线估计
  ieskf_opt.update_ba_ = true;    // ba 在线估计
  // 注意: g_ (重力向量) 总是会被更新, 这是修复 Z 轴漂移的关键
  ieskf_ = std::make_unique<IESKF>(ieskf_opt);

  // 后端 & 回环
  backend_ = std::make_unique<Backend>();
  loop_closure_ = std::make_unique<LoopClosure>();
  loop_closure_->setKeyframesPtr(&backend_->getKeyFrames());

  // 可视化
  viewer_ = std::make_unique<PangolinViewer>();
}

void Frontend::init(const State& init_state) {
  state_ = init_state;
  // 设置 IESKF 全量状态 (含 bg, ba, g)
  ieskf_->setState(init_state.q, init_state.p, init_state.v, init_state.bg, init_state.ba, init_state.g,
                   init_state.timestamp);

  // 初始化协方差
  Eigen::Matrix<double, 18, 18> P_init = Eigen::Matrix<double, 18, 18>::Identity() * 1e-4;
  P_init.block<3, 3>(6, 6) = M3d::Identity() * 0.1;  // 旋转不确定度
  ieskf_->setCovariance(P_init);
}

State Frontend::process(const CloudPtr& cloud, const std::string& kf_save_dir) {
  // 初始化
  if (!initialized_) {
    CloudPtr init_cloud = dsCloud(cloud, 0.2f);
    map_->addCloud(init_cloud);
    map_->setLocalCenter(state_.p);
    initialized_ = true;
    LOG(INFO) << "初始化完成，地图点数: " << map_->size();
    return state_;
  }

  frame_count_++;

  // 1. 降采样 + 特征采样
  CloudPtr ds_cloud = dsCloud(cloud, 0.1f);
  auto feature_cloud = featureSample(ds_cloud);

  // 2. IEKF + NDT 配准 (对齐 slam_tools 架构)
  // NDT 作为 IEKF 的观测回调, 在每次 IEKF 迭代中重新线性化
  //   - IMU 先验 (协方差 P) + NDT 观测 (HTVH/HTVr) 在每次迭代中同时起作用
  //   - 不再先做独立 NDT 优化再喂给 ESKF
  reg_->setSource(feature_cloud);
  int effective_points = 0;

  auto tic = std::chrono::steady_clock::now();
  bool reg_success = ieskf_->updateUsingCustomObserve(
      [this, &effective_points](const SE3& input_pose, Eigen::Matrix<double, 18, 18>& HTVH,
                                Eigen::Matrix<double, 18, 1>& HTVr) -> int {
        effective_points = reg_->computeResidualAndJacobians(map_.get(), input_pose, HTVH, HTVr);
        return effective_points;
      },
      50);  // min_effective = 50
  auto toc = std::chrono::steady_clock::now();
  double reg_ms = std::chrono::duration<double, std::milli>(toc - tic).count();

  // 3. 配准失败处理
  if (!reg_success) {
    LOG(ERROR) << "NDT+IEKF 配准失败! effective=" << effective_points;
    if (pending_rebuild_) {
      consecutive_stable_ = 0;
      LOG(WARNING) << "[LoopClosure] 配准失败, 重置稳定计数器";
    }
    diverged_ = true;
    last_reg_success_ = false;
    return state_;
  }
  last_reg_success_ = true;

  // 回环滞后应用: 全局优化后等连续 N 帧配准成功, 再平滑接入
  if (pending_rebuild_) {
    consecutive_stable_++;
    if (consecutive_stable_ >= kPendingRebuildThreshold) {
      const auto& kfs = backend_->getKeyFrames();
      if (!kfs.empty()) {
        const auto& last_kf = kfs.back();
        double pos_diff = (last_kf.p - state_.p).norm();
        double angle_diff = 2.0 * std::acos(std::min(1.0, std::abs((state_.q.inverse() * last_kf.q).w())));
        LOG(INFO) << "[LoopClosure] 应用优化结果, 位姿偏差: dp=" << pos_diff << "m, dθ=" << angle_diff << "rad";
        state_.p = last_kf.p;
        state_.q = last_kf.q;
        resetESKFWithOptimizedPose(state_);
      }
      rebuildMapFromKeyframes();
      pending_rebuild_ = false;
      consecutive_stable_ = 0;
      LOG(INFO) << "[LoopClosure] 回环优化已应用, 地图已重建";
    } else {
      LOG(INFO) << "[LoopClosure] 稳定帧 " << consecutive_stable_ << "/" << kPendingRebuildThreshold;
    }
  }

  // 4. 获取更新后的全量状态
  double state_ts = state_.timestamp;
  state_ = ieskf_->getNominalState();
  if (state_.timestamp == 0.0) {
    state_.timestamp = state_ts;
  }

  LOG(INFO) << "[Process] p=" << state_.p.transpose() << " v=" << state_.v.transpose()
            << " bg=" << state_.bg.transpose() << " ba=" << state_.ba.transpose() << " g=" << state_.g.transpose()
            << " effective=" << effective_points << " reg_ms=" << reg_ms;

  // 5. 构建信息矩阵 (从 IESKF 位姿协方差提取 6×6 块)
  Eigen::Matrix<double, 6, 6> info_mat = Eigen::Matrix<double, 6, 6>::Identity();
  {
    Eigen::Matrix<double, 6, 6> pose_cov = ieskf_->getPoseCovariance();
    // 正则化 + 求逆
    pose_cov.diagonal() += Eigen::Matrix<double, 6, 1>::Constant(1e-3);
    pose_cov = (pose_cov + pose_cov.transpose()) / 2.0;

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6>> eig(pose_cov);
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

  // 6. 后端关键帧管理
  bool is_keyframe = backend_->addKeyFrame(state_, feature_cloud, info_mat);

  if (is_keyframe) {
    // 保存关键帧点云
    if (!kf_save_dir.empty()) {
      if (!std::filesystem::exists(kf_save_dir)) {
        std::filesystem::create_directories(kf_save_dir);
      }
      const auto& kfs = backend_->getKeyFrames();
      std::string kf_path = kf_save_dir + "kf_" + std::to_string(kfs.back().id) + ".pcd";
      pcl::io::savePCDFileBinary(kf_path, *feature_cloud);
      LOG(INFO) << "保存关键帧点云: " << kf_path;
    }

    int kf_count = backend_->getKeyframeCount();
    if (kf_count < 3) {
      LOG(INFO) << "[KF] kf_count=" << kf_count << " (<3)，累积关键帧，暂不优化";
      mergeOptimizedKeyframesToMap();
    } else {
      bool opt_ok = backend_->slideWindowOptimize();
      if (opt_ok) {
        const auto& kfs = backend_->getKeyFrames();
        if (!kfs.empty()) {
          const auto& last_kf = kfs.back();
          double pos_diff = (last_kf.p - state_.p).norm();
          double angle_diff = 2.0 * std::acos(std::min(1.0, std::abs((state_.q.inverse() * last_kf.q).w())));

          if (pos_diff < 1.0 || angle_diff < 0.2) {
            state_.p = last_kf.p;
            state_.q = last_kf.q;
            resetESKFWithOptimizedPose(state_);
            LOG(INFO) << "滑窗优化更新位姿，IESKF 已重置: dp=" << pos_diff << "m, dθ=" << angle_diff << "rad";
          } else {
            LOG(WARNING) << "滑窗优化结果偏离过大，拒绝更新: dp=" << pos_diff << "m, dθ=" << angle_diff << "rad";
          }
        }
        mergeOptimizedKeyframesToMap();
      }
    }

    // 回环检测: 新关键帧触发
    // tryLoopClosure();
  }

  // 7. 更新地图局部中心
  map_->setLocalCenter(state_.p);

  last_feature_cloud_ = feature_cloud;

  // 8. 可视化 (高频操作每帧执行, 低频操作隔帧执行以降低CPU)
  if (is_use_viewer_ && viewer_ && viewer_->isRunning()) {
    // 每帧: 当前点云 + 位姿 (轻量)
    CloudPtr world_current_cloud(new PointCloudType());
    M4f T_cur = M4f::Identity();
    T_cur.block<3, 3>(0, 0) = state_.q.toRotationMatrix().cast<float>();
    T_cur.block<3, 1>(0, 3) = state_.p.cast<float>();
    pcl::transformPointCloud(*cloud, *world_current_cloud, T_cur);
    viewer_->updateCurrentCloud(world_current_cloud);
    viewer_->updatePose(state_.p, state_.q, state_.timestamp);
    viewer_->updateMotionInfo(state_.v, state_.bg, state_.ba);

    // 每3帧: 局部地图 (getCloud 遍历全部体素, 重)
    if (frame_count_ % 3 == 0) {
      auto local_map_cloud = map_->getCloud();
      CloudPtr ds_local_map = dsCloud(local_map_cloud, 1.0f);
      viewer_->updateLocalMap(ds_local_map);
    }

    // 每5帧: 全局轨迹 (appendGlobalMap 累积点云, 持续增长)
    if (frame_count_ % 5 == 0) {
      CloudPtr world_cloud(new PointCloudType());
      M4f T_world = M4f::Identity();
      T_world.block<3, 3>(0, 0) = state_.q.toRotationMatrix().cast<float>();
      T_world.block<3, 1>(0, 3) = state_.p.cast<float>();
      pcl::transformPointCloud(*cloud, *world_cloud, T_world);
      viewer_->appendGlobalMap(world_cloud);
    }
  }

  return state_;
}

CloudPtr Frontend::featureSample(const CloudPtr& cloud) const {
  constexpr float VOXEL_SIZE = 0.3f;

  std::unordered_map<VoxelKey, PointType, VoxelHash> voxel_map;

  for (const auto& pt : cloud->points) {
    VoxelKey key{static_cast<int>(std::floor(pt.x / VOXEL_SIZE)), static_cast<int>(std::floor(pt.y / VOXEL_SIZE)),
                 static_cast<int>(std::floor(pt.z / VOXEL_SIZE))};

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
  const auto& current_kf = kfs.back();

  // 始终添加描述子到检索库 (否则前29帧的描述子丢失, 永远找不到足够老的候选)
  loop_closure_->addKeyframe(current_kf);

  if (static_cast<int>(kfs.size()) < 30) {
    return;
  }

  kf_since_loop_check_++;
  if (kf_since_loop_check_ < loop_closure_interval_) {
    return;
  }
  kf_since_loop_check_ = 0;

  {
    std::vector<LoopPair> loop_pairs;
    if (loop_closure_->detect(current_kf, loop_pairs)) {
      LOG(INFO) << "检测到 " << loop_pairs.size() << " 个回环, 执行全局优化";
      backend_->globalOptimize(loop_pairs);

      // 全局优化后暂不立即重置IESKF和重建地图
      // 等连续N帧NDT配准成功后, 再平滑应用优化结果
      pending_rebuild_ = true;
      consecutive_stable_ = 0;
      LOG(INFO) << "[LoopClosure] 等待 " << kPendingRebuildThreshold << " 帧连续配准成功后重建地图...";
    }
  }
}

void Frontend::saveMap(const std::string& save_dir) {
  map_->clearAll();
  const auto& kfs = backend_->getKeyFrames();

  for (size_t i = 0; i < kfs.size(); ++i) {
    const auto& kf = kfs[i];
    std::string kf_path = save_dir + "kf_" + std::to_string(kf.id) + ".pcd";

    CloudPtr cloud(new PointCloudType());
    if (pcl::io::loadPCDFile<PointType>(kf_path, *cloud) == -1) {
      continue;
    }

    M4f T = M4f::Identity();
    T.block<3, 3>(0, 0) = kf.q.toRotationMatrix().cast<float>();
    T.block<3, 1>(0, 3) = kf.p.cast<float>();

    CloudPtr world_cloud(new PointCloudType());
    pcl::transformPointCloud(*cloud, *world_cloud, T);
    map_->addCloud(world_cloud);
  }

  CloudPtr all = map_->getCloud();
  std::string map_path = save_dir + "all_map.pcd";
  pcl::io::savePCDFileBinary(map_path, *all);
  LOG(INFO) << "地图保存完成: " << map_path << ", 点数: " << all->size();
}

void Frontend::resetESKFWithOptimizedPose(const State& state) {
  // 用优化后轨迹的相邻 KF 位姿差分估算速度
  V3d v_est = V3d::Zero();
  const auto& kfs = backend_->getKeyFrames();
  if (kfs.size() >= 2) {
    const auto& prev_kf = kfs[kfs.size() - 2];
    double dt = state.timestamp - prev_kf.timestamp;
    if (dt > 0.01) {
      v_est = (state.p - prev_kf.p) / dt;
    }
  }

  // 设置 IESKF 全量状态
  // 保留当前的 bg, ba, g 估计 (它们不受后端优化的影响)
  State cur = ieskf_->getNominalState();
  ieskf_->setState(state.q, state.p, v_est, cur.bg, cur.ba, cur.g, state.timestamp);

  // 重置协方差
  Eigen::Matrix<double, 18, 18> P_new = Eigen::Matrix<double, 18, 18>::Identity() * 1e-4;
  P_new.block<3, 3>(0, 0) *= 0.05;  // 位置: 0.05 m²
  P_new.block<3, 3>(3, 3) *= 1.0;   // 速度: 1.0 (m/s)²
  P_new.block<3, 3>(6, 6) *= 0.01;  // 旋转: 0.01 rad²
  ieskf_->setCovariance(P_new);

  LOG(INFO) << "[IESKF Reset] p=" << state.p.transpose() << " v_est=" << v_est.transpose()
            << " g=" << cur.g.transpose();
}

void Frontend::mergeOptimizedKeyframesToMap() {
  const auto& kfs = backend_->getKeyFrames();
  if (kfs.empty()) {
    return;
  }

  std::vector<int> merged_ids;
  for (size_t i = 0; i < kfs.size(); ++i) {
    const auto& kf = kfs[i];
    if (kf.merged) {
      continue;
    }
    if (!kf.cloud || kf.cloud->empty()) {
      continue;
    }

    CloudPtr world_cloud(new PointCloudType());
    M4f T = M4f::Identity();
    T.block<3, 3>(0, 0) = kf.q.toRotationMatrix().cast<float>();
    T.block<3, 1>(0, 3) = kf.p.cast<float>();
    pcl::transformPointCloud(*kf.cloud, *world_cloud, T);

    CloudPtr new_cloud(new PointCloudType());
    new_cloud->reserve(world_cloud->size());
    for (const auto& pt : world_cloud->points) {
      if (!map_->hasNearbyCell(pt, 0.5f, NearbyType::CENTER)) {
        new_cloud->push_back(pt);
      }
    }

    map_->addCloud(new_cloud);
    merged_ids.push_back(kf.id);
    LOG(INFO) << "[MapMerge] KF#" << kf.id << " 添加 " << new_cloud->size() << " 点, 地图总数: " << map_->size();
  }

  if (!merged_ids.empty()) {
    backend_->markKeyframesMerged(merged_ids);
  }
}

void Frontend::rebuildMapFromKeyframes() {
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

    CloudPtr ds_world_cloud = dsCloud(world_cloud, 0.1f);
    map_->addCloud(ds_world_cloud);
  }

  for (auto& kf : kfs) {
    kf.merged = true;
  }

  LOG(INFO) << "[MapRebuild] 回环后地图重建完成, 点数: " << map_->size();

  if (is_use_viewer_ && viewer_ && viewer_->isRunning()) {
    viewer_->clearGlobalMap();
    auto rebuilt_cloud = map_->getCloud();
    viewer_->appendGlobalMap(rebuilt_cloud);
  }
}

void Frontend::propagateFromTrustedPose(const std::deque<sensor_msgs::msg::Imu>& imu_datas, double cloud_time,
                                        double g_norm) {
  if (!initialized_ || imu_datas.size() < 2) {
    return;
  }

  // 起点: 上一帧的可靠状态 (含在线估计的 bg, ba, g)
  // bg 以 ImuProcessor 标定值为初值，IESKF 在线微调，跨帧保持
  ieskf_->setState(state_.q, state_.p, state_.v, state_.bg, state_.ba, state_.g, state_.timestamp);

  double start_time = state_.timestamp;
  if (start_time <= 0.0) {
    LOG(ERROR) << "[IMU Propagate] state_.timestamp 异常(" << start_time << "), 跳过递推";
    return;
  }

  int predict_count = 0;
  for (size_t i = 0; i < imu_datas.size() - 1; ++i) {
    const auto& imu0 = imu_datas[i];
    const auto& imu1 = imu_datas[i + 1];

    double t0 = imu0.header.stamp.sec + imu0.header.stamp.nanosec * 1e-9;
    double t1 = imu1.header.stamp.sec + imu1.header.stamp.nanosec * 1e-9;

    // 跳过 start_time 之前的 IMU，超过 cloud_time 停止
    if (t1 < start_time) {
      continue;
    }
    if (t0 > cloud_time) {
      break;
    }

    double dt = t1 - t0;
    if (dt <= 0.0 || dt > 0.1) {
      continue;
    }

    // 直接使用原始 IMU 陀螺仪和加速度计测量值
    V3d gyr0(imu0.angular_velocity.x, imu0.angular_velocity.y, imu0.angular_velocity.z);
    V3d gyr1(imu1.angular_velocity.x, imu1.angular_velocity.y, imu1.angular_velocity.z);
    V3d acc0(imu0.linear_acceleration.x, imu0.linear_acceleration.y, imu0.linear_acceleration.z);
    V3d acc1(imu1.linear_acceleration.x, imu1.linear_acceleration.y, imu1.linear_acceleration.z);

    // g → m/s²
    acc0 *= g_norm;
    acc1 *= g_norm;

    // 中值积分
    V3d gyr_mid = 0.5 * (gyr0 + gyr1);
    V3d acc_mid = 0.5 * (acc0 + acc1);

    // IESKF::predict() 内部减去在线估计的 bg_/ba_
    ieskf_->predict(gyr_mid, acc_mid, dt);
    predict_count++;
  }

  // 更新当前 state_ 为递推结果
  state_ = ieskf_->getNominalState();
  state_.timestamp = cloud_time;

  VLOG(1) << "[IMU Propagate] predict_count=" << predict_count << " pred_p=" << state_.p.transpose()
          << " pred_bg=" << state_.bg.transpose() << " pred_g=" << state_.g.transpose();
}

void Frontend::initViewer() {
  if (is_use_viewer_ && viewer_ && !viewer_->isRunning()) {
    viewer_->start();
  }
}
