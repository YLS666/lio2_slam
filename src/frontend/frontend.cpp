#include "frontend/frontend.hpp"
#include <glog/logging.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include "cloud_utils/point_type.hpp"
#include "utils/parallel.hpp"

Frontend::Frontend(AllConfig config)
    : is_use_viewer_(config.is_use_ui),
      last_feature_cloud_(new pcl::PointCloud<PointType>()),
      num_threads_(config.num_threads) {
  // 体素地图
  map_ = std::make_unique<VoxelMap>(0.5f, 20.0f, 4);
  map_->setNumThreads(num_threads_);

  // NDT 配准 (设为 IEKF 回调模式)
  reg_ = std::make_unique<NDTRegistration>(5);
  reg_->setNumThreads(num_threads_);
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
  ieskf_->setGravityNorm(config.g_norm);  // 重力范数约束值

  // 后端(只负责关键帧管理)
  backend_ = std::make_unique<Backend>();

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

  // 2. IEKF + NDT 配准
  // NDT 作为 IEKF 的观测回调, 在每次 IEKF 迭代中重新线性化
  //   - IMU 先验 (协方差 P) + NDT 观测 (HTVH/HTVr) 在每次迭代中同时起作用
  //   - 不再先做独立 NDT 优化再喂给 ESKF
  reg_->setSource(feature_cloud);
  int effective_points = 0;

  SE3 pre_pose = ieskf_->getNominalSE3();

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
    diverged_ = true;
    last_reg_success_ = false;
    return state_;
  }
  last_reg_success_ = true;

  // 4. 获取更新后的全量状态
  double state_ts = state_.timestamp;
  state_ = ieskf_->getNominalState();
  if (state_.timestamp == 0.0) {
    state_.timestamp = state_ts;
  }

  // NDT 对 IMU 旋转预测的修正量
  SE3 post_pose = ieskf_->getNominalSE3();
  float ndt_rot_correction = static_cast<float>((pre_pose.inverse() * post_pose).so3().log().norm());

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
  bool is_keyframe = backend_->addKeyFrame(state_, feature_cloud, info_mat, effective_points, ndt_rot_correction);

  if (is_keyframe) {
    // 保存关键帧点云 + 位姿
    if (!kf_save_dir.empty()) {
      // 每次程序运行只清空一次目录，避免上一次运行残留的数据混入
      static bool kf_dir_cleared = false;
      if (!kf_dir_cleared) {
        if (std::filesystem::exists(kf_save_dir)) {
          std::filesystem::remove_all(kf_save_dir);
        }
        std::filesystem::create_directories(kf_save_dir);
        kf_dir_cleared = true;
      }

      const auto& kfs = backend_->getKeyFrames();
      const auto& kf = kfs.back();

      std::string kf_path = kf_save_dir + "kf_" + std::to_string(kf.id) + ".pcd";
      pcl::io::savePCDFileBinary(kf_path, *feature_cloud);

      // 追加关键帧位姿到文本文件 (每行: id timestamp px py pz qx qy qz qw i00..i55)
      std::string pose_path = kf_save_dir + "keyframe_poses.txt";
      bool write_header = !std::filesystem::exists(pose_path);
      std::ofstream fout(pose_path, std::ios::app);
      if (fout.is_open()) {
        fout << std::fixed << std::setprecision(9);
        if (write_header) {
          fout << "# id timestamp px py pz qx qy qz qw i00 i01 ... i55\n";
        }
        const Qd& q = kf.q;
        fout << kf.id << " " << kf.timestamp << " " << kf.p.x() << " " << kf.p.y() << " " << kf.p.z() << " " << q.x()
             << " " << q.y() << " " << q.z() << " " << q.w();
        for (int r = 0; r < 6; ++r) {
          for (int c = 0; c < 6; ++c) {
            fout << " " << kf.info_mat(r, c);
          }
        }
        fout << "\n";
        fout.close();
      } else {
        LOG(ERROR) << "无法写入关键帧位姿文件: " << pose_path;
      }
    }

    // 仅合并到临时地图 (供在线可视化 / 临时 all_map 查看)，不做任何优化/回环
    mergeOptimizedKeyframesToMap();
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

  tbb::concurrent_unordered_map<VoxelKey, PointType, VoxelHash> voxel_map;

  tbb::task_arena arena(lio::effectiveThreads(num_threads_));
  arena.execute([&] {
    tbb::parallel_for(tbb::blocked_range<size_t>(0, cloud->size(), 4096), [&](const tbb::blocked_range<size_t>& range) {
      for (size_t i = range.begin(); i != range.end(); ++i) {
        const auto& pt = (*cloud)[i];
        VoxelKey key{static_cast<int>(std::floor(pt.x / VOXEL_SIZE)), static_cast<int>(std::floor(pt.y / VOXEL_SIZE)),
                     static_cast<int>(std::floor(pt.z / VOXEL_SIZE))};
        voxel_map.emplace(key, pt);  // 已存在则保留首个点, 线程安全
      }
    });
  });

  CloudPtr out(new PointCloudType);
  out->reserve(voxel_map.size());
  for (const auto& kv : voxel_map) {
    out->push_back(kv.second);
  }
  return out;
}

void Frontend::saveMap(const std::string& save_dir) {
  map_->clearAll();
  const auto& kfs = backend_->getKeyFrames();
  const size_t N = kfs.size();

  constexpr size_t kChunk = 256;  // 每批关键帧数, 限制峰值内存
  std::vector<CloudPtr> batch(kChunk);

  for (size_t base = 0; base < N; base += kChunk) {
    const size_t end = std::min(base + kChunk, N);
    const size_t cnt = end - base;

    // 并行: 加载 PCD + 变换到世界系 (I/O 与矩阵乘法并行, 离线不限制核数)
    tbb::parallel_for(tbb::blocked_range<size_t>(base, end), [&](const tbb::blocked_range<size_t>& range) {
      for (size_t i = range.begin(); i != range.end(); ++i) {
        const auto& kf = kfs[i];
        std::string kf_path = save_dir + "kf_" + std::to_string(kf.id) + ".pcd";

        CloudPtr cloud(new PointCloudType());
        if (pcl::io::loadPCDFile<PointType>(kf_path, *cloud) == -1) {
          batch[i - base].reset();
          continue;
        }

        M4f T = M4f::Identity();
        T.block<3, 3>(0, 0) = kf.q.toRotationMatrix().cast<float>();
        T.block<3, 1>(0, 3) = kf.p.cast<float>();

        batch[i - base] = std::make_shared<PointCloudType>();
        pcl::transformPointCloud(*cloud, *batch[i - base], T);
      }
    });

    // 串行合并进体素地图 (addCloud 内部已并行; 多次调用需串行以保证线程安全)
    for (size_t i = 0; i < cnt; ++i) {
      if (batch[i]) {
        map_->addCloud(batch[i]);
      }
    }
  }

  CloudPtr all = map_->getCloud();
  std::string map_path = save_dir + "all_map_1.pcd";
  pcl::io::savePCDFileBinary(map_path, *all);
  LOG(INFO) << "地图保存完成: " << map_path << ", 点数: " << all->size();
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

    // 退化信号
    double ang_vel = 0.0;
    if (i > 0) {
      const auto& prev_kf = kfs[i - 1];
      double dt = kf.timestamp - prev_kf.timestamp;
      if (dt > 0.01) {
        Qd delta_q = prev_kf.q.inverse() * kf.q;
        double angle = 2.0 * std::acos(std::min(1.0, std::abs(delta_q.w())));
        ang_vel = angle / dt;
      }
    }

    LOG(INFO) << "[MapMerge] KF#" << kf.id << " " << " (ang=" << ang_vel << " ndtDR=" << kf.ndt_rot_correction
              << " eff=" << kf.ndt_effective;

    CloudPtr world_cloud(new PointCloudType());
    M4f T = M4f::Identity();
    T.block<3, 3>(0, 0) = kf.q.toRotationMatrix().cast<float>();
    T.block<3, 1>(0, 3) = kf.p.cast<float>();
    pcl::transformPointCloud(*kf.cloud, *world_cloud, T);

    CloudPtr new_cloud(new PointCloudType());
    new_cloud->reserve(world_cloud->size());
    for (const auto& pt : world_cloud->points) {
      if (!map_->hasNearbyCell(pt, NearbyType::CENTER)) {
        new_cloud->push_back(pt);
      }
    }

    map_->addCloud(new_cloud);
    merged_ids.push_back(kf.id);
    LOG(INFO) << "[MapMerge] KF#" << kf.id << " +" << new_cloud->size() << "点, 地图体素:" << map_->size();
  }

  if (!merged_ids.empty()) {
    backend_->markKeyframesMerged(merged_ids);
  }
}

void Frontend::propagateFromTrustedPose(const std::deque<Imu>& imu_datas, double cloud_time, double g_norm,
                                        double acc_scale) {
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
    acc0 *= g_norm * acc_scale;
    acc1 *= g_norm * acc_scale;

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
