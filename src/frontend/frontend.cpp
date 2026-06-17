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
  eskf_->setState(init_state.q, init_state.p, init_state.v);
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
    // 降体素避免地图过密
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

  // 1. 降体素
  CloudPtr ds_cloud(new PointCloudType);
  static pcl::VoxelGrid<PointType> voxel;
  voxel.setInputCloud(cloud);
  voxel.setLeafSize(0.1f, 0.1f, 0.1f);
  voxel.filter(*ds_cloud);
  LOG(INFO) << "ds_cloud size: " << ds_cloud->size();

  // 2. 特征采样
  auto feature_cloud = featureSample(ds_cloud);
  LOG(INFO) << "feature_cloud size: " << feature_cloud->size();

  // 3. 配准
  auto start = std::chrono::steady_clock::now();

  State reg_init = state_;  // 用ESKF预测作为初值
  bool reg_success = reg_->align(feature_cloud, map_.get(), reg_init);
  VLOG(1) << "[Diag] match=" << reg_->getMatchCount() << " inlier=" << reg_->getInlierCount()
          << " pred_p=" << state_.p.transpose() << " reg_p=" << reg_init.p.transpose()
          << " dp=" << (reg_init.p - state_.p).norm();

  static int consecutive_failures = 0;
  if (!reg_success) {
    consecutive_failures++;
    LOG(WARNING) << "配准失败（连续 " << consecutive_failures << " 次）, 维持预测状态";
    if (consecutive_failures > 10) {
      LOG(ERROR) << "连续配准失败过多，系统可能已发散!";
    }
    return state_;
  }
  consecutive_failures = 0;

  // 保存原始观测 (用于下次配准的初值)
  raw_obs_state_ = reg_init;

  auto end = std::chrono::steady_clock::now();
  std::chrono::duration<double> elapsed = end - start;
  LOG(INFO) << "registration time: " << elapsed.count() * 1000.0 << " ms";

  // 4. ESKF观测更新
  eskf_->observePose(reg_init.q, reg_init.p);

  // 获取校正后的状态
  state_ = eskf_->getNominalState();

  // 5. 更新地图
  {
    // 转换当前帧到世界系
    CloudPtr world_cloud(new PointCloudType());
    M4f T = M4f::Identity();
    T.block<3, 3>(0, 0) = state_.q.toRotationMatrix().cast<float>();
    T.block<3, 1>(0, 3) = state_.p.cast<float>();
    pcl::transformPointCloud(*feature_cloud, *world_cloud, T);

    // 只添加新体素（使用 NearbyType::CENTER 快速检查）
    CloudPtr new_cloud(new PointCloudType());
    new_cloud->reserve(world_cloud->size());
    for (const auto& pt : world_cloud->points) {
      if (!map_->hasNearbyPoint(pt, 0.2f, NearbyType::CENTER)) {
        new_cloud->push_back(pt);
      }
    }

    map_->addCloud(new_cloud);
    LOG(INFO) << "map add: " << new_cloud->size();
    LOG(INFO) << "map size: " << map_->size();

    map_->setLocalCenter(state_.p);
  }

  // 6. 后端关键帧管理
  Eigen::Matrix<double, 6, 6> info_mat = Eigen::Matrix<double, 6, 6>::Identity();
  Eigen::Matrix<double, 6, 6> cov = reg_->getCovariance();
  if (cov.norm() > 1e-10 && cov.norm() < 1e10) {
    // 协方差正则化：对角线加小量防止奇异
    Eigen::Matrix<double, 6, 6> cov_reg = cov;
    cov_reg.diagonal() += Eigen::Matrix<double, 6, 1>::Constant(1e-6);
    info_mat = cov_reg.inverse();

    // 裁剪过大元素（防止 Hessian 病态导致 Cholesky 失败）
    constexpr double MAX_INFO = 1e6;
    for (int i = 0; i < 6; ++i) {
      for (int j = 0; j < 6; ++j) {
        if (info_mat(i, j) > MAX_INFO) {
          info_mat(i, j) = MAX_INFO;
        }
        if (info_mat(i, j) < -MAX_INFO) {
          info_mat(i, j) = -MAX_INFO;
        }
      }
    }
  }

  bool is_keyframe = backend_->addKeyFrame(state_, feature_cloud, info_mat);

  if (is_keyframe) {
    backend_->slideWindowOptimize();

    const auto& kfs = backend_->getKeyFrames();
    if (!kfs.empty()) {
      const auto& last_kf = kfs.back();

      double pos_diff = (last_kf.p - state_.p).norm();
      double angle_diff = 2.0 * std::acos(std::min(1.0, std::abs((state_.q.inverse() * last_kf.q).w())));

      if (pos_diff < 2.0 && angle_diff < 0.5) {  // 2m 和 ~30° 以内才接受
        state_.p = last_kf.p;
        state_.q = last_kf.q;
        // 同时重置 ESKF 状态以保持一致性
        eskf_->setState(state_.q, state_.p, state_.v);
        LOG(INFO) << "滑窗优化更新位姿: dp=" << pos_diff << "m, dθ=" << angle_diff << "rad";
      } else {
        LOG(WARNING) << "滑窗优化结果异常，拒绝更新: dp=" << pos_diff << "m, dθ=" << angle_diff << "rad";
      }
    }

    tryLoopClosure();
  }

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
      backend_->globalOptimize(loop_pairs);

      const auto& updated_kfs = backend_->getKeyFrames();
      if (!updated_kfs.empty()) {
        state_.p = updated_kfs.back().p;
        state_.q = updated_kfs.back().q;
        // 更新 ESKF 状态
        eskf_->setState(state_.q, state_.p, state_.v);
      }
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