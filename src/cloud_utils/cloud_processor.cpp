#include "cloud_processor.hpp"
#include <pcl/filters/voxel_grid.h>
#include <rcl/time.h>
#include <tbb/blocked_range.h>
#include <tbb/parallel_for.h>
#include <chrono>
#include <sophus/se3.hpp>
#include "cloud_utils/point_type.hpp"

void CloudProcessor::setExtrinsic(const Eigen::Quaterniond& q, const Eigen::Vector3d& t) {
  q_il_ = q;
  t_il_ = t;
}

void CloudProcessor::pre_process(const pcl::PointCloud<FullPointType>::Ptr& cloud,
                                 pcl::PointCloud<FullPointType>::Ptr& out_cloud) {
  out_cloud->clear();
  out_cloud->reserve(cloud->size());

  double start_time = cloud->points.front().timestamp * 1e-9;
  double end_time = cloud->points.back().timestamp * 1e-9;
  double dt = (end_time - start_time) / (cloud->size() - 1);

  for (size_t i = 0; i < cloud->size(); ++i) {
    const auto& pt = cloud->points[i];
    FullPointType new_pt = pt;
    new_pt.timestamp = static_cast<double>(start_time + i * dt) * 1e9;  // 转换回纳秒

    // 过滤异常点（NAN）
    if (!std::isfinite(new_pt.x) || !std::isfinite(new_pt.y) || !std::isfinite(new_pt.z)) {
      continue;
    }

    // 过滤雷达高度以下的点
    if (new_pt.z < 0) {
      continue;
    }

    // 略掉过近的点
    if (new_pt.getVector3fMap().norm() < 0.5) {
      continue;
    }

    out_cloud->push_back(new_pt);
  }

  out_cloud->width = out_cloud->points.size();
  out_cloud->height = 1;
  out_cloud->is_dense = true;
  return;
}

pcl::PointCloud<PointType>::Ptr CloudProcessor::process(const MeasureGroup& measures, ImuProcessor* imu_processor) {
  pcl::PointCloud<PointType>::Ptr output_cloud(new pcl::PointCloud<PointType>());

  if (!measures.lidar) {
    return output_cloud;
  }
  if (measures.lidar->empty()) {
    return output_cloud;
  }
  if (measures.imu_states.empty()) {
    std::cout << "imu states empty, cannot deskew!" << std::endl;
    return output_cloud;
  }
  if (measures.imu_datas.size() < 2) {
    std::cout << "imu states not enough" << std::endl;
    return nullptr;
  }

  const auto& cloud = measures.lidar;
  const auto& imu_states = measures.imu_states;

  // scan结束时刻状态
  ImuState end_state = imu_processor->interpolate(measures.lidar_end_time);
  Sophus::SE3d T_end_inv = end_state.T.inverse();

  output_cloud->resize(cloud->size());

  std::cout << std::fixed << std::setprecision(9);
  std::cout << "\n===== DESKEW =====" << std::endl;
  std::cout << "scan begin : " << measures.lidar_begin_time << std::endl;
  std::cout << "scan end : " << measures.lidar_end_time << std::endl;
  std::cout << "cloud size : " << cloud->size() << std::endl;

  auto start = std::chrono::system_clock::now();

  // pose间隔
  constexpr double POSE_DT = 0.001;
  // scan时间
  double scan_begin = measures.lidar_begin_time;
  double scan_end = measures.lidar_end_time;
  double scan_duration = scan_end - scan_begin;
  // pose数量
  size_t pose_num = static_cast<size_t>(scan_duration / POSE_DT) + 2;
  // resize
  pose_table_.resize(pose_num);
  // imu索引
  size_t imu_idx = 0;

  // 生成稀疏pose table
  for (size_t i = 0; i < pose_num; ++i) {
    double pose_time = scan_begin + i * POSE_DT;

    // 防止超scan结束
    if (pose_time > scan_end) {
      pose_time = scan_end;
    }

    // imu区间
    while ((imu_idx + 1) < imu_states.size() && imu_states[imu_idx + 1].timestamp < pose_time) {
      imu_idx++;
    }

    if ((imu_idx + 1) >= imu_states.size()) {
      break;
    }

    const auto& s1 = imu_states[imu_idx];
    const auto& s2 = imu_states[imu_idx + 1];
    double dt = s2.timestamp - s1.timestamp;
    double ratio = (pose_time - s1.timestamp) / dt;

    // 只做100次slerp！！！
    Eigen::Quaterniond q = s1.T.unit_quaternion().slerp(ratio, s2.T.unit_quaternion());
    Eigen::Vector3d p = (1.0 - ratio) * s1.T.translation() + ratio * s2.T.translation();

    pose_table_[i].R = q.toRotationMatrix();
    pose_table_[i].t = p;
  }

  // TBB 并行 deskew
  tbb::parallel_for(tbb::blocked_range<size_t>(0, cloud->size(), 1024),

                    [&](const tbb::blocked_range<size_t>& range) {
                      Eigen::Vector3d p_lidar;
                      Eigen::Vector3d p_imu;
                      Eigen::Vector3d p_world;
                      Eigen::Vector3d p_end;

                      for (size_t i = range.begin(); i < range.end(); ++i) {
                        const auto& pt = cloud->points[i];

                        auto& new_pt = output_cloud->points[i];

                        // 点时间
                        double pt_time = pt.timestamp * 1e-9;
                        // 查表index
                        size_t pose_idx = static_cast<size_t>((pt_time - scan_begin) / POSE_DT);
                        // 防止越界
                        pose_idx = std::min(pose_idx, pose_table_.size() - 1);
                        // pose
                        const auto& pose = pose_table_[pose_idx];

                        // lidar点
                        p_lidar << pt.x, pt.y, pt.z;
                        // lidar -> imu
                        p_imu = q_il_ * p_lidar + t_il_;
                        // imu -> world
                        p_world = pose.R * p_imu + pose.t;
                        // world -> end frame
                        p_end = T_end_inv * p_world;

                        // 写入输出
                        new_pt.x = static_cast<float>(p_end.x());
                        new_pt.y = static_cast<float>(p_end.y());
                        new_pt.z = static_cast<float>(p_end.z());
                        new_pt.intensity = pt.intensity;
                      }
                    });

  output_cloud->width = output_cloud->size();
  output_cloud->height = 1;
  output_cloud->is_dense = true;  // 包含无效点
  std::cout << "deskew finished." << std::endl;

  auto end = std::chrono::system_clock::now();
  std::chrono::duration<double> elapsed_seconds = end - start;
  std::cout << "deskew time: " << elapsed_seconds.count() * 1000 << "ms" << std::endl;

  return output_cloud;
}