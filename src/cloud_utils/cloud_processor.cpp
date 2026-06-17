#include "cloud_utils/cloud_processor.hpp"
#include <glog/logging.h>
#include <pcl/filters/voxel_grid.h>
#include <rcl/time.h>
#include <tbb/blocked_range.h>
#include <tbb/parallel_for.h>
#include <chrono>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

template <typename MessageT>
using PointCloud2ConstIterator = sensor_msgs::PointCloud2ConstIterator<MessageT>;

CloudProcessor::CloudProcessor(AllConfig& config) {
  q_il_ = vecToMat(config.r_imu_lidar).normalized();
  t_il_ = V3d(config.t_imu_lidar.data());
}

void CloudProcessor::pre_process(const sensor_msgs::msg::PointCloud2::SharedPtr& cloud, FullCloudPtr& out_cloud) {
  try {
    PointCloud2ConstIterator<float> iter_x(*cloud, "x");
    PointCloud2ConstIterator<float> iter_y(*cloud, "y");
    PointCloud2ConstIterator<float> iter_z(*cloud, "z");
    PointCloud2ConstIterator<float> iter_i(*cloud, "intensity");
    PointCloud2ConstIterator<double> iter_t(*cloud, "timestamp");

    out_cloud->clear();
    out_cloud->reserve(cloud->width);

    // 获取点云第一个点时间
    uint64_t start =
        static_cast<uint64_t>(cloud->header.stamp.sec) * 1000000000ULL + cloud->header.stamp.nanosec;  // 以ns为单位
    // 获取点云最后一个点时间
    PointCloud2ConstIterator<double> end_t(*cloud, "timestamp");
    end_t += (static_cast<int32_t>(cloud->width) - 1);
    uint64_t end = static_cast<uint64_t>(*end_t);
    // 注意，这里复用了pcl点云类型的stamp和seq，分别用于存储点云起始时间和一帧点云持续时间，单位均为ns
    out_cloud->header.stamp = start;
    out_cloud->header.seq = end - start;

    for (uint32_t point_index = 0; point_index < cloud->width; ++point_index) {
      FullPointType new_pt;
      new_pt.x = *iter_x;
      new_pt.y = *iter_y;
      new_pt.z = *iter_z;
      new_pt.intensity = static_cast<uint8_t>(std::clamp(std::round(*iter_i), 0.0f, 65535.0f));
      new_pt.timestamp = (*iter_t - static_cast<double>(start)) * 1e-9;
      new_pt.ring = 0;

      ++iter_x;
      ++iter_y;
      ++iter_z;
      ++iter_i;
      ++iter_t;

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
  } catch (const std::exception& e) {
    LOG(ERROR) << "错误的激光点字段，请检查雷达类型配置是否正确";
  }
}

CloudPtr CloudProcessor::process(const MeasureGroup& measures, ImuProcessor* imu_processor) {
  CloudPtr output_cloud(new PointCloudType());

  if (!measures.lidar) {
    return output_cloud;
  }
  if (measures.lidar->empty()) {
    return output_cloud;
  }
  if (measures.imu_states.empty()) {
    LOG(WARNING) << "imu states empty, cannot deskew!";
    return output_cloud;
  }
  if (measures.imu_datas.size() < 2) {
    LOG(WARNING) << "imu states not enough";
    return nullptr;
  }

  const auto& cloud = measures.lidar;
  const auto& imu_states = measures.imu_states;

  // scan结束时刻状态
  ImuState end_state = imu_processor->interpolate(measures.lidar_end_time);
  SE3 T_end_inv = end_state.T.inverse();

  output_cloud->resize(cloud->size());

  LOG(INFO) << std::fixed << std::setprecision(9);
  LOG(INFO) << "===== DESKEW =====";
  LOG(INFO) << "scan begin : " << measures.lidar_begin_time;
  LOG(INFO) << "scan end : " << measures.lidar_end_time;
  LOG(INFO) << "cloud size : " << cloud->size();

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
    double pose_time = scan_begin + static_cast<int>(i) * POSE_DT;

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
    Qd q = s1.T.unit_quaternion().slerp(ratio, s2.T.unit_quaternion());
    V3d p = (1.0 - ratio) * s1.T.translation() + ratio * s2.T.translation();

    pose_table_[i].R = q.toRotationMatrix();
    pose_table_[i].t = p;
  }

  // TBB 并行 deskew
  tbb::parallel_for(tbb::blocked_range<size_t>(0, cloud->size(), 1024),

                    [&](const tbb::blocked_range<size_t>& range) {
                      V3d p_lidar;
                      V3d p_imu;
                      V3d p_world;
                      V3d p_end;

                      for (size_t i = range.begin(); i < range.end(); ++i) {
                        const auto& pt = cloud->points[i];

                        auto& new_pt = output_cloud->points[i];

                        // 点时间
                        double pt_offset = pt.timestamp;  // 秒级偏移，无需再乘 1e-9
                        size_t pose_idx = static_cast<size_t>(pt_offset / POSE_DT);
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
  LOG(INFO) << "deskew finished.";

  auto end = std::chrono::system_clock::now();
  std::chrono::duration<double> elapsed_seconds = end - start;
  LOG(INFO) << "deskew time: " << elapsed_seconds.count() * 1000 << "ms";

  return output_cloud;
}