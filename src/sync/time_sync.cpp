#include "sync/time_sync.hpp"

#include <iostream>

TimeSync::TimeSync(ImuProcessor* imu_processor) : imu_processor_(imu_processor) {}

void TimeSync::pushImu(const sensor_msgs::msg::Imu& imu) {
  imu_buffer_.push_back(imu);

  if (imu_buffer_.size() > 20000) {
    imu_buffer_.pop_front();
  }
}

void TimeSync::pushCloud(pcl::PointCloud<FullPointType>::Ptr cloud) {
  cloud_buffer_.push_back(cloud);

  if (cloud_buffer_.size() > 100) {
    cloud_buffer_.pop_front();
  }
}

bool TimeSync::syncMeasure(MeasureGroup& measures) {
  if (cloud_buffer_.empty()) {
    return false;
  }

  auto cloud = cloud_buffer_.front();

  if (cloud->empty()) {
    cloud_buffer_.pop_front();
    return false;
  }

  double lidar_begin_time = cloud->points.front().timestamp * 1e-9;
  double lidar_end_time = cloud->points.back().timestamp * 1e-9;

  if (imu_buffer_.empty()) {
    return false;
  }

  double imu_begin_time = imu_buffer_.front().header.stamp.sec + imu_buffer_.front().header.stamp.nanosec * 1e-9;
  double imu_last_time = imu_buffer_.back().header.stamp.sec + imu_buffer_.back().header.stamp.nanosec * 1e-9;

  // 情况1：lidar太旧
  // 当前imu缓存已经走到更后面了
  // 当前scan永远不可能再被覆盖
  // 直接丢弃scan
  if (lidar_end_time < imu_begin_time) {
    std::cout << "lidar too old, drop this scan" << std::endl;
    cloud_buffer_.pop_front();
    return false;
  }

  // 情况2：imu还没覆盖scan
  // 继续等待imu
  if (imu_last_time < lidar_end_time) {
    return false;
  }

  measures.lidar = cloud;
  measures.lidar_begin_time = lidar_begin_time;
  measures.lidar_end_time = lidar_end_time;
  measures.imu_datas.clear();

  for (const auto& imu : imu_buffer_) {
    double imu_time = imu.header.stamp.sec + imu.header.stamp.nanosec * 1e-9;

    if (imu_time < lidar_begin_time - 0.01) {
      continue;
    }

    if (imu_time > lidar_end_time + 0.01) {
      break;
    }

    measures.imu_datas.push_back(imu);
  }

  // 从ImuProcessor裁剪状态
  const auto& all_states = imu_processor_->getStates();
  auto begin_it = std::lower_bound(all_states.begin(), all_states.end(), lidar_begin_time - 0.01,
                                   [](const ImuState& s, double t) { return s.timestamp < t; });

  auto end_it = std::lower_bound(all_states.begin(), all_states.end(), lidar_end_time + 0.01,
                                 [](const ImuState& s, double t) { return s.timestamp < t; });

  measures.imu_states.assign(begin_it, end_it);

  cloud_buffer_.pop_front();

  std::cout << std::fixed << std::setprecision(9);
  std::cout << "\n===== TIME SYNC =====" << std::endl;
  std::cout << "cloud size : " << cloud->size() << std::endl;
  std::cout << "imu size : " << measures.imu_datas.size() << std::endl;
  std::cout << "imu state size : " << measures.imu_states.size() << std::endl;
  std::cout << "lidar begin : " << lidar_begin_time << std::endl;
  std::cout << "lidar end : " << lidar_end_time << std::endl;

  while (!imu_buffer_.empty()) {
    double imu_time = imu_buffer_.front().header.stamp.sec + imu_buffer_.front().header.stamp.nanosec * 1e-9;

    // 只删除：
    // 比当前最老lidar还旧很多的imu
    if (imu_time < lidar_begin_time - 1.0) {
      imu_buffer_.pop_front();
    } else {
      break;
    }
  }

  return true;
}