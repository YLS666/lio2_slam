#include "sync/time_sync.hpp"
#include <glog/logging.h>
#include "cloud_utils/point_type.hpp"

TimeSync::TimeSync(ImuProcessor* imu_processor) : imu_processor_(imu_processor) {}

void TimeSync::pushImu(const sensor_msgs::msg::Imu& imu) {
  imu_buffer_.push_back(imu);

  if (imu_buffer_.size() > 20000) {
    imu_buffer_.pop_front();
  }
}

void TimeSync::pushCloud(FullCloudPtr cloud) {
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

  double lidar_begin_time = cloud->header.stamp * 1e-9 + cloud->points.front().timestamp;  // s
  double lidar_end_time = cloud->header.stamp * 1e-9 + cloud->points.back().timestamp;

  if (imu_buffer_.empty()) {
    return false;
  }

  double imu_begin_time = imu_buffer_.front().header.stamp.sec + imu_buffer_.front().header.stamp.nanosec * 1e-9;  // s
  double imu_last_time = imu_buffer_.back().header.stamp.sec + imu_buffer_.back().header.stamp.nanosec * 1e-9;

  // 情况1：lidar太旧
  // 当前imu缓存已经走到更后面了
  // 当前scan永远不可能再被覆盖
  // 直接丢弃scan
  if (lidar_end_time < imu_begin_time) {
    LOG(WARNING) << "lidar too old, drop this scan";
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

  LOG(INFO) << std::fixed << std::setprecision(9);
  LOG(INFO) << "===== TIME SYNC =====";
  LOG(INFO) << "cloud size : " << cloud->size();
  LOG(INFO) << "imu size : " << measures.imu_datas.size();
  LOG(INFO) << "imu state size : " << measures.imu_states.size();
  LOG(INFO) << "lidar begin : " << lidar_begin_time;
  LOG(INFO) << "lidar end : " << lidar_end_time;

  // 删除所有时间戳 < lidar_end_time + 0.01 的IMU (已消费的帧)
  while (!imu_buffer_.empty()) {
    double imu_time = imu_buffer_.front().header.stamp.sec + imu_buffer_.front().header.stamp.nanosec * 1e-9;

    if (imu_time < lidar_end_time + 0.01) {
      imu_buffer_.pop_front();
    } else {
      break;
    }
  }

  return true;
}