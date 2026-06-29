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
  if (lidar_end_time < imu_begin_time) {
    LOG(WARNING) << "lidar too old, drop this scan";
    cloud_buffer_.pop_front();
    return false;
  }

  // 情况2：imu还没覆盖scan
  if (imu_last_time < lidar_end_time) {
    return false;
  }

  measures.lidar = cloud;
  cloud_buffer_.pop_front();
  measures.lidar_begin_time = lidar_begin_time;
  measures.lidar_end_time = lidar_end_time;
  measures.imu_datas.clear();

  // 保留： scan开始前一个imu - scan结束后一个imu
  bool found_begin = false;

  for (size_t i = 0; i < imu_buffer_.size(); ++i) {
    const auto& imu = imu_buffer_[i];

    double imu_time = imu.header.stamp.sec + imu.header.stamp.nanosec * 1e-9;

    if (!found_begin) {
      if (imu_time >= lidar_begin_time) {
        // 保留scan开始前一个imu
        if (i > 0) {
          measures.imu_datas.push_back(imu_buffer_[i - 1]);
        }

        found_begin = true;
      } else {
        continue;
      }
    }

    measures.imu_datas.push_back(imu);

    // 保留scan结束之后一个imu
    if (imu_time > lidar_end_time) {
      break;
    }
  }

  if (measures.imu_datas.size() < 2) {
    LOG(WARNING) << "Not enough imu data.";
    return false;
  }

  double measure_start_time =
      measures.imu_datas.front().header.stamp.sec + measures.imu_datas.front().header.stamp.nanosec * 1e-9;  // s
  double measure_end_time =
      measures.imu_datas.back().header.stamp.sec + measures.imu_datas.back().header.stamp.nanosec * 1e-9;  // s

  // 从ImuProcessor裁剪状态
  const auto& all_states = imu_processor_->getStates();
  auto begin_it = std::lower_bound(all_states.begin(), all_states.end(), measure_start_time,
                                   [](const ImuState& s, double t) { return s.timestamp < t; });
  if (begin_it != all_states.begin()) {
    --begin_it;
  }

  auto end_it = std::upper_bound(all_states.begin(), all_states.end(), measure_end_time,
                                 [](double t, const ImuState& s) { return t < s.timestamp; });

  if (end_it != all_states.end()) {
    ++end_it;
  }

  measures.imu_states.assign(begin_it, end_it);

  LOG(INFO) << std::fixed << std::setprecision(9);
  LOG(INFO) << "===== TIME SYNC =====";
  LOG(INFO) << "cloud size : " << cloud->size();
  LOG(INFO) << "imu size : " << measures.imu_datas.size();
  LOG(INFO) << "imu state size : " << measures.imu_states.size();
  LOG(INFO) << std::fixed << std::setprecision(9) << "lidar begin : " << lidar_begin_time
            << "\nlidar end : " << lidar_end_time;
  LOG(INFO) << std::fixed << std::setprecision(9) << "imu begin : " << imu_begin_time
            << "\nlast imu : " << imu_last_time;
  LOG(INFO) << std::fixed << std::setprecision(9) << "measure start : " << measure_start_time
            << "\nmeasure end : " << measure_end_time;

  // 删除已经消费的IMU，但保留最后一个IMU，供下一帧作为scan开始之前那个IMU
  while (imu_buffer_.size() > 1) {
    double next_time = imu_buffer_[1].header.stamp.sec + imu_buffer_[1].header.stamp.nanosec * 1e-9;

    if (next_time <= lidar_end_time) {
      imu_buffer_.pop_front();
    } else {
      break;
    }
  }

  return true;
}