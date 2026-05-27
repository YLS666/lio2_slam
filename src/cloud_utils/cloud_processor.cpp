#include "cloud_processor.hpp"
#include <rcl/time.h>
#include <tbb/blocked_range.h>
#include <tbb/parallel_for.h>
#include <chrono>

void CloudProcessor::setExtrinsic(const Eigen::Quaterniond& q, const Eigen::Vector3d& t) {
  q_il_ = q;
  t_il_ = t;
}

pcl::PointCloud<PointXYZIT>::Ptr CloudProcessor::process(const MeasureGroup& measures, ImuProcessor* imu_processor) {
  pcl::PointCloud<PointXYZIT>::Ptr output_cloud(new pcl::PointCloud<PointXYZIT>());

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
  Eigen::Quaterniond q_end = end_state.q;
  Eigen::Vector3d p_end = end_state.p;
  Eigen::Quaterniond q_end_inv = q_end.inverse();

  output_cloud->resize(cloud->size());

  std::cout << std::fixed << std::setprecision(9);
  std::cout << "\n===== DESKEW =====" << std::endl;
  std::cout << "scan begin : " << measures.lidar_begin_time << std::endl;
  std::cout << "scan end : " << measures.lidar_end_time << std::endl;
  std::cout << "cloud size : " << cloud->size() << std::endl;

  auto start = std::chrono::system_clock::now();

  // 预计算：
  // 每个点对应的 imu 区间索引
  //
  // point_imu_index[i] = k
  // 表示：
  // imu_states[k] <= pt_time < imu_states[k+1]
  std::vector<int> point_imu_index(cloud->size());

  {
    size_t imu_idx = 0;

    for (size_t i = 0; i < cloud->size(); ++i) {
      const auto& pt = cloud->points[i];

      double pt_time = pt.timestamp * 1e-9;

      // 超范围
      if (pt_time < imu_states.front().timestamp) {
        continue;
      }
      if (pt_time > imu_states.back().timestamp) {
        continue;
      }

      while ((imu_idx + 1) < imu_states.size() && imu_states[imu_idx + 1].timestamp < pt_time) {
        imu_idx++;
      }

      if ((imu_idx + 1) >= imu_states.size()) {
        break;
      }

      point_imu_index[i] = static_cast<int>(imu_idx);
    }
  }

  // TBB 并行 deskew
  tbb::parallel_for(tbb::blocked_range<size_t>(0, cloud->size(), 1024),

                    [&](const tbb::blocked_range<size_t>& range) {
                      Eigen::Vector3d p_lidar;
                      Eigen::Vector3d p_imu;
                      Eigen::Vector3d p_world;
                      Eigen::Vector3d p_end_frame;

                      for (size_t i = range.begin(); i < range.end(); ++i) {
                        const auto& pt = cloud->points[i];

                        auto& new_pt = output_cloud->points[i];

                        int idx = point_imu_index[i];

                        const auto& s1 = imu_states[idx];
                        const auto& s2 = imu_states[idx + 1];
                        double pt_time = pt.timestamp * 1e-9;

                        // 插值比例
                        double dt = s2.timestamp - s1.timestamp;
                        double ratio = dt > 1e-6 ? (pt_time - s1.timestamp) / dt : 0.0;
                        // 位姿插值
                        Eigen::Quaterniond q_i = s1.q.slerp(ratio, s2.q);
                        Eigen::Vector3d p_i = (1.0 - ratio) * s1.p + ratio * s2.p;

                        // lidar点
                        p_lidar.x() = pt.x;
                        p_lidar.y() = pt.y;
                        p_lidar.z() = pt.z;

                        // lidar -> imu
                        p_imu = q_il_ * p_lidar + t_il_;
                        // imu -> world
                        p_world = q_i * p_imu + p_i;
                        // world -> end frame
                        p_end_frame = q_end_inv * (p_world - p_end);

                        // 写入输出
                        new_pt.x = static_cast<float>(p_end_frame.x());
                        new_pt.y = static_cast<float>(p_end_frame.y());
                        new_pt.z = static_cast<float>(p_end_frame.z());
                        new_pt.intensity = pt.intensity;
                        new_pt.timestamp = pt.timestamp;
                      }
                    });

  output_cloud->width = output_cloud->size();
  output_cloud->height = 1;
  output_cloud->is_dense = false;  // 包含无效点
  std::cout << "deskew finished." << std::endl;

  auto end = std::chrono::system_clock::now();
  std::chrono::duration<double> elapsed_seconds = end - start;
  std::cout << "deskew time: " << elapsed_seconds.count() * 1000 << "ms" << std::endl;

  return output_cloud;
}