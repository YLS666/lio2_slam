#include <iostream>
#include "cloud_utils/cloud_processor.hpp"
#include "config_def.hpp"
#include "frontend/frontend.hpp"
#include "imu_utils/imu_processor.hpp"
#include "ros_bridge/bag_io.hpp"
#include "sync/time_sync.hpp"

int main() {
  std::string CONFIG_PATH = "/home/yls/test_ros/src/lio2_slam/config/config.yaml";

  AllConfig config;
  config.init(CONFIG_PATH);

  ImuProcessor imu_processor(config);
  CloudProcessor cloud_processor(config);

  auto frontend = std::make_shared<Frontend>();
  frontend->setESKFParams(1.7e-4,  // 陀螺仪噪声密度
                          2.0e-3,  // 加速度计噪声密度
                          0.05,    // 角度观测噪声 (rad, ~2.86°)
                          0.1      // 位置观测噪声 (m)
  );
  frontend->setKeyframeParams(0.5,  // 关键帧距离阈值 (米)
                              0.35  // 关键帧角度阈值 (rad, ~20°)
  );
  // 初始状态 (原点)
  State init_state;
  init_state.q.setIdentity();
  init_state.p.setZero();
  init_state.v.setZero();
  frontend->init(init_state);

  TimeSync time_sync(&imu_processor);

  BagIO bag(config);
  // int frame_count = 0;  // 记录帧数，用于保存map文件名
  bag.run(
      [&](const sensor_msgs::msg::Imu& imu_msg) {
        if (imu_processor.processImu(imu_msg)) {
          time_sync.pushImu(imu_msg);
          // IMU 预测 (前向传播给 ESKF)
          // 从 imu_processor 获取最新状态
          const auto& states = imu_processor.getStates();
          if (states.size() >= 2) {
            const auto& s2 = states.back();
            const auto& s1 = states[states.size() - 2];
            double dt = s2.timestamp - s1.timestamp;

            if (dt > 0 && dt < 0.1) {
              // 传入已减 bias 的 IMU 数据
              frontend->predict(s2.T.so3().log() / dt,    // 从姿态估算角速度
                                Eigen::Vector3d::Zero(),  // 简化: 传0加速度
                                dt);
            }
          }
        }
      },
      [&](const pcl::PointCloud<FullPointType>::Ptr& cloud) {
        pcl::PointCloud<FullPointType>::Ptr out_cloud(new pcl::PointCloud<FullPointType>());
        cloud_processor.pre_process(cloud, out_cloud);  // 先过滤异常点并补全时间戳

        time_sync.pushCloud(out_cloud);

        MeasureGroup measures;

        while (time_sync.syncMeasure(measures)) {
          if (measures.imu_datas.size() < 2) {
            std::cout << "imu不足，跳过当前scan" << std::endl;
            return;
          }

          auto deskew_cloud = cloud_processor.process(measures, &imu_processor);

          frontend->process(deskew_cloud);
        }
      });

  frontend->saveMap(config.save_map_path + "all_map.pcd");
  std::cout << "关键帧数: " << frontend->getKeyframes().size() << std::endl;
  std::cout << "最终位姿: " << frontend->getState().p.transpose() << std::endl;

  return 0;
}