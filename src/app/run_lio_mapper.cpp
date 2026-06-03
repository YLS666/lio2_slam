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
  std::shared_ptr<Frontend> frontend = std::make_shared<Frontend>();
  TimeSync time_sync(&imu_processor);

  BagIO bag(config);
  bag.run(
      [&](const sensor_msgs::msg::Imu& imu_msg) {
        if (imu_processor.processImu(imu_msg)) {
          time_sync.pushImu(imu_msg);
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

  return 0;
}