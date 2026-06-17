#include <glog/logging.h>
#include <filesystem>
#include <iostream>
#include <sensor_msgs/msg/detail/point_cloud2__struct.hpp>
#include "cloud_utils/cloud_processor.hpp"
#include "config_def.hpp"
#include "frontend/frontend.hpp"
#include "imu_utils/imu_processor.hpp"
#include "ros_bridge/bag_io.hpp"
#include "sync/time_sync.hpp"
#include "utils/eigen_types.hpp"

int main(int argc, char** argv) {
  (void)argc;

  // std::string log_dir = std::string(std::getenv("HOME")) + "/.kx/log";
  std::string log_dir = "/home/yls/test_ros/src/lio2_slam/log";  // 替换为你的日志目录路径
  if (!std::filesystem::exists(log_dir)) {
    LOG(ERROR) << "日志目录不存在: " << log_dir;
    std::filesystem::create_directories(log_dir);
  }

  google::InitGoogleLogging(argv[0]);
  FLAGS_stderrthreshold = 0;      // 所有级别(INFO)都输出到stderr
  FLAGS_colorlogtostderr = true;  // 终端彩色输出
  FLAGS_log_dir = log_dir;        // 日志文件存放路径
  FLAGS_max_log_size = 20;        // 单个日志文件最大 20MB
  FLAGS_file_line_printf = true;  // 日志中打印文件位置
  FLAGS_rfc3339_format = false;   // 不使用RFC3339格式
  // FLAGS_logbuflevel = -1;       // 可选: 关闭缓存立即刷盘

  std::string CONFIG_PATH = "/home/yls/test_ros/src/lio2_slam/config/config.yaml";
  AllConfig config;
  if (!config.init(CONFIG_PATH)) {
    LOG(ERROR) << "配置文件加载失败: " << CONFIG_PATH;
    return -1;
  }
  LOG(INFO) << "配置文件加载成功: " << CONFIG_PATH;

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

  LOG(INFO) << "前端模块初始化完成";

  TimeSync time_sync(&imu_processor);

  BagIO bag(config);

  LOG(INFO) << "开始处理 bag: " << config.bag_file;

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
              // s1→s2 的相对旋转: R_rel = R1^T * R2
              // 增量角速度: omega = Log(R_rel) / dt
              SE3 T_rel = s1.T.inverse() * s2.T;
              V3d gyr = T_rel.so3().log() / dt;

              V3d acc_body(imu_msg.linear_acceleration.x * config.g_norm, imu_msg.linear_acceleration.y * config.g_norm,
                           imu_msg.linear_acceleration.z * config.g_norm);
              // 传入已减 bias 的 IMU 数据
              frontend->predict(gyr, acc_body, dt, config.g_norm);
            }
          }
        }
      },
      [&](const sensor_msgs::msg::PointCloud2::SharedPtr& cloud) {
        pcl::PointCloud<FullPointType>::Ptr out_cloud(new pcl::PointCloud<FullPointType>());
        cloud_processor.pre_process(cloud, out_cloud);  // 先过滤异常点并补全时间戳

        time_sync.pushCloud(out_cloud);

        MeasureGroup measures;

        while (time_sync.syncMeasure(measures)) {
          if (measures.imu_datas.size() < 2) {
            LOG(WARNING) << "IMU数据不足,跳过当前scan";
            return;
          }

          auto deskew_cloud = cloud_processor.process(measures, &imu_processor);

          frontend->process(deskew_cloud);
        }
      });

  frontend->saveMap(config.save_map_path + "all_map.pcd");
  LOG(INFO) << "关键帧数: " << frontend->getKeyframes().size();
  LOG(INFO) << "最终位姿: " << frontend->getState().p.transpose();

  google::ShutdownGoogleLogging();
  return 0;
}