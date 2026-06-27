#include <glog/logging.h>
#include <pcl/io/pcd_io.h>
#include <filesystem>
#include <iostream>
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
  std::string log_dir = "./src/lio2_slam/log";  // 替换为你的日志目录路径
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

  std::string CONFIG_PATH = "./src/lio2_slam/config/config.yaml";
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

  LOG(INFO) << "前端模块初始化完成";

  TimeSync time_sync(&imu_processor);

  BagIO bag(config);

  LOG(INFO) << "开始处理 bag: " << config.bag_file;

  // int count = 0;

  bag.run(
      [&](const sensor_msgs::msg::Imu& imu_msg) {
        if (imu_processor.processImu(imu_msg)) {
          static bool init_done = false;
          if (!init_done && !frontend->isInitialized()) {
            const auto& states = imu_processor.getStates();
            if (!states.empty()) {
              State init_state;
              init_state.q = states.front().T.unit_quaternion();
              init_state.p = V3d::Zero();
              init_state.v = V3d::Zero();
              init_state.timestamp = states.front().timestamp;
              frontend->init(init_state);
              init_done = true;
              LOG(INFO) << "ESKF 用 IMU 重力对齐姿态初始化: q=" << init_state.q.coeffs().transpose();
            }
          }
          time_sync.pushImu(imu_msg);
        }
      },
      [&](const sensor_msgs::msg::PointCloud2::SharedPtr& cloud) {
        pcl::PointCloud<FullPointType>::Ptr out_cloud(new pcl::PointCloud<FullPointType>());
        cloud_processor.pre_process(cloud, out_cloud);
        time_sync.pushCloud(out_cloud);

        MeasureGroup measures;
        while (time_sync.syncMeasure(measures)) {
          if (measures.imu_datas.size() < 2) {
            LOG(WARNING) << "IMU数据不足,跳过当前scan";
            continue;
          }

          auto deskew_cloud = cloud_processor.process(measures, &imu_processor);
          if (!deskew_cloud || deskew_cloud->empty()) {
            continue;
          }

          // std::string map_path = config.save_map_path + "frame_" + std::to_string(count++) + ".pcd";
          // pcl::io::savePCDFileBinary(map_path, *deskew_cloud);
          // LOG(INFO) << "保存关键帧地图: " << map_path;

          // 短期 IMU 递推
          // 将 imu_processor 的 states_ 队列转换为 vector 传入
          const auto& imu_queue = imu_processor.getStates();
          std::vector<ImuState> imu_vec(imu_queue.begin(), imu_queue.end());
          double cloud_time = measures.lidar_end_time;
          frontend->propagateFromTrustedPose(imu_vec, measures.imu_datas, cloud_time, config.g_norm);

          frontend->process(deskew_cloud);
        }

        // 对齐 IMU 姿态链到 ESKF 估计值
        State latest = frontend->getState();
        imu_processor.resetStates(SE3(latest.q, latest.p), latest.v);
      });

  frontend->saveMap(config.save_map_path + "all_map.pcd");
  LOG(INFO) << "关键帧数: " << frontend->getKeyframes().size();
  LOG(INFO) << "最终位姿: " << frontend->getState().p.transpose();

  google::ShutdownGoogleLogging();
  return 0;
}