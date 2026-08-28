#include <glog/logging.h>
#include <pcl/io/pcd_io.h>
#include <filesystem>
#include <iostream>
#include <string>
#include "config_def.hpp"
#include "postprocess/pose_graph_optimizer.hpp"

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

  std::string slam_data = config.save_map_path;
  std::string pose_file = slam_data + "keyframe_poses.txt";
  std::string cloud_dir = slam_data;
  std::string out_pose_file = slam_data + "optimized_poses.txt";
  std::string out_map_file = slam_data + "all_map_2.pcd";

  // 1. 加载关键帧位姿
  std::deque<KeyFrame> keyframes;
  if (!postprocess::loadKeyframePoses(pose_file, keyframes)) {
    LOG(ERROR) << "加载位姿文件失败";
    return -1;
  }

  // 2. 加载关键帧点云 (回环 ICP + 重建地图需要)
  if (!postprocess::loadKeyframeClouds(cloud_dir, keyframes)) {
    LOG(ERROR) << "加载点云失败";
    return -1;
  }

  // 3. 回环检测
  std::vector<LoopPair> loop_pairs;
  postprocess::detectLoopClosures(keyframes, loop_pairs);

  // 4. 全局位姿图优化 (g2o 内部迭代收敛, 即「再优化」)
  postprocess::globalOptimize(keyframes, loop_pairs);

  // 5. 保存优化后的位姿
  postprocess::saveKeyframePoses(out_pose_file, keyframes);

  // 6. 用优化后的位姿重建最终地图
  auto map = postprocess::rebuildGlobalMap(keyframes, 0.1f);
  pcl::io::savePCDFileBinary(out_map_file, *map);
  LOG(INFO) << "最终地图已保存: " << out_map_file << " 点数: " << map->size();

  google::ShutdownGoogleLogging();
  return 0;
}
