#include <glog/logging.h>
#include "config_def.hpp"
#include "pose_graph_opti/pose_graph_optimizer.hpp"

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

  std::string config_path = LIO2_SLAM_DATA_PATH + "/config/config.yaml";
  AllConfig config;
  if (!config.init(config_path)) {
    LOG(ERROR) << "配置文件加载失败: " << config_path;
    return -1;
  }
  LOG(INFO) << "配置文件加载成功: " << config_path;

  std::string map_path = LIO2_SLAM_DATA_PATH + "/map/map_" + std::to_string(config.map_id);

  pose_graph_opti pg_opti(map_path);
  pg_opti.run();

  google::ShutdownGoogleLogging();
  return 0;
}
