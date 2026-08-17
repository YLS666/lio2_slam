#include <glog/logging.h>
#include <pcl/io/pcd_io.h>
#include <iostream>
#include <string>
#include "postprocess/pose_graph_optimizer.hpp"

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  FLAGS_stderrthreshold = 0;
  FLAGS_colorlogtostderr = true;

  if (argc < 5) {
    std::cerr << "用法: run_optimizer <pose_file> <cloud_dir> <output_pose_file> <output_map.pcd>\n"
              << "示例: run_optimizer /path/to/map/keyframe_poses.txt /path/to/map/ "
                 "/path/to/map/optimized_poses.txt /path/to/map/optimized_map.pcd\n";
    return -1;
  }

  std::string pose_file = argv[1];
  std::string cloud_dir = argv[2];
  std::string out_pose_file = argv[3];
  std::string out_map_file = argv[4];

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
