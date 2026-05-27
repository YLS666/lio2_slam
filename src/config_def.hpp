#include <string>
#include <vector>
#include "yaml-cpp/yaml.h"

class AllConfig {
 public:
  explicit AllConfig() = default;
  ~AllConfig() = default;
  std::string imu_topic;
  std::string lidar_topic;
  std::string odom_topic;
  std::string map_topic;
  std::string deskew_cloud_topic;
  std::vector<double> t_imu_lidar;
  std::vector<double> r_imu_lidar;

  bool init(std::string config_file_path) {
    YAML::Node config = YAML::LoadFile(config_file_path);
    if (!config["imu_topic"]) {
      return false;
    }
    imu_topic = config["imu_topic"].as<std::string>();

    if (!config["lidar_topic"]) {
      return false;
    }
    lidar_topic = config["lidar_topic"].as<std::string>();

    if (!config["odom_topic"]) {
      return false;
    }
    odom_topic = config["odom_topic"].as<std::string>();

    if (!config["map_topic"]) {
      return false;
    }
    map_topic = config["map_topic"].as<std::string>();

    if (!config["deskew_cloud_topic"]) {
      return false;
    }
    deskew_cloud_topic = config["deskew_cloud_topic"].as<std::string>();

    if (!config["t_imu_lidar"]) {
      return false;
    }
    t_imu_lidar = config["t_imu_lidar"].as<std::vector<double>>();

    if (!config["r_imu_lidar"]) {
      return false;
    }
    r_imu_lidar = config["r_imu_lidar"].as<std::vector<double>>();
    return true;
  };
};