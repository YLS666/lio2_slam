#pragma once

#include <glog/logging.h>
#include <yaml-cpp/yaml.h>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

class YamlConfig {
 public:
  /**
   * @brief Construct a new Yaml Config object
   *
   * @param path 配置文件路径
   */
  explicit YamlConfig(const std::string &path) {
    try {
      std::ifstream fin(path);
      if (!fin.is_open()) {
        LOG(WARNING) << "配置文件路径错误，请检查：" << path;
        valid_ = false;
        return;
      }

      node_ = YAML::Load(fin);

      valid_ = true;
      LOG(INFO) << "加载配置文件路径为：" << path;
    } catch (const YAML::ParserException &e) {
      LOG(ERROR) << "配置文件格式错误：" << e.what();
      valid_ = false;
    }
  }

  /**
   * @brief 是否合法
   *
   * @return true 合法
   * @return false 不合法
   */
  bool valid() const { return valid_; }

  /**
   * @brief key读取
   *
   * @tparam T 模板类型
   * @param key key
   * @param default_value 默认值
   * @return T 返回key值
   */
  template <typename T>
  T get(const std::string &key, const T &default_value) const {
    if (!valid_ || !node_[key]) {
      LOG(WARNING) << "使用默认值：" << key << " = " << toString(default_value);
      return default_value;
    }

    try {
      T value = node_[key].as<T>();
      LOG(INFO) << "读取配置项：" << key << " = " << toString(value);
      return value;
    } catch (const YAML::TypedBadConversion<T> &) {
      LOG(WARNING) << "类型转换错误：" << key << "，使用默认值 " << toString(default_value);
      return default_value;
    }
  }

  /**
   * @brief 多层key读取（支持 \. 作为字面点）
   *
   * @tparam T 模板类型
   * @param path YAML文件路径
   * @param default_value 默认值
   * @return T 返回key值
   */
  template <typename T>
  T getPath(const std::string &path, const T &default_value) const {
    if (!valid_) {
      LOG(WARNING) << "非法配置文件，使用默认值：" << path;
      return default_value;
    }

    std::vector<std::string> keys = splitPath(path);

    const YAML::Node *cur = &node_;

    for (const auto &key : keys) {
      if (!cur->IsMap()) {
        LOG(WARNING) << "配置项不存在：" << path;
        return default_value;
      }

      const YAML::Node next = (*cur)[key];
      if (!next) {
        LOG(WARNING) << "配置项不存在：" << path;
        return default_value;
      }

      cur = &next;  // 继续向下
    }

    try {
      T value = cur->as<T>();
      LOG(INFO) << "配置项：" << path << " = " << toString(value);
      return value;
    } catch (...) {
      LOG(WARNING) << "类型转换错误：" << path << "，使用默认值";
      return default_value;
    }
  }

  /**
   * @brief 打印所有key
   *
   */
  void printAllKeys() const {
    if (!valid_) {
      LOG(WARNING) << "配置文件无效，无法打印 keys";
      return;
    }
    printNode(node_, "");
  }

 private:
  YAML::Node node_;     ///< yaml解析节点
  bool valid_ = false;  ///< 是否合法

  /**
   * @brief 路径分割函数（支持 \. ）
   *
   * @param path 路径
   * @return std::vector<std::string> 分割后字符串
   */
  static std::vector<std::string> splitPath(const std::string &path) {
    std::vector<std::string> result;
    std::string cur;
    bool escape = false;

    for (char c : path) {
      if (escape) {
        cur.push_back(c);  // 字面字符
        escape = false;
      } else if (c == '\\') {
        escape = true;  // 进入转义状态
      } else if (c == '.') {
        result.push_back(cur);
        cur.clear();
      } else {
        cur.push_back(c);
      }
    }

    result.push_back(cur);
    return result;
  }

  /**
   * @brief  类型统一转string
   *
   * @tparam U 模板变量类型
   * @param value key值
   * @return std::string 返回字符串
   */
  template <typename U>
  static std::string toString(const U &value) {
    std::ostringstream oss;
    oss << value;
    return oss.str();
  }

  /**
   * @brief vector 类型转换特化
   *
   * @tparam U 模板变量类型
   * @param vec key值
   * @return std::string 返回字符串
   */
  template <typename U>
  static std::string toString(const std::vector<U> &vec) {
    std::ostringstream oss;
    oss << "[";
    for (size_t i = 0; i < vec.size(); ++i) {
      oss << vec[i];
      if (i + 1 < vec.size()) {
        oss << ", ";
      }
    }
    oss << "]";
    return oss.str();
  }

  /**
   * @brief bool类型转换特化
   *
   * @param value key值
   * @return std::string 返回字符串
   */
  static std::string toString(bool value) { return value ? "true" : "false"; }

  /**
   * @brief 字符串类型转换特化
   *
   * @param value key值
   * @return std::string 返回字符串
   */
  static std::string toString(const std::string &value) { return "\"" + value + "\""; }

  /**
   * @brief 打印所有节点
   *
   * @param node 根节点
   * @param prefix 叶节点key
   */
  static void printNode(const YAML::Node &node, const std::string &prefix) {
    if (node.IsMap()) {
      for (auto it = node.begin(); it != node.end(); ++it) {
        std::string key = it->first.as<std::string>();
        printNode(it->second, prefix.empty() ? key : prefix + "." + key);
      }
    } else if (node.IsSequence()) {
      for (size_t i = 0; i < node.size(); ++i) {
        printNode(node[i], prefix + "[" + std::to_string(i) + "]");
      }
    } else {
      // Scalar
      LOG(INFO) << prefix << " = " << node.as<std::string>();
    }
  }
};

/**
 * @brief 函数模板显式特化
 *
 * @tparam
 * @param key
 * @param default_value
 * @return YAML::Node
 */
template <>
inline YAML::Node YamlConfig::get<YAML::Node>(const std::string &key, const YAML::Node &default_value) const {
  if (!valid_ || !node_[key]) {
    return default_value;
  }
  return node_[key];
}

const std::string LIO2_SLAM_DATA_PATH = std::string(std::getenv("HOME")) + "/data/lio2_slam";

struct AllConfig {
  std::string bag_path;
  int map_id;
  std::string imu_topic;
  std::string lidar_topic;
  std::string odom_topic;
  std::vector<double> t_lidar_imu;
  std::vector<double> r_lidar_imu;
  double g_norm;
  bool is_use_ui;
  int num_threads;

  bool init(std::string config_file_path) {
    if (!std::filesystem::exists(config_file_path)) {
      LOG(ERROR) << "文件不存在," << config_file_path;
      return false;
    }

    YamlConfig yaml_config(config_file_path);
    bag_path = LIO2_SLAM_DATA_PATH + yaml_config.get("bag_path", std::string("/bag/"));
    map_id = yaml_config.get("map_id", 0);
    imu_topic = yaml_config.get("imu_topic", std::string("/livox/imu"));
    lidar_topic = yaml_config.get("lidar_topic", std::string("/livox/lidar"));
    odom_topic = yaml_config.get("odom_topic", std::string("/odom"));
    t_lidar_imu = yaml_config.get("t_lidar_imu", std::vector<double>{-0.011, -0.02329, 0.04412});
    r_lidar_imu = yaml_config.get("r_lidar_imu", std::vector<double>{1, 0, 0, 0, 1, 0, 0, 0, 1});
    g_norm = yaml_config.get("g_norm", 9.80655);
    is_use_ui = yaml_config.get("is_use_ui", true);
    num_threads = yaml_config.get("num_threads", 0);

    return true;
  }
};