#include "bag_io.hpp"
#include <pcl_conversions/pcl_conversions.h>
#include "cloud_utils/point_type.hpp"
#include "rclcpp/serialization.hpp"
#include "rclcpp/serialized_message.hpp"
#include "rosbag2_cpp/reader.hpp"

BagIO::BagIO(AllConfig& config) {
  bag_path_ = config.bag_file;
  imu_topic_ = config.imu_topic;
  lidar_topic_ = config.lidar_topic;
}

void BagIO::run(std::function<void(const sensor_msgs::msg::Imu&)> imu_callback,

                std::function<void(const FullCloudPtr&)> cloud_callback) {
  rosbag2_cpp::Reader reader;

  reader.open(bag_path_);

  while (reader.has_next()) {
    auto msg = reader.read_next();

    if (msg->topic_name == imu_topic_) {
      rclcpp::SerializedMessage serialized_msg(*msg->serialized_data);

      sensor_msgs::msg::Imu imu_msg;

      rclcpp::Serialization<sensor_msgs::msg::Imu> serializer;

      serializer.deserialize_message(&serialized_msg, &imu_msg);

      imu_callback(imu_msg);
    }

    else if (msg->topic_name == lidar_topic_) {
      rclcpp::SerializedMessage serialized_msg(*msg->serialized_data);

      sensor_msgs::msg::PointCloud2 cloud_msg;

      rclcpp::Serialization<sensor_msgs::msg::PointCloud2> serializer;

      serializer.deserialize_message(&serialized_msg, &cloud_msg);

      FullCloudPtr cloud(new FullCloudPointType());

      pcl::fromROSMsg(cloud_msg, *cloud);

      cloud_callback(cloud);
    }
  }
}