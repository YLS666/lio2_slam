#include "bag_io.hpp"
#include <pcl_conversions/pcl_conversions.h>
#include <iostream>
#include "rclcpp/serialization.hpp"
#include "rclcpp/serialized_message.hpp"
#include "rosbag2_cpp/reader.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

BagIO::BagIO(const std::string& bag_path) : bag_path_(bag_path) {}

void BagIO::run(std::function<void(const sensor_msgs::msg::Imu&)> imu_callback,

                std::function<void(const pcl::PointCloud<FullPointType>::Ptr&)> cloud_callback) {
  rosbag2_cpp::Reader reader;

  reader.open(bag_path_);

  while (reader.has_next()) {
    auto msg = reader.read_next();

    if (msg->topic_name == "/ns1102/livox/imu") {
      rclcpp::SerializedMessage serialized_msg(*msg->serialized_data);

      sensor_msgs::msg::Imu imu_msg;

      rclcpp::Serialization<sensor_msgs::msg::Imu> serializer;

      serializer.deserialize_message(&serialized_msg, &imu_msg);

      imu_callback(imu_msg);
    }

    else if (msg->topic_name == "/ns1102/livox/lidar") {
      rclcpp::SerializedMessage serialized_msg(*msg->serialized_data);

      sensor_msgs::msg::PointCloud2 cloud_msg;

      rclcpp::Serialization<sensor_msgs::msg::PointCloud2> serializer;

      serializer.deserialize_message(&serialized_msg, &cloud_msg);

      pcl::PointCloud<FullPointType>::Ptr cloud(new pcl::PointCloud<FullPointType>());

      pcl::fromROSMsg(cloud_msg, *cloud);

      cloud_callback(cloud);
    }
  }
}