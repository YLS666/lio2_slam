#pragma once

#include <sensor_msgs/point_cloud2_iterator.hpp>
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

using Imu = sensor_msgs::msg::Imu;
using ImuSharedPtr = Imu::SharedPtr;
using PointCloud2 = sensor_msgs::msg::PointCloud2;
using PointCloud2SharedPtr = PointCloud2::SharedPtr;

template <typename MessageT>
using PointCloud2ConstIterator = sensor_msgs::PointCloud2ConstIterator<MessageT>;
