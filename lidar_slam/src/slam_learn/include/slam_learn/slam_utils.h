#ifndef _LIDAR_SLAM_UTILS_H
#define _LIDAR_SLAM_UTILS_H

#include "other_msgs/msg/point.hpp"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace lidarslam{


// 重命名
using PointType = pcl::PointXYZI;
using CloudType = pcl::PointCloud<PointType>;
using CloudTypePtr = CloudType::Ptr;

using SubPointCloud = rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr;
using PubPointCloud = rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr;

} // namespace lidarslam

#endif // _LIDAR_SLAM_UTILS_H