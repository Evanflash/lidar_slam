#ifndef _LIDAR_SLAM_UTILS_H
#define _LIDAR_SLAM_UTILS_H

#include "other_msgs/msg/point.hpp"
#include "other_msgs/msg/seg_cloud.hpp"

#include <string>
#include <vector>
#include <queue>
#include <mutex>
#include <unordered_set>
#include <boost/circular_buffer.hpp>
#include <Eigen/Dense>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/filter.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <ceres/ceres.h>

namespace lidarslam{


// 重命名
using PointType = pcl::PointXYZI;
using CloudType = pcl::PointCloud<PointType>;
using CloudTypePtr = CloudType::Ptr;

using SubPointCloud = rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr;
using PubPointCloud = rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr;

struct smoothness{
    float curvature;    // 曲率
    int index;          // 序号
};

} // namespace lidarslam

#endif // _LIDAR_SLAM_UTILS_H