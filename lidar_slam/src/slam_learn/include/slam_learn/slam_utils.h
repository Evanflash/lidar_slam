#ifndef _LIDAR_SLAM_UTILS_H
#define _LIDAR_SLAM_UTILS_H

#include "other_msgs/msg/point.hpp"
#include "other_msgs/msg/seg_cloud.hpp"
#include "other_msgs/msg/all_cloud.hpp"

#include <string>
#include <vector>
#include <queue>
#include <mutex>
#include <chrono>
#include <unistd.h>
#include <unordered_set>
#include <boost/circular_buffer.hpp>
#include <Eigen/Dense>
#include <thread>
#include <deque>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/filter.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>

#include <ceres/ceres.h>

namespace lidarslam{


// 重命名
using PointType = pcl::PointXYZI;
using CloudType = pcl::PointCloud<PointType>;
using CloudTypePtr = CloudType::Ptr;

using SubPointCloud = rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr;
using PubPointCloud = rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr;

struct smoothness{
    smoothness(float c, int i)
        : curvature(c), index(i){}
    smoothness(){}
    float curvature;    // 曲率
    int index;          // 序号
};

struct pose{
    Eigen::Quaterniond q;
    Eigen::Vector3d t;
};

} // namespace lidarslam

#endif // _LIDAR_SLAM_UTILS_H