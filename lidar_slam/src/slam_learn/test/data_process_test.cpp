#include "other_msgs/msg/point.hpp"
#include "other_msgs/msg/seg_cloud.hpp"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

class DataProcessTest : public rclcpp::Node{
public:
    DataProcessTest(const std::string &name)
        : Node(name)
    {
        subSegMsg = this -> create_subscription<other_msgs::msg::SegCloud>(
            "/seg_cloud", 1, std::bind(&DataProcessTest::publishPointCloud, this, std::placeholders::_1));
        pubSegmentPointCloud = this -> create_publisher<sensor_msgs::msg::PointCloud2>(
            "/segmentcloud", 1);
        pubGroundPointCloud = this -> create_publisher<sensor_msgs::msg::PointCloud2>(
            "/groundcloud", 1);
    }
    void publishPointCloud(const other_msgs::msg::SegCloud &seg_msg){
        pcl::PointCloud<pcl::PointXYZI>::Ptr segmentCloud(new pcl::PointCloud<pcl::PointXYZI>());
        segmentCloud -> reserve(seg_msg.seg_cloud.size());
        pcl::PointCloud<pcl::PointXYZI>::Ptr groundCloud(new pcl::PointCloud<pcl::PointXYZI>());
        groundCloud -> reserve(seg_msg.ground_cloud.size());
        for(size_t i = 0; i < seg_msg.seg_cloud.size(); ++i){
            pcl::PointXYZI point;
            point.x = seg_msg.seg_cloud[i].x;
            point.y = seg_msg.seg_cloud[i].y;
            point.z = seg_msg.seg_cloud[i].z;
            point.intensity = seg_msg.seg_cloud[i].i;
            segmentCloud -> push_back(point);
        }
        for(size_t i = 0; i < seg_msg.ground_cloud.size(); ++i){
            pcl::PointXYZI point;
            point.x = seg_msg.ground_cloud[i].x;
            point.y = seg_msg.ground_cloud[i].y;
            point.z = seg_msg.ground_cloud[i].z;
            point.intensity = seg_msg.ground_cloud[i].i;
            groundCloud -> push_back(point);
        }

        auto pub = [](pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, 
                      rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubMsg){
            sensor_msgs::msg::PointCloud2 msg;
            pcl::toROSMsg(*cloud, msg);
            msg.header.frame_id = "velodyne";
            if(pubMsg -> get_subscription_count() != 0){
                pubMsg -> publish(msg);
            }
        };
        pub(segmentCloud, pubSegmentPointCloud);
        pub(groundCloud, pubGroundPointCloud);
    }

private:
    rclcpp::Subscription<other_msgs::msg::SegCloud>::SharedPtr subSegMsg;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubSegmentPointCloud;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubGroundPointCloud;
};


int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    auto dpt = std::make_shared<DataProcessTest>("data_process_test");
    rclcpp::spin(dpt);
    rclcpp::shutdown();
    return 0;
}