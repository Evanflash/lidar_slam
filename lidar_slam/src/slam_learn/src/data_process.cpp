#include "slam_utils.h"

namespace lidarslam{

class DataProcess : public rclcpp::Node{
private:
    // 初始点云
    SubPointCloud subLaserCloudIn;
    // 发布的点云
    PubPointCloud pubCornerCloudSharp;
    PubPointCloud pubCornerCloudLessSharp;
    PubPointCloud pubSurfCloudFlat;
    PubPointCloud pubSurfCloudLessFlat;
    PubPointCloud pubFullPointCloud;

    CloudTypePtr laserCloudIn;
    CloudTypePtr cornerCloudSharp;
    CloudTypePtr cornerCloudLessSharp;
    CloudTypePtr surfCloudFlat;
    CloudTypePtr surfCloudLessFlat;
    CloudTypePtr fullPointCloud;

public:
    


};

} // lidarslam

int main(){
    other_msgs::msg::Point p;
    
    
    return 0;
}