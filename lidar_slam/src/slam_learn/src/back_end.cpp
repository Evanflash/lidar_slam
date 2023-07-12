#include "slam_utils.h"

#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/nonlinear/ISAM2.h>

namespace lidarslam{
class BackEnd : public rclcpp::Node{
private:
    // 消息队列
    std::queue<other_msgs::msg::AllCloud> allCloudQueue;

    other_msgs::msg::AllCloud all_cloud;

    // 互斥锁
    std::mutex mtx;

    float transformBefoMapped[6];
    Eigen::Quaternionf q_odom;
    Eigen::Vector3f t_odom;
    float transformAftrMapped[6];

    // 滑动窗口
    std::deque<CloudTypePtr> recentCornerKeyFrames;
    std::deque<CloudTypePtr> recentSurfKeyFrames;
    std::deque<CloudTypePtr> recentGroundKeyFrames;

    // 子图
    CloudTypePtr laserCornerFromMap;
    CloudTypePtr laserSurfFromMap;
    CloudTypePtr laserGroundFromMap;

    // kdtree
    pcl::KdTreeFLANN<PointType>::Ptr kdtreeCornerFromMap;
    pcl::KdTreeFLANN<PointType>::Ptr kdtreeSurfFromMap;
    pcl::KdTreeFLANN<PointType>::Ptr kdtreeGroundFromMap;

    // 新一帧特征点
    CloudTypePtr cornerSharpLast;
    CloudTypePtr groundSurfLast;

    // 新一帧点云
    CloudTypePtr cornerCloud;
    CloudTypePtr surfCloud;
    CloudTypePtr groundCloud;

    // 当前帧下采样
    pcl::VoxelGrid<PointType>::Ptr downSampleCornerCurr;
    pcl::VoxelGrid<PointType>::Ptr downSampleSurfCurr;
    pcl::VoxelGrid<PointType>::Ptr downSampleGroundCurr;

    // 子图下采样滤波器
    pcl::VoxelGrid<PointType>::Ptr downSampleCornerFromMap;
    pcl::VoxelGrid<PointType>::Ptr downSampleSurfFromMap;
    pcl::VoxelGrid<PointType>::Ptr downSampleGroundFromMap;

    // 参数
    bool loop_closure_enabled;
    float surrounding_keyframe_search_radius;
    int surrounding_keyframe_search_num;
    float history_keyframe_search_radius;
    int history_keyframe_search_num;
    float history_search_fitness_score;

    bool isFirstFrame;

    std::thread run_thread;

public:
    BackEnd(const std::string &name)
        : Node(name)
    {
        
        init();
        // 开启子线程
        run_thread = std::thread(&BackEnd::run, this);
    }
    ~BackEnd(){
        run_thread.join();
    }

private:
    /**
     * 子线程
    */
    void run(){
        while(rclcpp::ok()){
            if(allCloudQueue.empty()){
                sleep(1);
                continue;
            }
            mtx.lock();
            all_cloud = allCloudQueue.front();
            allCloudQueue.pop();
            mtx.unlock();

            // 消息拆解
            msgAnalysis();
            // 提取子地图
            extractSurroundingKeyFrames();
            // 图优化
            scan2MapOptimization();
            // 增加约束，保存关键帧
            saveKeyFramesAndFactor();
            // 检测是否存在回环，若存在则更新
            correctPoses();
            // 发布消息
            publish();
            // 重置参数
            clearCloud();
        }
    }

    /**
     * 初始化 
    */
    void init(){

    }

    /**
     * 解析消息
    */
    void msgAnalysis(){
        // 消息转点云
        auto msgToPointCloud = [](CloudTypePtr cloud, std::vector<other_msgs::msg::Point> msg){
            cloud -> clear();
            PointType point;
            for(size_t i = 0; i < msg.size(); ++i){
                point.x = msg[i].x;
                point.y = msg[i].y;
                point.z = msg[i].z;
                point.intensity = msg[i].i;
                cloud -> push_back(point);
            }
        };
        // 获得初始6自由度参数
        for(int i = 0; i < 6; ++i){
            transformBefoMapped[i] = all_cloud.trans_form[i];
        }
        // x y z qx qy qz
        q_odom = (Eigen::AngleAxisf(transformBefoMapped[5], Eigen::Vector3d::UnitZ()) *
                  Eigen::AngleAxisf(transformBefoMapped[4], Eigen::Vector3d::UnitY()) *
                  Eigen::AngleAxisf(transformBefoMapped[3], Eigen::Vector3d::UnitX()));
        t_odom = Eigen::Vector3f(transformBefoMapped[0], transformBefoMapped[1], transformBefoMapped[2]);

        // 获得点云
        msgToPointCloud(cornerSharpLast, all_cloud.corner_sharp);
        msgToPointCloud(cornerCloud, all_cloud.corner_less_sharp);
        msgToPointCloud(surfCloud, all_cloud.surf_less_flat);
        msgToPointCloud(groundSurfLast, all_cloud.ground_flat);
        msgToPointCloud(groundCloud, all_cloud.ground_less_flat);

        // 下采样
        downSamplePointCloud(cornerCloud, downSampleCornerCurr);
        downSamplePointCloud(surfCloud, downSampleSurfCurr);
        downSamplePointCloud(groundCloud, downSampleGroundCurr);
    }
    /**
     * 下采样滤波
    */
    void downSamplePointCloud(CloudTypePtr cloud_in, pcl::VoxelGrid<PointType>::Ptr downSampleFilter){
        CloudTypePtr cloud_tmp(new CloudType());
        downSampleFilter -> setInputCloud(cloud_in);
        cloud_in.reset();
        downSampleFilter -> filter(*cloud_tmp);
        cloud_in = cloud_tmp;
    }

    /**
     * 提取关键帧
    */
    void extractSurroundingKeyFrames(){
        for(size_t i = 0; i < recentCornerKeyFrames.size(); ++i){
            *laserCornerFromMap += *recentCornerKeyFrames[i];
            *laserSurfFromMap += *recentSurfKeyFrames[i];
            *laserGroundFromMap += *recentCornerKeyFrames[i];
        }
        downSamplePointCloud(laserCornerFromMap, downSampleCornerFromMap);
        downSamplePointCloud(laserSurfFromMap, downSampleSurfFromMap);
        downSamplePointCloud(laserGroundFromMap, downSampleGroundFromMap);
    }

    /**
     * 图优化
    */
    void scan2MapOptimization(){
        // 使用初始位姿将点坐标转换
        auto trans = [&](PointType &point){
            Eigen::Vector3f p(point.x, point.y, point.z);
            p = q_odom * p + t_odom;
            point.x = p.x();
            point.y = p.y();
            point.z = p.z();
        };

        if(isFirstFrame){
            isFirstFrame = false;
        }else{
            kdtreeCornerFromMap -> setInputCloud(laserCornerFromMap);
            kdtreeGroundFromMap -> setInputCloud(laserGroundFromMap);

            int cornerPointsNum = cornerSharpLast -> size();
            int groundPointsNum = groundSurfLast -> size();

            PointType pointSel;
            std::vector<int> pointSearchInd;
            std::vector<float> pointSearchSqDis;

            // 地面点优化
            for(size_t opti_counter = 0; opti_counter < 2; ++opti_counter){
                    
                for(int i = 0; i < groundPointsNum; ++i){

                }
            }
            
            // 边缘点优化
            for(size_t opti_counter = 0; opti_counter < 2; ++opti_counter){

                for(int i = 0; i < cornerPointsNum; ++i){

                }
            }
        }
        

    }

    /**
     * 增加新的约束并保存关键帧
    */
    void saveKeyFramesAndFactor(){
        
    }

    /**
     * 存在回环时，更新位姿
    */
    void correctPoses(){

    }

    /**
     * 发布位姿消息
    */
    void publish(){

    }

    /**
     * 重置数据
    */
    void clearCloud(){
        laserCornerFromMap -> clear();
        laserSurfFromMap -> clear();
        laserGroundFromMap -> clear();
    }

}; // class BackEnd

} // namespace lidarslam

int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    auto be = std::make_shared<lidarslam::BackEnd>("back_end");
    RCLCPP_INFO(be -> get_logger(), "\033[1;32m---->\033[0m Started.");
    rclcpp::spin(be);
    rclcpp::shutdown();
    return 0;
}