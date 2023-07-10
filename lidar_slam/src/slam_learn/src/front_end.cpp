#include "slam_utils.h"
#include "lidar_factor.hpp"

namespace lidarslam{

static const std::string PARAM_VERTICAL_SCANS = "param.vertical_scans";
static const std::string PARAM_EDGE_THRESHOLD = "param.edge_threshold";
static const std::string PARAM_SURF_THRESHOLD = "param.surf_threshold";
static const std::string PARAM_DISTANCE = "param.distance";
static double para_q[4] = {0.0, 0.0, 0.0, 1.0};
static double para_t[3] = {0.0, 0.0, 0.0};

class FrontEnd : public rclcpp::Node{
private:
    // 处理后的点云
    rclcpp::Subscription<other_msgs::msg::SegCloud>::SharedPtr subSegMsg;
    rclcpp::TimerBase::SharedPtr loop_rate;

    // 消息
    other_msgs::msg::SegCloud seg_msg;

    // 当前点特征点集
    CloudTypePtr cornerSharp;
    CloudTypePtr cornerLessSharp;
    CloudTypePtr surfFlat;
    CloudTypePtr surfLessFlat;
    CloudTypePtr groundSurfFlat;
    CloudTypePtr groundSurfLessFlat;
    // 上一时刻点特征点集
    CloudTypePtr cornerLast;
    CloudTypePtr surfLast;
    CloudTypePtr groundSurfLast;

    // 曲率
    std::vector<smoothness> segmentCurvature;
    std::vector<smoothness> groundCurvature;
    // 标记周围是否被选取
    std::vector<int> segmentNeighborPicked;
    std::vector<int> groundNeighborPicked;

    // kd tree
    pcl::KdTreeFLANN<PointType>::Ptr kdtreeCornerLast;
    pcl::KdTreeFLANN<PointType>::Ptr kdtreeSurfLast;
    pcl::KdTreeFLANN<PointType>::Ptr kdtreeGroundLast;

    // 参数
    int vertical_scans;
    float edgeThreshold;
    float surfThreshold;
    float nearest_feature_dist_sqr;

    bool isFirstFrame;

    Eigen::Map<Eigen::Quaterniond> q_last_curr;
    Eigen::Map<Eigen::Vector3d> t_last_curr;

    Eigen::Quaterniond q_w_curr;
    Eigen::Vector3d t_w_curr;

    // 队列
    std::queue<other_msgs::msg::SegCloud> segCloudQueue;
    std::mutex mtx;

    // 路径
    nav_msgs::msg::Path path;

    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubPath;
    PubPointCloud pubtest;
    PubPointCloud pubtest2;

public:
    FrontEnd(const std::string &name)
        : Node(name),
          q_last_curr(para_q),
          t_last_curr(para_t),
          q_w_curr(1, 0, 0, 0),
          t_w_curr(0, 0, 0)
    {
        subSegMsg = this -> create_subscription<other_msgs::msg::SegCloud>(
            "/seg_cloud", 1, std::bind(&FrontEnd::pushQueue, this, std::placeholders::_1));
        
        loop_rate = this -> create_wall_timer(std::chrono::milliseconds(100), std::bind(&FrontEnd::run, this));

        pubPath = this -> create_publisher<nav_msgs::msg::Path>("/path", 1);
        pubtest = this -> create_publisher<sensor_msgs::msg::PointCloud2>("/test", 1);
        pubtest2 = this -> create_publisher<sensor_msgs::msg::PointCloud2>("/test2", 1);
        // 声明参数
        this -> declare_parameter(PARAM_VERTICAL_SCANS, vertical_scans);
        this -> declare_parameter(PARAM_EDGE_THRESHOLD, edgeThreshold);
        this -> declare_parameter(PARAM_SURF_THRESHOLD, surfThreshold);
        this -> declare_parameter(PARAM_DISTANCE, nearest_feature_dist_sqr);

        // 更新参数
        if(!this -> get_parameter(PARAM_VERTICAL_SCANS, vertical_scans)){
            RCLCPP_WARN(this -> get_logger(), "data_process_node: %s not found", PARAM_VERTICAL_SCANS.c_str());
        }
        if(!this -> get_parameter(PARAM_EDGE_THRESHOLD, edgeThreshold)){
            RCLCPP_WARN(this -> get_logger(), "data_process_node: %s not found", PARAM_EDGE_THRESHOLD.c_str());
        }
        if(!this -> get_parameter(PARAM_SURF_THRESHOLD, surfThreshold)){
            RCLCPP_WARN(this -> get_logger(), "data_process_node: %s not found", PARAM_SURF_THRESHOLD.c_str());
        }
        if(!this -> get_parameter(PARAM_DISTANCE, nearest_feature_dist_sqr)){
            RCLCPP_WARN(this -> get_logger(), "data_process_node: %s not found", PARAM_DISTANCE.c_str());
        }

        // 初始化
        init();
    }

    /**
     * 初始化
    */
    void init(){
        cornerSharp.reset(new CloudType());
        cornerLessSharp.reset(new CloudType());
        surfFlat.reset(new CloudType());
        surfLessFlat.reset(new CloudType());
        groundSurfFlat.reset(new CloudType());
        groundSurfLessFlat.reset(new CloudType());
        cornerLast.reset(new CloudType());
        surfLast.reset(new CloudType());
        groundSurfLast.reset(new CloudType());

        kdtreeCornerLast.reset(new pcl::KdTreeFLANN<PointType>());
        kdtreeSurfLast.reset(new pcl::KdTreeFLANN<PointType>());
        kdtreeGroundLast.reset(new pcl::KdTreeFLANN<PointType>());

        isFirstFrame = true;
    }

    /**
     * 回调函数
    */
    void pushQueue(const other_msgs::msg::SegCloud &msg){
        mtx.lock();
        segCloudQueue.push(msg);
        mtx.unlock();
    }

    /**
     * 循环运行
    */
    void run(){
        if(segCloudQueue.empty()){
            return;
        }
        mtx.lock();
        seg_msg = segCloudQueue.front();
        segCloudQueue.pop();
        mtx.unlock();
        // 变量重置
        resetParameters();
        // 计算曲率
        calculateSmoothness();
        // 提取特征点
        extractFeatures();
        // 里程计
        std::chrono::time_point<std::chrono::system_clock> start = std::chrono::system_clock::now();
        odometry();
        std::chrono::time_point<std::chrono::system_clock> end = std::chrono::system_clock::now();
        std::chrono::duration<double> elapsed_seconds = end - start;
        RCLCPP_INFO(this -> get_logger(), "odometry time: %f", elapsed_seconds.count() * 1000);
        // 发布消息
        publish();
    }

    /**
     * 重置变量
    */
    void resetParameters(){
        // 点云
        cornerSharp -> clear();
        cornerLessSharp -> clear();
        surfFlat -> clear();
        surfLessFlat -> clear();
        groundSurfFlat -> clear();
        groundSurfLessFlat -> clear();

        // 曲率
        segmentCurvature.clear();
        segmentCurvature.resize(seg_msg.seg_cloud.size());
        groundCurvature.clear();
        groundCurvature.resize(seg_msg.ground_cloud.size());
        
        segmentNeighborPicked.clear();
        segmentNeighborPicked.resize(seg_msg.seg_cloud.size(), 0);
        groundNeighborPicked.clear();
        groundNeighborPicked.resize(seg_msg.ground_cloud.size(), 0);
    }

    /**
     * 计算曲率
    */
    void calculateSmoothness(){
        auto cs = [](std::vector<other_msgs::msg::Point> &points,
                     std::vector<float> &ranges,
                     std::vector<smoothness> &curvatures){
            int size = points.size();
            for(int i = 5; i < size - 5; ++i){
                float diffRange = ranges[i - 1] + ranges[i - 2] + ranges[i - 3] + ranges[i - 4] + ranges[i - 5] +
                                  ranges[i + 1] + ranges[i + 2] + ranges[i + 3] + ranges[i + 4] + ranges[i + 5] -
                                  ranges[i] * 10;
                smoothness sns;
                sns.curvature = diffRange * diffRange;
                sns.index = i;
                curvatures[i] = sns;
            }
        };
        cs(seg_msg.ground_cloud, seg_msg.ground_range, groundCurvature);
        cs(seg_msg.seg_cloud, seg_msg.seg_range, segmentCurvature);
    }

    /**
     * 提取边缘点和平面点
    */
    void extractFeatures(){
        // 排序
        auto cmp = [](const smoothness &l, const smoothness &r){
            return l.curvature < r.curvature;
        };

        // 从Point消息转换为PointXYZI
        auto tran = [](const other_msgs::msg::Point &pmsg, PointType &p){
            p.x = pmsg.x;
            p.y = pmsg.y;
            p.z = pmsg.z;
            p.intensity = pmsg.i;
        };

        PointType point;

        // 分割点云特征提取
        for(int i = 0; i < vertical_scans; ++i){
            for(int j = 0; j < 6; ++j){
                int sp = seg_msg.seg_ring_str_ind[i] + 
                    (seg_msg.seg_ring_end_ind[i] - seg_msg.seg_ring_str_ind[i]) * j / 6;
                int ep = seg_msg.seg_ring_str_ind[i] +
                    (seg_msg.seg_ring_end_ind[i] - seg_msg.seg_ring_str_ind[i]) * (j + 1) / 6 - 1;
                
                if(sp >= ep) continue;

                std::sort(segmentCurvature.begin() + sp, segmentCurvature.begin() + ep + 1, cmp);
                // 从分割点云中提取边缘点和平面点
                // 边缘点
                int largestPickedNum = 0;
                for(int k = ep; k >= sp; --k){
                    int curInd = segmentCurvature[k].index;
                    if(segmentNeighborPicked[curInd] == 0 && 
                       segmentCurvature[k].curvature > edgeThreshold){
                        ++largestPickedNum;
                        if(largestPickedNum <= 2){
                            tran(seg_msg.seg_cloud[curInd], point);
                            cornerSharp -> push_back(point);
                            cornerLessSharp -> push_back(point);
                        }else if(largestPickedNum <= 20){
                            tran(seg_msg.seg_cloud[curInd], point);
                            cornerLessSharp -> push_back(point);
                        }else{
                            break;
                        }

                        // 标记被选取的关键点与离该关键点近的点
                        segmentNeighborPicked[curInd] = 1;
                        for(int l = 1; l <= 5; ++l){
                            float dx = seg_msg.seg_cloud[curInd + l].x -
                                       seg_msg.seg_cloud[curInd + l - 1].x;
                            float dy = seg_msg.seg_cloud[curInd + l].y -
                                       seg_msg.seg_cloud[curInd + l - 1].y;
                            float dz = seg_msg.seg_cloud[curInd + l].z -
                                       seg_msg.seg_cloud[curInd + l - 1].z;
                            
                            if(dx * dx + dy * dy + dz * dz > 0.05) break;
                            segmentNeighborPicked[curInd + l] = 1;
                        }
                        for(int l = -1; l >= -5; --l){
                            float dx = seg_msg.seg_cloud[curInd + l].x -
                                       seg_msg.seg_cloud[curInd + l + 1].x;
                            float dy = seg_msg.seg_cloud[curInd + l].y -
                                       seg_msg.seg_cloud[curInd + l + 1].y;
                            float dz = seg_msg.seg_cloud[curInd + l].z -
                                       seg_msg.seg_cloud[curInd + l + 1].z;
                            
                            if(dx * dx + dy * dy + dz * dz > 0.05) break;
                            segmentNeighborPicked[curInd + l] = 1;
                        }

                    }
                }
                
                // 平面点
                int smallestPickedNum = 1;
                for(int k = sp; k <= ep; ++k){
                    int curInd = segmentCurvature[k].index;
                    if(segmentNeighborPicked[curInd] == 0 && 
                       segmentCurvature[k].curvature < surfThreshold){
                        tran(seg_msg.seg_cloud[curInd], point);
                        surfFlat -> push_back(point);

                        if(++smallestPickedNum >= 1){
                            break;
                        }
                        
                        // 标记选取的关键点与离该关键点近的点
                        segmentNeighborPicked[curInd] = 1;
                        for(int l = 1; l <= 5; ++l){
                            float dx = seg_msg.seg_cloud[curInd + l].x -
                                       seg_msg.seg_cloud[curInd + l - 1].x;
                            float dy = seg_msg.seg_cloud[curInd + l].y -
                                       seg_msg.seg_cloud[curInd + l - 1].y;
                            float dz = seg_msg.seg_cloud[curInd + l].z -
                                       seg_msg.seg_cloud[curInd + l - 1].z;
                            
                            if(dx * dx + dy * dy + dz * dz > 0.05) break;
                            segmentNeighborPicked[curInd + l] = 1;
                        }
                        for(int l = -1; l >= -5; --l){
                            float dx = seg_msg.seg_cloud[curInd + l].x -
                                       seg_msg.seg_cloud[curInd + l + 1].x;
                            float dy = seg_msg.seg_cloud[curInd + l].y -
                                       seg_msg.seg_cloud[curInd + l + 1].y;
                            float dz = seg_msg.seg_cloud[curInd + l].z -
                                       seg_msg.seg_cloud[curInd + l + 1].z;
                            
                            if(dx * dx + dy * dy + dz * dz > 0.05) break;
                            segmentNeighborPicked[curInd + l] = 1;
                        }
                    }
                }    
            }
        }

        

        for(int i = 0; i < int(seg_msg.seg_cloud.size()); ++i){
            tran(seg_msg.seg_cloud[i], point);
            surfLessFlat -> push_back(point);
        }
        
        sensor_msgs::msg::PointCloud2 msg;
        pcl::toROSMsg(*surfLessFlat, msg);
        msg.header.frame_id = "map";
        pubtest -> publish(msg);

        pcl::toROSMsg(*surfFlat, msg);
        msg.header.frame_id = "map";
        pubtest2 -> publish(msg);
        
        // 地面点特征提取
        for(int i = 0; i < vertical_scans; ++i){
             for(int j = 0; j < 6; ++j){
                int sp = seg_msg.grd_ring_str_ind[i] + 
                    (seg_msg.grd_ring_end_ind[i] - seg_msg.grd_ring_str_ind[i]) * j / 6;
                int ep = seg_msg.grd_ring_str_ind[i] +
                    (seg_msg.grd_ring_end_ind[i] - seg_msg.grd_ring_str_ind[i]) * (j + 1) / 6 - 1;

                if(sp >= ep) continue;

                std::sort(groundCurvature.begin() + sp, groundCurvature.begin() + ep + 1, cmp);

                // 平面点
                int smallestPickedNum = 1;
                for(int k = sp; k <= ep; ++k){
                    int curInd = groundCurvature[k].index;
                    if(groundNeighborPicked[curInd] == 0 && 
                       groundCurvature[k].curvature < surfThreshold){
                        tran(seg_msg.ground_cloud[curInd], point);
                        groundSurfFlat -> push_back(point);

                        if(++smallestPickedNum >= 1){
                            break;
                        }
                        
                        groundNeighborPicked[curInd] = 1;
                        for(int l = 1; l <= 5; ++l){
                            float dx = seg_msg.ground_cloud[curInd + l].x -
                                       seg_msg.ground_cloud[curInd + l - 1].x;
                            float dy = seg_msg.ground_cloud[curInd + l].y -
                                       seg_msg.ground_cloud[curInd + l - 1].y;
                            float dz = seg_msg.ground_cloud[curInd + l].z -
                                       seg_msg.ground_cloud[curInd + l - 1].z;
                            
                            if(dx * dx + dy * dy + dz * dz > 0.05) break;
                            groundNeighborPicked[curInd + l] = 1;
                        }
                        for(int l = -1; l >= -5; --l){
                            float dx = seg_msg.ground_cloud[curInd + l].x -
                                       seg_msg.ground_cloud[curInd + l + 1].x;
                            float dy = seg_msg.ground_cloud[curInd + l].y -
                                       seg_msg.ground_cloud[curInd + l + 1].y;
                            float dz = seg_msg.ground_cloud[curInd + l].z -
                                       seg_msg.ground_cloud[curInd + l + 1].z;
                            
                            if(dx * dx + dy * dy + dz * dz > 0.05) break;
                            groundNeighborPicked[curInd + l] = 1;
                        }
                    }
                }    
            }
        }

        for(int i = 0; i < int(seg_msg.ground_cloud.size()); ++i){
            tran(seg_msg.ground_cloud[i], point);
            groundSurfLessFlat -> push_back(point);
        }

        // 使用ransac算法获得地面的法向量
        pcl::SampleConsensusModelPlane<PointType>::Ptr model_p(
            new pcl::SampleConsensusModelPlane<PointType>(groundSurfLessFlat));
        pcl::RandomSampleConsensus<PointType> ransac(model_p);
        ransac.setDistanceThreshold(0.1);
        ransac.computeModel();
        Eigen::VectorXf coeffs;
        ransac.getModelCoefficients(coeffs);
        
    }

    /**
     * 里程计
    */
    void odometry(){
        // 计算俩点之间的距离
        auto dis = [](const PointType &p1, const PointType &p2){
            double d = (p1.x - p2.x) * (p1.x - p2.x) +
                       (p1.y - p2.y) * (p1.y - p2.y) +
                       (p1.z - p2.z) * (p1.z - p2.z);
            return d;
        };
        // PointXYZI -> Vector3d
        auto pointxyziToVector3d = [](const PointType &point){
            Eigen::Vector3d v(point.x, point.y, point.z);
            return v;
        };
        // 使用初始位姿插值
        auto transform = [&](const PointType *const pi){
            Eigen::Quaterniond q_point_last = Eigen::Quaterniond::Identity().slerp(1.0, q_last_curr);
            Eigen::Vector3d t_point_last = 1.0 * t_last_curr;
            Eigen::Vector3d point(pi -> x, pi -> y, pi -> z);
            Eigen::Vector3d un_point = q_point_last * point + t_point_last;
            PointType p(un_point.x(), un_point.y(), un_point.z());
            return p;
        };

        // 降采样
        auto downSample = [](CloudTypePtr cloud_in){
            CloudTypePtr tmp(new CloudType());
            pcl::VoxelGrid<PointType> downSampleFilter;
            downSampleFilter.setLeafSize(0.5, 0.5, 0.5);
            downSampleFilter.setInputCloud(cloud_in);
            downSampleFilter.filter(*tmp);
            *cloud_in = *tmp;
        };

        if(isFirstFrame == true){
            isFirstFrame = false;
        }else{
            int cornerSharpNum = cornerSharp -> size();
            int surfFlatNum = surfFlat -> size();
            int groundFlatNum = groundSurfFlat -> size();

            // 两次优化
            for(size_t opti_counter = 0; opti_counter < 1; ++opti_counter){
                ceres::LossFunction *loss_function = new ceres::HuberLoss(0.1);
                ceres::Manifold *q_manifold = new ceres::EigenQuaternionManifold();
                ceres::Problem::Options problem_options;

                ceres::Problem problem(problem_options);
                problem.AddParameterBlock(para_q, 4, q_manifold);
                problem.AddParameterBlock(para_t, 3);

                PointType pointSel;
                std::vector<int> pointSearchInd;
                std::vector<float> pointSearchSqDis;

                // 分割点云的边缘点
                for(int i = 0; i < cornerSharpNum; ++i){
                    pointSel = transform(&cornerSharp -> points[i]);
                    kdtreeCornerLast -> nearestKSearch(pointSel, 1, pointSearchInd, pointSearchSqDis);
                    int closestPointInd = -1, minPointInd2 = -1;
                    if(pointSearchSqDis[0] < nearest_feature_dist_sqr){
                        closestPointInd = pointSearchInd[0];
                        int closestPointScanID = int(cornerLast -> points[closestPointInd].intensity);

                        double minPointSqDis2 = nearest_feature_dist_sqr;
                        for(int j = closestPointInd + 1; j < int(cornerLast -> size()); ++j){
                            // 是同一条激光线
                            if(int(cornerLast -> points[j].intensity) <= closestPointScanID)
                                continue;
                            // 距离太远
                            if(int(cornerLast -> points[j].intensity) > closestPointScanID + 2.5)
                                break;

                            // 计算距离
                            double pointSqDis = dis(cornerLast -> points[j], pointSel);
                            if(pointSqDis < minPointSqDis2){
                                minPointSqDis2 = pointSqDis;
                                minPointInd2 = j;
                            }
                        }

                        for(int j = closestPointInd; j >= 0; --j){
                            // 是同一条激光线
                            if(int(cornerLast -> points[j].intensity) >= closestPointScanID)
                                continue;
                            // 距离太远
                            if(int(cornerLast -> points[j].intensity) < closestPointScanID - 2.5)
                                break;
                            
                            // 计算距离
                            double pointSqDis = dis(cornerLast -> points[j], pointSel);
                            if(pointSqDis < minPointSqDis2){
                                minPointSqDis2 = pointSqDis;
                                minPointInd2 = j;
                            }
                        }

                        if(minPointInd2 >= 0){
                            Eigen::Vector3d curr_point = pointxyziToVector3d(cornerSharp -> points[i]);
                            Eigen::Vector3d last_point_a = pointxyziToVector3d(cornerLast -> points[closestPointInd]);
                            Eigen::Vector3d last_point_b = pointxyziToVector3d(cornerLast -> points[minPointInd2]);

                            double s = 1.0;
                            ceres::CostFunction *cost_function = 
                                LidarEdgeFactor::Create(curr_point, last_point_a, last_point_b, s);
                            problem.AddResidualBlock(cost_function, loss_function, para_q, para_t);
                        }
                    }
                }
                
                // 分割点云的平面点
                /*for(int i = 0; i < surfFlatNum; ++i){
                    pointSel = transform(&surfFlat -> points[i]);
                    kdtreeSurfLast -> nearestKSearch(pointSel, 1, pointSearchInd, pointSearchSqDis);
                    int closestPointInd = -1, minPointInd2 = -1, minPointInd3 = -1;
                    if(pointSearchSqDis[0] < nearest_feature_dist_sqr){
                        closestPointInd = pointSearchInd[0];
                        int closestPointScanID = int(surfLast -> points[closestPointInd].intensity);
                        
                        double minPointSqDis2 = nearest_feature_dist_sqr, minPointSqDis3 = nearest_feature_dist_sqr;
                        for(int j = closestPointInd + 1; j < int(surfLast -> size()); ++j){
                            // 不是附近的点
                            if(int(surfLast -> points[j].intensity) > closestPointScanID + 2.5)
                                break;
                            
                            double pointSqDis = dis(surfLast -> points[j], pointSel);
                            if(int(surfLast -> points[j].intensity) <= closestPointScanID &&
                                    pointSqDis < minPointSqDis2){
                                minPointSqDis2 = pointSqDis;
                                minPointInd2 = j;
                            }else if(int(surfLast -> points[j].intensity) > closestPointScanID &&
                                        pointSqDis < minPointSqDis3){
                                minPointSqDis3 = pointSqDis;
                                minPointInd3 = j;
                            }
                        }
                        for(int j = closestPointInd - 1; j >= 0; --j){
                            // 不是附近点
                            if(int(surfLast -> points[j].intensity) < closestPointScanID - 2.5)
                                break;

                            double pointSqDis = dis(surfLast -> points[j], pointSel);
                            if(int(surfLast -> points[j].intensity) >= closestPointScanID &&
                                    pointSqDis < minPointSqDis2){
                                minPointSqDis2 = pointSqDis;
                                minPointInd2 = j;
                            }else if(int(surfLast -> points[j].intensity) < closestPointScanID &&
                                        pointSqDis < minPointSqDis3){
                                minPointSqDis3 = pointSqDis;
                                minPointInd3 = j;
                            }
                        }

                        if(minPointInd2 >= 0 && minPointInd3 >= 0){
                            Eigen::Vector3d curr_point = pointxyziToVector3d(surfFlat -> points[i]);
                            Eigen::Vector3d last_point_a = pointxyziToVector3d(surfLast -> points[closestPointInd]);
                            Eigen::Vector3d last_point_b = pointxyziToVector3d(surfLast -> points[minPointInd2]);
                            Eigen::Vector3d last_point_c = pointxyziToVector3d(surfLast -> points[minPointInd3]);
                            
                            double s = 1.0;
                            ceres::CostFunction *cost_function = 
                                LidarPlaneFactor::Create(curr_point, last_point_a, last_point_b, last_point_c, s);
                            problem.AddResidualBlock(cost_function, loss_function, para_q, para_t);
                        }
                    }
                }*/

                // 地面点云的平面点
                /*for(int i = 0; i < groundFlatNum; ++i){
                    pointSel = transform(&groundSurfFlat -> points[i]);
                    kdtreeGroundLast -> nearestKSearch(pointSel, 1, pointSearchInd, pointSearchSqDis);

                    int closestPointInd = -1, minPointInd2 = -1, minPointInd3 = -1;
                    if(pointSearchSqDis[0] < nearest_feature_dist_sqr){
                        closestPointInd = pointSearchInd[0];
                        int closestPointScanID = int(groundSurfLast -> points[closestPointInd].intensity);
                        
                        double minPointSqDis2 = nearest_feature_dist_sqr, minPointSqDis3 = nearest_feature_dist_sqr;
                        for(int j = closestPointInd + 1; j < int(groundSurfLast -> size()); ++j){
                            if(int(groundSurfLast -> points[j].intensity) > closestPointScanID + 2.5)
                                break;
                            
                            double pointSqDis = dis(groundSurfLast -> points[j], pointSel);
                            if(int(groundSurfLast -> points[j].intensity) <= closestPointScanID &&
                                    pointSqDis < minPointSqDis2){
                                minPointSqDis2 = pointSqDis;
                                minPointInd2 = j;
                            }else if(int(groundSurfLast -> points[j].intensity) > closestPointScanID &&
                                        pointSqDis < minPointSqDis3){
                                minPointSqDis3 = pointSqDis;
                                minPointInd3 = j;
                            }
                        }
                        for(int j = closestPointInd - 1; j >= 0; --j){
                            if(int(groundSurfLast -> points[j].intensity) > closestPointScanID - 2.5)
                                break;
                            
                            double pointSqDis = dis(groundSurfLast -> points[j], pointSel);
                            if(int(groundSurfLast -> points[j].intensity) >= closestPointScanID &&
                                    pointSqDis < minPointSqDis2){
                                minPointSqDis2 = pointSqDis;
                                minPointInd2 = j;
                            }else if(int(groundSurfLast -> points[j].intensity) < closestPointScanID &&
                                        pointSqDis < minPointSqDis3){
                                minPointSqDis3 = pointSqDis;
                                minPointInd3 = j;
                            }
                        }

                        if(minPointInd2 >= 0 && minPointInd3 >= 0){
                            Eigen::Vector3d curr_point = pointxyziToVector3d(groundSurfFlat -> points[i]);
                            Eigen::Vector3d last_point_a = pointxyziToVector3d(groundSurfLast -> points[closestPointInd]);
                            Eigen::Vector3d last_point_b = pointxyziToVector3d(groundSurfLast -> points[minPointInd2]);
                            Eigen::Vector3d last_point_c = pointxyziToVector3d(groundSurfLast -> points[minPointInd3]);
                            
                            double s = 1.0;
                            ceres::CostFunction *cost_function = 
                                LidarPlaneFactor::Create(curr_point, last_point_a, last_point_b, last_point_c, s);
                            problem.AddResidualBlock(cost_function, loss_function, para_q, para_t);
                        }
                    }
                }*/
                
                // 求解
                ceres::Solver::Options options;
                options.linear_solver_type = ceres::DENSE_QR;
                options.max_num_iterations = 4;
                options.minimizer_progress_to_stdout = false;
                ceres::Solver::Summary summary;
                ceres::Solve(options, &problem, &summary);
            }
            
            t_w_curr = t_w_curr + q_w_curr * t_last_curr;
            q_w_curr = q_w_curr * q_last_curr;
        }
        
        // 更新上一帧点云
        CloudTypePtr cloudTmp;
        
        cloudTmp = cornerLessSharp;
        cornerLessSharp = cornerLast;
        cornerLast = cloudTmp;
        downSample(cornerLast);

        cloudTmp = surfLessFlat;
        surfLessFlat = surfLast;
        surfLast = cloudTmp;
        downSample(surfLast);

        cloudTmp = groundSurfLessFlat;
        groundSurfLessFlat = groundSurfLast;
        groundSurfLast = cloudTmp;
        downSample(groundSurfLast);
        
        kdtreeCornerLast -> setInputCloud(cornerLast);
        kdtreeSurfLast -> setInputCloud(surfLast);
        kdtreeGroundLast -> setInputCloud(groundSurfLast);
    }

    /**
     * 发布消息
    */
    void publish(){
        nav_msgs::msg::Odometry laserOdometry;
        laserOdometry.header.frame_id = "map";
        laserOdometry.child_frame_id = "/lidar_odom";
        laserOdometry.pose.pose.orientation.x = q_w_curr.x();
        laserOdometry.pose.pose.orientation.y = q_w_curr.y();
        laserOdometry.pose.pose.orientation.z = q_w_curr.z();
        laserOdometry.pose.pose.orientation.w = q_w_curr.w();
        laserOdometry.pose.pose.position.x = t_w_curr.x();
        laserOdometry.pose.pose.position.y = t_w_curr.y();
        laserOdometry.pose.pose.position.z = t_w_curr.z();
        // RCLCPP_INFO(this -> get_logger(), "%f, %f, %f, %f", q_w_curr.x(), q_w_curr.y(), q_w_curr.z(),q_w_curr.w());
        geometry_msgs::msg::PoseStamped laserPose;
        laserPose.header = laserOdometry.header;
        laserPose.pose = laserOdometry.pose.pose;
        path.poses.push_back(laserPose);
        path.header.frame_id = "map";
        
        pubPath -> publish(path);
    }

}; // class FrontEnd

} // lidarslam

int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    auto fe = std::make_shared<lidarslam::FrontEnd>("front_end");
    RCLCPP_INFO(fe -> get_logger(), "\033[1;32m---->\033[0m Started.");
    rclcpp::spin(fe);
    rclcpp::shutdown();
    return 0;
}