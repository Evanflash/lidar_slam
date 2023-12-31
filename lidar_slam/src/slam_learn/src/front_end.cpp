#include "slam_utils.h"
#include "lidar_factor.hpp"

namespace lidarslam{

static const std::string PARAM_VERTICAL_SCANS = "param.vertical_scans";
static const std::string PARAM_EDGE_THRESHOLD = "param.edge_threshold";
static const std::string PARAM_SURF_THRESHOLD = "param.surf_threshold";
static const std::string PARAM_DISTANCE = "param.distance";
static const std::string PARAM_MAP_FREQUENCY = "param.map_frequency";

class FrontEnd : public rclcpp::Node{
private:
    // 消息接收与控制
    rclcpp::Subscription<other_msgs::msg::SegCloud>::SharedPtr subSegMsg;
    rclcpp::Publisher<other_msgs::msg::AllCloud>::SharedPtr pubAllCloud;

    // 消息
    other_msgs::msg::SegCloud seg_msg;
    other_msgs::msg::AllCloud all_cloud;

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
    // 标记周围是否被选取
    std::vector<int> segmentNeighborPicked;

    // kd tree
    pcl::KdTreeFLANN<PointType>::Ptr kdtreeCornerLast;
    pcl::KdTreeFLANN<PointType>::Ptr kdtreeSurfLast;
    pcl::KdTreeFLANN<PointType>::Ptr kdtreeGroundLast;

    // 参数
    int vertical_scans;
    float edgeThreshold;
    float surfThreshold;
    float nearest_feature_dist_sqr;
    int map_frequency;
    bool isFirstFrame;

    // 位姿
    Eigen::Quaterniond q_last_curr;
    Eigen::Vector3d t_last_curr;

    Eigen::Quaterniond q_w_curr;
    Eigen::Vector3d t_w_curr;

    // 队列
    std::queue<other_msgs::msg::SegCloud> segCloudQueue;
    std::mutex mtx;

    // 子线程
    std::thread run_thread;

    // 传入地图的参数
    Eigen::Quaterniond q_for_map;
    Eigen::Vector3d t_for_map;
    CloudTypePtr cornerForMap;
    CloudTypePtr surfForMap;
    CloudTypePtr groundForMap;

    // 前端地图
    CloudTypePtr frontEndGlobalMap;
    PubPointCloud pubFrontEndGlobalMap;
    pcl::VoxelGrid<PointType>::Ptr frontEndGlobalMapFilter;


    // 路径, 测试
    nav_msgs::msg::Path path;

    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubPath;
    PubPointCloud pubtest2;

    
public:
    FrontEnd(const std::string &name)
        : Node(name),
          q_w_curr(1, 0, 0, 0),
          t_w_curr(0, 0, 0),
          q_for_map(1, 0, 0, 0),
          t_for_map(0, 0, 0)
    {
        subSegMsg = this -> create_subscription<other_msgs::msg::SegCloud>(
            "/seg_cloud", 100, std::bind(&FrontEnd::pushQueue, this, std::placeholders::_1));
        
        pubAllCloud = this -> create_publisher<other_msgs::msg::AllCloud>(
            "/all_cloud", 1);

        pubFrontEndGlobalMap = this -> create_publisher<sensor_msgs::msg::PointCloud2>(
            "/front_end_global_map", 1);


        // test
        pubPath = this -> create_publisher<nav_msgs::msg::Path>("/path", 1);
        pubtest2 = this -> create_publisher<sensor_msgs::msg::PointCloud2>("/test2", 1);

        // 声明参数
        this -> declare_parameter(PARAM_VERTICAL_SCANS, vertical_scans);
        this -> declare_parameter(PARAM_EDGE_THRESHOLD, edgeThreshold);
        this -> declare_parameter(PARAM_SURF_THRESHOLD, surfThreshold);
        this -> declare_parameter(PARAM_DISTANCE, nearest_feature_dist_sqr);
        this -> declare_parameter(PARAM_MAP_FREQUENCY, map_frequency);

        // 更新参数
        if(!this -> get_parameter(PARAM_VERTICAL_SCANS, vertical_scans)){
            RCLCPP_WARN(this -> get_logger(), "front_end_node: %s not found", PARAM_VERTICAL_SCANS.c_str());
        }
        if(!this -> get_parameter(PARAM_EDGE_THRESHOLD, edgeThreshold)){
            RCLCPP_WARN(this -> get_logger(), "front_end_node: %s not found", PARAM_EDGE_THRESHOLD.c_str());
        }
        if(!this -> get_parameter(PARAM_SURF_THRESHOLD, surfThreshold)){
            RCLCPP_WARN(this -> get_logger(), "front_end_node: %s not found", PARAM_SURF_THRESHOLD.c_str());
        }
        if(!this -> get_parameter(PARAM_DISTANCE, nearest_feature_dist_sqr)){
            RCLCPP_WARN(this -> get_logger(), "front_end_node: %s not found", PARAM_DISTANCE.c_str());
        }
        if(!this -> get_parameter(PARAM_MAP_FREQUENCY, map_frequency)){
            RCLCPP_WARN(this -> get_logger(), "front_end_node: %s not found", PARAM_MAP_FREQUENCY.c_str());
        }

        // 初始化
        init();
        run_thread = std::thread(&FrontEnd::run, this);
        
    }
    ~FrontEnd(){
        run_thread.join();
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


        // 地图参数
        cornerForMap.reset(new CloudType());
        surfForMap.reset(new CloudType());
        groundForMap.reset(new CloudType());

        // 地图
        frontEndGlobalMap.reset(new CloudType());
        frontEndGlobalMapFilter.reset(new pcl::VoxelGrid<PointType>());
        frontEndGlobalMapFilter -> setLeafSize(0.5, 0.5, 0.5);
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
        while(rclcpp::ok()){
            if(segCloudQueue.empty()){
                //return;
                sleep(1);
                continue;
            }
            mtx.lock();
            seg_msg = segCloudQueue.front();
            segCloudQueue.pop();
            mtx.unlock();
            std::chrono::time_point<std::chrono::system_clock> start = std::chrono::system_clock::now();
            // 变量重置
            resetParameters();
            // 计算曲率
            calculateSmoothness();
            // 去除不合适的点
            removeUselessPoints();
            // 提取特征点
            extractFeatures();
            // 里程计
            odometry();
            // 发布消息
            publish();
            std::chrono::time_point<std::chrono::system_clock> end = std::chrono::system_clock::now();
            std::chrono::duration<double> elapsed_seconds = end - start;
            // RCLCPP_INFO(this -> get_logger(), "whole time: %fms", elapsed_seconds.count() * 1000);
        }
        
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
        segmentCurvature.assign(seg_msg.seg_cloud.size(), smoothness(0, 0));
        // 标志周围该点是否被选取
        segmentNeighborPicked.assign(seg_msg.seg_cloud.size(), 0);

        // 消息重置
        all_cloud.trans_form.clear();
        all_cloud.corner_less_sharp.clear();
        all_cloud.surf_less_flat.clear();
        all_cloud.ground_less_flat.clear();
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
                sns.isflat = 0;
                curvatures[i] = sns;
            }
        };
        cs(seg_msg.seg_cloud, seg_msg.seg_range, segmentCurvature);
    }

    /**
     * 去除不适合的点
    */
    void removeUselessPoints(){
        int size = seg_msg.seg_cloud.size();
        // RCLCPP_INFO(this -> get_logger(), "%d", size);
        // 去除遮挡点
        for(int i = 5; i < size - 6; ++i){
            float d1 = seg_msg.seg_range[i];
            float d2 = seg_msg.seg_range[i + 1];
            int colDiff = std::abs(int(seg_msg.seg_cloud_col_ind[i + 1] - seg_msg.seg_cloud_col_ind[i]));

            if(colDiff < 10){
                if(d1 - d2 > 0.3){
                    segmentNeighborPicked[i - 5] = 1;
                    segmentNeighborPicked[i - 4] = 1;
                    segmentNeighborPicked[i - 3] = 1;
                    segmentNeighborPicked[i - 2] = 1;
                    segmentNeighborPicked[i - 1] = 1;
                    segmentNeighborPicked[i] = 1;
                }else if(d2 - d1 > 0.3){
                    segmentNeighborPicked[i + 1] = 1;
                    segmentNeighborPicked[i + 2] = 1;
                    segmentNeighborPicked[i + 3] = 1;
                    segmentNeighborPicked[i + 4] = 1;
                    segmentNeighborPicked[i + 5] = 1;
                    segmentNeighborPicked[i + 6] = 1;
                }
            }

            float diff1 = std::abs(float(seg_msg.seg_range[i - 1] - seg_msg.seg_range[i]));
            float diff2 = std::abs(float(seg_msg.seg_range[i + 1] - seg_msg.seg_range[i]));
            if(diff1 > 0.02 * seg_msg.seg_range[i] &&
               diff2 > 0.02 * seg_msg.seg_range[i]){
                segmentNeighborPicked[i] = 1;
            }
        }
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
        CloudTypePtr surf(new CloudType());
        CloudTypePtr ground(new CloudType());
        pcl::VoxelGrid<PointType>::Ptr downsamplefilter(new pcl::VoxelGrid<PointType>());
        downsamplefilter -> setLeafSize(0.2, 0.2, 0.2);


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
                       segmentCurvature[k].curvature > edgeThreshold && seg_msg.is_ground[curInd] == 0){
                        ++largestPickedNum;
                        if(largestPickedNum <= 2){
                            tran(seg_msg.seg_cloud[curInd], point);
                            cornerSharp -> push_back(point);
                            cornerLessSharp -> push_back(point);
                            segmentCurvature[k].isflat = 1;
                        }else if(largestPickedNum <= 20){
                            tran(seg_msg.seg_cloud[curInd], point);
                            cornerLessSharp -> push_back(point);
                            segmentCurvature[k].isflat = 2;
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
                int smallestPickedNum = 0;
                for(int k = sp; k <= ep; ++k){
                    int curInd = segmentCurvature[k].index;
                    if(segmentNeighborPicked[curInd] == 0 && 
                       segmentCurvature[k].curvature < surfThreshold && seg_msg.is_ground[curInd] == 1){
                        tran(seg_msg.seg_cloud[curInd], point);
                        groundSurfFlat -> push_back(point);

                        if(++smallestPickedNum >= 4){
                            break;
                        }
                        
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

                /*
                smallestPickedNum = 0;
                for(int k = sp; k <= ep; ++k){
                    int curInd = segmentCurvature[k].index;
                    if(segmentNeighborPicked[curInd] == 0 && 
                       segmentCurvature[k].curvature < surfThreshold && seg_msg.is_ground[curInd] == 0){
                        tran(seg_msg.seg_cloud[curInd], point);
                        surfFlat -> push_back(point);

                        if(++smallestPickedNum >= 4){
                            break;
                        }
                        
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
                */
                
                for(int k = sp; k <= ep; ++k){
                    if(segmentCurvature[k].isflat <= 0){
                        int curInd = segmentCurvature[k].index;
                        if(seg_msg.is_ground[curInd] == 1 && k % 5 == 0){
                            tran(seg_msg.seg_cloud[curInd], point);
                            ground -> push_back(point);
                        } else if(seg_msg.is_ground[curInd] == 0 && segmentCurvature[k].curvature < 0.3){
                            tran(seg_msg.seg_cloud[curInd], point);
                            surf -> push_back(point);
                        }
                    }
                }
  
            }
            CloudTypePtr surfFiltered(new CloudType());
            CloudTypePtr groundFiltered(new CloudType());
            downsamplefilter -> setInputCloud(surf);
            downsamplefilter -> filter(*surfFiltered);
            *surfLessFlat += *surfFiltered;

            downsamplefilter -> setInputCloud(ground);
            downsamplefilter -> filter(*groundFiltered);
            *groundSurfLessFlat += *groundFiltered;
        }
         
        // 测试
        sensor_msgs::msg::PointCloud2 msg;
        pcl::toROSMsg(*cornerLessSharp, msg);
        msg.header.frame_id = "map";
        pubtest2 -> publish(msg);

    }

    /**
     * 里程计
    */
    void odometry(){
        // q: [qx, qy, qz]
        // t: [tx, ty, tz]
        double q[3] = {0.0, 0.0, 0.0};
        double t[3] = {0.0, 0.0, 0.0};
        Eigen::Map<Eigen::Vector3d> t_odom(t);
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
        // 将当前点变换到前一帧坐标系下
        auto trans = [&](PointType &pi, PointType &po){
            Eigen::Quaterniond q_odom = Eigen::AngleAxisd(q[2], Eigen::Vector3d::UnitZ()) *
                                        Eigen::AngleAxisd(q[1], Eigen::Vector3d::UnitY()) *
                                        Eigen::AngleAxisd(q[0], Eigen::Vector3d::UnitX());
            Eigen::Vector3d point_in(pi.x, pi.y, pi.z);
            point_in = q_odom * point_in + t_odom;
            po.x = point_in.x();
            po.y = point_in.y();
            po.z = point_in.z();
            po.intensity = pi.intensity;
        };

        if(isFirstFrame == true){
            isFirstFrame = false;
        }else{
            // RCLCPP_INFO(this -> get_logger(), "********************pose%d**********************", num++);
            int cornerSharpNum = cornerSharp -> size();
            int surfFlatNum = surfFlat -> size();
            int groundFlatNum = groundSurfFlat -> size();
            // 地面点优化
            for(size_t opti_counter = 0; opti_counter < 2; ++opti_counter){
                ceres::LossFunction *loss_function = new ceres::HuberLoss(0.1);
                ceres::Problem::Options problem_options;

                ceres::Problem problem(problem_options);
                problem.AddParameterBlock(q, 2);
                problem.AddParameterBlock(t + 2, 1);

                PointType pointSel;
                std::vector<int> pointSearchInd;
                std::vector<float> pointSearchSqDis;

                for(int i = 0; i < groundFlatNum; ++i){
                    trans(groundSurfFlat -> points[i], pointSel);
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
                            
                            ceres::CostFunction *cost_function = 
                                GroundPlaneFactor::Create(curr_point, last_point_a, last_point_b, last_point_c);
                            problem.AddResidualBlock(cost_function, loss_function, q, t + 2);
                        }
                    }
                }
                // 求解
                ceres::Solver::Options options;
                options.linear_solver_type = ceres::DENSE_QR;
                options.max_num_iterations = 4;
                options.minimizer_progress_to_stdout = false;
                ceres::Solver::Summary summary;
                ceres::Solve(options, &problem, &summary);
            }
            
            // 边缘点优化
            for(size_t opti_counter = 0; opti_counter < 2; ++opti_counter){
                ceres::LossFunction *loss_function = new ceres::HuberLoss(0.1);
                ceres::Problem::Options problem_options;

                ceres::Problem problem(problem_options);
                problem.AddParameterBlock(q + 2, 1);
                problem.AddParameterBlock(t, 2);

                PointType pointSel;
                std::vector<int> pointSearchInd;
                std::vector<float> pointSearchSqDis;
                // qyx
                Eigen::AngleAxisd roll(Eigen::AngleAxisd(q[0], Eigen::Vector3d::UnitX()));
                Eigen::AngleAxisd pitch(Eigen::AngleAxisd(q[1], Eigen::Vector3d::UnitY()));
                Eigen::Matrix3d qyx;
                qyx = pitch * roll;
                int corner = 0;
                int surf = 0;

                // 分割点云的边缘点
                for(int i = 0; i < cornerSharpNum; ++i){
                    trans(cornerSharp -> points[i], pointSel);
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

                            ceres::CostFunction *cost_function = 
                                CornerFactor::Create(curr_point, last_point_a, last_point_b, qyx, t[2]);
                            problem.AddResidualBlock(cost_function, loss_function, q + 2, t);
                            corner++;
                        }
                    }
                }
                /*
                // 分割点云的平面点
                for(int i = 0; i < surfFlatNum; ++i){
                    trans(surfFlat -> points[i], pointSel);
                    kdtreeSurfLast -> nearestKSearch(pointSel, 1, pointSearchInd, pointSearchSqDis);

                    int closestPointInd = -1, minPointInd2 = -1, minPointInd3 = -1;
                    if(pointSearchSqDis[0] < nearest_feature_dist_sqr){
                        closestPointInd = pointSearchInd[0];
                        int closestPointScanID = int(surfLast -> points[closestPointInd].intensity);
                        
                        double minPointSqDis2 = nearest_feature_dist_sqr, minPointSqDis3 = nearest_feature_dist_sqr;
                        for(int j = closestPointInd + 1; j < int(surfLast -> size()); ++j){
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
                            if(int(surfLast -> points[j].intensity) > closestPointScanID - 2.5)
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
                            
                            ceres::CostFunction *cost_function = 
                                SurfFactor::Create(
                                    curr_point, last_point_a, last_point_b, last_point_c, qyx, t[2]);
                            problem.AddResidualBlock(cost_function, loss_function, q + 2, t);
                            surf++;
                        }
                    }
                }
                */
                // 求解
                ceres::Solver::Options options;
                options.linear_solver_type = ceres::DENSE_QR;
                options.max_num_iterations = 10;
                options.minimizer_progress_to_stdout = false;
                ceres::Solver::Summary summary;
                ceres::Solve(options, &problem, &summary);
                
                // RCLCPP_INFO(this -> get_logger(), "corner Total num:%d, surf Total num:%d",
                //     cornerSharpNum, surfFlatNum);
                // RCLCPP_INFO(this -> get_logger(), "corner num:%d, surf num:%d", corner, surf);
                // RCLCPP_INFO_STREAM(this -> get_logger(), summary.BriefReport());       
            }
        }

        // 更新位姿
        Eigen::AngleAxisd roll(Eigen::AngleAxisd(q[0], ::Eigen::Vector3d::UnitX()));
        Eigen::AngleAxisd pitch(Eigen::AngleAxisd(q[1], ::Eigen::Vector3d::UnitY()));
        Eigen::AngleAxisd yaw(Eigen::AngleAxisd(q[2], ::Eigen::Vector3d::UnitZ()));
        q_last_curr = yaw * pitch * roll;
        t_last_curr = Eigen::Vector3d(t[0], t[1], t[2]);

        // 传给地图的增量
        t_for_map = t_for_map + q_for_map * t_last_curr;
        q_for_map = q_for_map * q_last_curr;

        // 世界坐标系下位姿
        t_w_curr = t_w_curr + q_w_curr * t_last_curr;
        q_w_curr = q_w_curr * q_last_curr;
        
        // 更新上一帧点云
        CloudTypePtr cloudTmp;
        cloudTmp = cornerLessSharp;
        cornerLessSharp = cornerLast;
        cornerLast = cloudTmp;

        cloudTmp = surfLessFlat;
        surfLessFlat = surfLast;
        surfLast = cloudTmp;

        cloudTmp = groundSurfLessFlat;
        groundSurfLessFlat = groundSurfLast;
        groundSurfLast = cloudTmp;

        kdtreeCornerLast -> setInputCloud(cornerLast);
        kdtreeSurfLast -> setInputCloud(surfLast);
        kdtreeGroundLast -> setInputCloud(groundSurfLast);
    }

    /**
     * 发布消息
    */
    void publish(){
        // 发布all_cloud
        static int cycle_count = map_frequency - 1;
        cycle_count++;
        if(cycle_count == map_frequency){
            cycle_count = 0;
            // 组装all_cloud消息
            all_cloud.header = seg_msg.header;
            // 消息中加入位姿
            all_cloud.trans_form.push_back(t_for_map.x());
            all_cloud.trans_form.push_back(t_for_map.y());
            all_cloud.trans_form.push_back(t_for_map.z());
            all_cloud.trans_form.push_back(q_for_map.x());
            all_cloud.trans_form.push_back(q_for_map.y());
            all_cloud.trans_form.push_back(q_for_map.z());
            all_cloud.trans_form.push_back(q_for_map.w());
            pointType2MsgPoint(cornerLast, all_cloud.corner_less_sharp);
            pointType2MsgPoint(surfLast, all_cloud.surf_less_flat);
            pointType2MsgPoint(groundSurfLast, all_cloud.ground_less_flat);

            q_for_map = Eigen::Quaterniond{1, 0, 0, 0};
            t_for_map = Eigen::Vector3d{0, 0, 0};

            pubAllCloud -> publish(all_cloud);
        }
        // 前端地图
        *frontEndGlobalMap += cloudToWorld(surfLast);
        *frontEndGlobalMap += cloudToWorld(groundSurfLast);
        CloudTypePtr globalMapFiltered(new CloudType());
        frontEndGlobalMapFilter -> setInputCloud(frontEndGlobalMap);
        // frontEndGlobalMap -> clear();
        frontEndGlobalMapFilter -> filter(*globalMapFiltered);
        *frontEndGlobalMap = *globalMapFiltered;

        if(pubFrontEndGlobalMap -> get_subscription_count() != 0){
            sensor_msgs::msg::PointCloud2 msg;
            pcl::toROSMsg(*frontEndGlobalMap, msg);
            msg.header.frame_id = "map";
            pubFrontEndGlobalMap -> publish(msg);
        }

        

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

        geometry_msgs::msg::PoseStamped laserPose;
        laserPose.header = laserOdometry.header;
        laserPose.pose = laserOdometry.pose.pose;
        path.poses.push_back(laserPose);
        path.header.frame_id = "map";
        
        pubPath -> publish(path);

    }


    /**
     * pointtype -> point
    */
    void pointType2MsgPoint(CloudTypePtr cloud, std::vector<other_msgs::msg::Point> &pointMsg){
        other_msgs::msg::Point p;
        for(size_t i = 0; i < cloud -> size(); ++i){
            p.x = cloud -> points[i].x;
            p.y = cloud -> points[i].y;
            p.z = cloud -> points[i].z;
            p.i = cloud -> points[i].intensity;
            pointMsg.push_back(p);
        }
    }

    /**
     * 将当前帧变换到世界坐标系下 
    */
    CloudType cloudToWorld(CloudTypePtr cloud){
        CloudType cloud_out;
        PointType point;
        for(size_t i = 0; i < cloud -> size(); ++i){
            Eigen::Vector3d p(cloud -> points[i].x, cloud -> points[i].y, cloud -> points[i].z);
            p = q_w_curr * p + t_w_curr;
            point.x = p.x();
            point.y = p.y();
            point.z = p.z();
            point.intensity = cloud -> points[i].intensity;
            cloud_out.push_back(point);
        }
        return cloud_out;
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