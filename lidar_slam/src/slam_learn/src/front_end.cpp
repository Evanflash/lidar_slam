#include "slam_utils.h"
#include "lidar_factor.hpp"

namespace lidarslam{

static const std::string PARAM_VERTICAL_SCANS = "param.vertical_scans";
static const std::string PARAM_EDGE_THRESHOLD = "param.edge_threshold";
static const std::string PARAM_SURF_THRESHOLD = "param.surf_threshold";
static const std::string PARAM_DISTANCE = "param.distance";

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
    CloudTypePtr groundSurfFlat;
    CloudTypePtr groundSurfLessFlat;
    // 上一时刻点特征点集
    CloudTypePtr cornerLast;
    CloudTypePtr groundSurfLast;

    // 曲率
    std::vector<smoothness> segmentCurvature;
    std::vector<smoothness> groundCurvature;
    // 标记周围是否被选取
    std::vector<int> segmentNeighborPicked;
    std::vector<int> groundNeighborPicked;

    // kd tree
    pcl::KdTreeFLANN<PointType>::Ptr kdtreeCornerLast;
    pcl::KdTreeFLANN<PointType>::Ptr kdtreeGroundLast;

    // 参数
    int vertical_scans;
    float edgeThreshold;
    float surfThreshold;
    float nearest_feature_dist_sqr;

    bool isFirstFrame;

    Eigen::Quaterniond q_last_curr;
    Eigen::Vector3d t_last_curr;

    Eigen::Quaterniond q_w_curr;
    Eigen::Vector3d t_w_curr;

    // 队列
    std::queue<other_msgs::msg::SegCloud> segCloudQueue;
    std::mutex mtx;

    // 子线程
    std::thread run_thread;

    // 路径, 测试
    nav_msgs::msg::Path path;

    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubPath;
    PubPointCloud pubtest;
    PubPointCloud pubtest2;
    
public:
    FrontEnd(const std::string &name)
        : Node(name),
          q_w_curr(1, 0, 0, 0),
          t_w_curr(0, 0, 0)
    {
        subSegMsg = this -> create_subscription<other_msgs::msg::SegCloud>(
            "/seg_cloud", 1, std::bind(&FrontEnd::pushQueue, this, std::placeholders::_1));
        
        pubAllCloud = this -> create_publisher<other_msgs::msg::AllCloud>(
            "/all_cloud", 1);

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
        groundSurfFlat.reset(new CloudType());
        groundSurfLessFlat.reset(new CloudType());
        cornerLast.reset(new CloudType());
        groundSurfLast.reset(new CloudType());

        kdtreeCornerLast.reset(new pcl::KdTreeFLANN<PointType>());
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
        while(rclcpp::ok()){
            if(segCloudQueue.empty()){
                //return;
                sleep(1);
                continue;
            }
            mtx.lock();
            seg_msg = segCloudQueue.front();
            all_cloud.header = seg_msg.header;
            segCloudQueue.pop();
            mtx.unlock();
            std::chrono::time_point<std::chrono::system_clock> start = std::chrono::system_clock::now();
            // 变量重置
            resetParameters();
            // 计算曲率
            calculateSmoothness();
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
        groundSurfFlat -> clear();
        groundSurfLessFlat -> clear();

        // 曲率
        segmentCurvature.clear();
        segmentCurvature.resize(seg_msg.seg_cloud.size(), smoothness(0, 0));
        groundCurvature.clear();
        groundCurvature.resize(seg_msg.ground_cloud.size(), smoothness(0, 0));
        // 标志周围该点是否被选取
        segmentNeighborPicked.clear();
        segmentNeighborPicked.resize(seg_msg.seg_cloud.size(), 0);
        groundNeighborPicked.clear();
        groundNeighborPicked.resize(seg_msg.ground_cloud.size(), 0);

        // 消息重置
        all_cloud.trans_form.clear();
        all_cloud.corner_sharp.clear();
        all_cloud.corner_less_sharp.clear();
        all_cloud.surf_less_flat.clear();
        all_cloud.ground_flat.clear();
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
                        if(largestPickedNum <= 1){
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
            }
        }
        
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
                int smallestPickedNum = 0;
                for(int k = sp; k <= ep; ++k){
                    int curInd = groundCurvature[k].index;
                    if(groundNeighborPicked[curInd] == 0 && 
                       groundCurvature[k].curvature < surfThreshold){
                        tran(seg_msg.ground_cloud[curInd], point);
                        groundSurfFlat -> push_back(point);

                        if(++smallestPickedNum >= 2){
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

        // 组装all_cloud消息
        pointType2MsgPoint(cornerSharp, all_cloud.corner_sharp);
        pointType2MsgPoint(cornerLessSharp, all_cloud.corner_less_sharp);
        all_cloud.surf_less_flat = seg_msg.seg_cloud;
        pointType2MsgPoint(groundSurfFlat, all_cloud.ground_flat);
        all_cloud.ground_less_flat = seg_msg.ground_cloud;
        
        // 测试
        sensor_msgs::msg::PointCloud2 msg;
        pcl::toROSMsg(*groundSurfLast, msg);
        msg.header.frame_id = "map";
        pubtest -> publish(msg);
        
        pcl::toROSMsg(*groundSurfFlat, msg);
        msg.header.frame_id = "map";
        pubtest2 -> publish(msg);
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

        // q: [qx, qy, qz]
        // t: [tx, ty, tz]
        double q[3] = {0.0, 0.0, 0.0};
        double t[3] = {0.0, 0.0, 0.0};
        if(isFirstFrame == true){
            isFirstFrame = false;
        }else{
            int cornerSharpNum = cornerSharp -> size();
            int groundFlatNum = groundSurfFlat -> size();
           
            // 地面点优化
            for(size_t opti_counter = 0; opti_counter < 2; ++opti_counter){
                ceres::LossFunction *loss_function = new ceres::HuberLoss(0.1);
                ceres::Problem::Options problem_options;

                ceres::Problem problem(problem_options);
                problem.AddParameterBlock(q, 3);
                problem.AddParameterBlock(t, 3);

                PointType pointSel;
                std::vector<int> pointSearchInd;
                std::vector<float> pointSearchSqDis;

                for(int i = 0; i < groundFlatNum; ++i){
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
                            
                            ceres::CostFunction *cost_function = 
                                GroundPlaneFactor::Create(curr_point, last_point_a, last_point_b, last_point_c);
                            problem.AddResidualBlock(cost_function, loss_function, q, t);
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
            // RCLCPP_INFO(this -> get_logger(), "x%f, y%f, z%f, qx%f, qy%f, qz%F", 
            //     t[0], t[1], t[2], q[0], q[1], q[2]);

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
                Eigen::AngleAxisd roll(Eigen::AngleAxisd(q[0], Eigen::Vector3d::UnitX()));
                Eigen::AngleAxisd pitch(Eigen::AngleAxisd(q[1], Eigen::Vector3d::UnitY()));
                Eigen::Matrix3d qyx;
                qyx = pitch * roll;

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

                            ceres::CostFunction *cost_function = 
                                CornerFactor::Create(curr_point, last_point_a, last_point_b, qyx, t[2]);
                            problem.AddResidualBlock(cost_function, loss_function, q + 2, t);
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
            // RCLCPP_INFO(this -> get_logger(), "front end : x%f, y%f, z%f, qx%f, qy%f, qz%F", 
            //     t[0], t[1], t[2], q[0], q[1], q[2]);
            
        }
        // 消息中加入位姿
        all_cloud.trans_form.push_back(t[0]);
        all_cloud.trans_form.push_back(t[1]);
        all_cloud.trans_form.push_back(t[2]);
        all_cloud.trans_form.push_back(q[0]);
        all_cloud.trans_form.push_back(q[1]);
        all_cloud.trans_form.push_back(q[2]);

        // 更新位姿
        Eigen::AngleAxisd roll(Eigen::AngleAxisd(q[0], ::Eigen::Vector3d::UnitX()));
        Eigen::AngleAxisd pitch(Eigen::AngleAxisd(q[1], ::Eigen::Vector3d::UnitY()));
        Eigen::AngleAxisd yaw(Eigen::AngleAxisd(q[2], ::Eigen::Vector3d::UnitZ()));
        q_last_curr = yaw * pitch * roll;
        t_last_curr = Eigen::Vector3d(t[0], t[1], t[2]);

        t_w_curr = t_w_curr + q_w_curr * t_last_curr;
        q_w_curr = q_w_curr * q_last_curr;
        
        // 更新上一帧点云
        CloudTypePtr cloudTmp;
        
        cloudTmp = cornerLessSharp;
        cornerLessSharp = cornerLast;
        cornerLast = cloudTmp;
        downSample(cornerLast);

        cloudTmp = groundSurfLessFlat;
        groundSurfLessFlat = groundSurfLast;
        groundSurfLast = cloudTmp;
        downSample(groundSurfLast);

        kdtreeCornerLast -> setInputCloud(cornerLast);
        kdtreeGroundLast -> setInputCloud(groundSurfLast);
    }

    /**
     * 发布消息
    */
    void publish(){
        // 发布all_cloud
        pubAllCloud -> publish(all_cloud);


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