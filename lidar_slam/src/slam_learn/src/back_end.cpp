#include "slam_utils.h"
#include "lidar_factor.hpp"

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
static double q_[4] = {0, 0, 0, 1};
static double t_[3] = {0, 0, 0};

class BackEnd : public rclcpp::Node{
private:
    // 消息队列
    std::queue<other_msgs::msg::AllCloud> allCloudQueue;
    other_msgs::msg::AllCloud all_cloud;
    rclcpp::Subscription<other_msgs::msg::AllCloud>::SharedPtr subAllCloud;

    nav_msgs::msg::Path path;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubPath;

    // 互斥锁
    std::mutex mtx;
    std::mutex globalMtx;

    // 地图
    std::queue<int> globalMapQueue;
    CloudTypePtr globalMap;
    PubPointCloud pubGlobalMap;

    Eigen::Quaterniond q_w_last;
    Eigen::Vector3d t_w_last;

    Eigen::Quaterniond q_keyframe_last;
    Eigen::Vector3d t_keyframe_last;

    Eigen::Quaterniond q_delta;
    Eigen::Vector3d t_delta;

    // 滑动窗口
    std::deque<int> recentKeyFrames;

    // 所有点云
    std::vector<CloudTypePtr> allCornerKeyFramse;
    std::vector<CloudTypePtr> allSurfKeyFrames;
    std::vector<CloudTypePtr> allGroundKeyFrames;

    // 对应位姿
    std::vector<pose> allKeyFramesPoses;

    // 子图
    CloudTypePtr laserCornerFromMap;
    CloudTypePtr laserSurfFromMap;
    CloudTypePtr laserGroundFromMap;

    // kdtree
    pcl::KdTreeFLANN<PointType>::Ptr kdtreeCornerPoints;
    pcl::KdTreeFLANN<PointType>::Ptr kdtreeSurfPoints;
    pcl::KdTreeFLANN<PointType>::Ptr kdtreeGroundPoints;


    // 新一帧点云
    CloudTypePtr cornerCloud;
    CloudTypePtr surfCloud;
    CloudTypePtr groundCloud;

    // 下采样滤波
    pcl::VoxelGrid<PointType>::Ptr downSampleSubMap;
    pcl::VoxelGrid<PointType>::Ptr downSampleKeyPoints;
    pcl::VoxelGrid<PointType>::Ptr downSampleGlobalMap;

    // 前一帧关键帧位姿
    PointType previousPosPoint;

    // 因子图
    gtsam::NonlinearFactorGraph gtsamGraph;
    gtsam::Values initialEstimate;
    gtsam::ISAM2 *isam;
    gtsam::Values isamCurrentEstimate;


    // 参数
    bool loop_closure_enabled;
    float surrounding_keyframe_search_radius;
    int surrounding_keyframe_search_num;
    float history_keyframe_search_radius;
    int history_keyframe_search_num;
    float history_search_fitness_score;

    std::thread run_thread;
    std::thread pub_global_map_thread;

public:
    BackEnd(const std::string &name)
        : Node(name),
          q_w_last(1, 0, 0, 0),
          t_w_last(0, 0, 0),
          q_delta(q_),
          t_delta(t_)
    {
        subAllCloud = this -> create_subscription<other_msgs::msg::AllCloud>(
            "/all_cloud", 10, std::bind(&BackEnd::getMsg, this, std::placeholders::_1));

        pubPath = this -> create_publisher<nav_msgs::msg::Path>("/global_path", 1);

        pubGlobalMap = this -> create_publisher<sensor_msgs::msg::PointCloud2>("/global_map", 1);



        init();
        // 开启子线程
        run_thread = std::thread(&BackEnd::run, this);
        pub_global_map_thread = std::thread(&BackEnd::publishGlobalMap, this);
    }
    ~BackEnd(){
        run_thread.join();
        pub_global_map_thread.join();
    }

private:
    /**
     * 接收消息
    */
    void getMsg(const other_msgs::msg::AllCloud &msg){
        mtx.lock();
        allCloudQueue.push(msg);
        mtx.unlock();
    }

    /**
     * 发布地图子线程
    */
    void publishGlobalMap(){
        while(rclcpp::ok()){
            // 1秒更新一次
            sleep(1);
            if(pubGlobalMap-> get_subscription_count() == 0)
                continue;
            globalMtx.lock();
            if(globalMapQueue.empty()){
                globalMtx.unlock();
                continue;
            }       
            std::vector<int> curPubCloud;
            for(size_t i = 0; i < globalMapQueue.size(); ++i){
                curPubCloud.push_back(globalMapQueue.front());
                globalMapQueue.pop();
            }
            globalMtx.unlock();

            for(size_t i = 0; i < curPubCloud.size(); ++i){
                *globalMap += transformPointCloud(allCornerKeyFramse[curPubCloud[i]],
                                                  allKeyFramesPoses[curPubCloud[i]]);
                *globalMap += transformPointCloud(allSurfKeyFrames[curPubCloud[i]],
                                                  allKeyFramesPoses[curPubCloud[i]]);
                *globalMap += transformPointCloud(allGroundKeyFrames[curPubCloud[i]],
                                                  allKeyFramesPoses[curPubCloud[i]]);
            }
            downSamplePointCloud(globalMap, downSampleGlobalMap);

            sensor_msgs::msg::PointCloud2 gl_map;
            pcl::toROSMsg(*globalMap, gl_map);

            gl_map.header.frame_id = "map";
            gl_map.header.stamp = this -> now();
            pubGlobalMap -> publish(gl_map);
        }
    }

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

            std::chrono::duration<double> elapsed_seconds;

            std::chrono::time_point<std::chrono::system_clock> t1 = std::chrono::system_clock::now();
            // 消息拆解
            msgAnalysis();
            std::chrono::time_point<std::chrono::system_clock> t2 = std::chrono::system_clock::now();
            elapsed_seconds = t2 - t1;

            // RCLCPP_INFO(this -> get_logger(), "msg analysis time: %fms", elapsed_seconds.count() * 1000);
            // 提取子地图
            extractSurroundingKeyFrames();
            std::chrono::time_point<std::chrono::system_clock> t3 = std::chrono::system_clock::now();
            elapsed_seconds = t3 - t2;
            // RCLCPP_INFO(this -> get_logger(), "extract sub map time: %fms", elapsed_seconds.count() * 1000);
            // 图优化
            scan2MapOptimization();
            std::chrono::time_point<std::chrono::system_clock> t4 = std::chrono::system_clock::now();
            elapsed_seconds = t4 - t3;
            // RCLCPP_INFO(this -> get_logger(), "map optimiazation time: %fms", elapsed_seconds.count() * 1000);
            // 增加约束，保存关键帧
            saveKeyFramesAndFactor();
            std::chrono::time_point<std::chrono::system_clock> t5 = std::chrono::system_clock::now();
            elapsed_seconds = t5 - t4;
            // RCLCPP_INFO(this -> get_logger(), "save key frames time: %fms", elapsed_seconds.count() * 1000);
            // 检测是否存在回环，若存在则更新
            correctPoses();
            // 发布消息
            publish();
            std::chrono::time_point<std::chrono::system_clock> t6 = std::chrono::system_clock::now();
            elapsed_seconds = t6 - t1;
            // std::chrono::duration<double> elapsed_seconds = t - t;
            // RCLCPP_INFO(this -> get_logger(), "back end whole time: %fms", elapsed_seconds.count() * 1000);
        }
    }

    /**
     * 初始化 
    */
    void init(){
        // 全局地图初始化
        globalMap.reset(new CloudType());
        downSampleGlobalMap.reset(new pcl::VoxelGrid<PointType>());
        downSampleGlobalMap -> setLeafSize(0.5, 0.5, 0.5);

        // submap
        laserCornerFromMap.reset(new CloudType());
        laserSurfFromMap.reset(new CloudType());
        laserGroundFromMap.reset(new CloudType());

        // kdtree
        kdtreeCornerPoints.reset(new pcl::KdTreeFLANN<PointType>());
        kdtreeSurfPoints.reset(new pcl::KdTreeFLANN<PointType>());
        kdtreeGroundPoints.reset(new pcl::KdTreeFLANN<PointType>());
        
        // 最新一帧点云
        cornerCloud.reset(new CloudType());
        surfCloud.reset(new CloudType());
        groundCloud.reset(new CloudType());

        // 下采样滤波
        downSampleSubMap.reset(new pcl::VoxelGrid<PointType>());
        downSampleSubMap -> setLeafSize(0.4, 0.4, 0.4);
        downSampleKeyPoints.reset(new pcl::VoxelGrid<PointType>());
        downSampleKeyPoints -> setLeafSize(0.2, 0.2, 0.2);
        
        // 因子图
        gtsam::ISAM2Params parameters;
        parameters.relinearizeThreshold = 0.01;
        parameters.relinearizeSkip = 1;
        parameters.factorization = gtsam::ISAM2Params::QR;
        isam = new gtsam::ISAM2(parameters);

        // 初始化前一帧位姿
        previousPosPoint.x = 0;
        previousPosPoint.y = 0;
        previousPosPoint.z = 0;
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
        // 初始6自由度参数
        q_delta = Eigen::Quaterniond(all_cloud.trans_form[6], all_cloud.trans_form[3],
            all_cloud.trans_form[4], all_cloud.trans_form[5]);
        t_delta = Eigen::Vector3d(all_cloud.trans_form[0], all_cloud.trans_form[1], all_cloud.trans_form[2]);

        // 获得新一帧点云
        msgToPointCloud(cornerCloud, all_cloud.corner_less_sharp);
        msgToPointCloud(surfCloud, all_cloud.surf_less_flat);
        msgToPointCloud(groundCloud, all_cloud.ground_less_flat);

        // 下采样
        downSamplePointCloud(cornerCloud, downSampleKeyPoints);
        downSamplePointCloud(surfCloud, downSampleKeyPoints);
        downSamplePointCloud(groundCloud, downSampleKeyPoints);

        // RCLCPP_INFO(this -> get_logger(), "corner size : %ld, surf size : %ld, ground size : %ld", 
        //     cornerCloud -> size(), surfCloud -> size(), groundCloud -> size());
    }
    
    /**
     * 提取关键帧
    */
    void extractSurroundingKeyFrames(){
        laserCornerFromMap -> clear();
        laserSurfFromMap -> clear();
        laserGroundFromMap -> clear();
        for(size_t i = 0; i < recentKeyFrames.size(); ++i){
            *laserCornerFromMap += transformPointCloud(allCornerKeyFramse[recentKeyFrames[i]],
                                                       allKeyFramesPoses[recentKeyFrames[i]]);
            *laserSurfFromMap += transformPointCloud(allSurfKeyFrames[recentKeyFrames[i]],
                                                       allKeyFramesPoses[recentKeyFrames[i]]);
            *laserGroundFromMap += transformPointCloud(allGroundKeyFrames[recentKeyFrames[i]],
                                                       allKeyFramesPoses[recentKeyFrames[i]]);                
        }

        downSamplePointCloud(laserCornerFromMap, downSampleSubMap);
        downSamplePointCloud(laserSurfFromMap, downSampleSubMap);
        downSamplePointCloud(laserGroundFromMap, downSampleSubMap);
        
        // RCLCPP_INFO(this -> get_logger(), "corner map:%ld, surf map:%ld, ground map:%ld", 
        //     laserCornerFromMap -> size(), laserSurfFromMap -> size(), laserGroundFromMap -> size());

        // globalMap -> clear();
        // *globalMap += *laserSurfFromMap;
        // *globalMap += *laserGroundFromMap;
        // sensor_msgs::msg::PointCloud2 gl_map;
        // pcl::toROSMsg(*globalMap, gl_map);

        // gl_map.header.frame_id = "map";
        // gl_map.header.stamp = this -> now();
        // pubGlobalMap -> publish(gl_map);
    }

    /**
     * 图优化
    */
    void scan2MapOptimization(){
        // 待优化四元数与偏移
        auto trans = [&](const PointType &pointFrom, PointType &pointTo){
            Eigen::Vector3d p(pointFrom.x, pointFrom.y, pointFrom.z);
            p = q_w_last * (q_delta * p + t_delta) + t_w_last;
            pointTo.x = p.x();
            pointTo.y = p.y();
            pointTo.z = p.z();
            pointTo.intensity = pointFrom.intensity;
        };

        if(!allKeyFramesPoses.empty()){
            kdtreeCornerPoints -> setInputCloud(laserCornerFromMap);
            kdtreeSurfPoints -> setInputCloud(laserSurfFromMap);
            kdtreeGroundPoints -> setInputCloud(laserGroundFromMap);

            int cornerSharpNum = cornerCloud -> size();
            int surfFlatNum = surfCloud -> size();
            int groundFlatNum = groundCloud -> size();

            // RCLCPP_INFO(this -> get_logger(), "corner:%d, surf:%d, ground:%d", 
            //     cornerSharpNum, surfFlatNum, groundFlatNum);

            PointType pointSel;
            std::vector<int> pointSearchInd;
            std::vector<float> pointSearchSqDis;

            for(size_t opti_counter = 0; opti_counter < 2; ++opti_counter){
                ceres::LossFunction *loss_function = new ceres::HuberLoss(0.1);
                ceres::Problem::Options problem_options;

                ceres::Problem problem(problem_options);
                ceres::Manifold *q_manifold = new ceres::EigenQuaternionManifold;
                problem.AddParameterBlock(q_, 4, q_manifold);
                problem.AddParameterBlock(t_, 3);

                // 边缘点
                for(int i = 0; i < cornerSharpNum; ++i){
                    trans(cornerCloud -> points[i], pointSel);
                    kdtreeCornerPoints -> nearestKSearch(pointSel, 5, pointSearchInd, pointSearchSqDis);

                    if(pointSearchSqDis[4] < 1.0){
                        Eigen::Matrix3f matA;
                        Eigen::Matrix<float, 1, 3> matD;
                        Eigen::Matrix3f matV;

                        float cx = 0, cy = 0, cz = 0;
                        for(int j = 0; j < 5; ++j){
                            cx += laserCornerFromMap -> points[pointSearchInd[j]].x;
                            cy += laserCornerFromMap -> points[pointSearchInd[j]].y;
                            cz += laserCornerFromMap -> points[pointSearchInd[j]].z;
                        }
                        cx /= 5;
                        cy /= 5;
                        cz /= 5;

                        float a11 = 0, a12 = 0, a13 = 0, a22 = 0, a23 = 0, a33 = 0;
                        for(int j = 0; j < 5; ++j){
                            float ax = laserCornerFromMap -> points[pointSearchInd[j]].x - cx;
                            float ay = laserCornerFromMap -> points[pointSearchInd[j]].y - cy;
                            float az = laserCornerFromMap -> points[pointSearchInd[j]].z - cz;

                            a11 += ax * ax;
                            a12 += ax * ay;
                            a13 += ax * az;
                            a22 += ay * ay;
                            a23 += ay * az;
                            a33 += az * az;
                        }
                        a11 /= 5;
                        a12 /= 5;
                        a13 /= 5;
                        a22 /= 5;
                        a23 /= 5;
                        a33 /= 5;

                        matA(0, 0) = a11;
                        matA(0, 1) = a12;
                        matA(0, 2) = a13;
                        matA(1, 0) = a12;
                        matA(1, 1) = a22;
                        matA(1, 2) = a23;
                        matA(2, 0) = a13;
                        matA(2, 1) = a12;
                        matA(2, 2) = a33;

                        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> esolver(matA);

                        matD = esolver.eigenvalues().real();
                        matV = esolver.eigenvectors().real();

                        if(matD[2] > 3 * matD[1]){
                            Eigen::Vector3d curr_p{cornerCloud -> points[i].x, 
                                                   cornerCloud -> points[i].y, 
                                                   cornerCloud -> points[i].z};
                            Eigen::Vector3d point_a{cx + 0.1 * matV(2, 0),
                                                    cy + 0.1 * matV(2, 1),
                                                    cz + 0.1 * matV(2, 2)};
                            Eigen::Vector3d point_b{cx - 0.1 * matV(2, 0),
                                                    cy - 0.1 * matV(2, 1),
                                                    cz - 0.1 * matV(2, 2)};
                            ceres::CostFunction *cost_function = 
                                LidarEdgeFactor::Create(curr_p, point_a, point_b);
                            problem.AddResidualBlock(
                                cost_function, loss_function, q_, t_);
                        }
                    }
                    
                }

                // 平面点
                for(int i = 0; i < surfFlatNum; ++i){
                    trans(surfCloud -> points[i], pointSel);
                    kdtreeSurfPoints -> nearestKSearch(pointSel, 5, pointSearchInd, pointSearchSqDis);

                    if(pointSearchSqDis[4] < 1.0){
                        Eigen::Matrix<double, 5, 1> matb;
                        matb.fill(-1);
                        Eigen::Matrix<double, 5, 3> matA;
                        Eigen::Matrix<double, 3, 1> matX;

                        for(int j = 0; j < 5; ++j){
                            matA(j, 0) = laserSurfFromMap -> points[pointSearchInd[j]].x;
                            matA(j, 1) = laserSurfFromMap -> points[pointSearchInd[j]].y;
                            matA(j, 2) = laserSurfFromMap -> points[pointSearchInd[j]].z;
                        }
                        // 求解Ax = b得到平面方程
                        matX = matA.colPivHouseholderQr().solve(matb);

                        double pa = matX(0, 0);
                        double pb = matX(1, 0);
                        double pc = matX(2, 0);
                        double pd = 1;
                        // 归一化
                        double ps = sqrt(pa * pa + pb * pb + pc * pc);
                        pa /= ps;
                        pb /= ps;
                        pc /= ps;
                        pd /= ps;


                        // 判断平面是否合理
                        bool planeValid = true;
                        for(int j = 0; j < 5; ++j){
                            if(abs(pa * laserSurfFromMap -> points[pointSearchInd[j]].x +
                                   pb * laserSurfFromMap -> points[pointSearchInd[j]].y +
                                   pc * laserSurfFromMap -> points[pointSearchInd[j]].z +
                                   pd) > 0.2){
                                planeValid = false;
                                break;
                            }
                        }

                        // 若平面合理，则优化
                        if(planeValid == true){
                            ceres::CostFunction *cost_function = 
                                LidarPlaneFactor::Create(pa, pb, pc, pd, surfCloud -> points[i]);
                            problem.AddResidualBlock(
                                cost_function, loss_function, q_, t_);
                        }
                    }  
                }
                
                // 地面点
                for(int i = 0; i < groundFlatNum; ++i){
                    trans(groundCloud -> points[i], pointSel);
                    kdtreeGroundPoints -> nearestKSearch(pointSel, 5, pointSearchInd, pointSearchSqDis);

                    if(pointSearchSqDis[4] < 1.0){
                        Eigen::Matrix<double, 5, 1> matb;
                        matb.fill(-1);
                        Eigen::Matrix<double, 5, 3> matA;
                        Eigen::Matrix<double, 3, 1> matX;

                        for(int j = 0; j < 5; ++j){
                            matA(j, 0) = laserGroundFromMap -> points[pointSearchInd[j]].x;
                            matA(j, 1) = laserGroundFromMap -> points[pointSearchInd[j]].y;
                            matA(j, 2) = laserGroundFromMap -> points[pointSearchInd[j]].z;
                        }
                        // 求解Ax = b得到平面方程
                        matX = matA.colPivHouseholderQr().solve(matb);

                        double pa = matX(0, 0);
                        double pb = matX(1, 0);
                        double pc = matX(2, 0);
                        double pd = 1;
                        // 归一化
                        double ps = sqrt(pa * pa + pb * pb + pc * pc);
                        pa /= ps;
                        pb /= ps;
                        pc /= ps;
                        pd /= ps;


                        // 判断平面是否合理
                        bool planeValid = true;
                        for(int j = 0; j < 5; ++j){
                            if(abs(pa * laserGroundFromMap -> points[pointSearchInd[j]].x +
                                   pb * laserGroundFromMap -> points[pointSearchInd[j]].y +
                                   pc * laserGroundFromMap -> points[pointSearchInd[j]].z +
                                   pd) > 0.2){
                                planeValid = false;
                                break;
                            }
                        }

                        // 若平面合理，则优化
                        if(planeValid == true){
                            ceres::CostFunction *cost_function = 
                                LidarPlaneFactor::Create(pa, pb, pc, pd, groundCloud -> points[i]);
                            problem.AddResidualBlock(
                                cost_function, loss_function, q_, t_);
                        }
                    }  
                }
                
                // 求解
                std::chrono::time_point<std::chrono::system_clock> t1 = std::chrono::system_clock::now();
                ceres::Solver::Options options;
                options.linear_solver_type = ceres::DENSE_QR;
                options.max_num_iterations = 5;
                options.minimizer_progress_to_stdout = false;
                ceres::Solver::Summary summary;
                ceres::Solve(options, &problem, &summary);
                std::chrono::time_point<std::chrono::system_clock> t2 = std::chrono::system_clock::now();
                std::chrono::duration<double> elapsed_seconds = t2 - t1;
                // RCLCPP_INFO(this -> get_logger(), "scan2map time: %fms", elapsed_seconds.count() * 1000);
                RCLCPP_INFO_STREAM(this -> get_logger(), "brief report" << summary.BriefReport());
            }
        }
        
        // 矫正位姿
        t_w_last = t_w_last + q_w_last * t_delta;
        q_w_last = q_w_last * q_delta;
        RCLCPP_INFO(this -> get_logger(), "x:%f, y:%f, z:%f", t_delta.x(), t_delta.y(), t_delta.z());
    }

    /**
     * 增加新的约束并保存关键帧
    */
    void saveKeyFramesAndFactor(){
        PointType currentPosPoint(t_w_last.x(), t_w_last.y(), t_w_last.z());

        // 噪声
        gtsam::Vector6 Vector6(6);
        Vector6 << 1e-6, 1e-6, 1e-6, 1e-8, 1e-8, 1e-6;
        auto priorNoise = gtsam::noiseModel::Diagonal::Variances(Vector6);
        auto odometryNoise = gtsam::noiseModel::Diagonal::Variances(Vector6);

        // 根据距离判断是否为新的关键帧
        bool saveThisKeyFrame = true;
        if(sqrt((previousPosPoint.x - currentPosPoint.x) *
                (previousPosPoint.x - currentPosPoint.x) +
                (previousPosPoint.y - currentPosPoint.y) *
                (previousPosPoint.y - currentPosPoint.y) +
                (previousPosPoint.z - currentPosPoint.z) *
                (previousPosPoint.z - currentPosPoint.z)) < 0.3){
            saveThisKeyFrame = false;
        }

        // 是关键帧或第一帧，则保存并更新
        if(saveThisKeyFrame == false && !allKeyFramesPoses.empty()) return;

        previousPosPoint = currentPosPoint;

        // 更新gtsam graph
        // 是第一帧
        if(allKeyFramesPoses.empty()){
            gtsamGraph.add(gtsam::PriorFactor<gtsam::Pose3>(
                0,
                gtsam::Pose3(gtsam::Rot3::Quaternion(q_w_last.w(), q_w_last.x(), q_w_last.y(), q_w_last.z()),
                             gtsam::Point3(t_w_last.x(), t_w_last.y(), t_w_last.z())),
                priorNoise));
            initialEstimate.insert(
                0,
                gtsam::Pose3(gtsam::Rot3::Quaternion(q_w_last.w(), q_w_last.x(), q_w_last.y(), q_w_last.z()),
                             gtsam::Point3(t_w_last.x(), t_w_last.y(), t_w_last.z())));
            q_keyframe_last = q_w_last;
            t_keyframe_last = t_w_last;
        }else{
            gtsam::Pose3 poseFrom = gtsam::Pose3(
                gtsam::Rot3::Quaternion(q_keyframe_last.w(), q_keyframe_last.x(), q_keyframe_last.y(), q_keyframe_last.z()),
                gtsam::Point3(t_keyframe_last.x(), t_keyframe_last.y(), t_keyframe_last.z()));
            gtsam::Pose3 poseTo = gtsam::Pose3(
                gtsam::Rot3::Quaternion(q_w_last.w(), q_w_last.x(), q_w_last.y(), q_w_last.z()),
                gtsam::Point3(t_w_last.x(), t_w_last.y(), t_w_last.z()));
            // 加入因子图中
            gtsamGraph.add(gtsam::BetweenFactor<gtsam::Pose3>(
                allKeyFramesPoses.size() - 1, allKeyFramesPoses.size(),
                poseFrom.between(poseTo), odometryNoise));
            initialEstimate.insert(
                allKeyFramesPoses.size(),
                gtsam::Pose3(
                    gtsam::Rot3::Quaternion(q_w_last.w(), q_w_last.x(), q_w_last.y(), q_w_last.z()),
                    gtsam::Point3(t_w_last.x(), t_w_last.y(), t_w_last.z())));
        }

        // 更新isam
        isam -> update(gtsamGraph, initialEstimate);
        isam -> update();

        gtsamGraph.resize(0);
        initialEstimate.clear();

        // 保存关键帧
        gtsam::Pose3 latestEstimate;

        isamCurrentEstimate = isam -> calculateEstimate();
        latestEstimate = isamCurrentEstimate.at<gtsam::Pose3>(isamCurrentEstimate.size() - 1);

        

        // 更新变换矩阵
        if(allKeyFramesPoses.size() > 1){
            q_w_last.w() = latestEstimate.rotation().toQuaternion().w();
            q_w_last.x() = latestEstimate.rotation().toQuaternion().x();
            q_w_last.y() = latestEstimate.rotation().toQuaternion().y();
            q_w_last.z() = latestEstimate.rotation().toQuaternion().z();
            t_w_last.x() = latestEstimate.translation().x();
            t_w_last.y() = latestEstimate.translation().y();
            t_w_last.z() = latestEstimate.translation().z();

            q_keyframe_last = q_w_last;
            t_keyframe_last = t_w_last;
        }
        pose thisPose;
        thisPose.q = q_w_last;
        thisPose.t = t_w_last;
        allKeyFramesPoses.push_back(thisPose);

        // 将关键帧插入滑动窗口
        
        allCornerKeyFramse.push_back(cornerCloud);
        cornerCloud.reset(new CloudType());
        allSurfKeyFrames.push_back(surfCloud);
        surfCloud.reset(new CloudType());
        allGroundKeyFrames.push_back(groundCloud);
        groundCloud.reset(new CloudType());


        int size = allKeyFramesPoses.size();

        recentKeyFrames.push_back(size - 1);
        if(recentKeyFrames.size() > 50){
            recentKeyFrames.pop_front();
        }
        
        // 将没发布的点云数据插入地图发布队列
        globalMtx.lock();
        globalMapQueue.push(size - 1);
        globalMtx.unlock();
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
        nav_msgs::msg::Odometry laserOdometry;
        laserOdometry.header.frame_id = "map";
        laserOdometry.child_frame_id = "/global_map";
        laserOdometry.pose.pose.orientation.x = q_w_last.x();
        laserOdometry.pose.pose.orientation.y = q_w_last.y();
        laserOdometry.pose.pose.orientation.z = q_w_last.z();
        laserOdometry.pose.pose.orientation.w = q_w_last.w();
        laserOdometry.pose.pose.position.x = t_w_last.x();
        laserOdometry.pose.pose.position.y = t_w_last.y();
        laserOdometry.pose.pose.position.z = t_w_last.z();

        geometry_msgs::msg::PoseStamped laserPose;
        laserPose.header = laserOdometry.header;
        laserPose.pose = laserOdometry.pose.pose;
        path.poses.push_back(laserPose);
        path.header.frame_id = "map";
        
        pubPath -> publish(path);

    }


/********工具类函数********/

    /**
     * 点云位姿变换
    */
    CloudType transformPointCloud(const CloudTypePtr cloud, const pose pose){
        CloudType tmp_cloud;
        PointType point;
        for(size_t i = 0; i < cloud -> size(); ++i){
            Eigen::Vector3d p(cloud -> points[i].x, cloud -> points[i].y, cloud -> points[i].z);
            p = pose.q * p + pose.t;
            point.x = p.x();
            point.y = p.y();
            point.z = p.z();
            point.intensity = cloud -> points[i].intensity;
            tmp_cloud.push_back(point);
        }
        return tmp_cloud;
    };

    /**
     * 下采样滤波
    */
    void downSamplePointCloud(CloudTypePtr cloud_in, pcl::VoxelGrid<PointType>::Ptr downSampleFilter){
        CloudTypePtr cloud_tmp(new CloudType());
        downSampleFilter -> setInputCloud(cloud_in);
        downSampleFilter -> filter(*cloud_tmp);
        *cloud_in = *cloud_tmp;
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