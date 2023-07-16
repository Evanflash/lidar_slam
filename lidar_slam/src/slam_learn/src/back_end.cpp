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
static double transformBefoMapped[6] = {0.0};


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

    // test
    Eigen::Quaterniond q_front_end;
    Eigen::Vector3d t_front_end;

    Eigen::Quaterniond q_keyframe_last;
    Eigen::Vector3d t_keyframe_last;

    // 滑动窗口
    std::deque<int> recentKeyFrames;

    // 所有点云
    std::vector<CloudTypePtr> allSegCloudKeyFrames;
    std::vector<CloudTypePtr> allGroundKeyFrames;

    // 对应位姿
    std::vector<pose> allKeyFramesPoses;

    // 子图
    CloudTypePtr keyPointsFromMap;

    // kdtree
    pcl::KdTreeFLANN<PointType>::Ptr kdtreeKeyPoints;

    // 新一帧特征点
    CloudTypePtr keyPointsLast;

    // 新一帧点云
    CloudTypePtr surfCloud;
    CloudTypePtr groundCloud;

    // 下采样滤波
    pcl::VoxelGrid<PointType>::Ptr downSampleAllCloud;
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
          t_w_last(0, 0, 0)
    {
        subAllCloud = this -> create_subscription<other_msgs::msg::AllCloud>(
            "/all_cloud", 1, std::bind(&BackEnd::getMsg, this, std::placeholders::_1));

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
                *globalMap += transformPointCloud(allSegCloudKeyFrames[curPubCloud[i]],
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
            std::chrono::time_point<std::chrono::system_clock> start = std::chrono::system_clock::now();
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
            std::chrono::time_point<std::chrono::system_clock> end = std::chrono::system_clock::now();
            std::chrono::duration<double> elapsed_seconds = end - start;
            // RCLCPP_INFO(this -> get_logger(), "back end whole time: %fms", elapsed_seconds.count() * 1000);
        }
    }

    /**
     * 初始化 
    */
    void init(){
        globalMap.reset(new CloudType());
        downSampleGlobalMap.reset(new pcl::VoxelGrid<PointType>());
        downSampleGlobalMap -> setLeafSize(1.0, 1.0, 1.0);

        keyPointsFromMap.reset(new CloudType());

        kdtreeKeyPoints.reset(new pcl::KdTreeFLANN<PointType>());

        keyPointsLast.reset(new CloudType());

        surfCloud.reset(new CloudType());
        groundCloud.reset(new CloudType());

        downSampleAllCloud.reset(new pcl::VoxelGrid<PointType>());
        downSampleAllCloud -> setLeafSize(0.4, 0.4, 0.4);

        gtsam::ISAM2Params parameters;
        parameters.relinearizeThreshold = 0.01;
        parameters.relinearizeSkip = 1;
        parameters.factorization = gtsam::ISAM2Params::QR;
        isam = new gtsam::ISAM2(parameters);
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
        // x y z qx qy qz
        for(int i = 0; i < 6; ++i){
            transformBefoMapped[i] = 0;
        }
        Eigen::Quaterniond q_odom = (Eigen::AngleAxisd(all_cloud.trans_form[5], Eigen::Vector3d::UnitZ()) *
                                     Eigen::AngleAxisd(all_cloud.trans_form[4], Eigen::Vector3d::UnitY()) *
                                     Eigen::AngleAxisd(all_cloud.trans_form[3], Eigen::Vector3d::UnitX()));
        Eigen::Vector3d t_odom = Eigen::Vector3d(all_cloud.trans_form[0], 
                                                 all_cloud.trans_form[1], 
                                                 all_cloud.trans_form[2]);
        // 将帧间变换参数转换为帧到地图的变换参数
        // RCLCPP_INFO(this -> get_logger(), "front end : x%f, y%f, z%f", t_odom.x(), t_odom.y(), t_odom.z());
        t_w_last = q_w_last * t_odom + t_w_last;
        q_w_last = q_w_last * q_odom;

        q_front_end = q_odom;
        t_front_end = t_odom;

        // 获得点云
        keyPointsLast -> clear();
        CloudTypePtr tmp(new CloudType());
        msgToPointCloud(tmp, all_cloud.surf_flat);
        *keyPointsLast += *tmp;
        tmp -> clear();
        msgToPointCloud(tmp, all_cloud.ground_flat);
        *keyPointsLast += *tmp;
        RCLCPP_INFO(this -> get_logger(), "key points size: %ld", keyPointsLast -> size());

        msgToPointCloud(surfCloud, all_cloud.surf_less_flat);
        msgToPointCloud(groundCloud, all_cloud.ground_less_flat);

    }
    
    /**
     * 提取关键帧
    */
    void extractSurroundingKeyFrames(){
        keyPointsFromMap -> clear();
        for(size_t i = 0; i < recentKeyFrames.size(); ++i){
            *keyPointsFromMap += transformPointCloud(allSegCloudKeyFrames[recentKeyFrames[i]],
                                                       allKeyFramesPoses[recentKeyFrames[i]]);
            *keyPointsFromMap += transformPointCloud(allGroundKeyFrames[recentKeyFrames[i]],
                                                       allKeyFramesPoses[recentKeyFrames[i]]);                
        }

        downSamplePointCloud(keyPointsFromMap, downSampleAllCloud);
    }

    /**
     * 图优化
    */
    void scan2MapOptimization(){
        auto trans = [&](const PointType &pointFrom, PointType &pointTo){
            Eigen::Vector3d p(pointFrom.x, pointFrom.y, pointFrom.z);
            p = q_w_last * p + t_w_last;
            pointTo.x = p.x();
            pointTo.y = p.y();
            pointTo.z = p.z();
            pointTo.intensity = pointFrom.intensity;
        };

        if(!allKeyFramesPoses.empty()){
            kdtreeKeyPoints -> setInputCloud(keyPointsFromMap);

            int keyPointsNum = keyPointsLast -> size();

            PointType pointSel;
            std::vector<int> pointSearchInd;
            std::vector<float> pointSearchSqDis;

            for(size_t opti_counter = 0; opti_counter < 2; ++opti_counter){
                ceres::LossFunction *loss_function = new ceres::HuberLoss(0.1);
                ceres::Problem::Options problem_options;

                ceres::Problem problem(problem_options);
                problem.AddParameterBlock(transformBefoMapped, 3);
                problem.AddParameterBlock(transformBefoMapped, 3);

                Eigen::Matrix<double, 5, 1> matb;
                matb.fill(-1);
                Eigen::Matrix<double, 5, 3> matA;
                Eigen::Matrix<double, 3, 1> matX;

                for(int i = 0; i < keyPointsNum; ++i){
                    trans(keyPointsLast -> points[i], pointSel);
                    kdtreeKeyPoints -> nearestKSearch(pointSel, 5, pointSearchInd, pointSearchSqDis);

                    if(pointSearchSqDis[4] < 1.0){
                        for(int j = 0; j < 5; ++j){
                            matA(j, 0) = keyPointsFromMap -> points[pointSearchInd[j]].x;
                            matA(j, 1) = keyPointsFromMap -> points[pointSearchInd[j]].y;
                            matA(j, 2) = keyPointsFromMap -> points[pointSearchInd[j]].z;
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
                            if(abs(pa * keyPointsFromMap -> points[pointSearchInd[j]].x +
                                   pb * keyPointsFromMap -> points[pointSearchInd[j]].y +
                                   pc * keyPointsFromMap -> points[pointSearchInd[j]].z +
                                   pd) > 0.2){
                                planeValid = false;
                                break;
                            }
                        }

                        // 若平面合理，则优化
                        if(planeValid == true){
                            ceres::CostFunction *cost_function = 
                                PlaneFactorBack::Create(pa, pb, pc, pd, pointSel);
                            problem.AddResidualBlock(
                                cost_function, loss_function, transformBefoMapped + 3, transformBefoMapped);
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
        }

        // if(!allKeyFramesPoses.empty()){
        //     kdtreeCornerFromMap -> setInputCloud(laserCornerFromMap);
        //     kdtreeGroundFromMap -> setInputCloud(laserGroundFromMap);

        //     int cornerPointsNum = cornerSharpLast -> size();
        //     int groundPointsNum = groundSurfLast -> size();

        //     PointType pointSel;
        //     std::vector<int> pointSearchInd;
        //     std::vector<float> pointSearchSqDis;

        //     // 地面点优化
        //     for(size_t opti_counter = 0; opti_counter < 2; ++opti_counter){
        //         ceres::LossFunction *loss_function = new ceres::HuberLoss(0.1);
        //         ceres::Problem::Options problem_options;

        //         ceres::Problem problem(problem_options);
        //         problem.AddParameterBlock(transformBefoMapped + 3, 2);
        //         problem.AddParameterBlock(transformBefoMapped + 2, 1);

        //         Eigen::Matrix<double, 5, 1> matb;
        //         matb.fill(-1);
        //         Eigen::Matrix<double, 5, 3> matA;
        //         Eigen::Matrix<double, 3, 1> matX;

        //         for(int i = 0; i < groundPointsNum; ++i){
        //             trans(groundSurfLast -> points[i], pointSel);
        //             kdtreeGroundFromMap -> nearestKSearch(pointSel, 5, pointSearchInd, pointSearchSqDis);

        //             if(pointSearchSqDis[4] < 1.0){
        //                 for(int j = 0; j < 5; ++j){
        //                     matA(j, 0) = laserGroundFromMap -> points[pointSearchInd[j]].x;
        //                     matA(j, 1) = laserGroundFromMap -> points[pointSearchInd[j]].y;
        //                     matA(j, 2) = laserGroundFromMap -> points[pointSearchInd[j]].z;
        //                 }
        //                 // 求解Ax = b得到平面方程
        //                 matX = matA.colPivHouseholderQr().solve(matb);

        //                 double pa = matX(0, 0);
        //                 double pb = matX(1, 0);
        //                 double pc = matX(2, 0);
        //                 double pd = 1;
        //                 // 归一化
        //                 double ps = sqrt(pa * pa + pb * pb + pc * pc);
        //                 pa /= ps;
        //                 pb /= ps;
        //                 pc /= ps;
        //                 pd /= ps;


        //                 // 判断平面是否合理
        //                 bool planeValid = true;
        //                 for(int j = 0; j < 5; ++j){
        //                     if(abs(pa * laserGroundFromMap -> points[pointSearchInd[j]].x +
        //                            pb * laserGroundFromMap -> points[pointSearchInd[j]].y +
        //                            pc * laserGroundFromMap -> points[pointSearchInd[j]].z +
        //                            pd) > 0.2){
        //                         planeValid = false;
        //                         break;
        //                     }
        //                 }

        //                 // 若平面合理，则优化
        //                 if(planeValid == true){
        //                     ceres::CostFunction *cost_function = 
        //                         GroundPlaneBack::Create(pa, pb, pc, pd, pointSel);
        //                     problem.AddResidualBlock(
        //                         cost_function, loss_function, transformBefoMapped + 3, transformBefoMapped + 2);
        //                 }
        //             }  
        //         }
        //         // 求解
        //         ceres::Solver::Options options;
        //         options.linear_solver_type = ceres::DENSE_QR;
        //         options.max_num_iterations = 4;
        //         options.minimizer_progress_to_stdout = false;
        //         ceres::Solver::Summary summary;
        //         ceres::Solve(options, &problem, &summary);
        //     }

        //     // RCLCPP_INFO(this -> get_logger(), "ground : x:%f, y:%f, z:%f, qx:%f, qy:%f, qz:%f", 
        //     //     transformBefoMapped[0], transformBefoMapped[1], transformBefoMapped[2], 
        //     //     transformBefoMapped[3], transformBefoMapped[4], transformBefoMapped[5]);
            
        //     // 计算 qyx
        //     Eigen::AngleAxisd roll(Eigen::AngleAxisd(transformBefoMapped[3], Eigen::Vector3d::UnitX()));
        //     Eigen::AngleAxisd pitch(Eigen::AngleAxisd(transformBefoMapped[4], Eigen::Vector3d::UnitY()));
        //     Eigen::Matrix3d qyx;
        //     qyx = pitch * roll;

        //     // 边缘点优化
        //     for(size_t opti_counter = 0; opti_counter < 2; ++opti_counter){
        //         ceres::LossFunction *loss_function = new ceres::HuberLoss(0.1);
        //         ceres::Problem::Options problem_options;

        //         ceres::Problem problem(problem_options);
        //         problem.AddParameterBlock(transformBefoMapped + 5, 1);
        //         problem.AddParameterBlock(transformBefoMapped, 2);
                
        //         // Eigen::Matrix<double, 3, 3> matA;
        //         // Eigen::Vector3d matD;
        //         // Eigen::Matrix3d matV;

        //         Eigen::Matrix<double, 5, 1> matb;
        //         matb.fill(-1);
        //         Eigen::Matrix<double, 5, 3> matA;
        //         Eigen::Matrix<double, 3, 1> matX;

        //         for(int i = 0; i < cornerPointsNum; ++i){
        //             trans(cornerSharpLast -> points[i], pointSel);
        //             kdtreeCornerFromMap -> nearestKSearch(pointSel, 5, pointSearchInd, pointSearchSqDis);

        //             if(pointSearchSqDis[4] < 1.0){
        //                 for(int j = 0; j < 5; ++j){
        //                     matA(j, 0) = laserCornerFromMap -> points[pointSearchInd[j]].x;
        //                     matA(j, 1) = laserCornerFromMap -> points[pointSearchInd[j]].y;
        //                     matA(j, 2) = laserCornerFromMap -> points[pointSearchInd[j]].z;
        //                 }
        //                 // 求解Ax = b得到平面方程
        //                 matX = matA.colPivHouseholderQr().solve(matb);

        //                 double pa = matX(0, 0);
        //                 double pb = matX(1, 0);
        //                 double pc = matX(2, 0);
        //                 double pd = 1;
        //                 // 归一化
        //                 double ps = sqrt(pa * pa + pb * pb + pc * pc);
        //                 pa /= ps;
        //                 pb /= ps;
        //                 pc /= ps;
        //                 pd /= ps;


        //                 // 判断平面是否合理
        //                 bool planeValid = true;
        //                 for(int j = 0; j < 5; ++j){
        //                     if(abs(pa * laserCornerFromMap -> points[pointSearchInd[j]].x +
        //                            pb * laserCornerFromMap -> points[pointSearchInd[j]].y +
        //                            pc * laserCornerFromMap -> points[pointSearchInd[j]].z +
        //                            pd) > 0.2){
        //                         planeValid = false;
        //                         break;
        //                     }
        //                 }

        //                 // 若平面合理，则优化
        //                 if(planeValid == true){
        //                     ceres::CostFunction *cost_function = 
        //                         CornerPlaneBack::Create(pa, pb, pc, pd, qyx, transformBefoMapped[2], pointSel);
        //                     problem.AddResidualBlock(
        //                         cost_function, loss_function, transformBefoMapped + 5, transformBefoMapped);
        //                 }
        //             }

        //             // if(pointSearchSqDis[4] < 1.0){
        //             //     float cx = 0, cy = 0, cz = 0;
        //             //     for(int j = 0; j < 5; ++j){
        //             //         cx += laserCornerFromMap -> points[pointSearchInd[j]].x;
        //             //         cy += laserCornerFromMap -> points[pointSearchInd[j]].y;
        //             //         cz += laserCornerFromMap -> points[pointSearchInd[j]].z;
        //             //     }
        //             //     cx /= 5;
        //             //     cy /= 5;
        //             //     cz /= 5;

        //             //     float a11 = 0, a12 = 0, a13 = 0, a22 = 0, a23 = 0, a33 = 0;
        //             //     for(int j = 0; j < 5; ++j){
        //             //         float ax = laserCornerFromMap -> points[pointSearchInd[j]].x - cx;
        //             //         float ay = laserCornerFromMap -> points[pointSearchInd[j]].y - cy;
        //             //         float az = laserCornerFromMap -> points[pointSearchInd[j]].z - cz;

        //             //         a11 += ax * ax;
        //             //         a12 += ax * ay;
        //             //         a13 += ax * az;
        //             //         a22 += ay * ay;
        //             //         a23 += ay * az;
        //             //         a33 += az * az;
        //             //     }
        //             //     a11 /= 5;
        //             //     a12 /= 5;
        //             //     a13 /= 5;
        //             //     a22 /= 5;
        //             //     a23 /= 5;
        //             //     a33 /= 5;

        //             //     matA << a11, a12, a13,
        //             //             a12, a22, a23,
        //             //             a13, a23, a33;
        //             //     Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> esolver(matA);
        //             //     matD = esolver.eigenvalues().real();
        //             //     matV = esolver.eigenvectors().real();

        //             //     // 判断是否为直线
        //             //     if(matD[2] > 3 * matD[1]){
        //             //         Eigen::Vector3d point_a{cx + 0.1 * matV(2, 0),
        //             //                                 cy + 0.1 * matV(2, 1),
        //             //                                 cz + 0.1 * matV(2, 2)};
        //             //         Eigen::Vector3d point_b{cx - 0.1 * matV(2, 0),
        //             //                                 cy - 0.1 * matV(2, 1),
        //             //                                 cz - 0.1 * matV(2, 2)};
        //             //         Eigen::Vector3d curr_point{pointSel.x, pointSel.y, pointSel.z};
        //             //         RCLCPP_INFO(this -> get_logger(), "%f, %f, %f", matV(0, 0), matV(0, 1), matV(0, 2));
        //             //         ceres::CostFunction *cost_function = 
        //             //             CornerFactor::Create(curr_point, point_a, point_b, qyx, transformBefoMapped[2]);
        //             //         problem.AddResidualBlock(
        //             //             cost_function, loss_function, transformBefoMapped + 5, transformBefoMapped);
        //             //     }
        //             // }
                     
        //         }
        //         // 求解
        //         ceres::Solver::Options options;
        //         options.linear_solver_type = ceres::DENSE_QR;
        //         options.max_num_iterations = 4;
        //         options.minimizer_progress_to_stdout = false;
        //         ceres::Solver::Summary summary;
        //         ceres::Solve(options, &problem, &summary);
        //     }
        // }
        // // if(transformBefoMapped[0] * transformBefoMapped[0] +
        // //    transformBefoMapped[1] * transformBefoMapped[1] +
        // //    transformBefoMapped[2] * transformBefoMapped[2] > 2){
        // //     return;
        // // }
       
        // 将帧间变换参数转换为帧到地图的变换参数
        // x y z qx qy qz
        Eigen::Quaterniond q_delta = (Eigen::AngleAxisd(transformBefoMapped[5], Eigen::Vector3d::UnitZ()) *
                                      Eigen::AngleAxisd(transformBefoMapped[4], Eigen::Vector3d::UnitY()) *
                                      Eigen::AngleAxisd(transformBefoMapped[3], Eigen::Vector3d::UnitX()));
        Eigen::Vector3d t_delta = Eigen::Vector3d(transformBefoMapped[0], 
                                                  transformBefoMapped[1], 
                                                  transformBefoMapped[2]);

        t_w_last = q_delta * t_w_last + t_delta;
        q_w_last = q_delta * q_w_last;

        t_front_end = q_delta * t_front_end + t_delta;
        q_front_end = q_delta * q_front_end;

        RCLCPP_INFO(this -> get_logger(), "back end : x:%f, y:%f, z:%f", 
            t_front_end.x(), t_front_end.y(), t_front_end.z());
    }

    /**
     * 增加新的约束并保存关键帧
    */
    void saveKeyFramesAndFactor(){
        PointType currentPosPoint(t_w_last.x(), t_w_last.x(), t_w_last.x());

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
                    gtsam::Rot3::Quaternion(q_keyframe_last.w(), q_keyframe_last.x(), q_keyframe_last.y(), q_keyframe_last.z()),
                    gtsam::Point3(t_keyframe_last.x(), t_keyframe_last.y(), t_keyframe_last.z())));
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
        
        allSegCloudKeyFrames.push_back(surfCloud);
        surfCloud.reset(new CloudType());
        allGroundKeyFrames.push_back(groundCloud);
        groundCloud.reset(new CloudType());

        if(allSegCloudKeyFrames.size() != allGroundKeyFrames.size()){
            RCLCPP_WARN(this -> get_logger(), "segcloud size != groundcloud size!");
        }
        int size = allSegCloudKeyFrames.size();

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