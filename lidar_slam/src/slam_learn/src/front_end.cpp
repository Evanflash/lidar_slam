#include "slam_utils.h"
#include "lidar_factor.hpp"

namespace lidarslam{

class FrontEnd : public rclcpp::Node{
private:
    // 处理后的点云
    rclcpp::Subscription<other_msgs::msg::SegCloud>::SharedPtr subSegMsg;

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
    double para_q[4];
    double para_t[3];

public:
    FrontEnd(const std::string &name)
        : Node(name)
    {

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

        isFirstFrame = true;
    }

    /**
     * 回调函数
    */
    void run(const other_msgs::msg::SegCloud &msg){
        seg_msg = msg;

        // 变量重置
        resetParameters();
        // 计算曲率
        calculateSmoothness();
        // 提取特征点
        extractFeatures();
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
        std::vector<smoothness>().swap(segmentCurvature);
        segmentCurvature.reserve(seg_msg.seg_cloud.size());
        std::vector<smoothness>().swap(groundCurvature);
        groundCurvature.reserve(seg_msg.ground_cloud.size());
        
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
                curvatures.push_back(sns);
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
                int sp = (seg_msg.seg_ring_str_ind[i] * (6 - j) + seg_msg.seg_ring_end_ind[i] * j) / 6;
                int ep = (seg_msg.seg_ring_str_ind[i] * (5 - j) + seg_msg.seg_ring_end_ind[i] * (j + 1)) / 6 - 1;

                if(sp >= ep) continue;

                std::sort(segmentCurvature.begin() + sp, segmentCurvature.begin() + ep, cmp);

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

                        if(++smallestPickedNum >= 4){
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
    
        for(int i = 0; i < seg_msg.seg_cloud.size(); ++i){
            tran(seg_msg.seg_cloud[i], point);
            surfLessFlat -> push_back(point);
        }

        // 地面点特征提取
        for(int i = 0; i < vertical_scans; ++i){
             for(int j = 0; j < 6; ++j){
                int sp = (seg_msg.grd_ring_str_ind[i] * (6 - j) + seg_msg.grd_ring_end_ind[i] * j) / 6;
                int ep = (seg_msg.grd_ring_str_ind[i] * (5 - j) + seg_msg.grd_ring_end_ind[i] * (j + 1)) / 6 - 1;

                if(sp >= ep) continue;

                std::sort(groundCurvature.begin() + sp, groundCurvature.begin() + ep, cmp);

                // 平面点
                int smallestPickedNum = 1;
                for(int k = sp; k <= ep; ++k){
                    int curInd = groundCurvature[k].index;
                    if(groundNeighborPicked[curInd] == 0 && 
                       groundCurvature[k].curvature < surfThreshold){
                        tran(seg_msg.ground_cloud[curInd], point);
                        groundSurfFlat -> push_back(point);

                        if(++smallestPickedNum >= 4){
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

        for(int i = 0; i < seg_msg.ground_cloud.size(); ++i){
            tran(seg_msg.ground_cloud[i], point);
            groundSurfLessFlat -> push_back(point);
        }
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

        if(isFirstFrame == true){
            isFirstFrame = false;
        }else{
            int cornerSharpNum = cornerSharp -> size();
            int surfFlatNum = surfFlat -> size();
            int groundFlatNum = groundSurfFlat -> size();

            // 两次优化
            for(size_t opti_counter = 0; opti_counter < 2; ++opti_counter){
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
                    pointSel = cornerSharp -> points[i];
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
                for(int i = 0; i < surfFlatNum; ++i){
                    pointSel = surfFlat -> points[i];
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
                            if(int(surfLast -> points[j].intensity) > closestPointScanID &&
                                    pointSqDis < minPointSqDis2){
                                minPointSqDis2 = pointSqDis;
                                minPointInd2 = j;
                            }else if(int(surfLast -> points[j].intensity) <= closestPointScanID &&
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
                }
            }
        }

        // 更新上一帧点云

    }

}; // class FrontEnd

} // lidarslam

int main(int argc, char** argv){
    rclcpp::init(argc, argv);

    rclcpp::shutdown();
    return 0;
}