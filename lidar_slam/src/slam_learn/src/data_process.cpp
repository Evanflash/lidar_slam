#include "slam_utils.h"

namespace lidarslam{

static const std::string PARAM_VERTICAL_SCANS = "laser.vertical_scans";
static const std::string PARAM_HORIZONTAL_SCANS = "laser.horizontal_scans";
static const std::string PARAM_ANGLE_BOTTOM = "laser.ang_bottom";
static const std::string PARAM_ANGLE_TOP = "laser.ang_top";
static const std::string PARAM_GROUND_INDEX = "laser.ground_scan_index";
static const std::string PARAM_SENSOR_ANGLE = "laser.sensor_mount_angle";
static const std::string PARAM_SEGMENT_THETA = "projection.segment_theta";
static const std::string PARAM_SEGMENT_VALID_POINT_NUM = "projection.segment_valid_point_num";
static const std::string PARAM_SEGMENT_VALID_LINE_NUM = "projection.segment_valid_line_num";


class DataProcess : public rclcpp::Node{
private:
    // 消息发布与接收
    SubPointCloud subLaserCloudIn;
    rclcpp::Publisher<other_msgs::msg::SegCloud>::SharedPtr pubSegCloud;

    PubPointCloud showSegCloud;
    CloudTypePtr cloud;

    // 消息
    other_msgs::msg::SegCloud seg_msg;

    // 中间变量
    Eigen::MatrixXf range_mat;
    Eigen::MatrixXf label_mat;
    Eigen::Matrix<int8_t, Eigen::Dynamic, Eigen::Dynamic> ground_mat;

    CloudTypePtr cloud_in;
    CloudTypePtr full_cloud;

    int label_count;

    // 分辨率
    float x_resolution;
    float y_resolution;

    // 配置参数
    int vertical_scans;
    int horizontal_scans;
    float ang_bottom;
    float ang_top;
    float segment_theta;
    int segment_valid_point_num;
    int segment_valid_line_num;
    int ground_scan_index;
    float sensor_mount_angle;

public:
    DataProcess(const std::string &name)
        : Node(name)
    {
        pubSegCloud = this -> create_publisher<other_msgs::msg::SegCloud>(
            "/seg_cloud", 1);
        subLaserCloudIn = this -> create_subscription<sensor_msgs::msg::PointCloud2>(
            "/velodyne_points", 1, std::bind(&DataProcess::run, this, std::placeholders::_1));

        showSegCloud = this -> create_publisher<sensor_msgs::msg::PointCloud2>(
            "/showseg", 1
        );

        // 声明参数
        this -> declare_parameter(PARAM_VERTICAL_SCANS, vertical_scans);
        this -> declare_parameter(PARAM_HORIZONTAL_SCANS, horizontal_scans);
        this -> declare_parameter(PARAM_ANGLE_BOTTOM, ang_bottom);
        this -> declare_parameter(PARAM_ANGLE_TOP, ang_top);
        this -> declare_parameter(PARAM_GROUND_INDEX, ground_scan_index);
        this -> declare_parameter(PARAM_SENSOR_ANGLE, sensor_mount_angle);
        this -> declare_parameter(PARAM_SEGMENT_THETA, segment_theta);
        this -> declare_parameter(PARAM_SEGMENT_VALID_POINT_NUM, segment_valid_point_num);
        this -> declare_parameter(PARAM_SEGMENT_VALID_LINE_NUM, segment_valid_line_num);

        // 更新参数
        if(!this -> get_parameter(PARAM_VERTICAL_SCANS, vertical_scans)){
            RCLCPP_WARN(this -> get_logger(), "data_process_node: %s not found", PARAM_VERTICAL_SCANS.c_str());
        }
        if(!this -> get_parameter(PARAM_HORIZONTAL_SCANS, horizontal_scans)){
            RCLCPP_WARN(this -> get_logger(), "data_process_node: %s not found", PARAM_HORIZONTAL_SCANS.c_str());
        }
        if(!this -> get_parameter(PARAM_ANGLE_BOTTOM, ang_bottom)){
            RCLCPP_WARN(this -> get_logger(), "data_process_node: %s not found", PARAM_ANGLE_BOTTOM.c_str());
        }
        if(!this -> get_parameter(PARAM_ANGLE_TOP, ang_top)){
            RCLCPP_WARN(this -> get_logger(), "data_process_node: %s not found", PARAM_ANGLE_TOP.c_str());
        }
        if(!this -> get_parameter(PARAM_GROUND_INDEX, ground_scan_index)){
            RCLCPP_WARN(this -> get_logger(), "data_process_node: %s not found", PARAM_GROUND_INDEX.c_str());
        }
        if(!this -> get_parameter(PARAM_SENSOR_ANGLE, sensor_mount_angle)){
            RCLCPP_WARN(this -> get_logger(), "data_process_node: %s not found", PARAM_SENSOR_ANGLE.c_str());
        }
        if(!this -> get_parameter(PARAM_SEGMENT_THETA, segment_theta)){
            RCLCPP_WARN(this -> get_logger(), "data_process_node: %s not found", PARAM_SEGMENT_THETA.c_str());
        }
        if(!this -> get_parameter(PARAM_SEGMENT_VALID_POINT_NUM, segment_valid_point_num)){
            RCLCPP_WARN(this -> get_logger(), "data_process_node: %s not found", PARAM_SEGMENT_VALID_POINT_NUM.c_str());
        }
        if(!this -> get_parameter(PARAM_SEGMENT_VALID_LINE_NUM, segment_valid_line_num)){
            RCLCPP_WARN(this -> get_logger(), "data_process_node: %s not found", PARAM_SEGMENT_VALID_LINE_NUM.c_str());
        }
        
        // 初始化
        init();
    }   

    void run(const sensor_msgs::msg::PointCloud2 &cloud_msg){
        pcl::fromROSMsg(cloud_msg, *cloud_in);
        std::vector<int> indices;
        pcl::removeNaNFromPointCloud(*cloud_in, *cloud_in, indices);

        seg_msg.header = cloud_msg.header;
    
        // 变量重置
        resetParameters();    
        // 投影点云
        projectPointCloud();
        // 去除地面点
        groundRemoval();
        // 聚类分割
        cloudSegmentation();
        // 发布消息
        publishSegmsg();
    }

private:
    /**
     * 初始化
    */
    void init(){
        // 参数初始化
        x_resolution = (M_PI * 2) / horizontal_scans;
        y_resolution = (M_PI / 180.0) * (ang_top - ang_bottom) / float(vertical_scans - 1);
        ang_bottom = (ang_bottom - 0.1) * (M_PI / 180.0);
        segment_theta *= (M_PI / 180.0);
        sensor_mount_angle *= (M_PI / 180.0);

        // 点云智能指针分配空间
        const size_t cloud_size = vertical_scans * horizontal_scans;
        cloud_in.reset(new CloudType());
        full_cloud.reset(new CloudType());

        // 分配大小
        full_cloud -> resize(cloud_size);

        // 矩阵分配大小
        range_mat.resize(vertical_scans, horizontal_scans);
        ground_mat.resize(vertical_scans, horizontal_scans);
        label_mat.resize(vertical_scans, horizontal_scans);

        cloud.reset(new CloudType());
    }

    /**
     * 重置中间变量，以进行下一帧操作
    */ 
    void resetParameters(){
        const size_t cloud_size = vertical_scans * horizontal_scans;
        PointType nanPoint(std::numeric_limits<float>::quiet_NaN(), 
                           std::numeric_limits<float>::quiet_NaN(), 
                           std::numeric_limits<float>::quiet_NaN(), 
                           -1);

        std::fill(full_cloud -> points.begin(), full_cloud -> points.end(), nanPoint);

        // 矩阵重置
        range_mat.fill(__FLT_MAX__);
        ground_mat.setZero();
        label_mat.setZero();

        // 消息重置
        std::vector<other_msgs::msg::Point>().swap(seg_msg.seg_cloud);
        seg_msg.seg_cloud.reserve(cloud_size);
        seg_msg.seg_range.assign(cloud_size, 0);
        seg_msg.is_ground.assign(cloud_size, 0);
        seg_msg.seg_ring_str_ind.assign(vertical_scans, 0);
        seg_msg.seg_ring_end_ind.assign(vertical_scans, 0);
        seg_msg.seg_cloud_col_ind.assign(cloud_size, 0);

        // 重置聚类标签
        label_count = 1;
    }

    /**
     * 将点云投影在二维平面上
    */ 
    void projectPointCloud(){
        const size_t cloud_size = cloud_in -> size();

        for(size_t i = 0; i < cloud_size; ++i){
            PointType curPoint = cloud_in -> points[i];

            float range = sqrt(curPoint.x * curPoint.x +
                               curPoint.y * curPoint.y +
                               curPoint.z * curPoint.z);

            // 排除距离过近的点 或 距离过远的点
            if(range < 2.0){
                continue;
            }

            // 获得在二维图像中，该点的横纵坐标
            float verticalAngle = std::asin(curPoint.z / range);
            int rowInd = (verticalAngle - ang_bottom) / y_resolution;
            if(rowInd < 0 || rowInd >= vertical_scans){
                continue;
            }

            float horizonAngle = std::atan2(curPoint.x, curPoint.y);
            int colInd = (horizonAngle + 2 * M_PI) / x_resolution;
            //-round((horizonAngle - M_PI_2) / x_resolution) + horizontal_scans * 0.5;
            if(colInd >= horizontal_scans){
                colInd -= horizontal_scans / 2;
            }
            if(colInd < 0 || colInd >= horizontal_scans){
                continue;
            }

            range_mat(rowInd, colInd) = range;
            
            curPoint.intensity = (float)rowInd + (float)colInd / 10000.0;
            size_t index = colInd + rowInd * horizontal_scans;
            full_cloud -> points[index] = curPoint;
        }

    }

    /**
     * 地面点滤除
    */
    void groundRemoval(){
        for(int j = 0; j < horizontal_scans; ++j){
            for(int i = 0; i < ground_scan_index; ++i){
                int lowerInd = j + i * horizontal_scans;
                int upperInd = j + (i + 1) * horizontal_scans;

                if(full_cloud -> points[lowerInd].intensity == -1 ||
                   full_cloud -> points[upperInd].intensity == -1){
                    ground_mat(i, j) = -1;
                    continue;
                }
                
                float dX = full_cloud -> points[upperInd].x - full_cloud -> points[lowerInd].x;
                float dY = full_cloud -> points[upperInd].y - full_cloud -> points[lowerInd].y;
                float dZ = full_cloud -> points[upperInd].z - full_cloud -> points[lowerInd].z;

                // 计算仰角，小于阈值则为地面点
                float vertical_angle = std::atan2(fabs(dZ), sqrt(dX * dX + dY * dY + dZ * dZ));
                if((vertical_angle - sensor_mount_angle) <= (M_PI / 18.0)){
                    ground_mat(i, j) = 1;
                    ground_mat(i + 1, j) = 1;
                }
            }
        }

        // 标记地面点和无效点
        for(int i = 0; i < vertical_scans; ++i){
            for(int j = 0; j < horizontal_scans; ++j){
                if(ground_mat(i, j) == 1 || range_mat(i, j) == __FLT_MAX__){
                    label_mat(i, j) = -1;
                }
            }
        }
    }

    /**
     * 点云分割
    */
    void cloudSegmentation(){
        // 聚类分割
        for(int i = 0; i < vertical_scans; ++i)
            for(int j = 0; j < horizontal_scans; ++j)
                if(label_mat(i, j) == 0) labelComponents(i, j);

        // 提取分割后点云
        int curInd = 0;
        for(int i = 0; i < vertical_scans; ++i){
            seg_msg.seg_ring_str_ind[i] = curInd - 1 + 5;
            for(int j = 0; j < horizontal_scans; ++j){
                if((label_mat(i, j) > 0 && label_mat(i, j) != 999999) || ground_mat(i, j) == 1){
                    if(ground_mat(i, j) == 1 && j % 5 != 0) 
                        continue;
                    int ind = j + i * horizontal_scans;
                    other_msgs::msg::Point point_msg;
                    point_msg.x = full_cloud -> points[ind].x;
                    point_msg.y = full_cloud -> points[ind].y;
                    point_msg.z = full_cloud -> points[ind].z;
                    point_msg.i = full_cloud -> points[ind].intensity;
                    seg_msg.seg_cloud.push_back(point_msg);
                    seg_msg.seg_range[curInd] = range_mat(i, j);
                    seg_msg.is_ground[curInd] = (ground_mat(i, j) == 1);
                    seg_msg.seg_cloud_col_ind[curInd] = j;
                    ++curInd;

                    cloud -> push_back(full_cloud -> points[ind]);
                    
                }
            }
            seg_msg.seg_ring_end_ind[i] = curInd - 1 - 5;
        }

        // RCLCPP_INFO(this -> get_logger(), "%ld", seg_msg.seg_cloud.size());
    }

   /**
    * 聚类并判断一个点云类是否有效
   */
    void labelComponents(int row, int col){
        const float segmentTheta = tan(segment_theta);

        std::unordered_set<int> lineHash;
        const size_t cloud_size = vertical_scans * horizontal_scans;
        using Coord2D = Eigen::Vector2i;
        boost::circular_buffer<Coord2D> queue(cloud_size);
        boost::circular_buffer<Coord2D> all_pushed(cloud_size);

        queue.push_back({row, col});
        all_pushed.push_back({row, col});
        
        const Coord2D neighbor[4] = {{0, -1}, {-1, 0}, {1, 0}, {0, 1}};

        while(queue.size() > 0){
            Coord2D fromInd = queue.front();
            queue.pop_front();
            // 分配标签
            label_mat(fromInd.x(), fromInd.y()) = label_count;
            // 查找临近点
            for(const auto& iter : neighbor){
                int nxtIndx = fromInd.x() + iter.x();
                int nxtIndy = fromInd.y() + iter.y();
                nxtIndy = (nxtIndy + horizontal_scans) % horizontal_scans;
                // 判断临近点的合法性以及是否访问过
                if(nxtIndx < 0 || nxtIndx >= vertical_scans || label_mat(nxtIndx, nxtIndy) != 0){
                    continue;
                }
                // 计算当前点与临近点的角度是否大于阈值
                float d1 = range_mat(fromInd.x(), fromInd.y());        
                float d2 = range_mat(nxtIndx, nxtIndy);
                
                if(d1 < d2) std::swap(d1, d2); // 保证d1 > d2

                float alpha = (iter.x() == 0) ? x_resolution : y_resolution;
                float tang = (d2 * sin(alpha)) / (d1 - d2 * cos(alpha));

                if(tang > segmentTheta){
                    queue.push_back({nxtIndx, nxtIndy});
                    label_mat(nxtIndx, nxtIndy) = label_count;
                    lineHash.insert(nxtIndx);
                    all_pushed.push_back({nxtIndx, nxtIndy});
                }
            }
        }

        // 判断分割点云是否合法
        if((all_pushed.size() >= 30)|| 
            (static_cast<int>(all_pushed.size()) >= segment_valid_point_num && 
             static_cast<int>(lineHash.size()) >= segment_valid_line_num)){
            ++label_count;
        }else{
            // 标记非法点
            for(size_t i = 0; i < all_pushed.size(); ++i){
                label_mat(all_pushed[i].x(), all_pushed[i].y()) = 999999;
            }
        }
    }

    /**
     * 发布消息
    */
    void publishSegmsg(){
        if(pubSegCloud -> get_subscription_count() != 0){
            pubSegCloud -> publish(seg_msg);
        }
        sensor_msgs::msg::PointCloud2 m;
        pcl::toROSMsg(*cloud, m);
        m.header.frame_id = "map";
        showSegCloud -> publish(m);
        // RCLCPP_INFO(this -> get_logger(), "data process: %ld", cloud -> size());
        cloud -> clear();
    }

};

} // lidarslam

int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    auto dp = std::make_shared<lidarslam::DataProcess>("data_process");
    RCLCPP_INFO(dp -> get_logger(), "\033[1;32m---->\033[0m Started.");
    rclcpp::spin(dp);
    rclcpp::shutdown();
    return 0;
}