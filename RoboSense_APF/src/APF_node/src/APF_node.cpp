
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <chrono>
#include <thread>
#include <Eigen/Dense>
#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <mutex>
#include <pcl/common/colors.h>  // 添加颜色处理头文件

class ObstacleAvoidance {
public:
    ObstacleAvoidance() : viewer_("3D Viewer") {
        // 初始化ROS节点
        ros::NodeHandle nh;
        
        // 创建点云订阅器
        cloud_sub_ = nh.subscribe("/rslidar_points", 1, &ObstacleAvoidance::cloudCallback, this);
        
        // 初始化参数
        k_attractive_ = 2.0f;
        k_repulsive_ = 2.0f;
        safe_distance_ = 1.5f;
        step_size_ = 0.15f;
        max_height_ = 3.0f;

        // 初始化位置
        current_position_ = Eigen::Vector3f(-2, -1, 1);
        target_point_ = Eigen::Vector3f(10, 5, 0);

        // 初始化点云指针（使用带强度的点云类型）
        current_cloud_.reset(new pcl::PointCloud<pcl::PointXYZI>);

        // 初始化可视化
        setupVisualization();
    }

    // 点云回调函数 - 处理带强度的点云
    void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg) {
        pcl::PointCloud<pcl::PointXYZI>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::fromROSMsg(*msg, *temp_cloud);
        
        // 使用互斥锁保护点云数据
        std::lock_guard<std::mutex> lock(cloud_mutex_);
        current_cloud_ = temp_cloud;
        new_cloud_received_ = true;  // 标记有新点云到达
        cloud_received_ = true;      // 标记已收到点云
    }

    void run() {
        ros::Rate rate(10);  // 控制循环频率为10Hz
        int step_count = 0;
        
        while (ros::ok() && !viewer_.wasStopped()) {
            auto start_time = std::chrono::high_resolution_clock::now();
            
            // 检查是否已收到点云
            if (!cloud_received_) {
                ros::spinOnce();
                rate.sleep();
                continue;
            }

            // 计算运动向量
            Eigen::Vector3f attractive, repulsive;
            {
                // 加锁访问点云
                std::lock_guard<std::mutex> lock(cloud_mutex_);
                attractive = calculateAttractiveForce();
                repulsive = calculateRepulsiveForce();
            }
            
            Eigen::Vector3f total_force = attractive + repulsive;

            // 更新位置
            if (total_force.norm() > 1e-3) {
                total_force.normalize();
                current_position_ += total_force * step_size_;
            }

            // 高度限制
            current_position_.z() = std::min(current_position_.z(), max_height_);

            // 更新可视化
            updateVisualization(attractive, repulsive, step_count++);
            viewer_.spinOnce(10);

            // ROS异步处理
            ros::spinOnce();
            rate.sleep();
        }
    }

private:
    void setupVisualization() {
        viewer_.setBackgroundColor(0, 0, 0);
        viewer_.addCoordinateSystem(1.0);

        // 目标点（蓝色球体）
        viewer_.addSphere(createPclPoint(target_point_), 0.3, 0.0, 0.0, 1.0, "target");

        // 机器人初始位置（绿色球体）
        viewer_.addSphere(createPclPoint(current_position_), 0.3, 0.0, 1.0, 0.0, "robot");
    }

    void updateVisualization(const Eigen::Vector3f& att, const Eigen::Vector3f& rep, int step) {
        // 更新点云显示（如果有新点云到达）
        {
            std::lock_guard<std::mutex> lock(cloud_mutex_);
            if (new_cloud_received_ && !current_cloud_->empty()) {
                // 创建强度颜色处理器
                pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> 
                    intensity_handler(current_cloud_, "intensity");
                
                if (!cloud_added_) {
                    // 第一次添加点云 - 使用强度着色
                    viewer_.addPointCloud<pcl::PointXYZI>(current_cloud_, intensity_handler, "obstacles");
                    
                    // 设置点云渲染属性
                    viewer_.setPointCloudRenderingProperties(
                        pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "obstacles");
                    
                    // 设置强度颜色映射范围（根据实际数据调整）
                    viewer_.setPointCloudRenderingProperties(
                        pcl::visualization::PCL_VISUALIZER_LUT_RANGE, 0.0, 255.0, "obstacles");
                    
                    // // 使用彩虹色映射
                    // viewer_.setPointCloudRenderingProperties(
                    //     pcl::visualization::PCL_VISUALIZER_COLOR_SCHEME, 
                    //     pcl::visualization::PCL_VISUALIZER_COLOR_SCHEME_RGB, 
                    //     "obstacles");
                    
                    cloud_added_ = true;
                } else {
                    // 更新现有点云
                    viewer_.updatePointCloud<pcl::PointXYZI>(current_cloud_, intensity_handler, "obstacles");
                }
                new_cloud_received_ = false;
            }
        }

        // 更新机器人位置
        viewer_.removeShape("robot");
        viewer_.addSphere(createPclPoint(current_position_), 0.3, 0.0, 1.0, 0.0, "robot");

        // 绘制轨迹
        if (step % 2 == 0) {
            std::string line_id = "line_" + std::to_string(step);
            if (step > 0) {
                Eigen::Vector3f prev_pos = current_position_ - (att + rep).normalized() * step_size_ * 5;
                viewer_.addLine(createPclPoint(prev_pos), createPclPoint(current_position_),
                              1.0, 1.0, 1.0, line_id);
            }
        }

        // 绘制力向量
        drawForceArrow(current_position_, att, "att_arrow", 0, 1, 1);
        drawForceArrow(current_position_, rep, "rep_arrow", 1, 0, 1);
    }

    pcl::PointXYZ createPclPoint(const Eigen::Vector3f& vec) const {
        return pcl::PointXYZ(vec.x(), vec.y(), vec.z());
    }

    void drawForceArrow(const Eigen::Vector3f& origin, const Eigen::Vector3f& force,
                       const std::string& id, float r, float g, float b) {
        viewer_.removeShape(id);
        if (force.norm() < 0.1f) return;

        pcl::PointXYZ start(origin.x(), origin.y(), origin.z());
        pcl::PointXYZ end(
            origin.x() + force.x() * 0.3f,
            origin.y() + force.y() * 0.3f,
            origin.z() + force.z() * 0.3f
        );
        viewer_.addArrow(start, end, r, g, b, false, id);
    }

    Eigen::Vector3f calculateAttractiveForce() {
        Eigen::Vector3f vec = target_point_ - current_position_;
        vec.z() = 0;  // 忽略高度方向
        float dist = vec.norm();
        if (dist < 0.1f) return Eigen::Vector3f::Zero();
        return k_attractive_ * vec.normalized();
    }

    Eigen::Vector3f calculateRepulsiveForce() {
        Eigen::Vector3f total_force = Eigen::Vector3f::Zero();
        for (const auto& p : current_cloud_->points) {
            // 使用带强度的点类型
            Eigen::Vector3f obstacle(p.x, p.y, p.z);
            Eigen::Vector3f vec = obstacle - current_position_;
            float dist = vec.norm();

            if (dist < safe_distance_ && p.z > 0.1f) {
                Eigen::Vector3f force_dir = -vec.normalized();
                total_force += force_dir;
            }
        }
        return total_force.normalized() * k_repulsive_;
    }

    pcl::visualization::PCLVisualizer viewer_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr current_cloud_;  // 使用带强度的点云类型
    Eigen::Vector3f current_position_;
    Eigen::Vector3f target_point_;
    float k_attractive_, k_repulsive_;
    float safe_distance_, step_size_, max_height_;

    // ROS相关成员
    ros::Subscriber cloud_sub_;
    bool cloud_received_ = false;
    bool new_cloud_received_ = false;
    bool cloud_added_ = false;
    std::mutex cloud_mutex_;  // 保护点云数据的互斥锁
};

int main(int argc, char** argv) {
    // 初始化ROS节点
    ros::init(argc, argv, "obstacle_avoidance_node");
    
    ObstacleAvoidance oa;
    oa.run();
    return 0;
}
