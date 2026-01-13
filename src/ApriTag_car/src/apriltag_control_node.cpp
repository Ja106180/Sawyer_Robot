#include <ros/ros.h>
#include <ros/package.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <Eigen/Dense>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <vector>

// 角度归一化函数
double normalizeAngle(double angle) {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

// 控制状态机
enum ControlState {
    STATE_GOING_TO_A = 0,
    STATE_GOING_TO_B = 1,
    STATE_GOING_TO_C = 2
};

class AprilTagControlNode {
public:
    AprilTagControlNode() : nh_("~"), current_state_(STATE_GOING_TO_A),
                           tag_pose_received_(false), imu_data_received_(false),
                           prev_distance_valid_(false), distance_increasing_counter_(0) {
        
        // 加载平面标定（单应性矩阵 H）
        std::string calibration_path = nh_.param<std::string>("plane_calibration_path", "");
        if (calibration_path.empty()) {
            // 使用 rospack 命令获取包路径（避免链接 rospack 库）
            FILE* pipe = popen("rospack find ApriTag_car", "r");
            if (pipe) {
                char buffer[512];
                if (fgets(buffer, sizeof(buffer), pipe) != nullptr) {
                    std::string pkg_path = buffer;
                    // 移除末尾的换行符
                    if (!pkg_path.empty() && pkg_path.back() == '\n') {
                        pkg_path.pop_back();
                    }
                    calibration_path = pkg_path + "/config/plane_calibration.yaml";
                }
                pclose(pipe);
            }
            // 如果 rospack 失败，使用默认路径
            if (calibration_path.empty()) {
                calibration_path = std::string(getenv("HOME")) + "/catkin_ws/src/ApriTag_car/config/plane_calibration.yaml";
            }
        }
        
        if (!loadPlaneCalibration(calibration_path)) {
            ROS_ERROR("Failed to load plane calibration!");
            ros::shutdown();
            return;
        }
        
        // 控制参数
        arrival_threshold_meters_ = nh_.param<double>("arrival_threshold_meters", 0.08);
        linear_kp_ = nh_.param<double>("linear_kp", 0.5);
        angular_kp_ = nh_.param<double>("angular_kp", 0.35);
        angular_kd_ = nh_.param<double>("angular_kd", 0.15);
        angular_dir_ = nh_.param<double>("angular_dir", -1.0);
        max_linear_speed_ = nh_.param<double>("max_linear_speed", 0.12);
        max_angular_speed_ = nh_.param<double>("max_angular_speed", 0.4);
        
        // 3 个目标点（平面坐标，米）- 先随便填，用户测试后会改
        target_points_ = {
            Eigen::Vector2d(0.3, 0.3),   // A点
            Eigen::Vector2d(0.7, 0.3),   // B点
            Eigen::Vector2d(0.5, 0.7)    // C点
        };
        
        // 订阅者
        tag_pose_sub_ = nh_.subscribe("/apriltag_car/tag_pose", 1,
                                      &AprilTagControlNode::tagPoseCallback, this);
        imu_sub_ = nh_.subscribe("/my_car_yolo/imu_processed", 1,
                                 &AprilTagControlNode::imuCallback, this);
        
        // 发布者
        cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/apriltag_car/cmd_vel", 1);
        
        ROS_INFO("AprilTag 控制节点已初始化");
        ROS_INFO("目标点: A(%.2f,%.2f) B(%.2f,%.2f) C(%.2f,%.2f)",
                 target_points_[0](0), target_points_[0](1),
                 target_points_[1](0), target_points_[1](1),
                 target_points_[2](0), target_points_[2](1));
        ROS_INFO("等待 tag 位姿数据...");
    }
    
    bool loadPlaneCalibration(const std::string& path) {
        try {
            YAML::Node config = YAML::LoadFile(path);
            
            // 加载单应性矩阵 H (3x3)
            auto H_data = config["homography_matrix"].as<std::vector<std::vector<double>>>();
            H_ = Eigen::Matrix3d::Zero();
            for (int i = 0; i < 3; i++) {
                for (int j = 0; j < 3; j++) {
                    H_(i, j) = H_data[i][j];
                }
            }
            
            ROS_INFO("平面标定已加载:");
            ROS_INFO("单应性矩阵 H:\n%s", H_.format(Eigen::IOFormat(4, 0, ", ", "\n", "[", "]")));
            
            return true;
        } catch (const std::exception& e) {
            ROS_ERROR("加载平面标定失败: %s", e.what());
            return false;
        }
    }
    
    void tagPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        tag_pose_received_ = true;
        
        // 获取 tag 在相机坐标系下的位置
        double camera_x = msg->pose.position.x;
        double camera_y = msg->pose.position.y;
        
        // 使用单应性矩阵 H 将相机坐标转换为平面坐标
        Eigen::Vector3d camera_point(camera_x, camera_y, 1.0);
        Eigen::Vector3d plane_point_homogeneous = H_ * camera_point;
        
        // 齐次坐标归一化
        double w = plane_point_homogeneous(2);
        if (std::abs(w) < 1e-6) {
            ROS_WARN_THROTTLE(1.0, "单应性变换异常，w 接近0");
            return;
        }
        
        current_plane_pos_(0) = plane_point_homogeneous(0) / w;
        current_plane_pos_(1) = plane_point_homogeneous(1) / w;
        
        // 从 tag 的 orientation 提取朝向（简化版本：用 tag 的朝向作为小车朝向）
        // 注意：这里需要根据实际 tag 安装方向调整
        tf::Quaternion q(msg->pose.orientation.x, msg->pose.orientation.y,
                        msg->pose.orientation.z, msg->pose.orientation.w);
        double roll, pitch, yaw;
        tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
        
        // 当前朝向（可能需要根据 tag 安装方向调整符号）
        current_yaw_ = angular_dir_ * yaw;
        
        // 执行控制
        controlLoop();
    }
    
    void imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
        imu_data_received_ = true;
        current_yaw_rate_ = angular_dir_ * msg->angular_velocity.z;
    }
    
    void controlLoop() {
        if (!tag_pose_received_) {
            return;
        }
        
        // 获取当前目标点
        Eigen::Vector2d target = target_points_[current_state_];
        
        // 计算到目标点的距离和方向
        Eigen::Vector2d diff = target - current_plane_pos_;
        double distance = diff.norm();
        double target_yaw = std::atan2(diff(1), diff(0));  // 注意：Y向下为正，所以用 atan2(y, x)
        
        // 角度误差
        double angle_error = normalizeAngle(target_yaw - current_yaw_);
        double angle_error_deg = std::abs(angle_error * 180.0 / M_PI);
        
        // 检查是否到达目标点
        if (distance < arrival_threshold_meters_) {
            ROS_INFO("到达目标点 %d！切换到下一个点", current_state_);
            switchToNextTarget();
            return;
        }
        
        // 距离反增检测（避免绕点转圈）
        if (prev_distance_valid_) {
            if (distance < 0.20 && distance > prev_distance_ + 0.01) {
                distance_increasing_counter_++;
            } else {
                distance_increasing_counter_ = 0;
            }
            if (distance_increasing_counter_ >= 3) {
                ROS_INFO("距离开始反增，触发到达并切换到下一个点");
                switchToNextTarget();
                prev_distance_ = distance;
                prev_distance_valid_ = true;
                distance_increasing_counter_ = 0;
                return;
            }
        }
        prev_distance_ = distance;
        prev_distance_valid_ = true;
        
        // 虚拟弹簧控制（参考 my_car_yolo_node.cpp）
        // 1) 线速度控制
        double linear_cmd = linear_kp_ * distance;
        if (linear_cmd > max_linear_speed_) {
            linear_cmd = max_linear_speed_;
        }
        
        const double min_move_distance = 0.15;
        const double min_linear_speed = 0.12;
        if (distance > min_move_distance && linear_cmd < min_linear_speed) {
            linear_cmd = min_linear_speed;
        }
        
        // 接近目标时柔和刹车
        if (distance < 0.20) {
            double v_cap_near = 0.02 + (0.10 * (distance / 0.20));
            if (linear_cmd > v_cap_near) linear_cmd = v_cap_near;
        }
        
        // 角度较大时限制前进速度
        double abs_angle_rad = std::abs(angle_error);
        if (abs_angle_rad > (60.0 * M_PI / 180.0)) {
            linear_cmd = 0.0;
        } else if (abs_angle_rad > (20.0 * M_PI / 180.0)) {
            linear_cmd *= std::cos(abs_angle_rad);
        }
        
        // 2) 角速度控制（PD）
        double angular_cmd = angular_dir_ * (angular_kp_ * angle_error);
        if (imu_data_received_) {
            angular_cmd -= angular_dir_ * angular_kd_ * current_yaw_rate_;
        }
        
        if (angular_cmd > max_angular_speed_) angular_cmd = max_angular_speed_;
        if (angular_cmd < -max_angular_speed_) angular_cmd = -max_angular_speed_;
        
        // 小角度死区
        if (angle_error_deg < 3.0) {
            if (imu_data_received_) {
                angular_cmd = angular_dir_ * (-angular_kd_ * current_yaw_rate_);
            } else {
                angular_cmd = 0.0;
            }
            if (std::fabs(angular_cmd) < 0.05) angular_cmd = 0.0;
        }
        
        // 近距离刹车
        if (distance < arrival_threshold_meters_) {
            linear_cmd *= 0.1;
        }
        
        // 防止只转不走
        if (linear_cmd > 0.02) {
            double ratio_limit = 1.0;
            if (std::fabs(angular_cmd) > ratio_limit * linear_cmd) {
                double sign = (angular_cmd >= 0.0) ? 1.0 : -1.0;
                angular_cmd = sign * ratio_limit * linear_cmd;
            }
        }
        
        // 发布速度指令
        geometry_msgs::Twist cmd_vel;
        cmd_vel.linear.x = linear_cmd;
        cmd_vel.angular.z = angular_cmd;
        cmd_vel_pub_.publish(cmd_vel);
        
        // 定期打印状态
        static int counter = 0;
        if (++counter % 50 == 0) {
            ROS_INFO("状态: 前往点%d | 位置(%.3f,%.3f) | 目标(%.3f,%.3f) | 距离:%.3fm | 角度误差:%.1f°",
                    current_state_, current_plane_pos_(0), current_plane_pos_(1),
                    target(0), target(1), distance, angle_error_deg);
        }
    }
    
    void switchToNextTarget() {
        // 停止运动
        geometry_msgs::Twist stop_cmd;
        stop_cmd.linear.x = 0.0;
        stop_cmd.angular.z = 0.0;
        cmd_vel_pub_.publish(stop_cmd);
        
        // 切换到下一个目标点
        current_state_ = (ControlState)((current_state_ + 1) % 3);
        prev_distance_valid_ = false;
        distance_increasing_counter_ = 0;
        
        ros::Duration(0.5).sleep();  // 短暂停顿
    }
    
private:
    ros::NodeHandle nh_;
    
    // 单应性矩阵 H（相机坐标 -> 平面坐标）
    Eigen::Matrix3d H_;
    
    // 当前状态和目标点
    ControlState current_state_;
    std::vector<Eigen::Vector2d> target_points_;
    
    // 当前位姿（平面坐标）
    Eigen::Vector2d current_plane_pos_;
    double current_yaw_;
    double current_yaw_rate_;
    
    // 控制参数
    double arrival_threshold_meters_;
    double linear_kp_, angular_kp_, angular_kd_;
    double angular_dir_;
    double max_linear_speed_, max_angular_speed_;
    
    // 状态标志
    bool tag_pose_received_;
    bool imu_data_received_;
    
    // 距离反增检测
    bool prev_distance_valid_;
    double prev_distance_;
    int distance_increasing_counter_;
    
    // ROS 接口
    ros::Subscriber tag_pose_sub_, imu_sub_;
    ros::Publisher cmd_vel_pub_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "apriltag_control_node");
    
    try {
        AprilTagControlNode node;
        ros::spin();
    } catch (const std::exception& e) {
        ROS_ERROR("节点异常: %s", e.what());
        return 1;
    }
    
    return 0;
}

