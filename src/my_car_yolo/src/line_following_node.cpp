#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <Eigen/Dense>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <vector>
#include <my_car_yolo/ObjectDetections.h>

// 角度归一化函数
double normalizeAngle(double angle) {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

// 线段结构
struct LineSegment {
    Eigen::Vector2d start;  // 起点（物理坐标，米）
    Eigen::Vector2d end;    // 终点（物理坐标，米）
    
    LineSegment(const Eigen::Vector2d& s, const Eigen::Vector2d& e) : start(s), end(e) {}
    
    // 计算线段长度
    double length() const {
        return (end - start).norm();
    }
    
    // 计算点到线段的最近点（投影点）
    Eigen::Vector2d nearestPoint(const Eigen::Vector2d& point) const {
        Eigen::Vector2d dir = end - start;
        double len = dir.norm();
        if (len < 1e-6) return start;  // 线段退化为点
        
        dir.normalize();
        Eigen::Vector2d to_point = point - start;
        double t = to_point.dot(dir);
        
        // 限制t在线段范围内
        t = std::max(0.0, std::min(t, len));
        
        return start + t * dir;
    }
    
    // 计算点到线段的距离
    double distanceToPoint(const Eigen::Vector2d& point) const {
        Eigen::Vector2d nearest = nearestPoint(point);
        return (point - nearest).norm();
    }
};

// 巡线状态
enum LineFollowState {
    STATE_IDLE = 0,            // 初始状态，等待开始
    STATE_FOLLOWING_LINE = 1,  // 沿当前线段直线行驶
    STATE_TURNING = 2          // 到达拐点后，原地转向 90°
};

class LineFollowingNode {
public:
    LineFollowingNode() : nh_("~"), 
                          current_segment_index_(0),
                          current_state_(STATE_IDLE),
                          current_yaw_(0.0),
                          current_yaw_rate_(0.0),
                          imu_data_received_(false),
                          control_started_(false),
                          detection_counter_(0),
                          servo_search_enabled_(true),
                          current_odometry_x_(0.0),
                          current_odometry_y_(0.0),
                          odom_yaw_(0.0),
                          odom_received_(false),
                          angular_dir_internal_(1.0),
                          sign_learned_(false),
                          last_error_distance_(std::numeric_limits<double>::quiet_NaN()),
                          error_trend_sum_(0.0),
                          sign_eval_counter_(0),
                          turning_from_segment_index_(0),
                          turning_to_segment_index_(0),
                          turning_start_yaw_(0.0),
                          turning_target_delta_yaw_(0.0) {
        // 获取参数
        calibration_path_ = nh_.param<std::string>("calibration_path", "");
        
        // 如果没有指定标定路径，使用默认路径
        if (calibration_path_.empty()) {
            std::string pkg_path = std::string(getenv("HOME")) + "/catkin_ws/src/my_car_yolo";
            calibration_path_ = pkg_path + "/config/hand_eye_calibration.yaml";
        }
        
        // 控制参数
        arrival_threshold_meters_ = nh_.param<double>("arrival_threshold_meters", 0.03);  // 3cm
        stable_frames_required_ = nh_.param<int>("stable_frames_required", 4);
        conf_threshold_ = nh_.param<double>("conf_threshold", 0.8);
        
        // 虚拟弹簧 / 巡线控制参数
        spring_kp_ = nh_.param<double>("spring_kp", 0.5);           // 最近点“拉回”权重（主要用于拐角预瞄）
        lateral_kp_ = nh_.param<double>("lateral_kp", 1.0);         // 横向误差 → 角速度 比例系数（减小，防止原地猛转）
        angular_kp_ = nh_.param<double>("angular_kp", 0.20);          // 角速度比例系数（再减小，降低左右摆动）
        angular_kd_ = nh_.param<double>("angular_kd", 0.12);          // 角速度阻尼系数
        angular_deadband_deg_ = nh_.param<double>("angular_deadband_deg", 8.0); // 角度死区，减小小幅摆动
        // 默认右侧目标右转：若仍反向可尝试改符号
        angular_dir_ = nh_.param<double>("angular_dir", -1.0);        // 转向方向符号
        imu_yaw_sign_ = nh_.param<double>("imu_yaw_sign", -1.0);     // IMU yaw符号
        
        // 速度参数
        constant_linear_speed_ = nh_.param<double>("constant_linear_speed", 0.1);  // 最大前进速度（m/s）
        max_angular_speed_ = nh_.param<double>("max_angular_speed", 0.3);          // 最大角速度（rad/s，稍微降一点，避免原地乱转）
        
        // 初始化4个点的像素坐标
        std::vector<std::pair<double, double>> points_pixel = {
            std::make_pair(45.0, 405.0),   // a点
            std::make_pair(160.0, 221.0),  // b点
            std::make_pair(328.0, 221.0),  // c点
            std::make_pair(357.0, 393.0)   // d点
        };
        
        // 加载手眼标定文件
        if (!loadHandEyeCalibration(calibration_path_)) {
            ROS_ERROR("Failed to load hand-eye calibration file: %s", calibration_path_.c_str());
            ros::shutdown();
            return;
        }
        
        // 将像素坐标转换为物理坐标，并创建线段
        std::vector<Eigen::Vector2d> points_world;
        for (const auto& pixel : points_pixel) {
            Eigen::Vector3d world = pixelToWorld(pixel.first, pixel.second);
            points_world.push_back(Eigen::Vector2d(world.x(), world.y()));
            ROS_INFO("Point pixel (%.0f, %.0f) -> world (%.3f, %.3f)", 
                     pixel.first, pixel.second, world.x(), world.y());
        }
        
        // 创建4条线段：ab, bc, cd, da（顺时针）
        line_segments_.push_back(LineSegment(points_world[0], points_world[1]));  // ab
        line_segments_.push_back(LineSegment(points_world[1], points_world[2]));  // bc
        line_segments_.push_back(LineSegment(points_world[2], points_world[3]));  // cd
        line_segments_.push_back(LineSegment(points_world[3], points_world[0]));  // da
        
        // 打印线段信息
        for (size_t i = 0; i < line_segments_.size(); ++i) {
            ROS_INFO("Segment %zu: start (%.3f, %.3f) -> end (%.3f, %.3f), length = %.3f m",
                     i, 
                     line_segments_[i].start.x(), line_segments_[i].start.y(),
                     line_segments_[i].end.x(), line_segments_[i].end.y(),
                     line_segments_[i].length());
        }
        
        // 初始化发布者和订阅者
        yolo_sub_ = nh_.subscribe("/line_following/detections", 1, &LineFollowingNode::yoloDetectionCallback, this);
        // IMU 和里程计目前不参与巡线控制，只保留接口以备后续扩展
        imu_sub_ = nh_.subscribe("/my_car_yolo/imu_processed", 1, &LineFollowingNode::imuCallback, this);
        odom_sub_ = nh_.subscribe("/odom", 1, &LineFollowingNode::odomCallback, this);
        cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/my_car_yolo/cmd_vel", 1);
        nearest_point_pub_ = nh_.advertise<geometry_msgs::Point>("/line_following/nearest_point", 1);
        servo_angle_pub_ = nh_.advertise<std_msgs::Float32>("/my_car_yolo/servo_angle", 1);

        // 周期性发布舵机（电机）控制命令（1.0=搜索，0.0=停止）
        servo_spin_timer_ = nh_.createTimer(ros::Duration(0.1), &LineFollowingNode::servoSpinTimerCallback, this);
        
        ROS_INFO("Line Following node initialized");
        ROS_INFO("Arrival threshold: %.3f m (3 cm)", arrival_threshold_meters_);
        ROS_INFO("Constant linear speed: %.3f m/s", constant_linear_speed_);
        ROS_INFO("Spring kp: %.2f, Angular kp: %.2f, kd: %.2f", spring_kp_, angular_kp_, angular_kd_);
        ROS_INFO("Waiting for YOLO detections from /line_following/detections...");
    }
    
    // IMU数据回调函数
    void imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
        tf::Quaternion q(msg->orientation.x, msg->orientation.y,
                        msg->orientation.z, msg->orientation.w);
        double roll, pitch, yaw;
        tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
        
        current_yaw_ = imu_yaw_sign_ * yaw;
        current_yaw_rate_ = imu_yaw_sign_ * msg->angular_velocity.z;
        imu_data_received_ = true;
    }
    
    // 里程计数据回调函数
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        current_odometry_x_ = msg->pose.pose.position.x;
        current_odometry_y_ = msg->pose.pose.position.y;

        // 直接从里程计姿态中读取 yaw（可以在原地转向时正常更新）
        tf::Quaternion q(msg->pose.pose.orientation.x,
                         msg->pose.pose.orientation.y,
                         msg->pose.pose.orientation.z,
                         msg->pose.pose.orientation.w);
        double roll, pitch, yaw;
        tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
        odom_yaw_ = yaw;
        odom_received_ = true;
    }
    
    // YOLO检测结果回调函数
    void yoloDetectionCallback(const my_car_yolo::ObjectDetections::ConstPtr& msg) {
        if (msg->objects.empty()) {
            detection_counter_ = 0;
            control_started_ = false;
            current_state_ = STATE_IDLE;
            servo_search_enabled_ = true;  // 没检测到时让标志电机开始慢速搜索
            stopMovement();
            return;
        }
        
        // 选择置信度最高的检测结果
        const my_car_yolo::ObjectDetection* best_detection = nullptr;
        float max_confidence = 0.0;
        
        for (const auto& obj : msg->objects) {
            if (obj.confidence > max_confidence && obj.confidence > conf_threshold_) {
                max_confidence = obj.confidence;
                best_detection = &obj;
            }
        }
        
        if (best_detection == nullptr) {
            detection_counter_ = 0;
            control_started_ = false;
            current_state_ = STATE_IDLE;
            servo_search_enabled_ = true;  // 无有效检测，继续搜索
            stopMovement();
            return;
        }
        
        // 检测到有效目标，停止搜索
        servo_search_enabled_ = false;

        // 增加连续检测计数器
        detection_counter_++;
        
        // 如果还没有开始控制，需要连续检测足够帧数后才开始
        if (!control_started_) {
            if (detection_counter_ >= stable_frames_required_) {
                control_started_ = true;
                current_state_ = STATE_FOLLOWING_LINE;
                ROS_INFO("连续检测到小车%d帧，开始巡线控制", detection_counter_);
            } else {
                ROS_INFO("检测计数器: %d/%d", detection_counter_, stable_frames_required_);
                return;
            }
        }
        
        // 执行巡线控制
        if (control_started_ && current_state_ == STATE_FOLLOWING_LINE) {
            followLine(*best_detection);
        }
    }
    
private:
    // 加载手眼标定文件
    bool loadHandEyeCalibration(const std::string& path) {
        try {
            YAML::Node config = YAML::LoadFile(path);
            
            if (config["method"] && config["method"].as<std::string>() == "intrinsics_based") {
                fx_ = config["fx"].as<double>();
                fy_ = config["fy"].as<double>();
                cx_ = config["cx"].as<double>();
                cy_ = config["cy"].as<double>();
                scale_meter_per_pixel_ = config["scale_meter_per_pixel"].as<double>();
                
                auto origin_pixel = config["origin_pixel"];
                origin_pixel_.first = origin_pixel[0].as<double>();
                origin_pixel_.second = origin_pixel[1].as<double>();
                
                auto origin_world = config["origin_world"];
                origin_world_.first = origin_world[0].as<double>();
                origin_world_.second = origin_world[1].as<double>();
                
                ROS_INFO("Hand-eye calibration loaded successfully");
                ROS_INFO("Scale: %.6f m/pixel", scale_meter_per_pixel_);
                return true;
            } else {
                ROS_ERROR("Unsupported calibration method");
                return false;
            }
        } catch (const std::exception& e) {
            ROS_ERROR("Failed to load calibration file: %s", e.what());
            return false;
        }
    }
    
    // 像素坐标转换为世界坐标
    Eigen::Vector3d pixelToWorld(double u, double v) {
        double pixel_offset_x = u - cx_;
        double pixel_offset_y = v - cy_;
        
        double world_x = pixel_offset_x * scale_meter_per_pixel_ + origin_world_.first;
        double world_y = pixel_offset_y * scale_meter_per_pixel_ + origin_world_.second;
        double world_z = 0.0;
        
        return Eigen::Vector3d(world_x, world_y, world_z);
    }
    
    // 计算点到线段的带符号横向误差（纯几何，不依赖IMU）
    // 正负号：采用 2D 叉积 sign，表示在线段一侧还是另一侧
    double signedLateralError(const LineSegment& seg, const Eigen::Vector2d& car_pos) const {
        Eigen::Vector2d d = seg.end - seg.start;
        double len = d.norm();
        if (len < 1e-6) return 0.0;
        Eigen::Vector2d d_norm = d / len;
        Eigen::Vector2d v = car_pos - seg.start;
        // 2D 叉积的 z 分量 = d.x * v.y - d.y * v.x
        double cross_z = d_norm.x() * v.y() - d_norm.y() * v.x();
        return cross_z;  // 单位：米，符号表示在线段左/右侧
    }
    
    // 原地转向：在拐点处将车头转到下一条线段的方向
    void handleTurning(const Eigen::Vector2d& car_pos) {
        if (!odom_received_) {
            // 没有里程计朝向信息时，保守起见先不转
            stopMovement();
            return;
        }

        // 采用相对旋转：例如每个拐角统一转 -90°（顺时针），避免依赖世界坐标系的绝对误差
        double desired_yaw = normalizeAngle(turning_start_yaw_ + turning_target_delta_yaw_);
        double heading_error = normalizeAngle(desired_yaw - odom_yaw_);

        // 若已基本对齐（例如误差小于 5°），认为转向完成，切换到下一线段直行
        const double yaw_tolerance = 5.0 * M_PI / 180.0;
        if (std::fabs(heading_error) < yaw_tolerance) {
            current_segment_index_ = turning_to_segment_index_;
            current_state_ = STATE_FOLLOWING_LINE;
            ROS_INFO("Turn finished. Now follow segment %zu (delta_yaw_target=%.1f deg)", 
                     current_segment_index_, turning_target_delta_yaw_ * 180.0 / M_PI);
            stopMovement();
            return;
        }

        // 按朝向误差原地转向
        double turn_kp = 1.0;
        double angular_cmd = turn_kp * heading_error;
        // 限幅
        if (angular_cmd > max_angular_speed_) angular_cmd = max_angular_speed_;
        if (angular_cmd < -max_angular_speed_) angular_cmd = -max_angular_speed_;

        geometry_msgs::Twist cmd_vel;
        cmd_vel.linear.x = 0.0;
        cmd_vel.angular.z = angular_cmd;
        cmd_vel_pub_.publish(cmd_vel);

        ROS_INFO_THROTTLE(0.5,
                          "Turning at corner: seg_from=%zu -> seg_to=%zu | heading_err=%.3f rad (%.1f deg) | ang=%.3f",
                          turning_from_segment_index_, turning_to_segment_index_,
                          heading_error, heading_error * 180.0 / M_PI,
                          angular_cmd);
    }
    
    // 巡线控制主函数：直线段 + 拐角原地转向
    void followLine(const my_car_yolo::ObjectDetection& detection) {
        // 若当前处于拐角转向状态，单独处理
        if (current_state_ == STATE_TURNING) {
            // 仍需更新 car_pos 以便日志查看，但控制只看朝向
            Eigen::Vector3d car_world_3d = pixelToWorld(detection.center_x, detection.center_y);
            Eigen::Vector2d car_pos(car_world_3d.x(), car_world_3d.y());
            handleTurning(car_pos);
            return;
        }

        // 1. 将小车位置从像素坐标转换为物理坐标
        Eigen::Vector3d car_world_3d = pixelToWorld(detection.center_x, detection.center_y);
        Eigen::Vector2d car_pos(car_world_3d.x(), car_world_3d.y());
        
        // 2. 获取当前线段 & 下一线段（用于转弯预瞄）
        const LineSegment& current_segment = line_segments_[current_segment_index_];
        const size_t next_index = (current_segment_index_ + 1) % line_segments_.size();
        const LineSegment& next_segment = line_segments_[next_index];
        const double seg_len = current_segment.length();

        // 3. 计算小车到线段的最近点（投影点），并获取未截断的投影进度
        Eigen::Vector2d dir = current_segment.end - current_segment.start;
        double dir_len = dir.norm();
        Eigen::Vector2d dir_norm = (dir_len > 1e-6) ? (dir / dir_len) : Eigen::Vector2d(1.0, 0.0);
        double t_raw = (car_pos - current_segment.start).dot(dir_norm);       // 未截断的投影距离
        double t_clamped = std::max(0.0, std::min(t_raw, seg_len));            // 截断后的投影距离
        Eigen::Vector2d nearest_point = current_segment.start + t_clamped * dir_norm;
        
        // 4. 计算虚拟弹簧误差向量（最近点）及距离
        Eigen::Vector2d error_vector = nearest_point - car_pos;
        double error_distance = error_vector.norm();
        
        // 5. 计算带符号横向误差（仅用几何，不用IMU）
        // 正负号决定往哪侧转向
        double lateral_error = signedLateralError(current_segment, car_pos);
        
        // 6. 线速度：直线段保持恒定前进速度（不再根据误差自动减速，方便走出“硬直线”）
        double linear_cmd = constant_linear_speed_;
        
        // 7. 拐角检测距离
        const double corner_enter_dist = 0.03; // 3 cm 内进入拐角状态
        double remain_to_end = seg_len - t_clamped;   // 剩余沿线距离（已截断）

        // 若已非常接近当前线段终点，且距离误差较小，则停止前进，进入“原地转向下一段”的状态
        if (remain_to_end < corner_enter_dist && error_distance < arrival_threshold_meters_) {
            turning_from_segment_index_ = current_segment_index_;
            turning_to_segment_index_ = next_index;
            // 记录进入拐角时的起始朝向
            turning_start_yaw_ = odom_yaw_;
            // 顺时针绕矩形：每个拐角统一转 -90°
            turning_target_delta_yaw_ = -M_PI_2;
            current_state_ = STATE_TURNING;
            ROS_INFO("Enter TURNING state at segment %zu -> %zu (remain_to_end=%.3f, err=%.3f, start_yaw=%.1f deg)",
                     turning_from_segment_index_, turning_to_segment_index_,
                     remain_to_end, error_distance,
                     turning_start_yaw_ * 180.0 / M_PI);
            stopMovement();
            return;
        }
        
        // 8. 角速度控制：仅用里程计朝向误差，使车头始终平行于当前线段（走出硬直线）
        double angular_cmd = 0.0;
        
        // 计算“期望朝向 - 里程计朝向”：期望朝向 = 当前线段方向
        double heading_error = 0.0;
        if (odom_received_) {
            double desired_yaw = std::atan2(dir_norm.y(), dir_norm.x());
            heading_error = normalizeAngle(desired_yaw - odom_yaw_);
            
            // 直线段使用朝向误差控制角速度
            const double heading_kp = 0.8;        // 朝向比例增益（越大越快对齐）
            const double heading_deadband = 3.0 * M_PI / 180.0; // 3° 死区
            if (std::fabs(heading_error) > heading_deadband) {
                angular_cmd = heading_kp * heading_error;
            } else {
                angular_cmd = 0.0;
            }
        }

        // 自动学习左右方向逻辑此时仅做保留，不再影响角速度（angular_cmd 不再乘以 angular_dir_internal_）
        
        // 9. 限制角速度
        if (angular_cmd > max_angular_speed_) angular_cmd = max_angular_speed_;
        if (angular_cmd < -max_angular_speed_) angular_cmd = -max_angular_speed_;
        
        // 10. 当横向误差很小的时候，不再频繁微调，减小抖动
        const double lateral_deadband = 0.01; // 1cm 死区
        if (std::fabs(lateral_error) < lateral_deadband) {
            angular_cmd = 0.0;
        }
        
        // 11. 发布控制命令
        geometry_msgs::Twist cmd_vel;
        cmd_vel.linear.x = linear_cmd;
        cmd_vel.angular.z = angular_cmd;
        cmd_vel_pub_.publish(cmd_vel);
        
        // 13. 发布最近点（用于可视化）
        geometry_msgs::Point nearest_point_msg;
        nearest_point_msg.x = nearest_point.x();
        nearest_point_msg.y = nearest_point.y();
        nearest_point_msg.z = 0.0;
        nearest_point_pub_.publish(nearest_point_msg);
        
        // 14. 日志输出（节流）
        static int log_counter = 0;
        if (++log_counter % 10 == 0) {
            ROS_INFO_THROTTLE(1.0, 
                "Seg %zu | Car (%.3f, %.3f) | Near (%.3f, %.3f) | Err: %.3f | Lat: %.3f | HeadErr: %.3f | Lin: %.3f | Ang: %.3f | DirInt: %.1f",
                current_segment_index_,
                car_pos.x(), car_pos.y(),
                nearest_point.x(), nearest_point.y(),
                error_distance,
                lateral_error,
                heading_error,
                linear_cmd,
                angular_cmd,
                angular_dir_internal_);
        }
    }
    
    // 停止运动
    void stopMovement() {
        geometry_msgs::Twist cmd_vel;
        cmd_vel.linear.x = 0.0;
        cmd_vel.angular.z = 0.0;
        cmd_vel_pub_.publish(cmd_vel);
    }

    // 定时向标志电机发布控制命令（1.0=搜索，0.0=停止）
    void servoSpinTimerCallback(const ros::TimerEvent&) {
        std_msgs::Float32 servo_cmd;
        if (servo_search_enabled_) {
            servo_cmd.data = 1.0f;  // 1.0 表示开始搜索（ESP32端执行点动/慢转）
        } else {
            servo_cmd.data = 0.0f;  // 0.0 表示停止
        }
        servo_angle_pub_.publish(servo_cmd);
    }
    
private:
    ros::NodeHandle nh_;
    
    // 订阅者和发布者
    ros::Subscriber yolo_sub_;
    ros::Subscriber imu_sub_;
    ros::Subscriber odom_sub_;
    ros::Publisher cmd_vel_pub_;
    ros::Publisher nearest_point_pub_;
    ros::Publisher servo_angle_pub_;
    
    // 标定参数
    std::string calibration_path_;
    double fx_, fy_, cx_, cy_;
    double scale_meter_per_pixel_;
    std::pair<double, double> origin_pixel_;
    std::pair<double, double> origin_world_;
    
    // 线段数据
    std::vector<LineSegment> line_segments_;
    size_t current_segment_index_;
    
    // 状态
    LineFollowState current_state_;
    bool control_started_;
    int detection_counter_;
    bool servo_search_enabled_;
    ros::Timer servo_spin_timer_;
    
    // IMU数据
    double current_yaw_;
    double current_yaw_rate_;
    bool imu_data_received_;
    
    // 里程计数据
    double current_odometry_x_;
    double current_odometry_y_;
    double odom_yaw_;           // 由里程计增量估算得到的车头朝向
    bool   odom_received_;      // 是否已有有效里程计
    // 自动学习左右方向相关
    double angular_dir_internal_;   // 实际使用的方向符号（可能与 angular_dir_ 相同或相反）
    bool   sign_learned_;           // 是否已经根据误差趋势确定好符号
    double last_error_distance_;    // 上一帧总误差
    double error_trend_sum_;        // 一段时间内误差变化的累积，用来判断在变好还是变坏
    int    sign_eval_counter_;      // 已经统计的帧数

    // 拐角转向相关
    size_t turning_from_segment_index_;
    size_t turning_to_segment_index_;
    double turning_start_yaw_;      // 进入拐角时的初始朝向
    double turning_target_delta_yaw_; // 相对旋转目标（例如 -90°）
    
    // 控制参数
    double arrival_threshold_meters_;
    int stable_frames_required_;
    double conf_threshold_;
    double spring_kp_;
    double lateral_kp_;          // 横向误差控制增益
    double angular_kp_;
    double angular_kd_;
    double angular_deadband_deg_;  // 角度死区（度）
    double angular_dir_;
    double imu_yaw_sign_;
    double constant_linear_speed_;
    double max_angular_speed_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "line_following_node");
    LineFollowingNode node;
    ros::spin();
    return 0;
}

