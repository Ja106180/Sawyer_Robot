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

// 角度归一化函数，避免360°跳变问题
double normalizeAngle(double angle) {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

// 点跳变状态机
enum PointState {
    POINT_A = 0
};

// 控制状态机
enum ControlState {
    STATE_IDLE = 0,
    STATE_TURNING_TO_TARGET = 1,      // 转向目标点
    STATE_VERIFYING_TURN = 2,         // 验证转向是否完成
    STATE_MOVING_FORWARD = 3,         // 前进到目标点
    STATE_VERIFYING_ARRIVAL = 4,      // 验证是否到达
    STATE_ARRIVED = 5                 // 已到达
};

class MyCarYoloNode {
public:
    MyCarYoloNode() : nh_("~"), current_point_(POINT_A), stable_counter_(0), current_yaw_(0.0),
                      imu_data_received_(false), control_started_(false), detection_counter_(0),
                      current_state_(STATE_IDLE), target_yaw_(0.0), target_distance_(0.0),
                      movement_started_(false), servo_search_enabled_(true) {
        // 获取参数
        calibration_path_ = nh_.param<std::string>("calibration_path", "");

        // 如果没有指定标定路径，使用默认路径
        if (calibration_path_.empty()) {
            std::string pkg_path = std::string(getenv("HOME")) + "/catkin_ws/src/my_car_yolo";
            calibration_path_ = pkg_path + "/config/hand_eye_calibration.yaml";
        }

        // 控制参数（全局变量，方便修改）
        // 到达距离阈值：视觉/标定在近距离会抖动，2.5cm 往往过于苛刻，容易导致“绕点转圈”
        // 先用更稳健的 8cm 作为默认，后续标定更准后再逐步缩小
        arrival_threshold_meters_ = nh_.param<double>("arrival_threshold_meters", 0.08);
        stable_frames_required_ = nh_.param<int>("stable_frames_required", 4);  // 需要连续稳定帧数
        hold_duration_ = nh_.param<double>("hold_duration", 2.0);               // 到达后保持时间（秒）
        conf_threshold_ = nh_.param<double>("conf_threshold", 0.8);             // YOLO置信度阈值
        target_speed_ = nh_.param<double>("target_speed", 0.1);                 // 目标速度（m/s，虚拟弹簧控制中的最大线速度）

        // 虚拟弹簧 + 阻尼控制参数
        // 说明：ESP32 端已经加入了“最小PWM死区补偿”，这里不需要再给很大的拉力系数，
        //       否则会出现“飞快冲向某个环，再来回大幅摆动”的现象。
        linear_kp_ = nh_.param<double>("linear_kp", 0.5);       // 位置误差对应的线速度比例系数（弹簧刚度）
        angular_kp_ = nh_.param<double>("angular_kp", 0.35);    // 角度误差对应的角速度比例系数（P）
        angular_kd_ = nh_.param<double>("angular_kd", 0.15);    // 角速度阻尼（D），使用 IMU gyro.z
        angular_dir_ = nh_.param<double>("angular_dir", -1.0);  // 转向方向（+1 或 -1），用于快速修正符号约定
        imu_yaw_sign_ = nh_.param<double>("imu_yaw_sign", -1.0); // IMU yaw/gyro.z 符号（+1 或 -1），用于对齐底盘旋转方向
        max_linear_speed_ = nh_.param<double>("max_linear_speed", 0.12);          // 线速度上限（m/s）
        max_angular_speed_ = nh_.param<double>("max_angular_speed", 0.4);         // 角速度上限（rad/s）
        
        // 舵机点动搜索参数（全局变量，方便你后续调整）
        servo_search_enabled_ = true;  // 是否启用舵机搜索（ROS启动后默认开启）

        // 初始化目标点坐标（像素坐标，方便修改）
        target_points_pixel_ = {
            std::make_pair(282.0, 127.0)   // A点（用于通用性测试）
        };

        // 初始化发布者和订阅者
        yolo_sub_ = nh_.subscribe("/my_car_yolo/detections", 1, &MyCarYoloNode::yoloDetectionCallback, this);
        imu_sub_ = nh_.subscribe("/my_car_yolo/imu_processed", 1, &MyCarYoloNode::imuCallback, this);
        odom_sub_ = nh_.subscribe("/odom", 1, &MyCarYoloNode::odomCallback, this);
        target_point_pub_ = nh_.advertise<geometry_msgs::Point>("/my_car_yolo/target_point", 1);
        cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/my_car_yolo/cmd_vel", 1);
        servo_angle_pub_ = nh_.advertise<std_msgs::Float32>("/my_car_yolo/servo_angle", 1);

        // 周期性发布舵机控制命令（1.0=搜索，0.0=停止）
        servo_spin_timer_ = nh_.createTimer(ros::Duration(0.1), &MyCarYoloNode::servoSpinTimerCallback, this);

        // 加载手眼标定文件
        if (!loadHandEyeCalibration(calibration_path_)) {
            ROS_ERROR("Failed to load hand-eye calibration file: %s", calibration_path_.c_str());
            ros::shutdown();
            return;
        }

        ROS_INFO("MyCarYolo node initialized");
        ROS_INFO("Calibration path: %s", calibration_path_.c_str());
        ROS_INFO("Arrival method: pure physical distance (%.1f cm)", arrival_threshold_meters_ * 100.0);
        ROS_INFO("Stable frames required: %d", stable_frames_required_);
        ROS_INFO("Hold duration: %.1f seconds", hold_duration_);
        ROS_INFO("Target point: A(%.0f,%.0f)",
                 target_points_pixel_[0].first, target_points_pixel_[0].second);
        ROS_INFO("Waiting for YOLO detections from /my_car_yolo/detections (confidence > %.1f)...", conf_threshold_);
        ROS_INFO("Virtual spring control: linear_kp=%.2f, angular_kp=%.2f, max_v=%.2f m/s, max_w=%.2f rad/s",
                 linear_kp_, angular_kp_, max_linear_speed_, max_angular_speed_);
    }

    // IMU数据回调函数
    void imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
        // 从IMU消息的四元数中提取航向角
        tf::Quaternion q(msg->orientation.x, msg->orientation.y,
                        msg->orientation.z, msg->orientation.w);
        double roll, pitch, yaw;
        tf::Matrix3x3(q).getRPY(roll, pitch, yaw);

        // 更新当前航向角
        current_yaw_ = imu_yaw_sign_ * yaw;
        current_yaw_rate_ = imu_yaw_sign_ * msg->angular_velocity.z;
        imu_data_received_ = true;  // 标记已收到IMU数据

        ROS_DEBUG("IMU yaw: %.2f degrees", yaw * 180.0 / M_PI);
    }

    // 里程计数据回调函数
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        current_odometry_x_ = msg->pose.pose.position.x;
        current_odometry_y_ = msg->pose.pose.position.y;

        ROS_DEBUG("里程计位置: (%.3f, %.3f)", current_odometry_x_, current_odometry_y_);
    }

private:
    // 加载手眼标定文件
    // 加载手眼标定文件
    bool loadHandEyeCalibration(const std::string& path) {
        try {
            YAML::Node config = YAML::LoadFile(path);

            // 检查是否是基于内参的标定结果
            if (config["method"] && config["method"].as<std::string>() == "intrinsics_based") {
                // 读取相机内参
                fx_ = config["fx"].as<double>();
                fy_ = config["fy"].as<double>();
                cx_ = config["cx"].as<double>();
                cy_ = config["cy"].as<double>();

                // 读取比例尺
                scale_meter_per_pixel_ = config["scale_meter_per_pixel"].as<double>();

                // 读取原点
                auto origin_pixel = config["origin_pixel"];
                origin_pixel_.first = origin_pixel[0].as<double>();
                origin_pixel_.second = origin_pixel[1].as<double>();

                auto origin_world = config["origin_world"];
                origin_world_.first = origin_world[0].as<double>();
                origin_world_.second = origin_world[1].as<double>();

                // 读取单应矩阵（备用）
                if (config["homography_matrix"]) {
                    auto matrix_data = config["homography_matrix"];
                    for (int i = 0; i < 3; ++i) {
                        for (int j = 0; j < 3; ++j) {
                            homography_matrix_(i, j) = matrix_data[i][j].as<double>();
                        }
                    }
                }

                ROS_INFO("Hand-eye calibration loaded successfully (intrinsics-based)");
                ROS_INFO("Camera intrinsics: fx=%.2f, fy=%.2f, cx=%.2f, cy=%.2f",
                         fx_, fy_, cx_, cy_);
                ROS_INFO("Scale: %.6f m/pixel", scale_meter_per_pixel_);
                ROS_INFO("Origin pixel: (%.1f, %.1f), world: (%.3f, %.3f)",
                         origin_pixel_.first, origin_pixel_.second, origin_world_.first, origin_world_.second);

                return true;
            } else {
                ROS_ERROR("Unsupported calibration method. Please use hand_eye_calibration.py to generate calibration file.");
                return false;
            }
        } catch (const std::exception& e) {
            ROS_ERROR("Failed to load hand-eye calibration file: %s", e.what());
            return false;
        }
    }

    // 像素坐标转换为世界坐标（基于内参）
    Eigen::Vector3d pixelToWorld(double u, double v) {
        // 使用内参和比例尺进行坐标转换
        // 世界坐标 = (像素坐标 - 主点) × 比例尺 + 原点偏移

        double pixel_offset_x = u - cx_;
        double pixel_offset_y = v - cy_;

        double world_x = pixel_offset_x * scale_meter_per_pixel_ + origin_world_.first;
        double world_y = pixel_offset_y * scale_meter_per_pixel_ + origin_world_.second;
        double world_z = 0.0;  // 地面

        return Eigen::Vector3d(world_x, world_y, world_z);
    }

    // YOLO检测结果回调函数
    void yoloDetectionCallback(const my_car_yolo::ObjectDetections::ConstPtr& msg) {
        if (msg->objects.empty()) {
            // 没有检测到物体，重置检测计数器，恢复搜索
            detection_counter_ = 0;
            control_started_ = false;
            current_state_ = STATE_IDLE;
            servo_search_enabled_ = true;  // 恢复搜索

            // 没有目标时，停止小车运动（防止继续惯性运动）
            stopMovement();
            return;
        }

        ROS_INFO("Received %zu YOLO detections (threshold: %.1f)", msg->objects.size(), conf_threshold_);

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
            // 没有达到置信度阈值的检测，重置计数器，继续搜索
            detection_counter_ = 0;
            control_started_ = false;
            current_state_ = STATE_IDLE;
            servo_search_enabled_ = true;  // 继续搜索

            // 没有可靠目标时，停止小车运动
            stopMovement();
            return;
        }

        // 检测到有效目标，停止舵机搜索
        servo_search_enabled_ = false;

        // 增加连续检测计数器
        detection_counter_++;

        // 如果还没有开始控制，需要连续检测足够帧数后才开始
        if (!control_started_) {
            if (detection_counter_ >= stable_frames_required_) {
                control_started_ = true;
                current_state_ = STATE_MOVING_FORWARD;  // 使用连续跟随（虚拟弹簧）控制，不再分离“先转再走”
                ROS_INFO("连续检测到标志物%d帧，开始虚拟弹簧跟随控制", detection_counter_);
            } else {
                ROS_INFO("检测计数器: %d/%d", detection_counter_, stable_frames_required_);
                return;
            }
        }

        // 使用虚拟弹簧 + 阻尼控制进行连续跟随
        if (control_started_ && current_state_ != STATE_ARRIVED) {
            followTarget(*best_detection);
        }
    }


    // 执行控制逻辑（原状态机版本，现已不再主动调用，保留作参考）
    void executeControlLogic(const my_car_yolo::ObjectDetection& detection) {
        switch (current_state_) {
            case STATE_TURNING_TO_TARGET:
                handleTurningToTarget(detection);
                break;
            case STATE_VERIFYING_TURN:
                handleVerifyingTurn();
                break;
            case STATE_MOVING_FORWARD:
                handleMovingForward(detection);
                break;
            case STATE_VERIFYING_ARRIVAL:
                handleVerifyingArrival(detection);
                break;
            case STATE_ARRIVED:
                // 已到达，什么都不做
                break;
            default:
                break;
        }
    }

    // 状态1: 转向目标点
    void handleTurningToTarget(const my_car_yolo::ObjectDetection& detection) {
        // 计算小车当前位置到目标点的角度
        calculateTargetAngleAndDistance(detection);

        // ROS_INFO("状态: 转向目标点 | 目标角度: %.1f° | 当前角度: %.1f°",
        //          target_yaw_ * 180.0 / M_PI, current_yaw_ * 180.0 / M_PI);

        // 切换到验证转向状态，在该状态中通过 /cmd_vel 控制小车自身旋转
        current_state_ = STATE_VERIFYING_TURN;
        turn_start_time_ = ros::Time::now();
    }

    // 状态2: 验证转向是否完成
    void handleVerifyingTurn() {
        if (!imu_data_received_) {
            ROS_WARN("等待IMU数据...");
            return;
        }

        // 计算角度误差（相对动作层的思想说明）：
        // - 上层：已经在 calculateTargetAngleAndDistance() 中，根据“当前车中心”和“目标点 A”
        //         计算出了目标朝向 target_yaw_（在与 IMU 一致的坐标系下）。
        // - 下层（这里）：只关心“从当前朝向再转多少度”这一件事。
        //   由于 target_yaw_ 和 current_yaw_ 是在同一坐标定义下，
        //   直接做差即可得到“还需要转的相对角度”。
        //
        // 这里没有去“清零 IMU”，而是只把 (target_yaw_ - current_yaw_) 看作当前这一段的相对控制目标。
        double angle_error = normalizeAngle(target_yaw_ - current_yaw_);
        double angle_error_deg = std::abs(angle_error * 180.0 / M_PI);

        // ROS_INFO("状态: 验证转向 | 角度误差: %.1f°", angle_error_deg);

        // 使用简单P控制，通过 /cmd_vel 让小车原地旋转到目标角度
        double kp = 0.8;                      // 角度P增益（可根据实际调整）
        double angular_cmd = kp * angle_error;
        double max_angular = 0.8;             // 最大角速度限制 (rad/s)
        if (angular_cmd > max_angular) angular_cmd = max_angular;
        if (angular_cmd < -max_angular) angular_cmd = -max_angular;

        // 如果角度误差大于阈值，则继续旋转
        if (angle_error_deg >= 5.0) {
            geometry_msgs::Twist cmd_vel;
            cmd_vel.linear.x = 0.0;          // 原地旋转，不前进
            cmd_vel.angular.z = angular_cmd;
            cmd_vel_pub_.publish(cmd_vel);
        }

        // 如果角度误差小于5度，认为转向完成
        if (angle_error_deg < 5.0) {
            ROS_INFO("转向完成，开始前进");
            // 停止转向
            stopMovement();
            current_state_ = STATE_MOVING_FORWARD;
            movement_started_ = false;
        } else if ((ros::Time::now() - turn_start_time_).toSec() > 10.0) {
            // 超时，重新计算
            ROS_WARN("转向超时，重新计算");
            current_state_ = STATE_TURNING_TO_TARGET;
        }
    }

    // 状态3: 前进到目标点
    void handleMovingForward(const my_car_yolo::ObjectDetection& detection) {
        // 在前进阶段，结合“相对动作 + 视觉纠偏”的思路：
        // 1）每次收到新的 YOLO 检测时，都可以重新根据当前画面计算一次到 A 点的目标方向 target_yaw_。
        //    这样可以避免只依赖“进前进状态那一刻”的一次性计算。
        // 2）如果在前进过程中，发现当前朝向与新的 target_yaw_ 之间的误差过大，
        //    可以中断前进，重新回到转向阶段进行校正。

        // 先根据当前检测结果更新一次目标角度和距离（上层重新评估）
        calculateTargetAngleAndDistance(detection);

        // 如果还没开始移动，发布前进命令
        if (!movement_started_) {
            ROS_INFO("开始前进 | 目标距离: %.0f cm", target_distance_ * 100.0);
            publishMoveCommand();
            movement_started_ = true;
            movement_start_time_ = ros::Time::now();
            initial_odometry_x_ = current_odometry_x_;
            initial_odometry_y_ = current_odometry_y_;
        }

        // 在前进过程中监控当前朝向误差，如果偏差过大则先停止前进，重新进入转向阶段
        if (imu_data_received_) {
            double angle_error = normalizeAngle(target_yaw_ - current_yaw_);
            double angle_error_deg = std::abs(angle_error * 180.0 / M_PI);

            // 这里使用一个比“完成转向”更宽松的阈值，例如 10 度，
            // 一旦偏差超过该阈值，就先停下并重新进入转向流程进行修正。
            if (angle_error_deg > 10.0) {
                ROS_WARN("前进过程中朝向偏差过大(%.1f°)，暂停前进并重新转向", angle_error_deg);
                stopMovement();
                movement_started_ = false;
                current_state_ = STATE_TURNING_TO_TARGET;
                return;
            }
        }

        // 检查是否到达目标距离
        checkDistanceArrival();

        // 检查是否视觉到达（A点在包围框内）
        checkVisualArrival(detection);
    }

    // 虚拟弹簧 + 阻尼连续跟随控制（声明，具体实现在类外）
    void followTarget(const my_car_yolo::ObjectDetection& detection);

    // 状态4: 验证是否到达（旧逻辑，已废弃，保留空实现以兼容状态机接口）
    void handleVerifyingArrival(const my_car_yolo::ObjectDetection& detection) {
        (void)detection;
    }

    // 计算目标角度和距离
    void calculateTargetAngleAndDistance(const my_car_yolo::ObjectDetection& detection) {
        // 坐标系约定说明（非常重要）：
        // 1. 图像坐标 & 世界坐标保持一致的方向约定：
        //    - 向右：X 轴正方向
        //    - 向下：Y 轴正方向
        //    - 向上：Y 轴负方向
        // 2. 小车上电时，车头朝向 Y 轴负方向（也就是画面“正上方”）被定义为 0 度朝向。
        //    IMU 处理节点应当保证 current_yaw_ = 0 时，车头朝“向上”（Y-）方向。
        // 3. 本函数需要计算的是：
        //    - 从“小车当前世界坐标”指向“目标点 A 世界坐标”的方向角（在上述坐标系下）
        //    - 以及两者之间的直线距离
        //
        // 具体做法：
        // 1）先在世界坐标系下，用标准数学定义计算角度：
        //    theta_math = atan2(dy, dx)
        //    其中 theta_math 以 X 轴正方向为 0 弧度，逆时针为正（标准 atan2 定义）。
        // 2）由于我们约定“小车 0 度朝向 = Y 轴负方向（画面向上）”，
        //    而数学坐标中 Y 轴负方向对应的角度是 -90 度（-π/2 弧度），
        //    为了让“小车 0 度朝向”和 IMU current_yaw_ 的定义对齐，
        //    我们将目标角度整体平移 +90 度（+π/2 弧度）：
        //        target_yaw_ = normalizeAngle(theta_math + π/2)
        //    这样，当目标方向正好指向画面“正上方”（Y-）时，target_yaw_ ≈ 0，
        //    与 IMU current_yaw_ 的 0 度定义保持一致。

        // 将小车中心点转换为世界坐标
        Eigen::Vector3d car_world = pixelToWorld(detection.center_x, detection.center_y);

        // 获取目标点世界坐标
        auto target_pixel = target_points_pixel_[current_point_];
        Eigen::Vector3d target_world = pixelToWorld(target_pixel.first, target_pixel.second);

        // 计算从“小车中心”指向“目标点 A”的向量
        double dx = target_world(0) - car_world(0);
        double dy = target_world(1) - car_world(1);

        // 1）先得到以 X+ 为 0 度、逆时针为正的标准数学角度
        double theta_math = atan2(dy, dx);

        // 2）将 0 度基准从 X+ 旋转到 Y-（画面向上），与“小车 0 度朝向”对齐
        //    数学上：Y- = X+ 逆时针旋转 -90 度，因此这里整体平移 +90 度（+π/2）
        target_yaw_ = normalizeAngle(theta_math + M_PI / 2.0);

        // 计算距离
        target_distance_ = sqrt(dx * dx + dy * dy);

        // ROS_INFO("计算结果 | 目标角度: %.1f° | 目标距离: %.0f cm",
        //          target_yaw_ * 180.0 / M_PI, target_distance_ * 100.0);
    }

    // 发布前进命令
    void publishMoveCommand() {
        geometry_msgs::Twist cmd_vel;
        cmd_vel.linear.x = target_speed_;  // 恒定速度前进
        cmd_vel.angular.z = 0.0;           // 不旋转，靠转向角度保持方向
        cmd_vel_pub_.publish(cmd_vel);

        ROS_INFO("发布前进命令: 速度 %.2f m/s", target_speed_);
    }

    // 检查距离到达（里程计）
    void checkDistanceArrival() {
        // 计算移动距离
        double moved_distance = sqrt(pow(current_odometry_x_ - initial_odometry_x_, 2) +
                                   pow(current_odometry_y_ - initial_odometry_y_, 2));

        double distance_error = std::abs(target_distance_ - moved_distance);
        double distance_error_cm = distance_error * 100.0;

        ROS_DEBUG("距离检查 | 目标: %.0f cm | 当前: %.0f cm | 误差: %.0f cm",
                 target_distance_ * 100.0, moved_distance * 100.0, distance_error_cm);

        // 如果距离误差小于5cm，认为到达
        if (distance_error_cm < 5.0) {
            ROS_INFO("里程计到达目标距离，停止前进");
            stopMovement();
            current_state_ = STATE_VERIFYING_ARRIVAL;
        }
    }

    // 检查视觉到达（包围框）——已弃用，仅保留空实现以兼容旧代码
    void checkVisualArrival(const my_car_yolo::ObjectDetection& detection) {
        (void)detection;
    }

    // 停止运动
    void stopMovement() {
        geometry_msgs::Twist stop_cmd;
        stop_cmd.linear.x = 0.0;
        stop_cmd.angular.z = 0.0;
        cmd_vel_pub_.publish(stop_cmd);

        ROS_INFO("停止小车运动");
    }

    // 到达目标点后的处理
    void onArrival() {
        arrival_time_ = ros::Time::now();
        ROS_INFO("🎯 到达目标点A! A点已在标志物包围框内");

        // 停止运动
        stopMovement();

        // 切换到已到达状态
        current_state_ = STATE_ARRIVED;

        ROS_INFO("小车已停止，任务完成！");
    }

    // 预留接口：未来如需根据IMU自动补偿舵机，可在此实现并主动调用
    void publishServoCompensation() {}

    // 定时向舵机发布控制命令（1.0=搜索，0.0=停止）
    void servoSpinTimerCallback(const ros::TimerEvent&) {
        std_msgs::Float32 servo_cmd;
        if (servo_search_enabled_) {
            servo_cmd.data = 1.0f;  // 1.0 表示开始搜索（ESP32端会执行点动逻辑）
        } else {
            servo_cmd.data = 0.0f;  // 0.0 表示停止
        }
        servo_angle_pub_.publish(servo_cmd);
    }

    // ROS节点句柄
    ros::NodeHandle nh_;
    ros::Subscriber yolo_sub_;
    ros::Subscriber imu_sub_;
    ros::Subscriber odom_sub_;
    ros::Publisher target_point_pub_;
    ros::Publisher cmd_vel_pub_;
    ros::Publisher servo_angle_pub_;

    // 参数
    std::string calibration_path_;
    double arrival_threshold_meters_;
    int stable_frames_required_;
    double hold_duration_;
    double conf_threshold_;
    double target_speed_;

    // 虚拟弹簧 + 阻尼控制参数
    double linear_kp_;          // 位置误差 → 线速度 比例系数
    double angular_kp_;         // 角度误差 → 角速度 比例系数
    double angular_kd_;         // 角速度阻尼（使用 IMU gyro.z）
    double angular_dir_;        // 转向方向（+1 或 -1）
    double imu_yaw_sign_;       // IMU yaw/gyro.z 符号（+1 或 -1）
    double max_linear_speed_;   // 线速度上限
    double max_angular_speed_;  // 角速度上限

    // 状态变量
    PointState current_point_;
    ControlState current_state_;
    int stable_counter_;
    ros::Time arrival_time_;
    ros::Time turn_start_time_;
    ros::Time movement_start_time_;
    double current_yaw_;  // 当前IMU航向角（弧度）
    double current_yaw_rate_ = 0.0;  // 当前 IMU yaw 角速度（rad/s）

    // 用于“距离反增”判定（避免绕点转圈/冲过头）
    double prev_distance_ = 0.0;
    bool prev_distance_valid_ = false;
    int distance_increasing_counter_ = 0;
    double current_odometry_x_;  // 当前里程计X
    double current_odometry_y_;  // 当前里程计Y
    double initial_odometry_x_;  // 移动开始时的里程计X
    double initial_odometry_y_;  // 移动开始时的里程计Y
    bool imu_data_received_;  // 是否收到过IMU数据
    bool control_started_;    // 是否已经开始控制（需要连续检测到标志物后才开始）
    bool movement_started_;   // 是否已经开始移动
    int detection_counter_;   // 连续检测到标志物的帧数
    double target_yaw_;       // 目标转向角度
    double target_distance_;  // 目标移动距离
    bool servo_search_enabled_;  // 是否启用舵机搜索（true=搜索，false=停止）
    ros::Timer servo_spin_timer_;

    // 目标点坐标（像素）
    std::vector<std::pair<double, double>> target_points_pixel_;

    // 手眼标定参数
    double fx_, fy_, cx_, cy_;           // 相机内参
    double scale_meter_per_pixel_;       // 米/像素比例尺
    std::pair<double, double> origin_pixel_;  // 像素坐标原点
    std::pair<double, double> origin_world_;  // 世界坐标原点
Eigen::Matrix3d homography_matrix_;  // 单应矩阵（备用）
};

// ==============================
// 虚拟弹簧 + 阻尼连续跟随控制实现
// ==============================
void MyCarYoloNode::followTarget(const my_car_yolo::ObjectDetection& detection) {
    if (!imu_data_received_) {
        ROS_WARN_THROTTLE(1.0, "尚未收到 IMU 数据，无法进行跟随控制");
        stopMovement();
        return;
    }

    // 根据当前检测结果计算目标角度和距离（上层：目标评估）
    calculateTargetAngleAndDistance(detection);

    // 使用“虚拟弹簧”思想：距离越远，线速度越大；越近越小
    double distance = target_distance_;  // 米

    // 近距离（靠近 A 点）视觉距离会抖动：用“距离反增”来判定是否已经越过目标，避免绕点转圈
    if (prev_distance_valid_) {
        // 当已经进入“接近区”后，如果距离连续变大，说明已经冲过/绕过 A 点
        if (distance < 0.20 && distance > prev_distance_ + 0.01) { // 1cm 以上认为有意义反增
            distance_increasing_counter_++;
        } else {
            distance_increasing_counter_ = 0;
        }
        if (distance_increasing_counter_ >= 3) {
            ROS_INFO("距离开始反增（疑似已越过目标/绕点），触发到达并刹车");
            onArrival();
            prev_distance_ = distance;
            prev_distance_valid_ = true;
            distance_increasing_counter_ = 0;
            return;
        }
    }
    prev_distance_ = distance;
    prev_distance_valid_ = true;

    // 角度误差：从当前朝向转到“指向 A 点”的目标朝向
    // 统一使用常见定义：error = target - current
    // 如发现转向方向整体相反，用参数 angular_dir（+1/-1）来快速翻转，而不是反复改代码。
    double angle_error = normalizeAngle(target_yaw_ - current_yaw_);
    double angle_error_deg = std::abs(angle_error * 180.0 / M_PI);

    // 1）线速度控制：v = Kp * 距离，限制到最大速度，并在“距离较远”时加一个最小速度
    double linear_cmd = linear_kp_ * distance;
    if (linear_cmd > max_linear_speed_) {
        linear_cmd = max_linear_speed_;
    }
    // 如果距离还比较远，但算出来的速度太小，地面摩擦会让车几乎不动，这里给一个“最小推进力”
    const double min_move_distance = 0.15;   // 距离大于 15cm 认为“还比较远”
    const double min_linear_speed  = 0.12;   // 远距离时的最小线速度（m/s）
    if (distance > min_move_distance && linear_cmd < min_linear_speed) {
        linear_cmd = min_linear_speed;
    }

    // 接近目标时强制柔和刹车，避免冲过头和绕点转圈
    if (distance < 0.20) {
        // 0~20cm 内：线速度上限随距离线性下降
        double v_cap_near = 0.02 + (0.10 * (distance / 0.20)); // distance=0.20 -> 0.12, distance=0 -> 0.02
        if (linear_cmd > v_cap_near) linear_cmd = v_cap_near;
    }

    // 角度较大时限制前进速度（先把方向追稳）
    double abs_angle_rad = std::abs(angle_error);
    if (abs_angle_rad > (60.0 * M_PI / 180.0)) {          // > 60°
        linear_cmd = 0.0;
    } else if (abs_angle_rad > (20.0 * M_PI / 180.0)) {   // 20~60°
        linear_cmd *= std::cos(abs_angle_rad);
    }

    // 2）角速度控制：PD 控制（P：追角；D：用 gyro.z 抑制过冲/摆动）
    // angular_dir_ 用于修正整体转向方向（+1 或 -1）
    double angular_cmd = angular_dir_ * (angular_kp_ * angle_error - angular_kd_ * current_yaw_rate_);
    if (angular_cmd > max_angular_speed_) angular_cmd = max_angular_speed_;
    if (angular_cmd < -max_angular_speed_) angular_cmd = -max_angular_speed_;

    // 小角度死区：抑制“10°来回摆动”那种抖动（仍保留阻尼项）
    if (angle_error_deg < 3.0) {
        angular_cmd = angular_dir_ * (-angular_kd_ * current_yaw_rate_);
        if (std::fabs(angular_cmd) < 0.05) angular_cmd = 0.0;
    }

    // 如果已经非常接近目标（距离很小），线速度进一步减小，逐渐“拉紧弹簧”停下
    if (distance < arrival_threshold_meters_) {
        linear_cmd *= 0.1;  // 更强的近距离刹车
    }

    // 3）防止“只转不走”：限制角速度相对线速度的比例
    // 如果角速度远大于线速度（|w| >> v），差速映射后会出现“几乎只有一侧轮子在转”，
    // 现场表现就是你看到的那种“只用左轮在原地转圈”。
    if (linear_cmd > 0.02) {  // 有明显前进意图时才做比例限制
        double ratio_limit = 1.0;  // 经验值：|w| 不要大于 v 的约 1 倍
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

    ROS_INFO_THROTTLE(0.5,
                      "跟随控制 | dir=%.0f imu_sign=%.0f | dist=%.2f m | err=%.1f deg | yaw=%.1f deg | tgt=%.1f deg | v=%.2f | w=%.2f | yaw_rate=%.2f",
                      angular_dir_,
                      imu_yaw_sign_,
                      distance,
                      angle_error_deg,
                      current_yaw_ * 180.0 / M_PI,
                      target_yaw_ * 180.0 / M_PI,
                      linear_cmd,
                      angular_cmd,
                      current_yaw_rate_);

        // 物理到达判定：仅基于车中心与 A 点之间的物理距离
        // 进入距离阈值内后，需要连续若干帧都满足条件才算真正到达，防止单帧抖动。
        if (distance < arrival_threshold_meters_) {
            stable_counter_++;
            ROS_INFO_THROTTLE(0.5, "已进入到达距离阈值内（<%.1f cm），稳定计数: %d/%d",
                              arrival_threshold_meters_ * 100.0,
                              stable_counter_, stable_frames_required_);

            if (stable_counter_ >= stable_frames_required_) {
                onArrival();
                stable_counter_ = 0;
            }
        } else {
            stable_counter_ = 0;
        }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "my_car_yolo_node");

    MyCarYoloNode node;

    ros::spin();

    return 0;
}
