 /*
 * balance_control_node.cpp - LQR平衡控制节点
 * 
 * 功能：
 * 1. 订阅IMU数据，获取pitch角和角速度
 * 2. 订阅编码器数据，计算位置和速度
 * 3. 两阶段控制：起立（Bang-Bang）+ 平衡（LQR）
 * 4. 键盘控制叠加（在LQR输出上叠加速度指令）
 * 5. 发布控制指令到 /ApriTag_car/cmd_vel
 */

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Int32MultiArray.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>
#include <Eigen/Dense>
#include <cmath>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>

// 控制模式
enum ControlMode {
    MODE_STANDUP,    // 起立模式
    MODE_BALANCE     // 平衡模式
};

class BalanceControlNode {
public:
    BalanceControlNode() : nh_("~"), mode_(MODE_STANDUP) {
        // 获取物理参数
        wheel_radius_ = nh_.param<double>("wheel_radius", 0.0335);  // 0.0335m
        wheel_base_ = nh_.param<double>("wheel_base", 0.194);       // 0.194m
        center_height_ = nh_.param<double>("center_height", 0.075);  // 0.075m
        mass_ = nh_.param<double>("mass", 1.5);                     // 1.5kg
        
        // 控制参数
        control_freq_ = nh_.param<double>("control_freq", 100.0);  // 100Hz
        max_velocity_ = nh_.param<double>("max_velocity", 0.3);   // m/s
        
        // 起立控制参数
        standup_max_vel_ = nh_.param<double>("standup_max_vel", 0.15);  // m/s
        standup_medium_vel_ = nh_.param<double>("standup_medium_vel", 0.08);  // m/s
        standup_angle_thresh1_ = nh_.param<double>("standup_angle_thresh1", 10.0 * M_PI / 180.0);  // 10度
        standup_angle_thresh2_ = nh_.param<double>("standup_angle_thresh2", 60.0 * M_PI / 180.0);  // 60度
        standup_angle_thresh3_ = nh_.param<double>("standup_angle_thresh3", 85.0 * M_PI / 180.0);  // 85度
        
        // 安全保护参数
        max_pitch_error_ = nh_.param<double>("max_pitch_error", 45.0 * M_PI / 180.0);  // 45度
        imu_timeout_ = nh_.param<double>("imu_timeout", 0.1);  // 100ms
        
        // 键盘控制参数
        keyboard_linear_speed_ = nh_.param<double>("keyboard_linear_speed", 0.1);  // m/s
        keyboard_angular_speed_ = nh_.param<double>("keyboard_angular_speed", 0.05);  // rad/s
        
        // 编码器参数
        ticks_per_rev_ = nh_.param<double>("ticks_per_rev", 1320.0);
        
        // 状态变量
        pitch_ = 0.0;
        pitch_rate_ = 0.0;
        position_ = 0.0;
        velocity_ = 0.0;
        last_imu_time_ = ros::Time(0);  // 初始化为0，表示未收到数据
        last_control_time_ = ros::Time::now();
        
        // 编码器累计计数
        left_encoder_total_ = 0;
        right_encoder_total_ = 0;
        last_encoder_time_ = ros::Time::now();
        
        // 键盘输入
        keyboard_linear_ = 0.0;
        keyboard_angular_ = 0.0;
        setupTerminal();
        
        // 初始化LQR增益矩阵（需要根据实际模型调整）
        initLQR();
        
        // 订阅者和发布者
        imu_sub_ = nh_.subscribe("/ApriTag_car/imu_angles", 10, 
                                 &BalanceControlNode::imuCallback, this);
        encoder_sub_ = nh_.subscribe("/ApriTag_car/wheel_encoders", 10,
                                     &BalanceControlNode::encoderCallback, this);
        cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/ApriTag_car/cmd_vel", 1);
        status_pub_ = nh_.advertise<std_msgs::Float32>("/ApriTag_car/control_status", 1);
        
        ROS_INFO("LQR balance control node started");
        ROS_INFO("Control frequency: %.1f Hz", control_freq_);
        ROS_INFO("Physical parameters: r=%.3fm, L=%.3fm, h=%.3fm, m=%.2fkg",
                 wheel_radius_, wheel_base_, center_height_, mass_);
    }
    
    ~BalanceControlNode() {
        restoreTerminal();
        // 停车
        geometry_msgs::Twist stop;
        cmd_vel_pub_.publish(stop);
        ros::Duration(0.1).sleep();
    }
    
    void spin() {
        ros::Rate rate(control_freq_);
        
        while (ros::ok()) {
            // 读取键盘输入
            readKeyboard();
            
            // 执行控制
            controlLoop();
            
            ros::spinOnce();
            rate.sleep();
        }
    }
    
private:
    void initLQR() {
        // 倒立摆模型参数（线性化在0°附近，即垂直平衡点）
        double g = 9.8;  // 重力加速度
        double h = center_height_;
        
        // 状态空间模型：x = [θ, θ_dot, x, x_dot]ᵀ
        // 其中θ是pitch角度（0度=垂直，负=前倾，正=后倾）
        // 简化模型（需要根据实际小车调整）
        // A = [0  1  0  0;
        //      g/h 0  0  0;
        //      0  0  0  1;
        //      g/h 0  0  0]
        // B = [0; 1/h; 0; 1/h]
        
        // 权重矩阵
        Eigen::Matrix4d Q;
        Q << 1000.0, 0.0, 0.0, 0.0,      // θ权重（最重要）
             0.0, 100.0, 0.0, 0.0,       // θ_dot权重
             0.0, 0.0, 1.0, 0.0,         // x权重（允许移动）
             0.0, 0.0, 0.0, 1.0;          // x_dot权重
        
        Eigen::Matrix<double, 1, 1> R;
        R << 0.1;  // 控制权重
        
        // 状态矩阵A（简化模型）
        Eigen::Matrix4d A;
        A << 0.0, 1.0, 0.0, 0.0,
             g / h, 0.0, 0.0, 0.0,
             0.0, 0.0, 0.0, 1.0,
             g / h, 0.0, 0.0, 0.0;
        
        // 控制矩阵B（简化模型）
        Eigen::Matrix<double, 4, 1> B;
        B << 0.0, 1.0 / h, 0.0, 1.0 / h;
        
        // 求解Riccati方程得到LQR增益矩阵K
        // 使用迭代方法求解（简化版，实际应该用更精确的方法）
        K_ = solveLQR(A, B, Q, R);
        
        ROS_INFO("LQR gain matrix K:");
        ROS_INFO("  K = [%.3f, %.3f, %.3f, %.3f]",
                 K_(0, 0), K_(0, 1), K_(0, 2), K_(0, 3));
    }
    
    Eigen::Matrix<double, 1, 4> solveLQR(const Eigen::Matrix4d& A,
                                          const Eigen::Matrix<double, 4, 1>& B,
                                          const Eigen::Matrix4d& Q,
                                          const Eigen::Matrix<double, 1, 1>& R) {
        // 使用迭代方法求解离散时间Riccati方程
        // P_{k+1} = Q + A^T * P_k * A - A^T * P_k * B * (R + B^T * P_k * B)^(-1) * B^T * P_k * A
        
        Eigen::Matrix4d P = Q;  // 初始猜测
        Eigen::Matrix<double, 1, 4> K;
        
        // 迭代求解（最多200次迭代）
        for (int i = 0; i < 200; i++) {
            // 计算 K = (R + B^T * P * B)^(-1) * B^T * P * A
            Eigen::Matrix<double, 1, 1> R_BPB = R + B.transpose() * P * B;
            K = R_BPB.inverse() * B.transpose() * P * A;
            
            // 计算新的P
            Eigen::Matrix4d P_new = Q + A.transpose() * P * A - 
                                     A.transpose() * P * B * K;
            
            // 检查收敛
            double diff = (P_new - P).norm();
            if (diff < 1e-8) {
                ROS_INFO("LQR solution converged, iterations: %d", i + 1);
                break;
            }
            P = P_new;
        }
        
        return K;
    }
    
    void imuCallback(const geometry_msgs::Vector3::ConstPtr& msg) {
        // ESP32发布的角度数据格式：x=Pitch（俯仰角），y=Yaw（航向角），z=0（单位：度）
        // 保存上一次的pitch值，用于计算pitch_rate
        static double prev_pitch_deg_ = 0.0;
        static ros::Time prev_time_ = ros::Time(0);
        
        ros::Time current_time = ros::Time::now();
        
        // 提取pitch角（单位：度）- ESP32发布在x字段
        double pitch_deg = msg->x;
        
        // 转换为弧度（内部计算使用弧度）
        pitch_ = pitch_deg * M_PI / 180.0;
        
        // 计算pitch角速度（通过差分，单位：rad/s）
        if (!prev_time_.isZero()) {
            double dt = (current_time - prev_time_).toSec();
            if (dt > 0 && dt < 0.1) {  // 合理的时间间隔
                double pitch_rate_deg_per_sec = (pitch_deg - prev_pitch_deg_) / dt;
                pitch_rate_ = pitch_rate_deg_per_sec * M_PI / 180.0;  // 度/秒转rad/s
            }
        }
        prev_pitch_deg_ = pitch_deg;
        prev_time_ = current_time;
        
        // ESP32角度定义：0度=垂直，负角度=前倾（车头向下），正角度=后倾（车头向上）
        // 保留原始角度值，不取绝对值，以便控制逻辑能区分前倾和后倾
        
        last_imu_time_ = current_time;
    }
    
    void encoderCallback(const std_msgs::Int32MultiArray::ConstPtr& msg) {
        if (msg->data.size() < 2) return;
        
        ros::Time current_time = ros::Time::now();
        double dt = (current_time - last_encoder_time_).toSec();
        
        if (dt <= 0 || dt > 0.1) {
            last_encoder_time_ = current_time;
            return;
        }
        
        // 累计编码器计数
        left_encoder_total_ += msg->data[0];
        right_encoder_total_ += msg->data[1];
        
        // 计算位置（米）
        double ticks_per_meter = ticks_per_rev_ / (2.0 * M_PI * wheel_radius_);
        double left_distance = left_encoder_total_ / ticks_per_meter;
        double right_distance = right_encoder_total_ / ticks_per_meter;
        position_ = (left_distance + right_distance) / 2.0;
        
        // 计算速度（m/s）
        if (dt > 0) {
            double left_vel = (msg->data[0] / ticks_per_meter) / dt;
            double right_vel = (msg->data[1] / ticks_per_meter) / dt;
            velocity_ = (left_vel + right_vel) / 2.0;
        }
        
        last_encoder_time_ = current_time;
    }
    
    void readKeyboard() {
        char c = 0;
        if (readKey(c)) {
            handleKey(c);
        }
    }
    
    bool readKey(char &c) {
        int n = ::read(STDIN_FILENO, &c, 1);
        return (n == 1);
    }
    
    void handleKey(char c) {
        switch (c) {
            case 'w':
            case 'W':
                keyboard_linear_ = keyboard_linear_speed_;
                keyboard_angular_ = 0.0;
                break;
            case 's':
            case 'S':
                keyboard_linear_ = -keyboard_linear_speed_;
                keyboard_angular_ = 0.0;
                break;
            case 'a':
            case 'A':
                keyboard_linear_ = 0.0;
                keyboard_angular_ = keyboard_angular_speed_;
                break;
            case 'd':
            case 'D':
                keyboard_linear_ = 0.0;
                keyboard_angular_ = -keyboard_angular_speed_;
                break;
            case 'x':
            case 'X':
            case ' ':
                keyboard_linear_ = 0.0;
                keyboard_angular_ = 0.0;
                break;
            case 'q':
            case 'Q':
                ros::shutdown();
                break;
        }
    }
    
    void setupTerminal() {
        tcgetattr(STDIN_FILENO, &old_terminal_);
        struct termios new_terminal = old_terminal_;
        new_terminal.c_lflag &= ~(ICANON | ECHO);
        new_terminal.c_cc[VMIN] = 0;
        new_terminal.c_cc[VTIME] = 0;
        tcsetattr(STDIN_FILENO, TCSANOW, &new_terminal);
        fcntl(STDIN_FILENO, F_SETFL, O_NONBLOCK);
    }
    
    void restoreTerminal() {
        tcsetattr(STDIN_FILENO, TCSANOW, &old_terminal_);
    }
    
    void controlLoop() {
        ros::Time current_time = ros::Time::now();
        
        // 检查IMU数据超时（如果从未收到数据，last_imu_time_为0）
        if (last_imu_time_.isZero()) {
            static bool first_warning = true;
            if (first_warning) {
                ROS_ERROR("No IMU data received yet!");
                ROS_ERROR("Check ESP32 connection:");
                ROS_ERROR("  1. Is ESP32 powered on?");
                ROS_ERROR("  2. Is ESP32 connected to WiFi?");
                ROS_ERROR("  3. Is ESP32 connected to ROS master?");
                ROS_ERROR("  4. Check topics: rostopic list | grep ApriTag");
                ROS_ERROR("  5. Check IMU angles data: rostopic echo /ApriTag_car/imu_angles");
                first_warning = false;
            }
            ROS_WARN_THROTTLE(5.0, "Waiting for IMU data...");
            publishStop();
            return;
        }
        
        double time_since_imu = (current_time - last_imu_time_).toSec();
        if (time_since_imu > imu_timeout_) {
            static bool first_warning = true;
            if (first_warning) {
                ROS_ERROR("IMU data timeout! Time since last data: %.2f seconds", time_since_imu);
                ROS_ERROR("Check ESP32 connection:");
                ROS_ERROR("  1. Is ESP32 powered on?");
                ROS_ERROR("  2. Is ESP32 connected to WiFi?");
                ROS_ERROR("  3. Is ESP32 connected to ROS master?");
                ROS_ERROR("  4. Check if /ApriTag_car/imu_angles topic exists: rostopic list");
                ROS_ERROR("  5. Check if IMU data is published: rostopic echo /ApriTag_car/imu_angles");
                first_warning = false;
            }
            ROS_WARN_THROTTLE(5.0, "IMU data timeout (%.1f s), stopping control", time_since_imu);
            publishStop();
            return;
        }
        static bool first_imu_received = true;
        if (first_imu_received) {
            ROS_INFO("IMU data received! Starting control...");
            first_imu_received = false;
        }
        
        // 计算pitch角误差（目标角度=0度，即垂直）
        // ESP32角度定义：0度=垂直，负角度=前倾，正角度=后倾
        double pitch_error = pitch_;  // 目标角度是0度，所以误差就是pitch本身
        
        // 安全保护：只在平衡模式下限制角度
        if (mode_ == MODE_BALANCE && fabs(pitch_) > max_pitch_error_) {
            ROS_WARN_THROTTLE(1.0, "Pitch angle too large: %.1f degrees, stopping control",
                             pitch_ * 180.0 / M_PI);
            publishStop();
            mode_ = MODE_STANDUP;  // 回到起立模式
            return;
        }
        
        // 控制模式切换
        double pitch_deg = pitch_ * 180.0 / M_PI;  // 转换为度：0度=垂直，负=前倾，正=后倾
        
        if (mode_ == MODE_STANDUP) {
            // 起立模式
            // ESP32角度：0度=垂直，负角度=前倾，正角度=后倾
            // 如果pitch接近0度（垂直），切换到平衡模式
            if (fabs(pitch_deg) < 15.0) {
                // 接近垂直（±15度内）：切换到平衡模式
                mode_ = MODE_BALANCE;
                ROS_INFO("Switching to balance mode (pitch: %.1f degrees)", pitch_deg);
            } else if (pitch_deg < -60.0) {
                // 前倾很大（<-60度，车头向下）：向后加速让车立起来
                publishVelocity(-standup_max_vel_, 0.0);
            } else if (pitch_deg > 60.0) {
                // 后倾很大（>60度，车头向上）：向前加速让车立起来
                publishVelocity(standup_max_vel_, 0.0);
            } else if (pitch_deg < -30.0) {
                // 前倾中等（-60到-30度）：继续向后加速
                publishVelocity(-standup_max_vel_, 0.0);
            } else if (pitch_deg > 30.0) {
                // 后倾中等（30到60度）：继续向前加速
                publishVelocity(standup_max_vel_, 0.0);
            } else {
                // 接近垂直但还在起立模式（-30到30度，但不在±15度内）：减速
                if (pitch_deg < 0) {
                    publishVelocity(-standup_medium_vel_, 0.0);  // 前倾，向后减速
                } else {
                    publishVelocity(standup_medium_vel_, 0.0);   // 后倾，向前减速
                }
            }
        } else {
            // 平衡模式：LQR控制
            // 状态向量：x = [θ, θ_dot, x, x_dot]ᵀ（目标角度是0度，所以θ就是误差）
            Eigen::Vector4d state;
            state << pitch_error,      // 角度误差（目标0度，所以就是pitch本身）
                     pitch_rate_,      // 角速度
                     position_,        // 位置
                     velocity_;        // 速度
            
            // LQR控制：u = -K * x
            Eigen::Matrix<double, 1, 1> u = -K_ * state;
            double lqr_velocity = u(0, 0);
            
            // 限制LQR输出
            lqr_velocity = std::max(-max_velocity_, std::min(max_velocity_, lqr_velocity));
            
            // 叠加键盘控制
            double final_linear = lqr_velocity + keyboard_linear_;
            double final_angular = keyboard_angular_;
            
            // 限制最终输出
            final_linear = std::max(-max_velocity_, std::min(max_velocity_, final_linear));
            
            publishVelocity(final_linear, final_angular);
            
            // 发布状态信息（用于调试）
            std_msgs::Float32 status_msg;
            status_msg.data = pitch_deg;
            status_pub_.publish(status_msg);
        }
    }
    
    void publishVelocity(double linear, double angular) {
        geometry_msgs::Twist cmd;
        cmd.linear.x = linear;
        cmd.angular.z = angular;
        cmd_vel_pub_.publish(cmd);
    }
    
    void publishStop() {
        geometry_msgs::Twist cmd;
        cmd.linear.x = 0.0;
        cmd.angular.z = 0.0;
        cmd_vel_pub_.publish(cmd);
    }
    
    ros::NodeHandle nh_;
    
    // 物理参数
    double wheel_radius_;
    double wheel_base_;
    double center_height_;
    double mass_;
    double ticks_per_rev_;
    
    // 控制参数
    double control_freq_;
    double max_velocity_;
    
    // 起立控制参数
    double standup_max_vel_;
    double standup_medium_vel_;
    double standup_angle_thresh1_;
    double standup_angle_thresh2_;
    double standup_angle_thresh3_;
    
    // 安全保护参数
    double max_pitch_error_;
    double imu_timeout_;
    
    // 键盘控制参数
    double keyboard_linear_speed_;
    double keyboard_angular_speed_;
    
    // 状态变量
    double pitch_;
    double pitch_rate_;
    double position_;
    double velocity_;
    ros::Time last_imu_time_;
    ros::Time last_control_time_;
    
    // 编码器
    int32_t left_encoder_total_;
    int32_t right_encoder_total_;
    ros::Time last_encoder_time_;
    
    // 键盘输入
    double keyboard_linear_;
    double keyboard_angular_;
    struct termios old_terminal_;
    
    // LQR增益矩阵
    Eigen::Matrix<double, 1, 4> K_;
    
    // 控制模式
    ControlMode mode_;
    
    // ROS接口
    ros::Subscriber imu_sub_;
    ros::Subscriber encoder_sub_;
    ros::Publisher cmd_vel_pub_;
    ros::Publisher status_pub_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "balance_control_node");
    
    BalanceControlNode node;
    
    ROS_INFO("Keyboard control:");
    ROS_INFO("  w/W: Forward");
    ROS_INFO("  s/S: Backward");
    ROS_INFO("  a/A: Turn left");
    ROS_INFO("  d/D: Turn right");
    ROS_INFO("  x/X or SPACE: Stop");
    ROS_INFO("  q/Q: Quit");
    
    node.spin();
    
    return 0;
}

