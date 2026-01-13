#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/Imu.h>

class ImuBridgeNode {
public:
    ImuBridgeNode() : nh_("~") {
        // 订阅ESP32发布的陀螺仪和加速度计数据
        gyro_sub_ = nh_.subscribe("/ApriTag_car/imu_gyro", 10, &ImuBridgeNode::gyroCallback, this);
        accel_sub_ = nh_.subscribe("/ApriTag_car/imu_accel", 10, &ImuBridgeNode::accelCallback, this);

        // 发布标准IMU消息
        imu_pub_ = nh_.advertise<sensor_msgs::Imu>("/ApriTag_car/imu_data", 10);

        ROS_INFO("IMU Bridge Node started");
        ROS_INFO("Subscribing to: /ApriTag_car/imu_gyro, /ApriTag_car/imu_accel");
        ROS_INFO("Publishing to: /ApriTag_car/imu_data");
    }

    void gyroCallback(const geometry_msgs::Vector3::ConstPtr& msg) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        gyro_data_ = *msg;
        gyro_updated_ = true;
        publishIfDataReady();
    }

    void accelCallback(const geometry_msgs::Vector3::ConstPtr& msg) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        accel_data_ = *msg;
        accel_updated_ = true;
        publishIfDataReady();
    }

private:
    void publishIfDataReady() {
        // 只有当两个数据都收到过才发布
        if (!gyro_updated_ || !accel_updated_) {
            return;
        }

        sensor_msgs::Imu imu_msg;

        // 设置时间戳和坐标系
        imu_msg.header.stamp = ros::Time::now();
        imu_msg.header.frame_id = "imu_link";

        // 朝向未知，设为单位四元数
        imu_msg.orientation.x = 0.0;
        imu_msg.orientation.y = 0.0;
        imu_msg.orientation.z = 0.0;
        imu_msg.orientation.w = 1.0;
        // 协方差设为-1表示未知
        for (int i = 0; i < 9; i++) {
            imu_msg.orientation_covariance[i] = -1.0;
        }

        // 不做坐标系变换，直接使用原始IMU数据
        // IMU坐标系：X=左侧，Y=后方，Z=向下
        // 直接传递给ROS（后续在角度计算中处理）

        // 设置角速度
        imu_msg.angular_velocity.x = gyro_data_.x;
        imu_msg.angular_velocity.y = gyro_data_.y;
        imu_msg.angular_velocity.z = gyro_data_.z;
        // 协方差设为-1表示未知
        for (int i = 0; i < 9; i++) {
            imu_msg.angular_velocity_covariance[i] = -1.0;
        }

        // 设置线加速度
        imu_msg.linear_acceleration.x = accel_data_.x;
        imu_msg.linear_acceleration.y = accel_data_.y;
        imu_msg.linear_acceleration.z = accel_data_.z;
        // 协方差设为-1表示未知
        for (int i = 0; i < 9; i++) {
            imu_msg.linear_acceleration_covariance[i] = -1.0;
        }

        imu_pub_.publish(imu_msg);
    }

    ros::NodeHandle nh_;
    ros::Subscriber gyro_sub_;
    ros::Subscriber accel_sub_;
    ros::Publisher imu_pub_;

    geometry_msgs::Vector3 gyro_data_;
    geometry_msgs::Vector3 accel_data_;
    bool gyro_updated_ = false;
    bool accel_updated_ = false;
    std::mutex data_mutex_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "imu_bridge_node");
    ImuBridgeNode node;
    ros::spin();
    return 0;
}
