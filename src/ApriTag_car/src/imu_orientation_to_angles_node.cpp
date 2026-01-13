#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Vector3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

class ImuOrientationToAnglesNode {
public:
    ImuOrientationToAnglesNode() : nh_("~") {
        // 订阅滤波后的IMU数据（包含姿态）
        imu_sub_ = nh_.subscribe("/imu/data", 10, &ImuOrientationToAnglesNode::imuCallback, this);

        // 发布角度数据给平衡控制节点
        angles_pub_ = nh_.advertise<geometry_msgs::Vector3>("/ApriTag_car/imu_angles", 10);

        ROS_INFO("IMU Orientation to Angles Node started");
        ROS_INFO("Subscribing to: /imu/data");
        ROS_INFO("Publishing to: /ApriTag_car/imu_angles");
    }

    void imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
        // 从四元数提取欧拉角
        tf2::Quaternion q(
            msg->orientation.x,
            msg->orientation.y,
            msg->orientation.z,
            msg->orientation.w
        );

        // 转换为欧拉角 (roll, pitch, yaw)
        double roll, pitch, yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

        // 转换为度数
        roll = roll * 180.0 / M_PI;
        pitch = pitch * 180.0 / M_PI;
        yaw = yaw * 180.0 / M_PI;

        // 对于平衡车，我们需要根据IMU的实际安装方向调整角度
        // IMU坐标系：X=左侧，Y=后方，Z=向下
        // 平衡车pitch：前倾负，后倾正
        double corrected_pitch = pitch;  // 暂时直接使用

        // 发布角度数据
        geometry_msgs::Vector3 angles_msg;
        angles_msg.x = corrected_pitch;  // Pitch（平衡控制需要）
        angles_msg.y = yaw;              // Yaw（转向控制需要）
        angles_msg.z = 0.0;              // Roll（平衡车不需要，设为0）

        angles_pub_.publish(angles_msg);
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber imu_sub_;
    ros::Publisher angles_pub_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "imu_orientation_to_angles_node");
    ImuOrientationToAnglesNode node;
    ros::spin();
    return 0;
}
