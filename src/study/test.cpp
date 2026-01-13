#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Vector3.h>
#include <cmath>

class MadgwickAHRS{
public:
    MadgwickAHRS(double beta = 0.1) : beta_(beta){
        q0_ = 1.0;
        q1_ = 0.0;
        q2_ = 0.0;
        q3_ = 0.0;
    }

    void updateIMU(double gx, double gy, double gz, double ax, double ay, double az, double dt){
        double norm = sqrt(ax*ax + ay*ay + az*az);
        if(norm < 1e-6) return;

        ax /= norm;
        ay /= norm;
        az /= norm;


        double f1 = 2.0 * (q1_*q3_ - q0_*q2_) - ax;
        double f2 = 2.0 * (q0_*q1_ + q2_*q3_) - ay;
        double f3 = 2.0 * (0.5 - q1_*q1_ - q2_*q2_) - az;

        double J11or24 = 2.0 * q2_;
        double J12or23 = 2.0 * q3_;
        double J13or22 = 2.0 * q0_;
        double J14or21 = 2.0 * q1_;
        double J32 = 2.0 * J14or21;
        double J33 = 2.0 * J11or24;
        double J41 = 2.0 * q3_;
        double J42 = 2.0 * q0_;
        double J43 = 2.0 * q1_;
        double J44 = 2.0 * q2_;
        double J51 = J32;
        double J52 = J33;
        double J53 = 2.0 * J13or22;
        double J54 = 2.0 * J12or23;
        double J61 = J42;
        double J62 = J41;
        double J63 = J44;
        double J64 = J43;

        double s1 = J11or24*f1 - J41*f2 + J51*f3;
        double s2 = J12or23*f1 + J42*f2 + J52*f3;
        double s3 = J13or22*f1 - J43*f2 - J53*f3;
        double s4 = J14or21*f1 - J44*f2 + J54*f3;

        norm = sqrt(s1*s1 + s2*s2 + s3*s3 + s4*s4);
        if(norm < 1e-6) return;

        s1 /= norm;
        s2 /= norm;
        s3 /= norm;
        s4 /= norm;

        double qDot1 = 0.5 * (-q1_*gx - q2_*gy - q3_*gz);
        double qDot2 = 0.5 * (q0_*gx + q2_*gz - q3_*gy);
        double qDot3 = 0.5 * (q0_*gy - q1_*gz + q3_*gx);
        double qDot4 = 0.5 * (q0_*gz + q1_*gy - q2_*gx);

        qDot1 -= beta_ * s1;
        qDot2 -= beta_ * s2;
        qDot3 -= beta_ * s3;
        qDot4 -= beta_ * s4;

        q0_ += qDot1 * dt;
        q1_ += qDot2 * dt;
        q2_ += qDot3 * dt;
        q3_ += qDot4 * dt;

        norm = sqrt(q0_*q0_ + q1_*q1_ + q2_*q2_ + q3_*q3_);
        if(norm < 1e-6) {
            q0_ = 1.0;
            q1_ = q2_ = q3_ = 0.0;
        } else {
            q0_ /= norm;
            q1_ /= norm;    
            q2_ /= norm;
            q3_ /= norm;
        }       
    }

    void getEulerAngles(double& roll, double& pitch, double& yaw){
        roll = atan2(2.0*(q0_*q1_ + q2_*q3_), 1.0 - 2.0*(q1_*q1_ + q2_*q2_));
        double sinp = 2.0*(q0_*q2_ - q3_*q1_);
        if(fabs(sinp) >= 1.0)
            pitch = copysign(M_PI / 2.0, sinp);
        else
            pitch = asin(sinp);
        yaw = atan2(2.0*(q0_*q3_ + q1_*q2_), 1.0 - 2.0*(q2_*q2_ + q3_*q3_));

        if(roll > M_PI) roll -= 2.0*M_PI;
        if(roll < -M_PI) roll += 2.0*M_PI;
        if(pitch > M_PI) pitch -= 2.0*M_PI;
        if(pitch < -M_PI) pitch += 2.0*M_PI;
        if(yaw > M_PI) yaw -= 2.0*M_PI;
        if(yaw < -M_PI) yaw += 2.0*M_PI;

    }

    void initFromAccel(double ax, double ay, double az){
        double norm = sqrt(ax*ax + ay*ay + az*az);
        if(norm < 1e-6) return;

        ax /= norm;
        ay /= norm;
        az /= norm;
        
        double pitch = asin(-ax);
        double roll = atan2(ay, -az);

        double cy = cos(roll * 0.5);
        double sy = sin(roll * 0.5);
        double cp = cos(pitch * 0.5);
        double sp = sin(pitch * 0.5);
        double cz = 1.0;
        double sz = 0.0;

        q0_ = cy * cp * cz + sy * sp * sz;
        q1_ = cy * cp * sz - sy * sp * cz;
        q2_ = sy * cp * cz + cy * sp * sz;
        q3_ = cy * sp * cz - sy * cp * sz;

        norm = sqrt(q0_*q0_ + q1_*q1_ + q2_*q2_ + q3_*q3_);
        if(norm > 1e-6) {
            q0_ /= norm;
            q1_ /= norm;
            q2_ /= norm;
            q3_ /= norm;
        } else {
            q0_ = 1.0;
            q1_ = q2_ = q3_ = 0.0;  
        }
    }

    void setBeta(double beta){  
        beta_ = beta;
    }

private:
    double q0_, q1_, q2_, q3_;
    double beta_;
};


class IMUProcessor{
public:
    IMUProcessor() : nh_("~"){
        beta_ = nh_.param<double>("beta", 0.1);
        gyro_bias_alpha_ = nh_.param<double>("gyro_bias_alpha", 0.005);
        gyro_static_thresh_ = nh_.param<double>("gyro_static_thresh", 0.02);
        init_samples_ = nh_.param<int>("init_samples", 200);    

        ahrs_.setBeta(beta_);

        roll_ = 0.0;
        pitch_ = 0.0;
        yaw_ = 0.0;
        last_time_ = ros::Time(0);

        gyro_bias_x_ = 0.0;
        gyro_bias_y_ = 0.0;
        gyro_bias_z_ = 0.0;
        initialized_ = false;
        init_count_ = 0;

        gyro_sum_x_ = 0.0;
        gyro_sum_y_ = 0.0;
        gyro_sum_z_ = 0.0;

        imu_sub_ = nh_.subscribe("/imu_raw", 100, &IMUProcessor::imuCallback, this);
        angles_pub_ = nh_.advertise<geometry_msgs::Vector3>("imu_angles", 10);
        status_timer_ = nh_.createTimer(ros::Duration(2.0), &IMUProcessor::statusCallback, this);
        last_data_time_ = ros::Time(0);

        ROS_INFO("IMU Processor Node");
        ROS_INFO("  Beta: %.3f", beta_);
        ROS_INFO("  Gyro bias alpha: %.4f", gyro_bias_alpha_);
        ROS_INFO("  Static threshold: %.4f rad/s", gyro_static_thresh_);
        ROS_INFO("  Init samples: %d", init_samples_);
        ROS_INFO("Waiting for IMU data from /imu_raw...");
    }

    void statusCallback(const ros::TimerEvent& event){
        ros::Time now = ros::Time::now();
        if(last_data_time_.isZero()){
            ROS_WARN_THROTTLE(5.0, "Waiting for IMU data...");
        } else {
            double time_since_last = (now - last_data_time_).toSec();
            if(time_since_last > 5.0){
                ROS_ERROR_THROTTLE(5.0, "IMU data timeout! Last data: %.1f seconds ago", time_since_last);
            }
        }
    }

    void imuCallback(const sensor_msgs::Imu::ConstPtr& msg){
        last_data_time_ = ros::Time::now();

        ros::Time current_time = msg->header.stamp;
        if(current_time.isZero()){
            current_time = ros::Time::now();
        }

        double dt = 0.01;
        if(!last_time_.isZero()){
            dt = (current_time - last_time_).toSec();
            if(dt <= 0 || dt > 0.1){
                dt = 0.01;
                ROS_WARN_THROTTLE(1.0, "Invalid time delta, using default 0.01s");
            }
        }

        double accel_x = msg->linear_acceleration.x;
        double accel_y = msg->linear_acceleration.y;
        double accel_z = msg->linear_acceleration.z;

        double gyro_x = msg->angular_velocity.x;
        double gyro_y = msg->angular_velocity.y;
        double gyro_z = msg->angular_velocity.z;

        double gyro_x_corrected = gyro_x - gyro_bias_x_;
        double gyro_y_corrected = gyro_y - gyro_bias_y_;
        double gyro_z_corrected = gyro_z - gyro_bias_z_;

        if(!initialized_){
            gyro_sum_x_ += gyro_x_corrected;
            gyro_sum_y_ += gyro_y_corrected;
            gyro_sum_z_ += gyro_z_corrected;
            init_count_++;

            if(init_count_ % 50 == 0){  

            if(init_count_ >= init_samples_){
                gyro_bias_x_ = gyro_sum_x_ / init_count_;
                gyro_bias_y_ = gyro_sum_y_ / init_count_;
                gyro_bias_z_ = gyro_sum_z_ / init_count_;

                initialized_ = true;
                ROS_INFO("Gyro bias calibration completed:");
                ROS_INFO("  bias_x = %.6f rad/s", gyro_bias_x_);
                ROS_INFO("  bias_y = %.6f rad/s", gyro_bias_y_);
                ROS_INFO("  bias_z = %.6f rad/s", gyro_bias_z_);

                ahrs_.initFromAccel(accel_x, accel_y, accel_z);
                ahrs_.getEulerAngles(roll_, pitch_, yaw_);
                ROS_INFO("Initial orientation:");
                ROS_INFO("  Roll = %.2f° (%.4f rad)", roll_ * 180.0 / M_PI, roll_);
                ROS_INFO("  Pitch = %.2f° (%.4f rad)", pitch_ * 180.0 / M_PI, pitch_);
                ROS_INFO("  Yaw = %.2f° (%.4f rad)", yaw_ * 180.0 / M_PI, yaw_);
                ROS_INFO("IMU processing started!");
            } else {
                last_time_ = current_time;
                return;
            }
        }

        double gyro_x_corrected = gyro_x - gyro_bias_x_;
        double gyro_y_corrected = gyro_y - gyro_bias_y_;
        double gyro_z_corrected = gyro_z - gyro_bias_z_;

        if(fabs(gyro_x_corrected) < gyro_static_thresh_){
            gyro_bias_x_ = (1.0 - gyro_bias_alpha_) * gyro_bias_x_ + gyro_bias_alpha_ * gyro_x;
        }
        if(fabs(gyro_y_corrected) < gyro_static_thresh_){
            gyro_bias_y_ = (1.0 - gyro_bias_alpha_) * gyro_bias_y_ + gyro_bias_alpha_ * gyro_y;
        }
        if(fabs(gyro_z_corrected) < gyro_static_thresh_){
            gyro_bias_z_ = (1.0 - gyro_bias_alpha_) * gyro_bias_z_ + gyro_bias_alpha_ * gyro_z;
        }

        ahrs_.updateIMU(gyro_x_corrected, gyro_y_corrected, gyro_z_corrected, accel_x, accel_y, accel_z, dt);   
        ahrs_.getEulerAngles(roll_, pitch_, yaw_);

        geometry_msgs::Vector3 angles_msg;
        angles_msg.x = roll_ * 180.0 / M_PI;
        angles_msg.y = pitch_ * 180.0 / M_PI;
        angles_msg.z = yaw_ * 180.0 / M_PI;
        angles_pub_.publish(angles_msg);

        last_time_ = current_time;
    }   

    bool isInitialized() const { return initialized_; }

private:
    ros::NodeHandle nh_;
    double beta_;
    double gyro_bias_alpha_;
    double gyro_static_thresh_;
    int init_samples_;
    MadgwickAHRS ahrs_;
    double roll_, pitch_, yaw_;
    ros::Time last_time_;   
    double gyro_bias_x_, gyro_bias_y_, gyro_bias_z_;
    bool initialized_;
    int init_count_;
    double gyro_sum_x_, gyro_sum_y_, gyro_sum_z_;
    ros::Subscriber imu_sub_;
    ros::Publisher angles_pub_;
    ros::Timer status_timer_;
    ros::Time last_data_time_;
};

int main(int argc, char** argv){
    ros::init(argc, argv, "imu_processor_node");
    IMUProcessor processor;
    ros::spin();

    return 0;
}