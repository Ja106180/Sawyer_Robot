# ImuProcessed.msg
# 处理后的IMU数据

std_msgs/Header header
float64 yaw                    # 航向角 (度，0-360，已处理360°跳变)
float64 yaw_filtered           # 滤波后的航向角
float64 yaw_rate               # 角速度 (rad/s)
float64 gyro_bias_x            # X轴陀螺仪零偏
float64 gyro_bias_y            # Y轴陀螺仪零偏
float64 gyro_bias_z            # Z轴陀螺仪零偏
float64 accel_x                # X轴加速度 (m/s²)
float64 accel_y                # Y轴加速度 (m/s²)
float64 accel_z                # Z轴加速度 (m/s²)
