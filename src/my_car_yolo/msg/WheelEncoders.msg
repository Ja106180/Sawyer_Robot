# WheelEncoders.msg
# 轮子编码器原始数据

std_msgs/Header header
int32 left_pulses              # 左轮编码器脉冲数
int32 right_pulses             # 右轮编码器脉冲数
float64 left_velocity          # 左轮速度 (m/s)
float64 right_velocity         # 右轮速度 (m/s)
