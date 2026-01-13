# ESP32功能测试指南

## 准备工作

### 1. 确保ESP32已连接并运行
- ESP32已上传代码并连接到WiFi
- ESP32的串口监视器显示"ROS连接成功！"

### 2. 启动ROS核心和rosserial节点（不启动控制节点）

打开第一个终端：

```bash
cd ~/catkin_ws
source devel/setup.bash

# 启动roscore（如果还没运行）
roscore
```

打开第二个终端：

```bash
cd ~/catkin_ws
source devel/setup.bash

# 启动rosserial TCP节点（ESP32连接）
rosrun rosserial_python serial_node.py tcp 11411
```

**检查连接**：如果看到类似 "Connected to ESP32" 的消息，说明连接成功。

---

## 测试1：电机控制测试

### 测试1.1：让左电机前转

打开第三个终端：

```bash
cd ~/catkin_ws
source devel/setup.bash

# 发送命令：只让左轮转（线速度0.1 m/s，角速度0，这样左轮慢右轮快，但我们可以用负角速度让左轮单独转）
# 方法：设置线速度0，角速度为正，这样左轮会反转，右轮正转
# 或者：设置线速度0.1，角速度-0.5，这样左轮慢/停，右轮快

# 方案A：让左轮单独前转（差速控制）
# 线速度0.1 m/s，角速度-0.3 rad/s
# 左轮速度 = 0.1 - (-0.3 * 0.1475 / 2) = 0.1 + 0.022 = 0.122 m/s
# 右轮速度 = 0.1 + (-0.3 * 0.1475 / 2) = 0.1 - 0.022 = 0.078 m/s
# 这样左轮会转得快一些

# 更简单的方法：直接设置较大的角速度差
rostopic pub -r 10 /yolo_car/cmd_vel geometry_msgs/Twist "linear:
  x: 0.1
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: -0.4"
```

**观察**：左轮应该前转，右轮应该慢转或停转。

**停止电机**：

```bash
# 发送零速度指令
rostopic pub -1 /yolo_car/cmd_vel geometry_msgs/Twist "linear:
  x: 0.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0"
```

### 测试1.2：让右电机前转

```bash
# 线速度0.1 m/s，角速度+0.4 rad/s（正角速度让右轮转得快）
rostopic pub -r 10 /yolo_car/cmd_vel geometry_msgs/Twist "linear:
  x: 0.1
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.4"
```

**观察**：右轮应该前转，左轮应该慢转或停转。

**停止电机**：

```bash
rostopic pub -1 /yolo_car/cmd_vel geometry_msgs/Twist "linear:
  x: 0.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0"
```

### 测试1.3：让两个电机同时前转

```bash
# 只有线速度，没有角速度
rostopic pub -r 10 /yolo_car/cmd_vel geometry_msgs/Twist "linear:
  x: 0.1
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0"
```

**观察**：两个轮子应该同时前转。

**停止电机**：

```bash
rostopic pub -1 /yolo_car/cmd_vel geometry_msgs/Twist "linear:
  x: 0.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0"
```

---

## 测试2：数据接收测试

### 测试2.1：查看轮速数据

打开第四个终端：

```bash
cd ~/catkin_ws
source devel/setup.bash

# 查看轮速话题
rostopic echo /yolo_car/wheel_vel
```

**预期输出**：应该每50ms（20Hz）看到一次数据，格式如下：

```
header: 
  seq: 123
  stamp: 
    secs: 1234567890
    nsecs: 123456789
  frame_id: "base_link"
vector: 
  x: 0.05    # 左轮速度 (m/s)
  y: 0.05    # 右轮速度 (m/s)
  z: 0.0
---
```

**测试**：
1. 让电机转起来（使用测试1的命令）
2. 观察 `vector.x` 和 `vector.y` 是否有数值变化
3. 如果电机在转但数值为0，可能是编码器接线或参数问题

### 测试2.2：查看IMU原始数据

打开第五个终端（或使用Ctrl+C停止上一个echo，然后重新运行）：

```bash
cd ~/catkin_ws
source devel/setup.bash

# 查看IMU原始数据
rostopic echo /yolo_car/imu_raw
```

**预期输出**：应该每20ms（50Hz）看到一次数据，格式如下：

```
header: 
  seq: 456
  stamp: 
    secs: 1234567890
    nsecs: 123456789
  frame_id: "imu_link"
orientation: 
  x: 0.0
  y: 0.0
  z: 0.0
  w: 0.0
orientation_covariance: [0.0, ...]
angular_velocity: 
  x: 0.001    # X轴角速度 (rad/s)
  y: 0.002    # Y轴角速度 (rad/s)
  z: 0.003    # Z轴角速度 (rad/s)
angular_velocity_covariance: [0.0, ...]
linear_acceleration: 
  x: 0.1      # X轴加速度 (m/s²)
  y: 0.2      # Y轴加速度 (m/s²)
  z: 9.8      # Z轴加速度 (m/s²，静止时应该接近重力加速度)
linear_acceleration_covariance: [0.0, ...]
---
```

**测试**：
1. 静止时：`linear_acceleration.z` 应该接近 9.8 m/s²（重力）
2. 移动小车：观察 `angular_velocity` 和 `linear_acceleration` 是否有变化
3. 如果所有值都是0，可能是MPU6050接线或I2C通信问题

---

## 测试2.3：查看话题列表和频率

检查所有话题是否正常：

```bash
# 查看所有话题
rostopic list

# 应该看到：
# /yolo_car/cmd_vel
# /yolo_car/servo_angle
# /yolo_car/wheel_vel
# /yolo_car/imu_raw
# /rosout
# /rosout_agg

# 查看话题发布频率
rostopic hz /yolo_car/wheel_vel    # 应该约20Hz
rostopic hz /yolo_car/imu_raw      # 应该约50Hz

# 查看话题类型
rostopic type /yolo_car/wheel_vel
rostopic type /yolo_car/imu_raw
```

---

## 故障排查

### 如果电机不转：
1. 检查ESP32串口输出，看是否收到cmd_vel消息
2. 检查L298N接线（PWM和方向引脚）
3. 检查PWM值是否太小（MG310需要一定占空比才能启动）

### 如果轮速数据为0：
1. 检查编码器接线（A、B相）
2. 检查编码器参数（TICKS_PER_REV、WHEEL_RADIUS_M）是否正确
3. 在ESP32串口监视器查看是否有编码器计数

### 如果IMU数据为0或异常：
1. 检查MPU6050接线（SCL、SDA、VCC、GND）
2. 检查I2C地址是否正确（0x68）
3. 在ESP32串口监视器查看MPU6050初始化是否成功

---

## 快速测试命令汇总

```bash
# 1. 启动roscore（终端1）
roscore

# 2. 启动rosserial（终端2）
rosrun rosserial_python serial_node.py tcp 11411

# 3. 测试左轮（终端3）
rostopic pub -r 10 /yolo_car/cmd_vel geometry_msgs/Twist "linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: -0.4}"

# 4. 测试右轮（终端3，先停止上一个）
rostopic pub -r 10 /yolo_car/cmd_vel geometry_msgs/Twist "linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.4}"

# 5. 停止电机（终端3）
rostopic pub -1 /yolo_car/cmd_vel geometry_msgs/Twist "linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}"

# 6. 查看轮速（终端4）
rostopic echo /yolo_car/wheel_vel

# 7. 查看IMU（终端5）
rostopic echo /yolo_car/imu_raw
```

