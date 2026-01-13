# my_car_yolo 功能包

基于ROS1的智能视觉跟随小车系统，实现YOLO物体检测 + 点跳变路径跟踪控制。

## 📋 系统架构

### 🎯 核心功能
- **视觉伺服控制**：YOLO检测标志物，控制小车按顺序到达预设像素点（A→B→C→D循环）
- **手眼标定**：像素坐标系与世界坐标系转换
- **多传感器融合**：IMU + 编码器数据融合，提高定位精度
- **实时舵机控制**：根据IMU角度调整标志物朝向，始终面向摄像头

### 🏗️ 模块组成

#### 1. **ESP32-S3底层控制** (Arduino)
- **发布**：`/my_car_yolo/imu_raw`, `/my_car_yolo/wheel_encoders`
- **订阅**：`/my_car_yolo/cmd_vel`, `/my_car_yolo/target_point`, `/my_car_yolo/servo_angle`
- **功能**：电机控制、舵机控制、传感器数据采集、WiFi-ROS通信

#### 2. **YOLO主控制节点** (C++)
- **订阅**：摄像头图像
- **发布**：`/my_car_yolo/target_point`, `/my_car_yolo/cmd_vel`, `/my_car_yolo/servo_angle`
- **功能**：YOLO检测、手眼标定、点跳变状态机、到达检测

#### 3. **IMU处理器** (Python)
- **订阅**：`/my_car_yolo/imu_raw`
- **发布**：`/my_car_yolo/imu_processed`
- **功能**：角度转换、360°跳变处理、卡尔曼滤波、零偏校准

#### 4. **里程计处理器** (Python)
- **订阅**：`/my_car_yolo/wheel_encoders`, `/my_car_yolo/imu_processed`
- **发布**：`/odom`
- **功能**：运动学模型、航迹推演、互补滤波、数据融合

## 🔧 安装与配置

### 1. 系统依赖
```bash
# Ubuntu 20.04
sudo apt-get install ros-noetic-cv-bridge ros-noetic-sensor-msgs
sudo apt-get install python3-pip
pip3 install ultralytics opencv-python numpy

# ESP32开发环境
# 下载Arduino IDE并安装ESP32支持
```

### 2. 编译功能包
```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

### 3. 手眼标定
```bash
# 启动标定程序
rosrun my_car_yolo calibration.py

# 在图像窗口中：
# 'c' - 开始采集标定点
# 's' - 保存标定结果
# 'q' - 退出
```

### 4. ESP32配置
- 修改 `esp32/my_car_yolo_control.ino` 中的WiFi配置
- 接线按照注释连接硬件
- 上传代码到ESP32-S3

## 🚀 运行系统

### 一键启动
```bash
roslaunch my_car_yolo my_car_yolo.launch
```

### 分步启动
```bash
# 1. 启动rosserial (ESP32通信)
rosrun rosserial_python serial_node.py tcp 11411

# 2. 启动数据处理器
rosrun my_car_yolo imu_processor.py
rosrun my_car_yolo odometry_processor.py

# 3. 启动主控制节点
rosrun my_car_yolo my_car_yolo_node
```

## ⚙️ 参数配置

### 全局变量（在launch文件中修改）
```xml
<!-- YOLO检测参数 -->
<param name="conf_threshold" value="0.5"/>              <!-- 置信度阈值 -->
<param name="arrival_threshold" value="20.0"/>         <!-- 到达距离阈值(像素) -->
<param name="stable_frames_required" value="5"/>       <!-- 稳定帧数要求 -->

<!-- 控制参数 -->
<param name="target_speed" value="0.1"/>               <!-- 目标速度(m/s) -->

<!-- IMU处理参数 -->
<param name="sample_rate" value="50.0"/>              <!-- 采样率(Hz) -->
<param name="use_kalman" value="true"/>               <!-- 使用卡尔曼滤波 -->

<!-- 里程计参数 -->
<param name="ticks_per_rev" value="260.0"/>          <!-- 每转脉冲数 -->
<param name="wheel_radius" value="0.048"/>           <!-- 轮子半径(m) -->
<param name="wheel_base" value="0.1475"/>            <!-- 轮距(m) -->
```

## 📡 ROS话题

### 发布话题
- `/my_car_yolo/imu_processed` (my_car_yolo/ImuProcessed) - 处理后的IMU数据
- `/odom` (nav_msgs/Odometry) - 里程计数据
- `/my_car_yolo/target_point` (geometry_msgs/Point) - 当前目标点
- `/my_car_yolo/cmd_vel` (geometry_msgs/Twist) - 速度指令
- `/my_car_yolo/servo_angle` (std_msgs/Float32) - 舵机角度

### 订阅话题
- `/usb_cam/image_raw` (sensor_msgs/Image) - 摄像头图像
- `/my_car_yolo/imu_raw` (sensor_msgs/Imu) - 原始IMU数据
- `/my_car_yolo/wheel_encoders` (my_car_yolo/WheelEncoders) - 原始编码器数据

## 🔍 控制逻辑

### 点跳变流程
```
YOLO检测到标志物 → 像素坐标
    ↓
手眼标定 → 世界坐标
    ↓
状态机判断：是否到达当前目标点？
    ↓ 是 → 切换到下一个点 (A→B→C→D)
    ↓ 否 → 继续向当前点运动
    ↓
ESP32接收目标点 → PID控制 → 到达目标点
    ↓
同时：IMU角度 → 舵机补偿 → 标志物始终朝向摄像头
```

### 到达检测
- 计算检测到的标志中心点与目标点的像素距离
- 连续N帧距离小于阈值才认为到达
- 避免单帧噪声导致的误判

### 舵机控制
- IMU检测小车朝向角度变化
- 计算标志物需要转动的角度以保持朝向摄像头
- 实时调整舵机角度

## 🛠️ 故障排除

### ESP32连接问题
```bash
# 检查WiFi连接
ping 192.168.1.100  # ESP32 IP

# 检查rosserial连接
rostopic list | grep my_car_yolo
```

### 标定问题
- 确保标定板在合适位置
- 检查摄像头视野是否覆盖整个运动范围
- 标定点要均匀分布，避免集中在一个区域

### 控制精度问题
- 检查手眼标定精度
- 调整PID参数
- 验证IMU和编码器数据质量

## 📝 开发说明

### 代码结构
```
my_car_yolo/
├── src/                    # C++源代码
├── scripts/               # Python脚本
├── esp32/                 # ESP32代码
├── config/                # 配置文件
├── launch/                # 启动文件
└── msg/                   # ROS消息定义
```

### 扩展功能
- 添加更多目标点形状（圆形、自定义路径）
- 实现路径规划算法
- 添加障碍物检测和避障
- 集成激光雷达提高定位精度

## 📄 许可证

TODO - 请设置合适的许可证
