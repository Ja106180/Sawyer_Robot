# yolo_car 功能包

YOLO物体检测与小车控制系统

## 功能概述

1. **YOLO物体检测**：使用YOLO模型检测USB摄像头中的物体
2. **动态路径跟踪**：两个点沿矩形边线运动
3. **小车控制**：PID控制小车跟随红色点运动
4. **舵机扫描**：识别不到牌子时自动扫描

## 安装依赖

### 1. 安装 rosserial（ESP32通信必需）

```bash
sudo apt-get install ros-noetic-rosserial-arduino
sudo apt-get install ros-noetic-rosserial
```

### 2. 安装Arduino IDE和ESP32支持

1. 下载并安装Arduino IDE：https://www.arduino.cc/en/software
2. 在Arduino IDE中添加ESP32支持：
   - 文件 -> 首选项 -> 附加开发板管理器网址
   - 添加：`https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json`
   - 工具 -> 开发板 -> 开发板管理器 -> 搜索"ESP32" -> 安装

### 3. 安装rosserial Arduino库

在Arduino IDE中：
- 工具 -> 管理库 -> 搜索"ros_lib" -> 安装

或者手动安装：
```bash
cd ~/Arduino/libraries
rm -rf ros_lib
rosrun rosserial_arduino make_libraries.py .
```

## 编译和运行

### 1. 编译ROS包

```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

### 2. 上传ESP32代码

1. 打开Arduino IDE
2. 打开文件：`~/catkin_ws/src/yolo_car/esp32/yolo_car_control.ino`
3. 选择开发板：工具 -> 开发板 -> ESP32 Arduino -> ESP32S3 Dev Module
4. 选择端口：工具 -> 端口 -> 选择ESP32的串口
5. 上传代码

### 3. 配置WiFi（ESP32代码中）

修改 `yolo_car_control.ino` 中的WiFi配置：
```cpp
const char* ssid = "你的WiFi名称";
const char* password = "你的WiFi密码";
IPAddress server(192, 168, 31, 150);  // ROS master的IP地址
```

### 4. 启动系统

```bash
cd ~/catkin_ws
source devel/setup.bash
roslaunch yolo_car yolo_car.launch
```

### 5. rosserial节点（ESP32通信）

已在 `yolo_car.launch` 中自动启动 TCP 模式（默认端口 11411），无需手动运行。

如果需要手动或使用 USB，可参考：
```bash
# WiFi/TCP 手动启动
rosrun rosserial_python serial_node.py tcp 11411

# USB 连接
rosrun rosserial_python serial_node.py /dev/ttyUSB0 115200
```

## 话题说明

### 发布的话题

- `/yolo_car/detections` (yolo_car/ObjectDetections)：检测到的物体信息
- `/yolo_car/target_point` (geometry_msgs/Point)：红色点位置
- `/yolo_car/cmd_vel` (geometry_msgs/Twist)：速度指令（发送给ESP32）
- `/yolo_car/servo_angle` (std_msgs/Float32)：舵机角度指令（发送给ESP32）

### 订阅的话题

ESP32订阅：
- `/yolo_car/cmd_vel`：速度指令
- `/yolo_car/servo_angle`：舵机角度指令

## 参数配置

### 控制参数（可在launch文件中修改）

- `kp_x`, `ki_x`, `kd_x`：X方向（角速度）PID参数
- `kp_y`, `ki_y`, `kd_y`：Y方向（线速度）PID参数
- `deadzone_x`, `deadzone_y`：死区大小（像素）
- `max_linear_vel`：最大线速度（m/s）
- `max_angular_vel`：最大角速度（rad/s）
- `control_rate`：控制频率（Hz）
- `lost_timeout`：识别不到超时时间（秒）

## 硬件接线

### L298N电机驱动
- N1 -> ESP32 GPIO 3（左轮使能PWM）
- N3 -> ESP32 GPIO 5（右轮使能PWM）
- OUT1/OUT2 -> 左轮电机
- OUT3/OUT4 -> 右轮电机

### SG90舵机
- 信号线 -> ESP32 GPIO 2
- 电源 -> 5V
- 地 -> GND

## 故障排查

1. **ESP32无法连接ROS**：
   - 检查WiFi连接
   - 检查ROS master IP地址是否正确
   - 检查rosserial节点是否运行

2. **小车不运动**：
   - 检查L298N接线
   - 检查PWM输出是否正常
   - 检查速度指令是否发布

3. **舵机不工作**：
   - 检查舵机接线
   - 检查PWM频率设置（50Hz）

