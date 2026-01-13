# STM32F103C8T6 + ESP8266 (ESP-01S) ROS 控制配置说明

## 1. ros_lib 已生成
ros_lib 已自动生成在 `User/ros_lib/ros_lib/` 目录下。

## 2. Keil 工程配置

### 2.1 添加包含路径
在 Keil 工程设置中，添加以下包含路径（Include Paths）：
```
User
User/ros_lib/ros_lib
```

### 2.2 将 main.c 改为 C++ 编译
**重要**：`main.c` 需要使用 C++ 编译器编译，因为 rosserial 是 C++ 库。

方法1：在 Keil 中右键 `main.c` → Options → 取消勾选 "C/C++" → 改为 "C++"

方法2：将 `main.c` 重命名为 `main.cpp`（推荐）

### 2.3 添加源文件到工程
确保以下文件已添加到 Keil 工程：
- `User/main.c` (或 `main.cpp`)
- `User/motor.c`
- `User/servo.c`
- `User/usart.c`

## 3. 引脚配置

### 3.1 电机控制（L298N）
- **左轮 PWM**：PA6 (TIM3_CH1)
- **右轮 PWM**：PA7 (TIM3_CH2)
- **左轮 DIR**：PB0
- **右轮 DIR**：PB1

### 3.2 舵机控制（SG90）
- **信号线**：PA1 (TIM2_CH2)

### 3.3 ESP-01S 串口通信
- **TX**：PA9 (USART1_TX)
- **RX**：PA10 (USART1_RX)
- **波特率**：115200

## 4. ESP-01S AT 配置步骤

### 4.1 连接 ESP-01S 到串口调试工具
使用 USB 转串口模块连接 ESP-01S：
- VCC → 3.3V（注意：ESP-01S 需要 3.3V，不能接 5V）
- GND → GND
- TX → USB转串口的 RX
- RX → USB转串口的 TX

### 4.2 配置 ESP-01S 为 WiFi 透传模式
打开串口调试工具（如 PuTTY、Arduino IDE 串口监视器），波特率设为 115200，发送以下 AT 命令：

```
AT                          // 测试连接
AT+RST                      // 重启模块
AT+CWMODE=1                 // 设置为 STA 模式
AT+CWJAP="你的WiFi名称","WiFi密码"    // 连接 WiFi（等待连接成功）
AT+CIPMODE=1                // 设置为透传模式
AT+CIPSTART="TCP","192.168.31.64",11411    // 连接到 PC 的 rosserial TCP 服务器
AT+CIPSEND                  // 进入透传模式
```

**注意**：
- 将 `"你的WiFi名称"` 和 `"WiFi密码"` 替换为实际的 WiFi 名称和密码
- 将 `192.168.31.64` 替换为你的 PC IP 地址（运行 ROS 的机器）
- 端口 `11411` 是 rosserial TCP 服务器的默认端口

### 4.3 验证连接
配置完成后，ESP-01S 会自动进入透传模式。此时：
1. STM32 通过 USART1 发送的数据会通过 ESP-01S 的 WiFi 发送到 PC
2. PC 通过 rosserial TCP 服务器发送的数据会通过 ESP-01S 的 WiFi 发送到 STM32

## 5. ROS 端启动

在 PC 端运行：
```bash
cd ~/catkin_ws
source devel/setup.bash
roslaunch yolo_car yolo_car.launch
```

这会启动：
- `yolo_car_detection`：YOLO 检测节点
- `yolo_car_control`：控制节点（发布 `/yolo_car/cmd_vel` 和 `/yolo_car/servo_angle`）
- `rosserial_tcp`：TCP 服务器（监听 11411 端口）

## 6. 功能说明

### 6.1 电机控制
- 订阅 `/yolo_car/cmd_vel` (geometry_msgs/Twist)
- `linear.x`：线速度（m/s）
- `angular.z`：角速度（rad/s）
- 超时保护：500ms 未收到新指令则自动停止电机

### 6.2 舵机控制
- 订阅 `/yolo_car/servo_angle` (std_msgs/Float32)
- 角度范围：0~180°
- 频率：50Hz PWM

## 7. 故障排查

### 7.1 STM32 无法连接 ROS
- 检查 ESP-01S 是否已连接到 WiFi
- 检查 PC IP 地址是否正确
- 检查 rosserial TCP 服务器是否已启动（`roslaunch yolo_car yolo_car.launch`）
- 检查串口连接（PA9/PA10）是否正确

### 7.2 电机不转或方向错误
- 检查 L298N 接线是否正确
- 检查 PWM 和 DIR 引脚配置
- 如果左轮方向错误，可在 `motor.c` 中调整 `motor_set_cmd()` 函数的左轮方向逻辑

### 7.3 舵机不转
- 检查舵机信号线是否连接到 PA1
- 检查舵机供电是否足够（SG90 需要 5V）
- 检查 PWM 频率是否为 50Hz

## 8. 编译注意事项

1. **C++ 编译**：确保 `main.c` 使用 C++ 编译器
2. **包含路径**：确保添加了 `User/ros_lib/ros_lib` 到包含路径
3. **优化等级**：建议使用 `-O2` 优化等级
4. **标准库**：确保链接了标准 C++ 库

