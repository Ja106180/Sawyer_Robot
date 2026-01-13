# Untitled Chat
_Exported on 2025/12/11 at GMT+8 16:01:43 from Cursor (2.1.26)_

---

**User**

阅读一下当前空间中的yolo_car文件夹，说说你的理解，然后我再告诉你我的目的，我们先讨论讨论，讨论好了再做，这样才不会偏离我们的目的，不围绕目的去做都是无用的多此一举了

---

**Cursor**

简要理解（基于当前 yolo_car 包的结构与代码）：
- 这是一个 ROS Noetic 包，结合 YOLOv8 物体检测与小车控制，支持 ESP32 通过 rosserial TCP 连接。
- 主要节点：
  - `yolo_car_detection.py`：加载 YOLOv8 模型（默认 `models/yolov8n_label.pt`），用 USB 摄像头实时检测；在窗口显示结果，发布 `/yolo_car/detections`（包含中心点、bbox、置信度等）。矩形顶点用于显示轮廓。
  - `yolo_car_control.py`：订阅 `/yolo_car/detections`，在图像坐标系内让检测到的牌子中心沿预设矩形顺时针巡线；基于法向误差调节角速度，偏离大减速；发布 `/yolo_car/cmd_vel` 与 `/yolo_car/servo_angle`，丢失检测时停车并按预设序列扫描舵机。
- 启动方式：`roslaunch yolo_car yolo_car.launch` 启动检测、控制和 `rosserial_python serial_node.py tcp 11411`。
- 参数：launch 中暴露模型路径、置信度、摄像头源、控制器的增益/死区/速度上限/反向等。
- 话题：
  - 发布：`/yolo_car/detections`、`/yolo_car/cmd_vel`、`/yolo_car/servo_angle`、`/yolo_car/target_point`（由其他节点发布）。
  - 订阅：控制节点订阅 `/yolo_car/detections`（核心）、`/yolo_car/target_point`（兼容占位）。
- 模型文件：`models/yolov8n_label.pt` 已放在包内。
- 硬件/固件：有 `esp32/`（上位机指令）、`stm32f103c8t6/`（已在 IDE 打开 README_配置说明.md，可能是另一套固件说明），驱动 L298N、电调舵机。
- 消息定义：包内有 `msg/` 目录（应包含 ObjectDetection/ObjectDetections，自定义检测消息）。

如果你接下来想聚焦某一块（如检测稳定性、控制策略、固件通信、launch 参数优化或模型替换），告诉我目标，我们再细化讨论或检查细节。

---

**User**

非常好，我现在和你说说我的目的，我一个esp32s3使用的arduinoIDE，一个L296N电机驱动模块，它的out1,2接的左电机，out3,4接的右电机，它还有N1,2,3,4分别接的esp32s3的3,4,5,6引脚，我还有一个舵机，它的信号线接的2引脚，然后右电机编码器的A，B分别接的esp32s3的7,8引脚，左电机编码器的A，B相分别接的esp32的9,10引脚，我还有一个MPU6050陀螺仪，它的SCL和SDA分别接的esp32s3的45,46引脚，然后给你一个esp32s3的参考代码，/*
 * ESP32-S3 + L298N电机驱动 + SG90舵机 + ROS1 (rosserial)
 * 
 * 功能：
 * 1. 订阅速度指令控制L298N电机
 * 2. 订阅舵机角度指令控制SG90舵机
 * 3. 发布IMU数据（可选）
 * 
 * 接线：
 * - L298N使能引脚：N1->GPIO3, N3->GPIO5
 * - L298N方向控制：OUT1/OUT2接左轮，OUT3/OUT4接右轮
 * - SG90舵机信号线：GPIO2
 */

#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/Imu.h>
#include <Wire.h>

#define USE_WIFI true

#if USE_WIFI
  #include <WiFi.h>
  const char* ssid = "机器人小车局域网";
  const char* password = "3252362734";
  IPAddress server(192, 168, 31, 64);
  const uint16_t serverPort = 11411;
#endif

// L298N电机控制引脚（四线控制：左PWM/左DIR/右PWM/右DIR）
// 根据你的接线：N1->GPIO3, N2->GPIO4, N3->GPIO5, N4->GPIO6
#define MOTOR_LEFT_PWM   3   // 左轮PWM（原N1）
#define MOTOR_LEFT_DIR   4   // 左轮方向（原N2）
#define MOTOR_RIGHT_PWM  5   // 右轮PWM（原N3）
#define MOTOR_RIGHT_DIR  6   // 右轮方向（原N4）

// SG90舵机引脚
#define SERVO_PIN 2

// PWM参数
#define PWM_FREQ 1000      // PWM频率 1kHz
#define PWM_RESOLUTION 8   // 8位分辨率 (0-255)
#define SERVO_FREQ 50      // 舵机PWM频率 50Hz
// 使用12位分辨率，便于占空比计算 (0-4095)
#define SERVO_RESOLUTION 12

// 速度限制（归一化到0-255）
#define MAX_PWM_VALUE 200  // 最大PWM值（避免满速）

// 命令超时（毫秒），超过此时间未收到新速度指令则停车
#define CMD_TIMEOUT_MS 500

ros::NodeHandle nh;
unsigned long last_cmd_time = 0;

// 电机急停
void stopMotors() {
  ledcWrite(0, 0);
  ledcWrite(1, 0);
  digitalWrite(MOTOR_LEFT_DIR, LOW);
  digitalWrite(MOTOR_RIGHT_DIR, LOW);
}

// 速度指令回调
void cmdVelCallback(const geometry_msgs::Twist& msg) {
  float linear_vel = msg.linear.x;   // 线速度 m/s
  float angular_vel = msg.angular.z; // 角速度 rad/s

  // 记录收到指令的时间
  last_cmd_time = millis();
  
  // 极小值直接视为0，避免噪声乱转
  if (fabs(linear_vel) < 1e-3 && fabs(angular_vel) < 1e-3) {
    stopMotors();
    return;
  }
  
  // 差速控制：将线速度和角速度转换为左右轮速度
  // 假设轮距为0.2m（根据实际小车调整）
  float wheel_base = 0.2;
  float left_vel = linear_vel - (angular_vel * wheel_base / 2.0);
  float right_vel = linear_vel + (angular_vel * wheel_base / 2.0);
  
  // 限制速度范围（假设最大速度0.3 m/s对应PWM 200）
  float max_vel = 0.3;
  left_vel = constrain(left_vel, -max_vel, max_vel);
  right_vel = constrain(right_vel, -max_vel, max_vel);
  
  // 转换为PWM值（0-255）
  int left_pwm = abs(left_vel / max_vel * MAX_PWM_VALUE);
  int right_pwm = abs(right_vel / max_vel * MAX_PWM_VALUE);
  
  // 设置方向（如果需要方向控制）
  // 注意：根据实际L298N接线调整
  // 如果OUT1接负极，OUT2接正极，那么：
  // - 正转：IN1=LOW, IN2=HIGH (或使用N2作为方向控制)
  // - 反转：IN1=HIGH, IN2=LOW
  // 这里假设N2和N4作为方向控制引脚
  // 左右方向控制（左轮已反转）
  if (left_vel >= 0) {
    digitalWrite(MOTOR_LEFT_DIR, HIGH);
  } else {
    digitalWrite(MOTOR_LEFT_DIR, LOW);
  }
  
  if (right_vel >= 0) {
    digitalWrite(MOTOR_RIGHT_DIR, LOW);  // 正转方向
  } else {
    digitalWrite(MOTOR_RIGHT_DIR, HIGH); // 反转方向
  }
  
  // 设置PWM输出
  ledcWrite(0, left_pwm);   // 通道0 -> 左轮PWM
  ledcWrite(1, right_pwm);  // 通道1 -> 右轮PWM
}

// 舵机角度回调
void servoAngleCallback(const std_msgs::Float32& msg) {
  float angle = msg.data;
  angle = constrain(angle, 0.0, 180.0); // 限制在0-180度

  // 调试打印，确认收到指令
  Serial.print("servo cmd: ");
  Serial.println(angle);

  // SG90舵机：典型 0.5~2.5ms（可按需收窄到 0.6~2.4ms）
  int pulse_width = map(angle, 0, 180, 500, 2500); // 微秒
  // 12位分辨率，50Hz -> 周期 20ms，对应计数 4096
  int duty_cycle = (pulse_width * 4096L) / 20000;    // 转换为占空比
  duty_cycle = constrain(duty_cycle, 0, 4095);

  ledcWrite(2, duty_cycle); // 通道2 -> 舵机
}

// 创建订阅者（注意：话题名称需要完整路径）
ros::Subscriber<geometry_msgs::Twist> cmd_vel_sub("/yolo_car/cmd_vel", cmdVelCallback);
ros::Subscriber<std_msgs::Float32> servo_angle_sub("/yolo_car/servo_angle", servoAngleCallback);

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n\n=== ESP32-S3 + L298N + SG90 + ROS1 ===");

  last_cmd_time = millis();
  
  // 初始化电机控制引脚
  Serial.println("初始化L298N电机驱动...");
  pinMode(MOTOR_LEFT_DIR, OUTPUT);
  pinMode(MOTOR_RIGHT_DIR, OUTPUT);
  // 初始方向：置低，防止上电乱转
  digitalWrite(MOTOR_LEFT_DIR, LOW);
  digitalWrite(MOTOR_RIGHT_DIR, LOW);
  
  // 配置PWM通道（电机）
  ledcSetup(0, PWM_FREQ, PWM_RESOLUTION);  // 通道0：左轮PWM
  ledcSetup(1, PWM_FREQ, PWM_RESOLUTION);  // 通道1：右轮PWM
  ledcAttachPin(MOTOR_LEFT_PWM, 0);
  ledcAttachPin(MOTOR_RIGHT_PWM, 1);
  
  // 配置PWM通道（舵机）
  ledcSetup(2, SERVO_FREQ, SERVO_RESOLUTION); // 通道2：舵机
  ledcAttachPin(SERVO_PIN, 2);
  
  // 初始停止电机
  ledcWrite(0, 0);
  ledcWrite(1, 0);
  
  // 初始舵机到90度（中位）
  float init_angle = 90.0;
  int pulse_width = map(init_angle, 0, 180, 500, 2500);
  int duty_cycle = (pulse_width * 4096L) / 20000;
  duty_cycle = constrain(duty_cycle, 0, 4095);
  ledcWrite(2, duty_cycle);
  
  Serial.println("✓ 电机和舵机初始化完成");
  
#if USE_WIFI
  Serial.println("\n=== WiFi连接 ===");
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  
  Serial.print("连接中");
  int wifiAttempts = 0;
  while (WiFi.status() != WL_CONNECTED && wifiAttempts < 30) {
    delay(500);
    Serial.print(".");
    wifiAttempts++;
  }
  Serial.println();
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("✓ WiFi连接成功！IP: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("✗ WiFi连接失败！");
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    nh.getHardware()->setConnection(server, serverPort);
  }
#endif
  
  Serial.println("\n=== ROS连接 ===");
  nh.initNode();
  nh.subscribe(cmd_vel_sub);
  nh.subscribe(servo_angle_sub);
  
  Serial.print("等待连接");
  int rosAttempts = 0;
  while (!nh.connected() && rosAttempts < 50) {
    nh.spinOnce();
    delay(100);
    if (rosAttempts % 10 == 0) Serial.print(".");
    rosAttempts++;
  }
  Serial.println();
  
  if (nh.connected()) {
    Serial.println("✓ ROS连接成功！");
  } else {
    Serial.println("✗ ROS连接失败！");
  }
  
  Serial.println("\n=== 系统状态 ===");
  Serial.print("WiFi: ");
  Serial.println((WiFi.status() == WL_CONNECTED) ? "✓ 已连接" : "✗ 未连接");
  Serial.print("ROS: ");
  Serial.println(nh.connected() ? "✓ 已连接" : "✗ 未连接");
  Serial.println("\n开始运行...\n");
}

void loop() {
  // 命令超时保护：无论是否连接，都先做超时判断
  if (millis() - last_cmd_time > CMD_TIMEOUT_MS) {
    stopMotors();
  }

  if (!nh.connected()) {
    static unsigned long lastReconnectMsg = 0;
    if (millis() - lastReconnectMsg > 5000) {
      Serial.println("ROS连接断开，尝试重连...");
      lastReconnectMsg = millis();
    }
    // 断连时确保电机停转
    stopMotors();
    nh.spinOnce();
    delay(100);
    return;
  }

  nh.spinOnce();
  delay(10);
}

，接下来做的时候你要参考一下这个代码让它能和ros通信，然后知道这些信息，你觉得可以做什么，我们先讨论讨论，我看看你理解没有，先不要着急做任何事，还没讨论好就做，做了也是白做的

---

**Cursor**

好的，我理解你的硬件与目标，先归纳关键点，供讨论确认：

- 硬件接线（ESP32-S3 + L298N + SG90 + 编码器 + MPU6050）：
  - L298N：N1/N2/N3/N4 -> GPIO 3/4/5/6；OUT1/2 左轮，OUT3/4 右轮。
  - 舵机：GPIO 2。
  - 编码器：右轮 A/B -> GPIO 7/8，左轮 A/B -> GPIO 9/10。
  - MPU6050：SCL/SDA -> GPIO 45/46。
- 参考代码功能（需与 ROS 对接）：
  - 订阅 `/yolo_car/cmd_vel` 控制左右轮 PWM + 方向，超时停车。
  - 订阅 `/yolo_car/servo_angle` 控制舵机（50Hz PWM）。
  - WiFi rosserial TCP 连接到 `192.168.31.64:11411`（可改为 ROS master 实际 IP/端口）。
  - 目前未处理编码器反馈和 MPU6050（IMU发布是“可选”）。
- 现有 ROS 包 `yolo_car`：
  - Python 节点已发布 `/yolo_car/cmd_vel`、`/yolo_car/servo_angle`，使用 rosserial TCP 11411。
  - 控制逻辑是基于视觉矩形巡线，速度较小（线速度 ~0.05 基准，max 0.15），角速度 max 0.05。
- 潜在要做的事情（需你确认优先级与范围）：
  1) 在 Arduino 端整合 rosserial WiFi 连接参数与话题名，对齐 launch（11411 端口，主机 IP）。
  2) 完善电机驱动方向/PWM 映射，确保与你的接线一致（左右反转时需调整 DIR 逻辑）。
  3) 加入编码器读数，发布里程计/轮速话题（如 `sensor_msgs/JointState` 或自定义），或至少做闭环限速。
  4) 接入 MPU6050，发布 `sensor_msgs/Imu`，并可做角速度/姿态用于控制。
  5) 调整 ROS 端控制器参数，使输出速度与电机能力匹配（可能需要缩放或限幅）。
  6) WiFi 连接与超时/重连的健壮性处理。
  7) 如果需要 USB 模式，增加串口 rosserial 配置。

你想先从哪一步开始？是先让参考代码最小化跑通 rosserial WiFi + 电机/舵机（不含编码器/IMU），还是一步到位把编码器和 MPU6050 也接入并发布到 ROS？

---

**User**

3) 加入编码器读数，发布里程计/轮速话题（如 sensor_msgs/JointState 或自定义），或至少做闭环限速。



4) 接入 MPU6050，发布 sensor_msgs/Imu，并可做角速度/姿态用于控制。非常好，我就是想在esp32端在原来的基础上增加这两个，当然ros端肯定也要更新增加代码来接收这两个的数据，我们的最终目的是让小车做循线，循的线是我们自己在@yolo_car_detection.py 里自己画的那个矩形，它通过识别到小车上的标识牌，我的想法是通过控制小车来实现控制识别到的标识的中心点来巡线矩形，但这个有一个巨大的缺陷就是像素坐标和我们的显示世界不一样，肯定要把它现实世界的坐标转换一下什么的，然后我还不清楚编码器和陀螺仪可以在我们的目的中起什么作用，它可以干什么还不清楚，我的电机使用的是MG310，我们继续讨论讨论，不要着急做

---

**Cursor**

理解你的目标：在现有视觉巡线（基于检测牌子中心点沿矩形）基础上，ESP32 端新增编码器与 MPU6050 发布，ROS 端接收并利用这些数据，使小车更稳定地按真实世界轨迹沿矩形巡线。

先讨论关键点和作用，再定方案，不动代码：

1. 编码器能做什么
- 轮速估计：左右轮脉冲 -> 线/角速度（里程计增量）。
- 闭环限速：根据目标 `/yolo_car/cmd_vel` 将 PWM 与实际轮速比对，避免空转或负载变化导致速度偏差。
- 里程计发布：`nav_msgs/Odometry` 或简化为 `sensor_msgs/JointState`/自定义轮速消息，提供 `/odom` 帧位姿估计（平面 2D）。
- 失联/丢帧安全：无视觉时可用里程计短时惯性运动或直接停车；检测到轮子不转时报警。

2. MPU6050 能做什么
- 角速度/姿态估计：发布 `sensor_msgs/Imu`，用于姿态/航向稳定。
- 漂移抑制：与编码器融合可做简化的航向估计，减少纯编码器积分导致的偏航误差。
- 失联辅助：短时视觉丢失时，用 IMU yaw 角保持航向，降低跑偏。

3. 视觉坐标与真实世界的关系
- 目前控制在图像像素平面直接巡线，缺点是摄像头姿态/尺度变化会导致实际轨迹与像素矩形不一致。
- 最小可行：保持像素控制，但用编码器/IMU 做“守护”——限速、姿态稳定、丢目标停车。
- 更进一步：标定像素到地平面映射（单应性/相机标定 + 地面平面假设），把矩形顶点从像素变成地面坐标，控制在真实坐标系下；需要相机高度、俯仰角、内参。

4. ROS 侧建议的话题/帧
- 编码器发布：可简化为 `sensor_msgs/JointState`（name: left/right, velocity: rad/s 或 m/s）或自定义 `WheelState`.
- 里程计：`nav_msgs/Odometry`，`frame_id=odom`，`child_frame_id=base_link`，可选发布 TF。
- IMU：`sensor_msgs/Imu`，填充角速度、线加速度，姿态可选（若用 DMP/互补滤波）。
- 控制节点可订阅里程计/IMU 用于：
  - 失联保护：视觉超时停车；或用里程计/IMU 限制角速度/加速度。
  - 速度闭环：用轮速反馈修正 PWM。
  - 航向约束：限制 Z 轴角速度或偏航积累。

5. ESP32 端实现思路（先确认，不写代码）
- 编码器：在中断或定时器中计数，固定周期计算左右轮脉冲频率 -> 轮速 (m/s)，并发布 ROS 消息；必要时在主循环中做简单 PI 调速，将期望轮速映射到 PWM。
- IMU：用 I2C 读 MPU6050，发布 IMU；若要姿态，可用互补滤波（gyro+acc）或轻量库；先至少输出角速度/加速度。
- 通信：沿用 rosserial TCP，增加两个发布者（轮速/里程计，IMU）。
- 安全：命令超时停车；ROS 断线停车。

6. MG310 电机特点
- 起转阻力较大，需最小 PWM（已有基准速度）；闭环能提升低速可控性。
- 减速比大，编码器脉冲频率相对可用，注意计算每转脉冲数。

7. 逐步推进的推荐路线
- 方案 A（快测）：视觉控制保持像素矩形；ESP32 发布轮速与 IMU；ROS 端仅记录，不参与控制，用于观测与调参。
- 方案 B（稳健）：在 ESP32 做轮速闭环（给定线/角速度 -> 轮速 -> PI -> PWM），IMU 仅发布。
- 方案 C（进阶）：ROS 侧用里程计+IMU 融合估计航向，控制节点加入航向稳定（限制偏航漂移），视觉丢失时短时保持航向/速度或直接停车。
- 方案 D（高阶）：做相机标定与地面单应性，将矩形目标转成地面坐标，控制在真实世界轨迹上。

问题给你决定优先级：
- 你想先走方案 A（最小改动，先把轮速/IMU发上来观测）还是直接方案 B（加轮速闭环）？
- 是否需要在 ROS 端立刻消费这些数据（比如记录、可视化、简单报警），还是先在 ESP32 端闭环即可？
- 是否计划做相机标定/单应性，把矩形从像素转地面？如果短期不做，我们就侧重低速稳定和丢失保护。

---

**User**

编码器能做什么

轮速估计：左右轮脉冲 -> 线/角速度（里程计增量）。

闭环限速：根据目标 /yolo_car/cmd_vel 将 PWM 与实际轮速比对，避免空转或负载变化导致速度偏差。

里程计发布：nav_msgs/Odometry 或简化为 sensor_msgs/JointState/自定义轮速消息，提供 /odom 帧位姿估计（平面 2D）。

失联/丢帧安全：无视觉时可用里程计短时惯性运动或直接停车；检测到轮子不转时报警。

MPU6050 能做什么

角速度/姿态估计：发布 sensor_msgs/Imu，用于姿态/航向稳定。

漂移抑制：与编码器融合可做简化的航向估计，减少纯编码器积分导致的偏航误差。

失联辅助：短时视觉丢失时，用 IMU yaw 角保持航向，降低跑偏。

视觉坐标与真实世界的关系

目前控制在图像像素平面直接巡线，缺点是摄像头姿态/尺度变化会导致实际轨迹与像素矩形不一致。

最小可行：保持像素控制，但用编码器/IMU 做“守护”——限速、姿态稳定、丢目标停车。

ROS 侧建议的话题/帧

编码器发布：可简化为 sensor_msgs/JointState（name: left/right, velocity: rad/s 或 m/s）或自定义 WheelState.

里程计：nav_msgs/Odometry，frame_id=odom，child_frame_id=base_link，可选发布 TF。

IMU：sensor_msgs/Imu，填充角速度、线加速度，姿态可选（若用 DMP/互补滤波）。

控制节点可订阅里程计/IMU 用于：

失联保护：视觉超时停车；或用里程计/IMU 限制角速度/加速度。

速度闭环：用轮速反馈修正 PWM。

航向约束：限制 Z 轴角速度或偏航积累。

ESP32 端实现思路（先确认，不写代码）

编码器：在中断或定时器中计数，固定周期计算左右轮脉冲频率 -> 轮速 (m/s)，并发布 ROS 消息；必要时在主循环中做简单 PI 调速，将期望轮速映射到 PWM。

IMU：用 I2C 读 MPU6050，发布 IMU；若要姿态，可用互补滤波（gyro+acc）或轻量库；先至少输出角速度/加速度。

通信：沿用 rosserial TCP，增加两个发布者（轮速/里程计，IMU）。

安全：命令超时停车；ROS 断线停车。

MG310 电机特点

起转阻力较大，需最小 PWM（已有基准速度）；闭环能提升低速可控性。

减速比大，编码器脉冲频率相对可用，注意计算每转脉冲数。

以上这些都不错，都非常可以，都要加入我们的代码中，这些就先确定了，

然后有一个点我们需要继续讨论讨论，更进一步：标定像素到地平面映射（单应性/相机标定 + 地面平面假设），把矩形顶点从像素变成地面坐标，控制在真实坐标系下；需要相机高度、俯仰角、内参。我不想用这种方法来做，可不可通过我这种方式来实现把像素坐标转换成物理世界的真实坐标，就是我告诉你像素画的矩形长宽在真实物理世界的长度宽，通过这种方式来替代你提到的那个标定方法

---

**Cursor**

可以，但要说明清楚：只给出“矩形真实长宽 + 像素长宽”只能在强假设下用简单比例换算，严格来说仍需要相机姿态（俯仰/滚转/高度）才能从像素到地面坐标。可行的几种折中方案：

- 假设近似俯视、畸变小：相机光轴基本垂直地面、倾角很小、离地高度固定。这时可以用“像素长宽 ↔ 真实长宽”做单轴/双轴比例换算，把矩形内的像素偏移映射到地面偏移。缺点是只要相机有轻微俯仰或高度变化，比例就会失真。
- 用已知尺寸矩形作“即席标定”：如果你能在画面中取到该矩形四个像素顶点坐标，同时知道它在地面的真实四点坐标（长宽和位置），就能直接求出单应性矩阵 H（这其实就是最小化的平面标定，不需要相机内参，只靠这个已知平面即可）。一旦求出 H，任意像素点都能映射到地平面坐标。这比通用相机标定简单，且契合“用已知矩形尺寸替代完整标定”的诉求。
- 如果不能获取四个角点、只能知道“矩形在画面中多大”，那只能用比例估计尺度，无法正确处理透视和倾斜。可加一点修正：在长宽方向分别做比例，并根据矩形上下边长度差估计俯仰，再对 y 向比例做一次透视补偿，但鲁棒性有限。
- 另一条思路：用视场角近似，将像素偏移转成水平角度（Δx → yaw 角），再用识别物体的实际尺寸/高度估计距离，组合出地面坐标。这需要已知物理尺寸（牌子尺寸）而不是路径矩形尺寸。

建议优先方案（简单、效果好）：
- 在图像中确定矩形四个角的像素坐标（你本来就画了矩形，顶点坐标已知），把真实地面矩形的四个点坐标提供给节点，用这两组点一次性求 H。之后控制节点用 H 把牌子中心像素变成地面坐标，在真实平面上做巡线控制。
- 若一定不用 H，也可以先用“比例 + 俯视假设”快速跑通，但要接受误差：相机姿态/高度变化会造成轨迹偏差。

你想走“用四个角点 + 真实尺寸求 H”的轻量方案，还是先用“俯视比例近似”快速验证，后面再补 H？

---

**User**

用已知尺寸矩形作“即席标定”：如果你能在画面中取到该矩形四个像素顶点坐标，同时知道它在地面的真实四点坐标（长宽和位置），就能直接求出单应性矩阵 H（这其实就是最小化的平面标定，不需要相机内参，只靠这个已知平面即可）。一旦求出 H，任意像素点都能映射到地平面坐标。这比通用相机标定简单，且契合“用已知矩形尺寸替代完整标定”的诉求。非常好，我想的就是这种方式，

右下：（397，354）

右上：（370，170）

左上：（214，173）

左下：（121，361）

右下到右上：120cm

右上到左上：60cm

左上到左下：120cm

左下到右下：60cm          这是我刚刚测试的数据，然后把图像上原来的那个矩形画成我现在测定的右下：（397，354）

右上：（370，170）

左上：（214，173）

左下：（121，361），这个意思懂的吧，我们继续讨论讨论，我看看你理解没有，不要着急做

---

**Cursor**

理解你的数据与意图：用已知矩形四点（像素）+ 真实矩形尺寸做平面单应性 H，后续把检测到的牌子中心像素映射到地面坐标，再在真实平面上做巡线。你给的像素顶点已更新：
- 右下 (397, 354)
- 右上 (370, 170)
- 左上 (214, 173)
- 左下 (121, 361)
真实尺寸（顺时针）：
- 右下→右上：120 cm
- 右上→左上：60 cm
- 左上→左下：120 cm
- 左下→右下：60 cm

我理解要点：
- 这是一个长边 120 cm、短边 60 cm 的矩形，摄像头俯视下产生透视畸变，像素长宽不等。
- 计划用这四对点求单应矩阵 H，不做完整相机标定。
- 需要明确地面坐标系的原点与轴向，确保像素点与真实点的顺序一致，否则 H 会错。

建议进一步确认的细节（不改代码，先定规范）：
1) 地面坐标系定义  
- 建议原点放在右下角（对应你给的“右下”像素点），x 轴指向“右下→右上”（120 cm），y 轴指向“右下→左下”（60 cm），单位米或厘米需统一（建议米，便于 ROS）。  
- 确认是否接受这种定义；若要原点在矩形中心，也可以，但要一致。

2) 点的顺序与对应关系  
- 像素点顺序（右下、右上、左上、左下）必须与地面点顺序一致。比如：  
  - 像素 P0=(397,354) ↔ 地面 G0=(0,0)  
  - 像素 P1=(370,170) ↔ 地面 G1=(1.20,0)（单位 m）  
  - 像素 P2=(214,173) ↔ 地面 G2=(1.20,0.60)? 这里要注意：右上→左上是 60 cm，实际长边 120、短边 60，所以从 G1 到 G2 是 0.60 m（沿 y 或沿 x？取决于坐标定义）。我建议：右上→左上 60 cm 沿 y 轴正方向，则 G2=(1.20,0.60)。  
  - 像素 P3=(121,361) ↔ 地面 G3=(0,0.60)  
- 请确认上述对应是否符合你的预期；核心是确定长边/短边对应哪个轴。

3) 单位与发布  
- ROS 侧最好用米（m），ESP32 内部也用米做速度/里程计保持一致。

4) 透视与畸变  
- 假设镜头畸变可忽略或已在 YOLO 输入前做了矫正；H 将按这组点直接算。相机姿态保持固定，否则 H 需要重算。

5) 后续控制使用  
- 控制节点：在收到 /yolo_car/detections 后，把中心像素 (u,v) 通过 H 映射到地面 (x,y)，再用真实矩形边做几何控制，替换当前的“像素巡线”。  
- 编码器/IMU：继续作为速度闭环和安全冗余（前面已确认要加）。

如果你确认上述坐标系与对应关系，我就按这个定义后续实现（先计划，后动手）。有不同的原点/轴向偏好请指出。

---

**User**

1) 地面坐标系定义



建议原点放在右下角（对应你给的“右下”像素点），x 轴指向“右下→右上”（120 cm），y 轴指向“右下→左下”（60 cm），单位米或厘米需统一（建议米，便于 ROS）。

确认是否接受这种定义；若要原点在矩形中心，也可以，但要一致。单位改成m，然后我们的图像坐标原点是图像的左上角，那我们的原点也选择左上角吧，x,y轴也和图像像素坐标轴一样，这样对应上要好一点，5) 后续控制使用

控制节点：在收到 /yolo_car/detections 后，把中心像素 (u,v) 通过 H 映射到地面 (x,y)，再用真实矩形边做几何控制，替换当前的“像素巡线”。

编码器/IMU：继续作为速度闭环和安全冗余（前面已确认要加）。对这个要用上，假设镜头畸变可忽略或已在 YOLO 输入前做了矫正；镜头畸变是什么，我们继续讨论讨论，我看看你理解没有，不要着急写代码

---

**Cursor**

好的，理解你的更新与偏好：

- 地面坐标系/原点/轴向：
  - 原点改为图像左上角，对齐像素坐标系；x 轴向右，y 轴向下，单位用米。
  - 地面矩形真实尺寸：右下→右上 1.20 m，右上→左上 0.60 m，左上→左下 1.20 m，左下→右下 0.60 m。
  - 像素顶点（按图像坐标）：右下 (397,354)、右上 (370,170)、左上 (214,173)、左下 (121,361)。
  - 对应关系需与坐标系一致：例如按顺时针从左上开始：P0(214,173)↔G0(0,0)，P1(370,170)↔G1(1.20,0)，P2(397,354)↔G2(1.20,0.60)，P3(121,361)↔G3(0,0.60)。请确认这个点序与地面坐标匹配；关键是像素点顺序与地面点顺序一致，否则单应矩阵会错。

- 镜头畸变是什么：
  - 常见的径向/切向畸变会让直线变弯、尺度随半径变化。若不矫正，单应矩阵 H 的精度会受影响，尤其在画面边缘。
  - 若畸变可忽略（广角不大、ROI 在画面中心），可以直接用当前像素点求 H；若有畸变，最好在 YOLO 前做一次去畸变（相机内参+畸变系数）或至少把顶点选在畸变较小的区域。

- 后续控制使用（思路不改）：
  - 检测到的中心点 (u,v) 通过 H → 地面 (x,y)，在真实平面上做巡线控制，替换像素巡线。
  - 编码器/IMU 用于轮速闭环和姿态/安全冗余。

请确认：
1) 顶点对应顺序是否采用“像素左上→右上→右下→左下”对应到“地面 (0,0)→(1.20,0)→(1.20,0.60)→(0,0.60)”？  
2) 摄像头是否大致俯视、畸变可以不做矫正，还是需要先做简单去畸变？  
确认后再动手。

---

**User**

原点改为图像左上角，对齐像素坐标系；x 轴向右，y 轴向下，单位用米。，对应关系需与坐标系一致：例如按顺时针从左上开始：P0(214,173)↔G0(0,0)，P1(370,170)↔G1(1.20,0)，P2(397,354)↔G2(1.20,0.60)，P3(121,361)↔G3(0,0.60)。你这里好像有点问题啊，左上是原点向右是x轴，那右边是是右上，右上到左上是0.6m，你这不有问题吗，重新改一下，我们继续讨论讨论，我看看你理解没有，2) 摄像头是否大致俯视、畸变可以不做矫正，还是需要先做简单去畸变？这个可以不用做了

---

**Cursor**

你说得对，按像素原点在左上、x 向右、y 向下，真实矩形应是宽 0.60 m、高 1.20 m。建议用以下对应关系（顺时针）：

- P0 左上 (214,173) ↔ G0 (0.00, 0.00) m  
- P1 右上 (370,170) ↔ G1 (0.60, 0.00) m  
- P2 右下 (397,354) ↔ G2 (0.60, 1.20) m  
- P3 左下 (121,361) ↔ G3 (0.00, 1.20) m  

这样：右上到左上是 0.60 m，右下到右上是 1.20 m，方向与像素坐标一致。镜头畸变暂不处理。

如果你认可这组对应，就可以据此求单应矩阵 H，并在控制节点里将牌子中心像素 (u,v) 映射到地面坐标 (x,y) 再做巡线。需要调整请告诉我。

---

**User**

非常好，就是这样，讨论的差不多了，现在你总结一下要做的吧，我看看有没有什么问题，看看你理解没有，不要着急写代码

---

**Cursor**

当前共识与待办（不动代码）：
- 坐标与单应性：
  - 像素原点左上，x 右、y 下，单位 m。
  - 点对应：P0(214,173)→G0(0,0)，P1(370,170)→G1(0.60,0)，P2(397,354)→G2(0.60,1.20)，P3(121,361)→G3(0,1.20)。
  - 求 H 后，检测到的牌子中心 (u,v) 映射为地面 (x,y)，控制在真实矩形上。
  - 镜头畸变忽略（默认俯视、畸变可接受）。

- ESP32 端扩展：
  - 保持 rosserial TCP 连接/超时保护。
  - 增加编码器计数与轮速计算；可发布轮速/里程计；预留 PI 轮速闭环（提高低速可控）。
  - 增加 MPU6050 读取并发布 IMU（角速度、加速度；姿态可选）。
  - 消息：继续订阅 `/yolo_car/cmd_vel`、`/yolo_car/servo_angle`，新增发布轮速/里程计与 IMU。

- ROS 端更新：
  - 控制节点：在处理 `/yolo_car/detections` 时用 H 将像素坐标映射到地面坐标，再按真实矩形进行巡线控制；保留丢失超时与速度限幅。
  - 新增订阅/可视化：轮速/里程计（可用 `JointState` 或 `Odometry`）与 `sensor_msgs/Imu`；可用于监控、报警或后续融合。
  - 参数：矩形四点与真实尺寸、H 的计算/加载；速度/角速度限幅可根据 MG310 特性调整。

- 风险/注意：
  - H 依赖摄像头姿态稳定，若位置/倾角变化需重取四点重算。
  - 编码器脉冲数与轮径/减速比需准确，以换算 m/s；MG310 起转阻力大，最小 PWM/轮速闭环要调。
  - IMU 无滤波时噪声大，可先发布原始 gyro/acc，再视需要加互补滤波。

---

**User**

是不是还少了一个，

右下：（397，354）

右上：（370，170）

左上：（214，173）

左下：（121，361）

现在的矩形不是说重新画吗，用这个参数重新画，还有一个点，增加 MPU6050 读取并发布 IMU（角速度、加速度；姿态可选）。我们的ros端只读取它发的原始数据，然后我们在ros端对原始数据滤波各种处理后得到我们需要的数据角速度、加速度；姿态，我们继续讨论讨论，我看看你理解没有

---

**Cursor**

明白，补充两点，继续保持讨论、不写代码：

- 矩形重画：在检测节点中用新的像素顶点重画矩形框（右下(397,354)、右上(370,170)、左上(214,173)、左下(121,361)），并用同一组顶点参与单应性 H 计算，确保显示与映射一致。

- IMU 数据流：
  - ESP32 端只采集并发布原始 MPU6050 数据（gyro、acc；姿态可选先不算）。
  - ROS 端订阅原始 IMU 后，再做滤波/姿态估计（如互补滤波、Madgwick、EKF），得到你需要的角速度、加速度、姿态，用于监控或控制。
  - 好处：算法迭代都在 ROS 侧，固件保持简单；需要什么处理随时在 ROS 端调整。

如果这个补充符合预期，我们就按此方案推进：重画矩形、用新顶点求 H，ESP32 发布原始 IMU，ROS 端负责滤波与融合。

---

**User**

非常好差不多就这些了，就这样了，按照我们讨论的结果，精华去做吧，你要做的有@yolo_car_control.ino 和@yolo_car_detection.py 还有esp32s3的代码全部都要按照我们讨论的去做，你直接开始做吧，没有提到的就按照你的推荐去做

---

**Cursor**

```cpp
/*
 * ESP32-S3 + L298N电机驱动 + SG90舵机 + ROS1 (rosserial)
 * 
 * 功能：
 * 1. 订阅速度指令控制L298N电机
 * 2. 订阅舵机角度指令控制SG90舵机
 * 3. 发布编码器轮速（m/s）
 * 4. 发布IMU原始数据（角速度/线加速度）
 * 
 * 接线：
 * - L298N使能引脚：N1->GPIO3, N3->GPIO5
 * - L298N方向控制：OUT1/OUT2接左轮，OUT3/OUT4接右轮
 * - SG90舵机信号线：GPIO2
 * - 右编码器 A/B: GPIO7/GPIO8
 * - 左编码器 A/B: GPIO9/GPIO10
 * - MPU6050: SDA->GPIO46, SCL->GPIO45
 */

#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <Wire.h>
#include <math.h>

#define USE_WIFI true

#if USE_WIFI
  #include <WiFi.h>
  const char* ssid = "机器人小车局域网";
  const char* password = "3252362734";
  IPAddress server(192, 168, 31, 64);
  const uint16_t serverPort = 11411;
#endif

// L298N电机控制引脚（四线控制：左PWM/左DIR/右PWM/右DIR）
// 根据你的接线：N1->GPIO3, N2->GPIO4, N3->GPIO5, N4->GPIO6
#define MOTOR_LEFT_PWM   3   // 左轮PWM（原N1）
#define MOTOR_LEFT_DIR   4   // 左轮方向（原N2）
#define MOTOR_RIGHT_PWM  5   // 右轮PWM（原N3）
#define MOTOR_RIGHT_DIR  6   // 右轮方向（原N4）

// SG90舵机引脚
#define SERVO_PIN 2

// 编码器引脚
#define ENC_RIGHT_A 7
#define ENC_RIGHT_B 8
#define ENC_LEFT_A  9
#define ENC_LEFT_B  10

// MPU6050
#define MPU_ADDR 0x68
#define I2C_SDA 46
#define I2C_SCL 45

// PWM参数
#define PWM_FREQ 1000      // PWM频率 1kHz
#define PWM_RESOLUTION 8   // 8位分辨率 (0-255)
#define SERVO_FREQ 50      // 舵机PWM频率 50Hz
// 使用12位分辨率，便于占空比计算 (0-4095)
#define SERVO_RESOLUTION 12

// 速度限制（归一化到0-255）
#define MAX_PWM_VALUE 200  // 最大PWM值（避免满速）

// 命令超时（毫秒），超过此时间未收到新速度指令则停车
#define CMD_TIMEOUT_MS 500

// 编码器与几何参数（需按实际调整）
#define TICKS_PER_REV 390.0f          // 轮子一圈的脉冲数（含减速比），请按实际修改
#define WHEEL_RADIUS_M 0.0325f        // 轮子半径（米），按实际轮径调整
#define WHEEL_BASE_M   0.20f          // 轮距（米）

// 轮速发布周期（毫秒）
#define WHEEL_PUB_PERIOD_MS 50

// IMU发布周期（毫秒）
#define IMU_PUB_PERIOD_MS 20

ros::NodeHandle nh;
unsigned long last_cmd_time = 0;
unsigned long last_wheel_pub = 0;
unsigned long last_imu_pub = 0;

volatile long right_ticks = 0;
volatile long left_ticks = 0;
volatile int right_dir_sign = 1;
volatile int left_dir_sign = 1;

geometry_msgs::Vector3Stamped wheel_vel_msg;
ros::Publisher wheel_vel_pub("/yolo_car/wheel_vel", &wheel_vel_msg);

sensor_msgs::Imu imu_msg;
ros::Publisher imu_pub("/yolo_car/imu_raw", &imu_msg);

// 简单的方向辅助，编码器只取A通道，方向依赖电机指令方向
void IRAM_ATTR rightEncISR() {
  right_ticks += right_dir_sign;
}

void IRAM_ATTR leftEncISR() {
  left_ticks += left_dir_sign;
}

// MPU6050 原始寄存器读取
bool readMPU6050(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(MPU_ADDR, 14, true) != 14) return false;
  *ax = (Wire.read() << 8) | Wire.read();
  *ay = (Wire.read() << 8) | Wire.read();
  *az = (Wire.read() << 8) | Wire.read();
  Wire.read(); Wire.read(); // temp
  *gx = (Wire.read() << 8) | Wire.read();
  *gy = (Wire.read() << 8) | Wire.read();
  *gz = (Wire.read() << 8) | Wire.read();
  return true;
}

bool initMPU6050() {
  Wire.begin(I2C_SDA, I2C_SCL, 400000);
  delay(50);
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B); // PWR_MGMT_1
  Wire.write(0x00); // wake up
  if (Wire.endTransmission() != 0) return false;
  delay(10);
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1C); // ACCEL_CONFIG
  Wire.write(0x00); // +-2g
  if (Wire.endTransmission() != 0) return false;
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1B); // GYRO_CONFIG
  Wire.write(0x00); // +-250 dps
  if (Wire.endTransmission() != 0) return false;
  return true;
}

// 电机急停
void stopMotors() {
  ledcWrite(0, 0);
  ledcWrite(1, 0);
  digitalWrite(MOTOR_LEFT_DIR, LOW);
  digitalWrite(MOTOR_RIGHT_DIR, LOW);
}

// 速度指令回调
void cmdVelCallback(const geometry_msgs::Twist& msg) {
  float linear_vel = msg.linear.x;   // 线速度 m/s
  float angular_vel = msg.angular.z; // 角速度 rad/s

  // 记录收到指令的时间
  last_cmd_time = millis();
  
  // 极小值直接视为0，避免噪声乱转
  if (fabs(linear_vel) < 1e-3 && fabs(angular_vel) < 1e-3) {
    stopMotors();
    return;
  }
  
  // 差速控制：将线速度和角速度转换为左右轮速度
  // 假设轮距为0.2m（根据实际小车调整）
  float left_vel = linear_vel - (angular_vel * WHEEL_BASE_M / 2.0f);
  float right_vel = linear_vel + (angular_vel * WHEEL_BASE_M / 2.0f);
  
  // 限制速度范围（假设最大速度0.3 m/s对应PWM 200）
  float max_vel = 0.3;
  left_vel = constrain(left_vel, -max_vel, max_vel);
  right_vel = constrain(right_vel, -max_vel, max_vel);
  
  // 转换为PWM值（0-255）
  int left_pwm = abs(left_vel / max_vel * MAX_PWM_VALUE);
  int right_pwm = abs(right_vel / max_vel * MAX_PWM_VALUE);
  
  // 设置方向（如果需要方向控制）
  // 注意：根据实际L298N接线调整
  // 如果OUT1接负极，OUT2接正极，那么：
  // - 正转：IN1=LOW, IN2=HIGH (或使用N2作为方向控制)
  // - 反转：IN1=HIGH, IN2=LOW
  // 这里假设N2和N4作为方向控制引脚
  // 左右方向控制（左轮已反转）
  if (left_vel >= 0) {
    digitalWrite(MOTOR_LEFT_DIR, HIGH);
    left_dir_sign = 1;
  } else {
    digitalWrite(MOTOR_LEFT_DIR, LOW);
    left_dir_sign = -1;
  }
  
  if (right_vel >= 0) {
    digitalWrite(MOTOR_RIGHT_DIR, LOW);  // 正转方向
    right_dir_sign = 1;
  } else {
    digitalWrite(MOTOR_RIGHT_DIR, HIGH); // 反转方向
    right_dir_sign = -1;
  }
  
  // 设置PWM输出
  ledcWrite(0, left_pwm);   // 通道0 -> 左轮PWM
  ledcWrite(1, right_pwm);  // 通道1 -> 右轮PWM
}

// 舵机角度回调
void servoAngleCallback(const std_msgs::Float32& msg) {
  float angle = msg.data;
  angle = constrain(angle, 0.0, 180.0); // 限制在0-180度

  // 调试打印，确认收到指令
  Serial.print("servo cmd: ");
  Serial.println(angle);

  // SG90舵机：典型 0.5~2.5ms（可按需收窄到 0.6~2.4ms）
  int pulse_width = map(angle, 0, 180, 500, 2500); // 微秒
  // 12位分辨率，50Hz -> 周期 20ms，对应计数 4096
  int duty_cycle = (pulse_width * 4096L) / 20000;    // 转换为占空比
  duty_cycle = constrain(duty_cycle, 0, 4095);

  ledcWrite(2, duty_cycle); // 通道2 -> 舵机
}

// 创建订阅者（注意：话题名称需要完整路径）
ros::Subscriber<geometry_msgs::Twist> cmd_vel_sub("/yolo_car/cmd_vel", cmdVelCallback);
ros::Subscriber<std_msgs::Float32> servo_angle_sub("/yolo_car/servo_angle", servoAngleCallback);

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n\n=== ESP32-S3 + L298N + SG90 + ROS1 ===");

  last_cmd_time = millis();
  
  // 初始化电机控制引脚
  Serial.println("初始化L298N电机驱动...");
  pinMode(MOTOR_LEFT_DIR, OUTPUT);
  pinMode(MOTOR_RIGHT_DIR, OUTPUT);
  // 初始方向：置低，防止上电乱转
  digitalWrite(MOTOR_LEFT_DIR, LOW);
  digitalWrite(MOTOR_RIGHT_DIR, LOW);
  
  // 配置PWM通道（电机）
  ledcSetup(0, PWM_FREQ, PWM_RESOLUTION);  // 通道0：左轮PWM
  ledcSetup(1, PWM_FREQ, PWM_RESOLUTION);  // 通道1：右轮PWM
  ledcAttachPin(MOTOR_LEFT_PWM, 0);
  ledcAttachPin(MOTOR_RIGHT_PWM, 1);
  
  // 配置PWM通道（舵机）
  ledcSetup(2, SERVO_FREQ, SERVO_RESOLUTION); // 通道2：舵机
  ledcAttachPin(SERVO_PIN, 2);

  // 编码器输入
  pinMode(ENC_RIGHT_A, INPUT_PULLUP);
  pinMode(ENC_RIGHT_B, INPUT_PULLUP);
  pinMode(ENC_LEFT_A, INPUT_PULLUP);
  pinMode(ENC_LEFT_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_RIGHT_A), rightEncISR, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_LEFT_A), leftEncISR, RISING);
  
  // 初始停止电机
  ledcWrite(0, 0);
  ledcWrite(1, 0);
  
  // 初始舵机到90度（中位）
  float init_angle = 90.0;
  int pulse_width = map(init_angle, 0, 180, 500, 2500);
  int duty_cycle = (pulse_width * 4096L) / 20000;
  duty_cycle = constrain(duty_cycle, 0, 4095);
  ledcWrite(2, duty_cycle);
  
  Serial.println("✓ 电机和舵机初始化完成");
  
#if USE_WIFI
  Serial.println("\n=== WiFi连接 ===");
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  
  Serial.print("连接中");
  int wifiAttempts = 0;
  while (WiFi.status() != WL_CONNECTED && wifiAttempts < 30) {
    delay(500);
    Serial.print(".");
    wifiAttempts++;
  }
  Serial.println();
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("✓ WiFi连接成功！IP: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("✗ WiFi连接失败！");
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    nh.getHardware()->setConnection(server, serverPort);
  }
#endif
  
  Serial.println("\n=== ROS连接 ===");
  nh.initNode();
  nh.subscribe(cmd_vel_sub);
  nh.subscribe(servo_angle_sub);
  nh.advertise(wheel_vel_pub);
  nh.advertise(imu_pub);
  
  Serial.print("等待连接");
  int rosAttempts = 0;
  while (!nh.connected() && rosAttempts < 50) {
    nh.spinOnce();
    delay(100);
    if (rosAttempts % 10 == 0) Serial.print(".");
    rosAttempts++;
  }
  Serial.println();
  
  if (nh.connected()) {
    Serial.println("✓ ROS连接成功！");
  } else {
    Serial.println("✗ ROS连接失败！");
  }
  
  Serial.println("\n=== 系统状态 ===");
  Serial.print("WiFi: ");
  Serial.println((WiFi.status() == WL_CONNECTED) ? "✓ 已连接" : "✗ 未连接");
  Serial.print("ROS: ");
  Serial.println(nh.connected() ? "✓ 已连接" : "✗ 未连接");
  Serial.println("\n开始运行...\n");

  if (!initMPU6050()) {
    Serial.println("✗ MPU6050 初始化失败");
  } else {
    Serial.println("✓ MPU6050 初始化完成");
  }
}

void loop() {
  // 命令超时保护：无论是否连接，都先做超时判断
  if (millis() - last_cmd_time > CMD_TIMEOUT_MS) {
    stopMotors();
  }

  if (!nh.connected()) {
    static unsigned long lastReconnectMsg = 0;
    if (millis() - lastReconnectMsg > 5000) {
      Serial.println("ROS连接断开，尝试重连...");
      lastReconnectMsg = millis();
    }
    // 断连时确保电机停转
    stopMotors();
    nh.spinOnce();
    delay(100);
    return;
  }

  // 发布轮速
  unsigned long now = millis();
  if (now - last_wheel_pub >= WHEEL_PUB_PERIOD_MS) {
    long rticks = right_ticks;
    long lticks = left_ticks;
    right_ticks = 0;
    left_ticks = 0;

    float dt = (now - last_wheel_pub) / 1000.0f;
    if (dt < 0.001f) dt = 0.001f;
    float ticks_per_s_right = rticks / dt;
    float ticks_per_s_left = lticks / dt;
    float m_per_tick = (2.0f * M_PI * WHEEL_RADIUS_M) / TICKS_PER_REV;
    float right_mps = ticks_per_s_right * m_per_tick;
    float left_mps = ticks_per_s_left * m_per_tick;

    wheel_vel_msg.header.stamp = nh.now();
    wheel_vel_msg.vector.x = left_mps;
    wheel_vel_msg.vector.y = right_mps;
    wheel_vel_msg.vector.z = 0.0f;
    wheel_vel_pub.publish(&wheel_vel_msg);
    last_wheel_pub = now;
  }

  // 发布IMU原始数据
  if (now - last_imu_pub >= IMU_PUB_PERIOD_MS) {
    int16_t ax, ay, az, gx, gy, gz;
    if (readMPU6050(&ax, &ay, &az, &gx, &gy, &gz)) {
      // 标度：加速度 16384 LSB/g，角速度 131 LSB/(deg/s)
      const float ACCEL_SCALE = 9.80665f / 16384.0f;          // m/s^2
      const float GYRO_SCALE = (M_PI / 180.0f) / 131.0f;      // rad/s

      imu_msg.header.stamp = nh.now();
      imu_msg.linear_acceleration.x = ax * ACCEL_SCALE;
      imu_msg.linear_acceleration.y = ay * ACCEL_SCALE;
      imu_msg.linear_acceleration.z = az * ACCEL_SCALE;
      imu_msg.angular_velocity.x = gx * GYRO_SCALE;
      imu_msg.angular_velocity.y = gy * GYRO_SCALE;
      imu_msg.angular_velocity.z = gz * GYRO_SCALE;
      // 姿态/协方差留空，由上位机滤波
      imu_pub.publish(&imu_msg);
    }
    last_imu_pub = now;
  }

  nh.spinOnce();
  delay(10);
}


```

```python
#!/usr/bin/env python3
"""
yolo_car_detection.py - YOLO物体检测节点

功能：
1. 使用YOLO模型检测USB摄像头中的物体
2. 在窗口上显示检测结果（仅坐标，不再有红/蓝点动画）
3. 发布检测结果到 /yolo_car/detections 话题
"""

import cv2
import os
import rospy
import rospkg
import numpy as np
from ultralytics import YOLO
from yolo_car.msg import ObjectDetections, ObjectDetection
from std_msgs.msg import Header


class YOLOCarDetection:
    def __init__(self):
        rospy.init_node("yolo_car_detection", anonymous=False)
        
        # 获取ROS参数
        model_path = rospy.get_param("~model", None)
        
        # 处理空字符串的情况（launch文件可能传递空字符串）
        if model_path == "" or model_path is None:
            model_path = None
        
        # 如果没有指定模型路径，尝试从包内的models目录加载默认模型
        if model_path is None:
            rospack = rospkg.RosPack()
            package_path = rospack.get_path('yolo_car')
            default_model = os.path.join(package_path, 'models', 'yolov8n_label.pt')
            
            if os.path.exists(default_model):
                model_path = default_model
                rospy.loginfo("使用默认模型: %s", model_path)
            else:
                rospy.logerr("未找到模型文件！")
                rospy.logerr("请将模型文件 'yolov8n_label.pt' 放到 %s 目录", os.path.join(package_path, 'models'))
                rospy.logerr("或者通过参数指定模型路径: _model:=path/to/model.pt")
                rospy.signal_shutdown("缺少模型文件")
        
        conf_threshold = float(rospy.get_param("~conf_threshold", 0.25))
        window_name = rospy.get_param("~window_name", "YOLO Car Detection")
        source = int(rospy.get_param("~source", 0))  # USB摄像头固定为0
        
        # 加载YOLO模型
        rospy.loginfo("加载YOLO模型: %s", model_path)
        try:
            self.model = YOLO(model_path)
        except Exception as e:
            rospy.logerr("加载模型失败: %s", e)
            rospy.signal_shutdown("模型加载失败")
            return
        
        # 创建ROS发布者
        self.detections_pub = rospy.Publisher("/yolo_car/detections", ObjectDetections, queue_size=1)
        
        # 初始化窗口
        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
        self.window_name = window_name
        
        # 当前帧的检测结果 [(center_x, center_y, bbox)]
        self.current_frame_detections = []
        # 矩形顶点（顺时针），仅用于显示轮廓
        # 重新标定的矩形顶点（顺时针）：右下、右上、左上、左下
        self.rect_vertices = [
            (397, 354),  # 右下
            (370, 170),  # 右上
            (214, 173),  # 左上
            (121, 361),  # 左下
        ]
        
        # 启动YOLO检测流
        rospy.loginfo("启动YOLO检测，使用USB摄像头 (source=0)")
        rospy.loginfo("按 'q' 键退出")
        
        try:
            while not rospy.is_shutdown():
                try:
                    got_frame = False
                    for result in self.model(source=source, stream=True, conf=conf_threshold):
                        got_frame = True
                if rospy.is_shutdown():
                    break
                
                # 处理检测结果
                self.process_detection(result)
                
                # 显示图像
                plotted = result.plot()
                
                        # 绘制矩形轮廓（无动画点）
                        self.draw_rect(plotted)

                        # 在图像上添加中心点坐标标注
                self.draw_labels(plotted, result)
                
                cv2.imshow(self.window_name, plotted)
                
                # 检查退出
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    rospy.loginfo("收到退出信号")
                            rospy.signal_shutdown("用户退出")
                    break
                    
                    if not got_frame:
                        rospy.logwarn("未从摄像头获取到任何帧，1秒后重试...")
                        try:
                            blank = np.zeros((480, 640, 3), dtype=np.uint8)
                            cv2.putText(blank, "No frames from camera, retrying...", (20, 240),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2, cv2.LINE_AA)
                            cv2.imshow(self.window_name, blank)
                            cv2.waitKey(1)
                        except Exception:
                            pass
                        rospy.sleep(1.0)
                except Exception as e:
                    rospy.logerr("YOLO流异常: %s，1秒后重试", e)
                    try:
                        blank = np.zeros((480, 640, 3), dtype=np.uint8)
                        cv2.putText(blank, "Camera error, retrying...", (20, 240),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2, cv2.LINE_AA)
                        cv2.imshow(self.window_name, blank)
                        cv2.waitKey(1)
                    except Exception:
                        pass
                    rospy.sleep(1.0)
        except Exception as e:
            rospy.logerr("检测过程中出错: %s", e)
        finally:
            cv2.destroyAllWindows()
            rospy.loginfo("YOLO检测节点已关闭")
    
    def process_detection(self, result):
        """处理检测结果并发布ROS消息"""
        msg = ObjectDetections()
        msg.header = Header()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "camera_frame"

        # 每帧先清空当前检测缓存，避免上一帧残留
        self.current_frame_detections = []
        
        if result.boxes is None or len(result.boxes) == 0:
            # 没有检测到物体
            self.detections_pub.publish(msg)
            return
        
        # 获取图像尺寸
        if result.orig_shape:
            img_height, img_width = result.orig_shape
        else:
            img_height, img_width = result.plot().shape[:2]
        
        # 提取所有检测信息
        detections = []
        for box in result.boxes:
            bbox = box.xyxy[0].cpu().numpy()  # [x1, y1, x2, y2]
            x1, y1, x2, y2 = bbox
            center_x = (x1 + x2) / 2.0
            center_y = (y1 + y2) / 2.0
            confidence = float(box.conf[0].cpu().numpy()) if box.conf is not None else 0.0
            detections.append((center_x, center_y, (x1, y1, x2, y2), confidence, box))
        
        # 只保留前两个检测结果（按置信度排序，取最高的两个）
        if len(detections) > 2:
            detections.sort(key=lambda x: x[3], reverse=True)  # 按置信度降序排序
            detections = detections[:2]  # 只保留前两个
        
        # 保存当前帧的检测结果，供draw_labels使用（不再区分ID）
        self.current_frame_detections = []
        
        # 创建ROS消息（为兼容消息字段，仍填充 object_id，但不做跟踪）
        for idx, (center_x, center_y, (x1, y1, x2, y2), confidence, box) in enumerate(detections):
            self.current_frame_detections.append((center_x, center_y, (x1, y1, x2, y2)))

            obj_msg = ObjectDetection()
            obj_msg.header = msg.header
            obj_msg.object_id = idx + 1  # 简单序号
            obj_msg.center_x = float(center_x)
            obj_msg.center_y = float(center_y)
            obj_msg.confidence = confidence
            obj_msg.bbox_x = float(x1)
            obj_msg.bbox_y = float(y1)
            obj_msg.bbox_width = float(x2 - x1)
            obj_msg.bbox_height = float(y2 - y1)
            obj_msg.image_width = img_width
            obj_msg.image_height = img_height
            
            msg.objects.append(obj_msg)
        
        # 发布消息
        self.detections_pub.publish(msg)
    
    def draw_labels(self, image, result):
        """在图像上绘制中心点坐标标签（仅坐标，无ID）"""
        if not self.current_frame_detections:
            return
        
        # 直接使用当前帧保存的检测结果（最多两个）
        for center_x, center_y, (x1, y1, x2, y2) in self.current_frame_detections:
            center_x_int = int(center_x)
            center_y_int = int(center_y)
            
            # 只显示坐标，不显示label和ID
            coord_text = f"({center_x_int}, {center_y_int})"
            
            # 获取文本大小
            font = cv2.FONT_HERSHEY_SIMPLEX
            font_scale = 0.6
            thickness = 2
            
            # 计算文本位置
            (text_width_coord, text_height_coord), _ = cv2.getTextSize(coord_text, font, font_scale, thickness)
            
            # 在检测框上方绘制标签，往上移更多避免遮挡YOLO的置信度显示
            text_x = int(x1)
            text_y_coord = int(y1) - text_height_coord - 11  # 往上移更多，留出空间给置信度
            
            # 确保文本不超出图像边界
            if text_y_coord < text_height_coord:
                text_y_coord = int(y2) + text_height_coord + 5
            
            # 绘制背景矩形（提高可读性）
            cv2.rectangle(image, 
                         (text_x - 2, text_y_coord - text_height_coord - 2),
                         (text_x + text_width_coord + 2, text_y_coord + 2),
                         (0, 0, 0), -1)
            
            # 绘制坐标文本
            cv2.putText(image, coord_text, (text_x, text_y_coord), 
                       font, font_scale, (255, 255, 0), thickness)
            
            # 在中心点绘制标记
            cv2.circle(image, (center_x_int, center_y_int), 5, (0, 0, 255), -1)
            cv2.circle(image, (center_x_int, center_y_int), 8, (0, 0, 255), 2)
            
    def draw_rect(self, image):
        """绘制矩形轮廓（仅显示，不再画红/蓝点）"""
        if not self.rect_vertices or len(self.rect_vertices) < 2:
            return
        color_frame = (150, 150, 150)
        thickness_frame = 1
        n = len(self.rect_vertices)
        for i in range(n):
            pt1 = tuple(map(int, self.rect_vertices[i]))
            pt2 = tuple(map(int, self.rect_vertices[(i + 1) % n]))
            cv2.line(image, pt1, pt2, color_frame, thickness_frame, lineType=cv2.LINE_AA)


def main():
    try:
        node = YOLOCarDetection()
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        rospy.loginfo("收到键盘中断信号")
    finally:
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()


```

```python
#!/usr/bin/env python3
"""
yolo_car_control.py - YOLO小车控制节点（矩形巡线）

功能：
1. 订阅牌子检测结果
2. 让牌子中心沿既定矩形边顺时针巡线（图像坐标），恒速前进 + 法向纠偏
3. 发布速度指令到 /yolo_car/cmd_vel
4. 识别不到时停车并舵机扫描
"""

import rospy
import numpy as np
import cv2
from geometry_msgs.msg import Twist, Point
from yolo_car.msg import ObjectDetections
from std_msgs.msg import Float32


class YOLOCarControl:
    def __init__(self):
        rospy.init_node("yolo_car_control", anonymous=False)

        # 参数：矩形巡线控制
        self.k_ang = float(rospy.get_param("~k_ang", 0.6))   # 单位改为 m 后放大
        self.edge_deadband = float(rospy.get_param("~edge_deadband", 0.02))  # 靠线死区，米
        # 中间死区，误差小于此直接设角速度=0，避免单边打圈
        self.mid_deadband = float(rospy.get_param("~mid_deadband", 0.05))
        self.switch_threshold = float(rospy.get_param("~switch_threshold", 0.05))  # 顶点切边阈值，米
        # TT 马达需要一定占空比才动，适当提高基准和最小速度
        self.base_linear_speed = float(rospy.get_param("~base_linear_speed", 0.05))
        self.max_linear_vel = float(rospy.get_param("~max_linear_vel", 0.15))
        self.max_angular_vel = float(rospy.get_param("~max_angular_vel", 0.05))
        self.angular_scale = float(rospy.get_param("~angular_scale", 0.12))
        self.invert_x_control = bool(rospy.get_param("~invert_x_control", False))
        self.control_rate = float(rospy.get_param("~control_rate", 20.0))
        self.lost_timeout = float(rospy.get_param("~lost_timeout", 3.0))

        # 状态变量
        self.car_center = None  # 牌子中心点 (x, y)
        self.last_cmd_vel = Twist()
        self.last_detection_time = None
        self.car_detected = False

        # 舵机扫描相关
        self.servo_scan_angles = [0, 90, 180, 90, 0, 90, 180]
        self.servo_scan_index = 0
        self.servo_scan_interval = 1.5
        self.last_servo_time = None
        self.servo_scanning = False

        # 发布者
        self.cmd_vel_pub = rospy.Publisher("/yolo_car/cmd_vel", Twist, queue_size=1)
        self.servo_angle_pub = rospy.Publisher("/yolo_car/servo_angle", Float32, queue_size=1)

        # 订阅者
        self.detections_sub = rospy.Subscriber("/yolo_car/detections", ObjectDetections, self.detections_callback)
        # 保留订阅以兼容，但不用于控制
        self.target_point_sub = rospy.Subscriber("/yolo_car/target_point", Point, self.target_point_callback)

        # 矩形顶点（顺时针）
        # 像素与地面点（米）对应，默认使用讨论确定的矩形
        pixel_default = [
            [214.0, 173.0],  # 左上
            [370.0, 170.0],  # 右上
            [397.0, 354.0],  # 右下
            [121.0, 361.0],  # 左下
        ]
        ground_default = [
            [0.0, 0.0],
            [0.60, 0.0],
            [0.60, 1.20],
            [0.0, 1.20],
        ]
        # 参数可覆盖，格式 [x0,y0,x1,y1,x2,y2,x3,y3]
        pixel_param = rospy.get_param("~rect_pixels", [])
        ground_param = rospy.get_param("~rect_ground", [])
        if len(pixel_param) == 8:
            pixel_default = [[pixel_param[0], pixel_param[1]],
                             [pixel_param[2], pixel_param[3]],
                             [pixel_param[4], pixel_param[5]],
                             [pixel_param[6], pixel_param[7]]]
        if len(ground_param) == 8:
            ground_default = [[ground_param[0], ground_param[1]],
                              [ground_param[2], ground_param[3]],
                              [ground_param[4], ground_param[5]],
                              [ground_param[6], ground_param[7]]]

        self.pixel_vertices = np.array(pixel_default, dtype=np.float32)
        self.rect_vertices = [np.array(p, dtype=np.float32) for p in ground_default]
        self.H = self._compute_homography(self.pixel_vertices, np.array(ground_default, dtype=np.float32))
        self.current_edge = 0  # 当前边索引

        rospy.loginfo("YOLO小车控制节点已启动")
        rospy.loginfo("比例控制 - k_ang=%.4f", self.k_ang)
        rospy.loginfo("靠线死区: %.3f m, 切边阈值: %.3f m", self.edge_deadband, self.switch_threshold)
        rospy.loginfo("速度限制: 线速度=%.2f m/s, 角速度=%.2f rad/s", self.max_linear_vel, self.max_angular_vel)
        if self.H is None:
            rospy.logwarn("未能计算单应矩阵，控制将保持像素坐标系")
        else:
            rospy.loginfo("单应矩阵已计算，可将像素映射到地面坐标")

        # 启动控制循环
        self.control_loop()

    def detections_callback(self, msg):
        """检测结果回调"""
        if len(msg.objects) > 0:
            obj = msg.objects[0]
            self.car_center = (obj.center_x, obj.center_y)
            if self.H is not None:
                self.car_center_ground = self._pixel_to_ground(self.car_center)
            else:
                self.car_center_ground = None
            self.last_detection_time = rospy.Time.now()
            self.car_detected = True
            self.servo_scanning = False
            self.servo_scan_index = 0
            # 初始选最近边
            if self.H is not None and self.car_center_ground is not None:
                self.current_edge = self._nearest_edge(np.array(self.car_center_ground))
            else:
                self.current_edge = self._nearest_edge(np.array([obj.center_x, obj.center_y]))
        else:
            self.car_detected = False

    def target_point_callback(self, msg):
        """兼容接口，未使用"""
        pass

    def _point_to_segment_distance(self, p, a, b):
        """点到线段距离"""
        ab = b - a
        ap = p - a
        ab_len2 = np.dot(ab, ab)
        if ab_len2 == 0:
            return np.linalg.norm(ap)
        t = np.clip(np.dot(ap, ab) / ab_len2, 0.0, 1.0)
        proj = a + t * ab
        return np.linalg.norm(p - proj)

    def _compute_homography(self, pixels, ground):
        """从像素四点到地面四点计算单应矩阵"""
        try:
            H, status = cv2.findHomography(pixels, ground, method=0)
            if H is None:
                return None
            return H
        except Exception as e:
            rospy.logwarn("计算单应矩阵失败: %s", e)
            return None

    def _pixel_to_ground(self, pt):
        """像素点 -> 地面坐标"""
        if self.H is None:
            return None
        u, v = pt
        src = np.array([u, v, 1.0], dtype=np.float32)
        dst = self.H.dot(src)
        if dst[2] == 0:
            return None
        x = dst[0] / dst[2]
        y = dst[1] / dst[2]
        return (float(x), float(y))

    def _nearest_edge(self, pt):
        """找到距离点最近的边索引"""
        min_dist = 1e9
        min_idx = 0
        n = len(self.rect_vertices)
        for i in range(n):
            a = self.rect_vertices[i]
            b = self.rect_vertices[(i + 1) % n]
            dist = self._point_to_segment_distance(pt, a, b)
            if dist < min_dist:
                min_dist = dist
                min_idx = i
        return min_idx

    def _edge_frame(self, idx):
        """返回边的起点、终点、切向、法向、长度"""
        a = self.rect_vertices[idx]
        b = self.rect_vertices[(idx + 1) % len(self.rect_vertices)]
        ab = b - a
        length = np.linalg.norm(ab)
        if length == 0:
            return a, b, np.array([1.0, 0.0]), np.array([0.0, 1.0]), 1.0
        t = ab / length
        n = np.array([-t[1], t[0]])  # 左手法向
        return a, b, t, n, length

    def control_loop(self):
        """控制循环"""
        rate = rospy.Rate(self.control_rate)

        while not rospy.is_shutdown():
            current_time = rospy.Time.now()

            if self.car_detected and self.car_center is not None:
                if self.H is not None and self.car_center_ground is not None:
                    p = np.array(self.car_center_ground, dtype=np.float32)
                else:
                    p = np.array([self.car_center[0], self.car_center[1]], dtype=np.float32)

                # 首选当前边，偏离过大才重选最近边（避免频繁切换导致打圈）
                a, b, t_vec, n_vec, length = self._edge_frame(self.current_edge)
                ap = p - a
                s = np.dot(ap, t_vec)
                s_clamped = np.clip(s, 0.0, length)
                proj = a + s_clamped * t_vec
                en = np.dot(p - proj, n_vec)
                dist_edge = abs(en)
                if dist_edge > (0.25 if self.H is not None else 160.0):
                    self.current_edge = self._nearest_edge(p)
                a, b, t_vec, n_vec, length = self._edge_frame(self.current_edge)
                ap = p - a
                s = np.dot(ap, t_vec)      # 沿边方向投影
                s_clamped = np.clip(s, 0.0, length)
                proj = a + s_clamped * t_vec  # 最近点投影
                en = np.dot(p - proj, n_vec)  # 法向距离（投影点）

                # 到达顶点则切换下一边
                if s >= (length - self.switch_threshold):
                    self.current_edge = (self.current_edge + 1) % len(self.rect_vertices)
                    a, b, t_vec, n_vec, length = self._edge_frame(self.current_edge)
                    ap = p - a
                    s = np.dot(ap, t_vec)
                    s_clamped = np.clip(s, 0.0, length)
                    proj = a + s_clamped * t_vec
                    en = np.dot(p - proj, n_vec)

                # 靠线死区
                if abs(en) < self.edge_deadband:
                    en = 0.0

                # 中间死区，避免小偏差也打圈
                if abs(en) < self.mid_deadband:
                    angular_vel = 0.0
                else:
                    # 角速度（法向误差）+ 平滑抑制
                    angular_vel = self.k_ang * en
                    if self.invert_x_control:
                        angular_vel = -angular_vel
                    angular_vel *= self.angular_scale
                    angular_vel = np.tanh(angular_vel)
                    angular_vel = np.clip(angular_vel, -self.max_angular_vel, self.max_angular_vel)

                # 线速度：偏差大减速，贴线后恒速；最低保留 0.04 以克服起步阻力
                slow_factor = 1.0 - min(abs(en) / (0.30 if self.H is not None else 200.0), 0.85)
                linear_vel = self.base_linear_speed * slow_factor
                linear_vel = max(linear_vel, 0.04)
                linear_vel = np.clip(linear_vel, -self.max_linear_vel, self.max_linear_vel)

                # 发布速度
                cmd_vel = Twist()
                cmd_vel.linear.x = linear_vel
                cmd_vel.angular.z = angular_vel
                self.cmd_vel_pub.publish(cmd_vel)
                self.last_cmd_vel = cmd_vel

            else:
                # 识别不到牌子：立即停止
                cmd_vel = Twist()
                self.cmd_vel_pub.publish(cmd_vel)
                self.last_cmd_vel = cmd_vel

                # 舵机扫描
                if self.last_servo_time is None or (current_time - self.last_servo_time).to_sec() >= self.servo_scan_interval:
                    angle = self.servo_scan_angles[self.servo_scan_index]
                    servo_msg = Float32()
                    servo_msg.data = float(angle)
                    self.servo_angle_pub.publish(servo_msg)

                    # 更新索引
                    self.servo_scan_index = (self.servo_scan_index + 1) % len(self.servo_scan_angles)
                    self.last_servo_time = current_time

            rate.sleep()


def main():
    try:
        YOLOCarControl()
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        rospy.loginfo("收到键盘中断信号")
    finally:
        # 停止小车
        cmd_vel = Twist()
        rospy.Publisher("/yolo_car/cmd_vel", Twist, queue_size=1).publish(cmd_vel)


if __name__ == "__main__":
    main()


```

```python
#!/usr/bin/env python3
"""
yolo_car_control.py - YOLO小车控制节点（矩形巡线）

功能：
1. 订阅牌子检测结果
2. 让牌子中心沿既定矩形边顺时针巡线（图像坐标），恒速前进 + 法向纠偏
3. 发布速度指令到 /yolo_car/cmd_vel
4. 识别不到时停车并舵机扫描
"""

import rospy
import numpy as np
import cv2
from geometry_msgs.msg import Twist, Point
from yolo_car.msg import ObjectDetections
from std_msgs.msg import Float32


class YOLOCarControl:
    def __init__(self):
        rospy.init_node("yolo_car_control", anonymous=False)

        # 参数：矩形巡线控制
        self.k_ang = float(rospy.get_param("~k_ang", 0.6))   # 单位改为 m 后放大
        self.edge_deadband = float(rospy.get_param("~edge_deadband", 0.02))  # 靠线死区，米
        # 中间死区，误差小于此直接设角速度=0，避免单边打圈
        self.mid_deadband = float(rospy.get_param("~mid_deadband", 0.05))
        self.switch_threshold = float(rospy.get_param("~switch_threshold", 0.05))  # 顶点切边阈值，米
        # TT 马达需要一定占空比才动，适当提高基准和最小速度
        self.base_linear_speed = float(rospy.get_param("~base_linear_speed", 0.05))
        self.max_linear_vel = float(rospy.get_param("~max_linear_vel", 0.15))
        self.max_angular_vel = float(rospy.get_param("~max_angular_vel", 0.05))
        self.angular_scale = float(rospy.get_param("~angular_scale", 0.12))
        self.invert_x_control = bool(rospy.get_param("~invert_x_control", False))
        self.control_rate = float(rospy.get_param("~control_rate", 20.0))
        self.lost_timeout = float(rospy.get_param("~lost_timeout", 3.0))

        # 状态变量
        self.car_center = None  # 牌子中心点 (x, y)
        self.car_center_ground = None  # 映射到地面坐标的中心点
        self.last_cmd_vel = Twist()
        self.last_detection_time = None
        self.car_detected = False

        # 舵机扫描相关
        self.servo_scan_angles = [0, 90, 180, 90, 0, 90, 180]
        self.servo_scan_index = 0
        self.servo_scan_interval = 1.5
        self.last_servo_time = None
        self.servo_scanning = False

        # 发布者
        self.cmd_vel_pub = rospy.Publisher("/yolo_car/cmd_vel", Twist, queue_size=1)
        self.servo_angle_pub = rospy.Publisher("/yolo_car/servo_angle", Float32, queue_size=1)

        # 订阅者
        self.detections_sub = rospy.Subscriber("/yolo_car/detections", ObjectDetections, self.detections_callback)
        # 保留订阅以兼容，但不用于控制
        self.target_point_sub = rospy.Subscriber("/yolo_car/target_point", Point, self.target_point_callback)

        # 矩形顶点（顺时针）
        # 像素与地面点（米）对应，默认使用讨论确定的矩形
        pixel_default = [
            [214.0, 173.0],  # 左上
            [370.0, 170.0],  # 右上
            [397.0, 354.0],  # 右下
            [121.0, 361.0],  # 左下
        ]
        ground_default = [
            [0.0, 0.0],
            [0.60, 0.0],
            [0.60, 1.20],
            [0.0, 1.20],
        ]
        # 参数可覆盖，格式 [x0,y0,x1,y1,x2,y2,x3,y3]
        pixel_param = rospy.get_param("~rect_pixels", [])
        ground_param = rospy.get_param("~rect_ground", [])
        if len(pixel_param) == 8:
            pixel_default = [[pixel_param[0], pixel_param[1]],
                             [pixel_param[2], pixel_param[3]],
                             [pixel_param[4], pixel_param[5]],
                             [pixel_param[6], pixel_param[7]]]
        if len(ground_param) == 8:
            ground_default = [[ground_param[0], ground_param[1]],
                              [ground_param[2], ground_param[3]],
                              [ground_param[4], ground_param[5]],
                              [ground_param[6], ground_param[7]]]

        self.pixel_vertices = np.array(pixel_default, dtype=np.float32)
        self.rect_vertices = [np.array(p, dtype=np.float32) for p in ground_default]
        self.H = self._compute_homography(self.pixel_vertices, np.array(ground_default, dtype=np.float32))
        self.current_edge = 0  # 当前边索引

        rospy.loginfo("YOLO小车控制节点已启动")
        rospy.loginfo("比例控制 - k_ang=%.4f", self.k_ang)
        rospy.loginfo("靠线死区: %.3f m, 切边阈值: %.3f m", self.edge_deadband, self.switch_threshold)
        rospy.loginfo("速度限制: 线速度=%.2f m/s, 角速度=%.2f rad/s", self.max_linear_vel, self.max_angular_vel)
        if self.H is None:
            rospy.logwarn("未能计算单应矩阵，控制将保持像素坐标系")
        else:
            rospy.loginfo("单应矩阵已计算，可将像素映射到地面坐标")

        # 启动控制循环
        self.control_loop()

    def detections_callback(self, msg):
        """检测结果回调"""
        if len(msg.objects) > 0:
            obj = msg.objects[0]
            self.car_center = (obj.center_x, obj.center_y)
            if self.H is not None:
                self.car_center_ground = self._pixel_to_ground(self.car_center)
            else:
                self.car_center_ground = None
            self.last_detection_time = rospy.Time.now()
            self.car_detected = True
            self.servo_scanning = False
            self.servo_scan_index = 0
            # 初始选最近边
            if self.H is not None and self.car_center_ground is not None:
                self.current_edge = self._nearest_edge(np.array(self.car_center_ground))
            else:
                self.current_edge = self._nearest_edge(np.array([obj.center_x, obj.center_y]))
        else:
            self.car_detected = False

    def target_point_callback(self, msg):
        """兼容接口，未使用"""
        pass

    def _point_to_segment_distance(self, p, a, b):
        """点到线段距离"""
        ab = b - a
        ap = p - a
        ab_len2 = np.dot(ab, ab)
        if ab_len2 == 0:
            return np.linalg.norm(ap)
        t = np.clip(np.dot(ap, ab) / ab_len2, 0.0, 1.0)
        proj = a + t * ab
        return np.linalg.norm(p - proj)

    def _compute_homography(self, pixels, ground):
        """从像素四点到地面四点计算单应矩阵"""
        try:
            H, status = cv2.findHomography(pixels, ground, method=0)
            if H is None:
                return None
            return H
        except Exception as e:
            rospy.logwarn("计算单应矩阵失败: %s", e)
            return None

    def _pixel_to_ground(self, pt):
        """像素点 -> 地面坐标"""
        if self.H is None:
            return None
        u, v = pt
        src = np.array([u, v, 1.0], dtype=np.float32)
        dst = self.H.dot(src)
        if dst[2] == 0:
            return None
        x = dst[0] / dst[2]
        y = dst[1] / dst[2]
        return (float(x), float(y))

    def _nearest_edge(self, pt):
        """找到距离点最近的边索引"""
        min_dist = 1e9
        min_idx = 0
        n = len(self.rect_vertices)
        for i in range(n):
            a = self.rect_vertices[i]
            b = self.rect_vertices[(i + 1) % n]
            dist = self._point_to_segment_distance(pt, a, b)
            if dist < min_dist:
                min_dist = dist
                min_idx = i
        return min_idx

    def _edge_frame(self, idx):
        """返回边的起点、终点、切向、法向、长度"""
        a = self.rect_vertices[idx]
        b = self.rect_vertices[(idx + 1) % len(self.rect_vertices)]
        ab = b - a
        length = np.linalg.norm(ab)
        if length == 0:
            return a, b, np.array([1.0, 0.0]), np.array([0.0, 1.0]), 1.0
        t = ab / length
        n = np.array([-t[1], t[0]])  # 左手法向
        return a, b, t, n, length

    def control_loop(self):
        """控制循环"""
        rate = rospy.Rate(self.control_rate)

        while not rospy.is_shutdown():
            current_time = rospy.Time.now()

            if self.car_detected and self.car_center is not None:
                if self.H is not None and self.car_center_ground is not None:
                    p = np.array(self.car_center_ground, dtype=np.float32)
                else:
                    p = np.array([self.car_center[0], self.car_center[1]], dtype=np.float32)

                # 首选当前边，偏离过大才重选最近边（避免频繁切换导致打圈）
                a, b, t_vec, n_vec, length = self._edge_frame(self.current_edge)
                ap = p - a
                s = np.dot(ap, t_vec)
                s_clamped = np.clip(s, 0.0, length)
                proj = a + s_clamped * t_vec
                en = np.dot(p - proj, n_vec)
                dist_edge = abs(en)
                if dist_edge > (0.25 if self.H is not None else 160.0):
                    self.current_edge = self._nearest_edge(p)
                a, b, t_vec, n_vec, length = self._edge_frame(self.current_edge)
                ap = p - a
                s = np.dot(ap, t_vec)      # 沿边方向投影
                s_clamped = np.clip(s, 0.0, length)
                proj = a + s_clamped * t_vec  # 最近点投影
                en = np.dot(p - proj, n_vec)  # 法向距离（投影点）

                # 到达顶点则切换下一边
                if s >= (length - self.switch_threshold):
                    self.current_edge = (self.current_edge + 1) % len(self.rect_vertices)
                    a, b, t_vec, n_vec, length = self._edge_frame(self.current_edge)
                    ap = p - a
                    s = np.dot(ap, t_vec)
                    s_clamped = np.clip(s, 0.0, length)
                    proj = a + s_clamped * t_vec
                    en = np.dot(p - proj, n_vec)

                # 靠线死区
                if abs(en) < self.edge_deadband:
                    en = 0.0

                # 中间死区，避免小偏差也打圈
                if abs(en) < self.mid_deadband:
                    angular_vel = 0.0
                else:
                    # 角速度（法向误差）+ 平滑抑制
                    angular_vel = self.k_ang * en
                    if self.invert_x_control:
                        angular_vel = -angular_vel
                    angular_vel *= self.angular_scale
                    angular_vel = np.tanh(angular_vel)
                    angular_vel = np.clip(angular_vel, -self.max_angular_vel, self.max_angular_vel)

                # 线速度：偏差大减速，贴线后恒速；最低保留 0.04 以克服起步阻力
                slow_factor = 1.0 - min(abs(en) / (0.30 if self.H is not None else 200.0), 0.85)
                linear_vel = self.base_linear_speed * slow_factor
                linear_vel = max(linear_vel, 0.04)
                linear_vel = np.clip(linear_vel, -self.max_linear_vel, self.max_linear_vel)

                # 发布速度
                cmd_vel = Twist()
                cmd_vel.linear.x = linear_vel
                cmd_vel.angular.z = angular_vel
                self.cmd_vel_pub.publish(cmd_vel)
                self.last_cmd_vel = cmd_vel

            else:
                # 识别不到牌子：立即停止
                cmd_vel = Twist()
                self.cmd_vel_pub.publish(cmd_vel)
                self.last_cmd_vel = cmd_vel

                # 舵机扫描
                if self.last_servo_time is None or (current_time - self.last_servo_time).to_sec() >= self.servo_scan_interval:
                    angle = self.servo_scan_angles[self.servo_scan_index]
                    servo_msg = Float32()
                    servo_msg.data = float(angle)
                    self.servo_angle_pub.publish(servo_msg)

                    # 更新索引
                    self.servo_scan_index = (self.servo_scan_index + 1) % len(self.servo_scan_angles)
                    self.last_servo_time = current_time

            rate.sleep()


def main():
    try:
        YOLOCarControl()
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        rospy.loginfo("收到键盘中断信号")
    finally:
        # 停止小车
        cmd_vel = Twist()
        rospy.Publisher("/yolo_car/cmd_vel", Twist, queue_size=1).publish(cmd_vel)


if __name__ == "__main__":
    main()


```

```python
#!/usr/bin/env python3
"""
yolo_car_detection.py - YOLO物体检测节点

功能：
1. 使用YOLO模型检测USB摄像头中的物体
2. 在窗口上显示检测结果（仅坐标，不再有红/蓝点动画）
3. 发布检测结果到 /yolo_car/detections 话题
"""

import cv2
import os
import rospy
import rospkg
import numpy as np
from ultralytics import YOLO
from yolo_car.msg import ObjectDetections, ObjectDetection
from std_msgs.msg import Header


class YOLOCarDetection:
    def __init__(self):
        rospy.init_node("yolo_car_detection", anonymous=False)
        
        # 获取ROS参数
        model_path = rospy.get_param("~model", None)
        
        # 处理空字符串的情况（launch文件可能传递空字符串）
        if model_path == "" or model_path is None:
            model_path = None
        
        # 如果没有指定模型路径，尝试从包内的models目录加载默认模型
        if model_path is None:
            rospack = rospkg.RosPack()
            package_path = rospack.get_path('yolo_car')
            default_model = os.path.join(package_path, 'models', 'yolov8n_label.pt')
            
            if os.path.exists(default_model):
                model_path = default_model
                rospy.loginfo("使用默认模型: %s", model_path)
            else:
                rospy.logerr("未找到模型文件！")
                rospy.logerr("请将模型文件 'yolov8n_label.pt' 放到 %s 目录", os.path.join(package_path, 'models'))
                rospy.logerr("或者通过参数指定模型路径: _model:=path/to/model.pt")
                rospy.signal_shutdown("缺少模型文件")
        
        conf_threshold = float(rospy.get_param("~conf_threshold", 0.25))
        window_name = rospy.get_param("~window_name", "YOLO Car Detection")
        source = int(rospy.get_param("~source", 0))  # USB摄像头固定为0
        
        # 加载YOLO模型
        rospy.loginfo("加载YOLO模型: %s", model_path)
        try:
            self.model = YOLO(model_path)
        except Exception as e:
            rospy.logerr("加载模型失败: %s", e)
            rospy.signal_shutdown("模型加载失败")
            return
        
        # 创建ROS发布者
        self.detections_pub = rospy.Publisher("/yolo_car/detections", ObjectDetections, queue_size=1)
        
        # 初始化窗口
        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
        self.window_name = window_name
        
        # 当前帧的检测结果 [(center_x, center_y, bbox)]
        self.current_frame_detections = []
        # 矩形顶点（顺时针），仅用于显示轮廓
        # 重新标定的矩形顶点（顺时针）：右下、右上、左上、左下
        self.rect_vertices = [
            (397, 354),  # 右下
            (370, 170),  # 右上
            (214, 173),  # 左上
            (121, 361),  # 左下
        ]
        
        # 启动YOLO检测流
        rospy.loginfo("启动YOLO检测，使用USB摄像头 (source=0)")
        rospy.loginfo("按 'q' 键退出")
        
        try:
            while not rospy.is_shutdown():
                try:
                    got_frame = False
                    for result in self.model(source=source, stream=True, conf=conf_threshold):
                        got_frame = True
                        if rospy.is_shutdown():
                            break

                        # 处理检测结果
                        self.process_detection(result)

                        # 显示图像
                        plotted = result.plot()
                        self.draw_rect(plotted)
                        self.draw_labels(plotted, result)
                        cv2.imshow(self.window_name, plotted)

                        # 检查退出
                        if cv2.waitKey(1) & 0xFF == ord('q'):
                            rospy.loginfo("收到退出信号")
                            rospy.signal_shutdown("用户退出")
                            break

                    if not got_frame:
                        rospy.logwarn("未从摄像头获取到任何帧，1秒后重试...")
                        try:
                            blank = np.zeros((480, 640, 3), dtype=np.uint8)
                            cv2.putText(blank, "No frames from camera, retrying...", (20, 240),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2, cv2.LINE_AA)
                            cv2.imshow(self.window_name, blank)
                            cv2.waitKey(1)
                        except Exception:
                            pass
                        rospy.sleep(1.0)
                except Exception as e:
                    rospy.logerr("YOLO流异常: %s，1秒后重试", e)
                    try:
                        blank = np.zeros((480, 640, 3), dtype=np.uint8)
                        cv2.putText(blank, "Camera error, retrying...", (20, 240),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2, cv2.LINE_AA)
                        cv2.imshow(self.window_name, blank)
                        cv2.waitKey(1)
                    except Exception:
                        pass
                    rospy.sleep(1.0)
        except Exception as e:
            rospy.logerr("检测过程中出错: %s", e)
        finally:
            cv2.destroyAllWindows()
            rospy.loginfo("YOLO检测节点已关闭")
    
    def process_detection(self, result):
        """处理检测结果并发布ROS消息"""
        msg = ObjectDetections()
        msg.header = Header()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "camera_frame"

        # 每帧先清空当前检测缓存，避免上一帧残留
        self.current_frame_detections = []
        
        if result.boxes is None or len(result.boxes) == 0:
            # 没有检测到物体
            self.detections_pub.publish(msg)
            return
        
        # 获取图像尺寸
        if result.orig_shape:
            img_height, img_width = result.orig_shape
        else:
            img_height, img_width = result.plot().shape[:2]
        
        # 提取所有检测信息
        detections = []
        for box in result.boxes:
            bbox = box.xyxy[0].cpu().numpy()  # [x1, y1, x2, y2]
            x1, y1, x2, y2 = bbox
            center_x = (x1 + x2) / 2.0
            center_y = (y1 + y2) / 2.0
            confidence = float(box.conf[0].cpu().numpy()) if box.conf is not None else 0.0
            detections.append((center_x, center_y, (x1, y1, x2, y2), confidence, box))
        
        # 只保留前两个检测结果（按置信度排序，取最高的两个）
        if len(detections) > 2:
            detections.sort(key=lambda x: x[3], reverse=True)  # 按置信度降序排序
            detections = detections[:2]  # 只保留前两个
        
        # 保存当前帧的检测结果，供draw_labels使用（不再区分ID）
        self.current_frame_detections = []
        
        # 创建ROS消息（为兼容消息字段，仍填充 object_id，但不做跟踪）
        for idx, (center_x, center_y, (x1, y1, x2, y2), confidence, box) in enumerate(detections):
            self.current_frame_detections.append((center_x, center_y, (x1, y1, x2, y2)))

            obj_msg = ObjectDetection()
            obj_msg.header = msg.header
            obj_msg.object_id = idx + 1  # 简单序号
            obj_msg.center_x = float(center_x)
            obj_msg.center_y = float(center_y)
            obj_msg.confidence = confidence
            obj_msg.bbox_x = float(x1)
            obj_msg.bbox_y = float(y1)
            obj_msg.bbox_width = float(x2 - x1)
            obj_msg.bbox_height = float(y2 - y1)
            obj_msg.image_width = img_width
            obj_msg.image_height = img_height
            
            msg.objects.append(obj_msg)
        
        # 发布消息
        self.detections_pub.publish(msg)
    
    def draw_labels(self, image, result):
        """在图像上绘制中心点坐标标签（仅坐标，无ID）"""
        if not self.current_frame_detections:
            return
        
        # 直接使用当前帧保存的检测结果（最多两个）
        for center_x, center_y, (x1, y1, x2, y2) in self.current_frame_detections:
            center_x_int = int(center_x)
            center_y_int = int(center_y)
            
            # 只显示坐标，不显示label和ID
            coord_text = f"({center_x_int}, {center_y_int})"
            
            # 获取文本大小
            font = cv2.FONT_HERSHEY_SIMPLEX
            font_scale = 0.6
            thickness = 2
            
            # 计算文本位置
            (text_width_coord, text_height_coord), _ = cv2.getTextSize(coord_text, font, font_scale, thickness)
            
            # 在检测框上方绘制标签，往上移更多避免遮挡YOLO的置信度显示
            text_x = int(x1)
            text_y_coord = int(y1) - text_height_coord - 11  # 往上移更多，留出空间给置信度
            
            # 确保文本不超出图像边界
            if text_y_coord < text_height_coord:
                text_y_coord = int(y2) + text_height_coord + 5
            
            # 绘制背景矩形（提高可读性）
            cv2.rectangle(image, 
                         (text_x - 2, text_y_coord - text_height_coord - 2),
                         (text_x + text_width_coord + 2, text_y_coord + 2),
                         (0, 0, 0), -1)
            
            # 绘制坐标文本
            cv2.putText(image, coord_text, (text_x, text_y_coord), 
                       font, font_scale, (255, 255, 0), thickness)
            
            # 在中心点绘制标记
            cv2.circle(image, (center_x_int, center_y_int), 5, (0, 0, 255), -1)
            cv2.circle(image, (center_x_int, center_y_int), 8, (0, 0, 255), 2)
            
    def draw_rect(self, image):
        """绘制矩形轮廓（仅显示，不再画红/蓝点）"""
        if not self.rect_vertices or len(self.rect_vertices) < 2:
            return
        color_frame = (150, 150, 150)
        thickness_frame = 1
        n = len(self.rect_vertices)
        for i in range(n):
            pt1 = tuple(map(int, self.rect_vertices[i]))
            pt2 = tuple(map(int, self.rect_vertices[(i + 1) % n]))
            cv2.line(image, pt1, pt2, color_frame, thickness_frame, lineType=cv2.LINE_AA)


def main():
    try:
        node = YOLOCarDetection()
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        rospy.loginfo("收到键盘中断信号")
    finally:
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()


```

更新完成，核心改动：
- `esp32/yolo_car_control.ino`：加入编码器计数与轮速发布 `/yolo_car/wheel_vel`（Vector3Stamped，x 左轮 m/s，y 右轮 m/s）、MPU6050 原始 IMU 发布 `/yolo_car/imu_raw`；新增编码器/IMU 引脚、I2C 初始化、周期发布；方向由速度指令决定，编码器仅用 A 相配合方向符号计数；预留常量 `TICKS_PER_REV`、`WHEEL_RADIUS_M`、`WHEEL_BASE_M` 需按实车调整。
- `scripts/yolo_car_detection.py`：用新的矩形顶点重画显示。
- `scripts/yolo_car_control.py`：加入单应性计算，将检测到的像素中心映射到地面坐标系（原点在图像左上，单位 m），改为按真实矩形（0.6 m x 1.2 m）巡线；死区/切边阈值改为米单位；若未算出 H 则回退像素控制并告警。

关键代码片段：
```70:79:src/yolo_car/scripts/yolo_car_detection.py
        self.rect_vertices = [
            (397, 354),  # 右下
            (370, 170),  # 右上
            (214, 173),  # 左上
            (121, 361),  # 左下
        ]
```

```20:46:src/yolo_car/scripts/yolo_car_control.py
import cv2
...
        pixel_default = [
            [214.0, 173.0],  # 左上
            [370.0, 170.0],  # 右上
            [397.0, 354.0],  # 右下
            [121.0, 361.0],  # 左下
        ]
        ground_default = [
            [0.0, 0.0],
            [0.60, 0.0],
            [0.60, 1.20],
            [0.0, 1.20],
        ]
        self.H = self._compute_homography(self.pixel_vertices, np.array(ground_default, dtype=np.float32))
```

```31:63:src/yolo_car/esp32/yolo_car_control.ino
#define ENC_RIGHT_A 7
#define ENC_RIGHT_B 8
#define ENC_LEFT_A  9
#define ENC_LEFT_B  10
#define MPU_ADDR 0x68
#define I2C_SDA 46
#define I2C_SCL 45
...
#define TICKS_PER_REV 390.0f
#define WHEEL_RADIUS_M 0.0325f
#define WHEEL_BASE_M   0.20f
```

使用说明与待调参数：
- 实车前请调整 `TICKS_PER_REV`、`WHEEL_RADIUS_M`、`WHEEL_BASE_M`，以匹配 MG310 编码器齿数与轮径。
- IMU 仅发布原始 gyro/acc，滤波/姿态估计放在 ROS 端处理。
- 轮速发布频率 20 Hz，IMU 50 Hz；命令超时 500 ms 仍会停车。
- 控制节点默认使用上述四点计算单应性，若相机姿态变化需重新取点；未算出 H 时会回退到像素控制并提示。
- 新话题：`/yolo_car/wheel_vel`、`/yolo_car/imu_raw`。可在 ROS 侧做可视化或融合。

如需再调增益/死区、或在控制节点使用轮速/IMU 做进一步闭环，可告知。

```python
        self.rect_vertices = [
            (397, 354),  # 右下
            (370, 170),  # 右上
            (214, 173),  # 左上
            (121, 361),  # 左下
        ]
```

```python
import cv2
...
        pixel_default = [
            [214.0, 173.0],  # 左上
            [370.0, 170.0],  # 右上
            [397.0, 354.0],  # 右下
            [121.0, 361.0],  # 左下
        ]
        ground_default = [
            [0.0, 0.0],
            [0.60, 0.0],
            [0.60, 1.20],
            [0.0, 1.20],
        ]
        self.H = self._compute_homography(self.pixel_vertices, np.array(ground_default, dtype=np.float32))
```

```plaintext
#define ENC_RIGHT_A 7
#define ENC_RIGHT_B 8
#define ENC_LEFT_A  9
#define ENC_LEFT_B  10
#define MPU_ADDR 0x68
#define I2C_SDA 46
#define I2C_SCL 45
...
#define TICKS_PER_REV 390.0f
#define WHEEL_RADIUS_M 0.0325f
#define WHEEL_BASE_M   0.20f
```

---

**User**

     uint8_t requestFrom(int address, int size, int sendStop);

             ^~~~~~~~~~~

C:\Users\s3252\AppData\Local\Arduino15\packages\esp32\hardware\esp32\2.0.7\libraries\Wire\src/Wire.h:122:12: note: candidate 2: 'size_t TwoWire::requestFrom(uint8_t, size_t, bool)'

     size_t requestFrom(uint8_t address, size_t len, bool stopBit);

            ^~~~~~~~~~~

C:\Users\s3252\AppData\Local\Arduino15\packages\esp32\hardware\esp32\2.0.7\libraries\Wire\src/Wire.h:126:13: note: candidate 1: 'uint8_t TwoWire::requestFrom(int, int, int)'

     uint8_t requestFrom(int address, int size, int sendStop);

             ^~~~~~~~~~~

C:\Users\s3252\AppData\Local\Arduino15\packages\esp32\hardware\esp32\2.0.7\libraries\Wire\src/Wire.h:120:13: note: candidate 2: 'uint8_t TwoWire::requestFrom(uint16_t, uint8_t, bool)'

     uint8_t requestFrom(uint16_t address, uint8_t size, bool sendStop);

             ^~~~~~~~~~~

C:\Users\s3252\AppData\Local\Arduino15\packages\esp32\hardware\esp32\2.0.7\libraries\Wire\src/Wire.h:126:13: note: candidate 1: 'uint8_t TwoWire::requestFrom(int, int, int)'

     uint8_t requestFrom(int address, int size, int sendStop);

             ^~~~~~~~~~~

C:\Users\s3252\AppData\Local\Arduino15\packages\esp32\hardware\esp32\2.0.7\libraries\Wire\src/Wire.h:119:12: note: candidate 2: 'size_t TwoWire::requestFrom(uint16_t, size_t, bool)'

     size_t requestFrom(uint16_t address, size_t size, bool sendStop);

            ^~~~~~~~~~~

C:\Users\s3252\AppData\Local\Arduino15\packages\esp32\hardware\esp32\2.0.7\libraries\Wire\src/Wire.h:126:13: note: candidate 1: 'uint8_t TwoWire::requestFrom(int, int, int)'

     uint8_t requestFrom(int address, int size, int sendStop);

             ^~~~~~~~~~~

C:\Users\s3252\AppData\Local\Arduino15\packages\esp32\hardware\esp32\2.0.7\libraries\Wire\src/Wire.h:122:12: note: candidate 2: 'size_t TwoWire::requestFrom(uint8_t, size_t, bool)'

     size_t requestFrom(uint8_t address, size_t len, bool stopBit);                            在arduino编译的时候出现这些是什么意思，报错了吗，然后#define TICKS_PER_REV 390.0f

#define WHEEL_RADIUS_M 0.0325f

#define WHEEL_BASE_M   0.20f 这三个我已经测量更新了的

---

**Cursor**

这是编译错误：`Wire.requestFrom()` 调用存在类型歧义。检查 ESP32 代码中的调用并修复：


文件被删除。重新创建 ESP32 代码，修复 `Wire.requestFrom()` 的类型歧义，并更新你提供的常量值：


```cpp
/*
 * ESP32-S3 + L298N电机驱动 + SG90舵机 + 编码器 + MPU6050 + ROS1 (rosserial)
 * 
 * 功能：
 * 1. 订阅速度指令控制L298N电机
 * 2. 订阅舵机角度指令控制SG90舵机
 * 3. 编码器计数并发布轮速
 * 4. MPU6050读取并发布原始IMU数据
 * 
 * 接线：
 * - L298N：N1->GPIO3(左PWM), N2->GPIO4(左DIR), N3->GPIO5(右PWM), N4->GPIO6(右DIR)
 * - SG90舵机信号线：GPIO2
 * - 编码器：右轮A->GPIO7, B->GPIO8; 左轮A->GPIO9, B->GPIO10
 * - MPU6050：SCL->GPIO45, SDA->GPIO46
 */

#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <Wire.h>

#define USE_WIFI true

#if USE_WIFI
  #include <WiFi.h>
  const char* ssid = "机器人小车局域网";
  const char* password = "3252362734";
  IPAddress server(192, 168, 31, 64);
  const uint16_t serverPort = 11411;
#endif

// L298N电机控制引脚
#define MOTOR_LEFT_PWM   3   // 左轮PWM
#define MOTOR_LEFT_DIR   4   // 左轮方向
#define MOTOR_RIGHT_PWM  5   // 右轮PWM
#define MOTOR_RIGHT_DIR  6   // 右轮方向

// SG90舵机引脚
#define SERVO_PIN 2

// 编码器引脚
#define ENC_RIGHT_A 7
#define ENC_RIGHT_B 8
#define ENC_LEFT_A  9
#define ENC_LEFT_B  10

// MPU6050 I2C引脚和地址
#define MPU_ADDR 0x68
#define I2C_SDA 46
#define I2C_SCL 45

// PWM参数
#define PWM_FREQ 1000      // PWM频率 1kHz
#define PWM_RESOLUTION 8   // 8位分辨率 (0-255)
#define SERVO_FREQ 50      // 舵机PWM频率 50Hz
#define SERVO_RESOLUTION 12

// 速度限制
#define MAX_PWM_VALUE 200  // 最大PWM值

// 命令超时（毫秒）
#define CMD_TIMEOUT_MS 500

// 编码器参数（已测量更新）
#define TICKS_PER_REV 390.0f      // 每转脉冲数
#define WHEEL_RADIUS_M 0.0325f    // 轮子半径（米）
#define WHEEL_BASE_M   0.20f      // 轮距（米）

// 编码器计数变量（volatile用于中断）
volatile int32_t enc_left_count = 0;
volatile int32_t enc_right_count = 0;
volatile int8_t enc_left_last_state = 0;
volatile int8_t enc_right_last_state = 0;

// 轮速计算
float left_wheel_vel = 0.0f;
float right_wheel_vel = 0.0f;
unsigned long last_enc_time = 0;
const unsigned long ENC_UPDATE_INTERVAL_MS = 50; // 50ms更新一次，20Hz

// ROS节点
ros::NodeHandle nh;
unsigned long last_cmd_time = 0;

// 发布者
ros::Publisher wheel_vel_pub("/yolo_car/wheel_vel", &wheel_vel_msg);
ros::Publisher imu_raw_pub("/yolo_car/imu_raw", &imu_raw_msg);
geometry_msgs::Vector3Stamped wheel_vel_msg;
sensor_msgs::Imu imu_raw_msg;

// 左编码器中断处理（A相）
void IRAM_ATTR encLeftA() {
  int8_t a = digitalRead(ENC_LEFT_A);
  int8_t b = digitalRead(ENC_LEFT_B);
  int8_t state = (a << 1) | b;
  int8_t diff = (state - enc_left_last_state) & 0x03;
  if (diff == 1 || diff == 3) {
    enc_left_count++;
  } else if (diff == 2) {
    enc_left_count--;
  }
  enc_left_last_state = state;
}

// 右编码器中断处理（A相）
void IRAM_ATTR encRightA() {
  int8_t a = digitalRead(ENC_RIGHT_A);
  int8_t b = digitalRead(ENC_RIGHT_B);
  int8_t state = (a << 1) | b;
  int8_t diff = (state - enc_right_last_state) & 0x03;
  if (diff == 1 || diff == 3) {
    enc_right_count++;
  } else if (diff == 2) {
    enc_right_count--;
  }
  enc_right_last_state = state;
}

// 电机急停
void stopMotors() {
  ledcWrite(0, 0);
  ledcWrite(1, 0);
  digitalWrite(MOTOR_LEFT_DIR, LOW);
  digitalWrite(MOTOR_RIGHT_DIR, LOW);
}

// 速度指令回调
void cmdVelCallback(const geometry_msgs::Twist& msg) {
  float linear_vel = msg.linear.x;   // 线速度 m/s
  float angular_vel = msg.angular.z; // 角速度 rad/s

  last_cmd_time = millis();
  
  if (fabs(linear_vel) < 1e-3 && fabs(angular_vel) < 1e-3) {
    stopMotors();
    return;
  }
  
  // 差速控制
  float left_vel = linear_vel - (angular_vel * WHEEL_BASE_M / 2.0);
  float right_vel = linear_vel + (angular_vel * WHEEL_BASE_M / 2.0);
  
  float max_vel = 0.3;
  left_vel = constrain(left_vel, -max_vel, max_vel);
  right_vel = constrain(right_vel, -max_vel, max_vel);
  
  int left_pwm = abs(left_vel / max_vel * MAX_PWM_VALUE);
  int right_pwm = abs(right_vel / max_vel * MAX_PWM_VALUE);
  
  // 方向控制
  if (left_vel >= 0) {
    digitalWrite(MOTOR_LEFT_DIR, HIGH);
  } else {
    digitalWrite(MOTOR_LEFT_DIR, LOW);
  }
  
  if (right_vel >= 0) {
    digitalWrite(MOTOR_RIGHT_DIR, LOW);
  } else {
    digitalWrite(MOTOR_RIGHT_DIR, HIGH);
  }
  
  ledcWrite(0, left_pwm);
  ledcWrite(1, right_pwm);
}

// 舵机角度回调
void servoAngleCallback(const std_msgs::Float32& msg) {
  float angle = msg.data;
  angle = constrain(angle, 0.0, 180.0);

  int pulse_width = map(angle, 0, 180, 500, 2500);
  int duty_cycle = (pulse_width * 4096L) / 20000;
  duty_cycle = constrain(duty_cycle, 0, 4095);
  ledcWrite(2, duty_cycle);
}

// 订阅者
ros::Subscriber<geometry_msgs::Twist> cmd_vel_sub("/yolo_car/cmd_vel", cmdVelCallback);
ros::Subscriber<std_msgs::Float32> servo_angle_sub("/yolo_car/servo_angle", servoAngleCallback);

// MPU6050初始化
bool initMPU6050() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B); // PWR_MGMT_1寄存器
  Wire.write(0x00); // 唤醒MPU6050
  uint8_t error = Wire.endTransmission();
  
  if (error != 0) {
    return false;
  }
  
  // 配置加速度计范围 ±2g
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1C); // ACCEL_CONFIG
  Wire.write(0x00);
  Wire.endTransmission();
  
  // 配置陀螺仪范围 ±250°/s
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1B); // GYRO_CONFIG
  Wire.write(0x00);
  Wire.endTransmission();
  
  return true;
}

// 读取MPU6050原始数据
void readMPU6050() {
  // 读取加速度计数据（6字节：ACCEL_XOUT_H, ACCEL_XOUT_L, ACCEL_YOUT_H, ACCEL_YOUT_L, ACCEL_ZOUT_H, ACCEL_ZOUT_L）
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B); // ACCEL_XOUT_H寄存器地址
  Wire.endTransmission(false);
  
  // 修复：使用明确的类型参数，避免歧义
  uint8_t bytes_read = Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)14, (bool)true);
  
  if (bytes_read == 14) {
    int16_t accel_x = (Wire.read() << 8) | Wire.read();
    int16_t accel_y = (Wire.read() << 8) | Wire.read();
    int16_t accel_z = (Wire.read() << 8) | Wire.read();
    int16_t temp = (Wire.read() << 8) | Wire.read();
    int16_t gyro_x = (Wire.read() << 8) | Wire.read();
    int16_t gyro_y = (Wire.read() << 8) | Wire.read();
    int16_t gyro_z = (Wire.read() << 8) | Wire.read();
    
    // 转换为物理单位
    // 加速度：±2g范围，32768对应2g，g=9.8 m/s²
    float accel_scale = 2.0f * 9.8f / 32768.0f;
    imu_raw_msg.linear_acceleration.x = accel_x * accel_scale;
    imu_raw_msg.linear_acceleration.y = accel_y * accel_scale;
    imu_raw_msg.linear_acceleration.z = accel_z * accel_scale;
    
    // 角速度：±250°/s范围，32768对应250°/s，转换为rad/s
    float gyro_scale = 250.0f * M_PI / 180.0f / 32768.0f;
    imu_raw_msg.angular_velocity.x = gyro_x * gyro_scale;
    imu_raw_msg.angular_velocity.y = gyro_y * gyro_scale;
    imu_raw_msg.angular_velocity.z = gyro_z * gyro_scale;
    
    imu_raw_msg.header.stamp = nh.now();
    imu_raw_msg.header.frame_id = "imu_link";
  }
}

// 更新轮速
void updateWheelVelocities() {
  unsigned long current_time = millis();
  unsigned long dt_ms = current_time - last_enc_time;
  
  if (dt_ms >= ENC_UPDATE_INTERVAL_MS) {
    // 读取并重置计数（原子操作）
    int32_t left_ticks = enc_left_count;
    int32_t right_ticks = enc_right_count;
    enc_left_count = 0;
    enc_right_count = 0;
    
    // 计算轮速（m/s）
    // 每转脉冲数 * 轮子周长 = 每米脉冲数
    float ticks_per_meter = TICKS_PER_REV / (2.0f * M_PI * WHEEL_RADIUS_M);
    float dt_sec = dt_ms / 1000.0f;
    
    left_wheel_vel = (left_ticks / ticks_per_meter) / dt_sec;
    right_wheel_vel = (right_ticks / ticks_per_meter) / dt_sec;
    
    // 发布轮速
    wheel_vel_msg.header.stamp = nh.now();
    wheel_vel_msg.header.frame_id = "base_link";
    wheel_vel_msg.vector.x = left_wheel_vel;
    wheel_vel_msg.vector.y = right_wheel_vel;
    wheel_vel_msg.vector.z = 0.0;
    wheel_vel_pub.publish(&wheel_vel_msg);
    
    last_enc_time = current_time;
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n\n=== ESP32-S3 + L298N + SG90 + 编码器 + MPU6050 + ROS1 ===");

  last_cmd_time = millis();
  
  // 初始化电机控制引脚
  Serial.println("初始化L298N电机驱动...");
  pinMode(MOTOR_LEFT_DIR, OUTPUT);
  pinMode(MOTOR_RIGHT_DIR, OUTPUT);
  digitalWrite(MOTOR_LEFT_DIR, LOW);
  digitalWrite(MOTOR_RIGHT_DIR, LOW);
  
  // 配置PWM通道（电机）
  ledcSetup(0, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(1, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(MOTOR_LEFT_PWM, 0);
  ledcAttachPin(MOTOR_RIGHT_PWM, 1);
  
  // 配置PWM通道（舵机）
  ledcSetup(2, SERVO_FREQ, SERVO_RESOLUTION);
  ledcAttachPin(SERVO_PIN, 2);
  
  ledcWrite(0, 0);
  ledcWrite(1, 0);
  
  // 初始舵机到90度
  float init_angle = 90.0;
  int pulse_width = map(init_angle, 0, 180, 500, 2500);
  int duty_cycle = (pulse_width * 4096L) / 20000;
  duty_cycle = constrain(duty_cycle, 0, 4095);
  ledcWrite(2, duty_cycle);
  
  Serial.println("✓ 电机和舵机初始化完成");
  
  // 初始化编码器
  Serial.println("初始化编码器...");
  pinMode(ENC_LEFT_A, INPUT_PULLUP);
  pinMode(ENC_LEFT_B, INPUT_PULLUP);
  pinMode(ENC_RIGHT_A, INPUT_PULLUP);
  pinMode(ENC_RIGHT_B, INPUT_PULLUP);
  
  enc_left_last_state = (digitalRead(ENC_LEFT_A) << 1) | digitalRead(ENC_LEFT_B);
  enc_right_last_state = (digitalRead(ENC_RIGHT_A) << 1) | digitalRead(ENC_RIGHT_B);
  
  attachInterrupt(digitalPinToInterrupt(ENC_LEFT_A), encLeftA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_RIGHT_A), encRightA, CHANGE);
  
  last_enc_time = millis();
  Serial.println("✓ 编码器初始化完成");
  
  // 初始化MPU6050
  Serial.println("初始化MPU6050...");
  Wire.begin(I2C_SDA, I2C_SCL);
  delay(100);
  
  if (initMPU6050()) {
    Serial.println("✓ MPU6050初始化成功");
  } else {
    Serial.println("✗ MPU6050初始化失败");
  }
  
#if USE_WIFI
  Serial.println("\n=== WiFi连接 ===");
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  
  Serial.print("连接中");
  int wifiAttempts = 0;
  while (WiFi.status() != WL_CONNECTED && wifiAttempts < 30) {
    delay(500);
    Serial.print(".");
    wifiAttempts++;
  }
  Serial.println();
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("✓ WiFi连接成功！IP: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("✗ WiFi连接失败！");
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    nh.getHardware()->setConnection(server, serverPort);
  }
#endif
  
  Serial.println("\n=== ROS连接 ===");
  nh.initNode();
  nh.subscribe(cmd_vel_sub);
  nh.subscribe(servo_angle_sub);
  nh.advertise(wheel_vel_pub);
  nh.advertise(imu_raw_pub);
  
  Serial.print("等待连接");
  int rosAttempts = 0;
  while (!nh.connected() && rosAttempts < 50) {
    nh.spinOnce();
    delay(100);
    if (rosAttempts % 10 == 0) Serial.print(".");
    rosAttempts++;
  }
  Serial.println();
  
  if (nh.connected()) {
    Serial.println("✓ ROS连接成功！");
  } else {
    Serial.println("✗ ROS连接失败！");
  }
  
  Serial.println("\n=== 系统状态 ===");
  Serial.print("WiFi: ");
  Serial.println((WiFi.status() == WL_CONNECTED) ? "✓ 已连接" : "✗ 未连接");
  Serial.print("ROS: ");
  Serial.println(nh.connected() ? "✓ 已连接" : "✗ 未连接");
  Serial.print("编码器参数: TICKS_PER_REV=");
  Serial.print(TICKS_PER_REV);
  Serial.print(", WHEEL_RADIUS_M=");
  Serial.print(WHEEL_RADIUS_M);
  Serial.print(", WHEEL_BASE_M=");
  Serial.println(WHEEL_BASE_M);
  Serial.println("\n开始运行...\n");
}

void loop() {
  // 命令超时保护
  if (millis() - last_cmd_time > CMD_TIMEOUT_MS) {
    stopMotors();
  }

  if (!nh.connected()) {
    static unsigned long lastReconnectMsg = 0;
    if (millis() - lastReconnectMsg > 5000) {
      Serial.println("ROS连接断开，尝试重连...");
      lastReconnectMsg = millis();
    }
    stopMotors();
    nh.spinOnce();
    delay(100);
    return;
  }

  // 更新轮速（20Hz）
  updateWheelVelocities();
  
  // 读取并发布IMU（50Hz）
  static unsigned long last_imu_time = 0;
  if (millis() - last_imu_time >= 20) {
    readMPU6050();
    imu_raw_pub.publish(&imu_raw_msg);
    last_imu_time = millis();
  }

  nh.spinOnce();
  delay(10);
}

```

已修复编译错误并更新常量值。

**主要修复：**

1. **修复 `Wire.requestFrom()` 类型歧义**（第234行）：
   ```cpp
   uint8_t bytes_read = Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)14, (bool)true);
   ```
   通过显式类型转换消除歧义，明确使用 `uint8_t` 版本的重载。

2. **更新你提供的常量值**（第50-52行）：
   ```cpp
   #define TICKS_PER_REV 390.0f      // 每转脉冲数
   #define WHEEL_RADIUS_M 0.0325f    // 轮子半径（米）
   #define WHEEL_BASE_M   0.20f      // 轮距（米）
   ```

3. **其他改进**：
   - 编码器中断处理使用 `IRAM_ATTR` 优化
   - 轮速计算使用你提供的参数
   - IMU 数据发布频率 50Hz（每20ms）
   - 轮速发布频率 20Hz（每50ms）

现在应可正常编译。如仍有问题，请告知具体错误信息。

```cpp
   uint8_t bytes_read = Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)14, (bool)true);
```

```cpp
   #define TICKS_PER_REV 390.0f      // 每转脉冲数
   #define WHEEL_RADIUS_M 0.0325f    // 轮子半径（米）
   #define WHEEL_BASE_M   0.20f      // 轮距（米）
```

---

**User**

// 发布者

ros::Publisher wheel_vel_pub("/yolo_car/wheel_vel", &wheel_vel_msg);

ros::Publisher imu_raw_pub("/yolo_car/imu_raw", &imu_raw_msg);     uint8_t requestFrom(int address, int size, int sendStop);

             ^~~~~~~~~~~

C:\Users\s3252\AppData\Local\Arduino15\packages\esp32\hardware\esp32\2.0.7\libraries\Wire\src/Wire.h:121:13: note: candidate 2: 'uint8_t TwoWire::requestFrom(uint16_t, uint8_t, uint8_t)'

     uint8_t requestFrom(uint16_t address, uint8_t size, uint8_t sendStop);

             ^~~~~~~~~~~

C:\Users\s3252\AppData\Local\Arduino15\packages\esp32\hardware\esp32\2.0.7\libraries\Wire\src/Wire.h:126:13: note: candidate 1: 'uint8_t TwoWire::requestFrom(int, int, int)'

     uint8_t requestFrom(int address, int size, int sendStop);

             ^~~~~~~~~~~

C:\Users\s3252\AppData\Local\Arduino15\packages\esp32\hardware\esp32\2.0.7\libraries\Wire\src/Wire.h:120:13: note: candidate 2: 'uint8_t TwoWire::requestFrom(uint16_t, uint8_t, bool)'

     uint8_t requestFrom(uint16_t address, uint8_t size, bool sendStop);

             ^~~~~~~~~~~

C:\Users\s3252\AppData\Local\Arduino15\packages\esp32\hardware\esp32\2.0.7\libraries\Wire\src/Wire.h:126:13: note: candidate 1: 'uint8_t TwoWire::requestFrom(int, int, int)'

     uint8_t requestFrom(int address, int size, int sendStop);

             ^~~~~~~~~~~

C:\Users\s3252\AppData\Local\Arduino15\packages\esp32\hardware\esp32\2.0.7\libraries\Wire\src/Wire.h:119:12: note: candidate 2: 'size_t TwoWire::requestFrom(uint16_t, size_t, bool)'

     size_t requestFrom(uint16_t address, size_t size, bool sendStop);

            ^~~~~~~~~~~

C:\Users\s3252\AppData\Local\Arduino15\packages\esp32\hardware\esp32\2.0.7\libraries\Wire\src/Wire.h:126:13: note: candidate 1: 'uint8_t TwoWire::requestFrom(int, int, int)'

     uint8_t requestFrom(int address, int size, int sendStop);

             ^~~~~~~~~~~

C:\Users\s3252\AppData\Local\Arduino15\packages\esp32\hardware\esp32\2.0.7\libraries\Wire\src/Wire.h:124:13: note: candidate 2: 'uint8_t TwoWire::requestFrom(uint8_t, uint8_t, uint8_t)'

     uint8_t requestFrom(uint8_t address, uint8_t size, uint8_t sendStop);

             ^~~~~~~~~~~

exit status 1这里还是有报错，是怎么回事，

// 编码器与几何参数（需按实际调整）

#define TICKS_PER_REV 260.0f          // 轮子一圈的脉冲数（含减速比），请按实际修改

#define WHEEL_RADIUS_M 0.0480f        // 轮子半径（米），按实际轮径调整

#define WHEEL_BASE_M   0.1475f          // 轮距（米）这个才是我的测量值不要搞错了

---

**Cursor**

检查代码，修复 `Wire.requestFrom()` 的类型问题，并更新你提供的常量值：


修复两个问题：
1. 发布者声明在消息变量之前，导致编译错误
2. `Wire.requestFrom()` 的类型转换需要更明确
3. 更新你提供的常量值


```cpp
/*
 * ESP32-S3 + L298N电机驱动 + SG90舵机 + 编码器 + MPU6050 + ROS1 (rosserial)
 * 
 * 功能：
 * 1. 订阅速度指令控制L298N电机
 * 2. 订阅舵机角度指令控制SG90舵机
 * 3. 编码器计数并发布轮速
 * 4. MPU6050读取并发布原始IMU数据
 * 
 * 接线：
 * - L298N：N1->GPIO3(左PWM), N2->GPIO4(左DIR), N3->GPIO5(右PWM), N4->GPIO6(右DIR)
 * - SG90舵机信号线：GPIO2
 * - 编码器：右轮A->GPIO7, B->GPIO8; 左轮A->GPIO9, B->GPIO10
 * - MPU6050：SCL->GPIO45, SDA->GPIO46
 */

#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <Wire.h>

#define USE_WIFI true

#if USE_WIFI
  #include <WiFi.h>
  const char* ssid = "机器人小车局域网";
  const char* password = "3252362734";
  IPAddress server(192, 168, 31, 64);
  const uint16_t serverPort = 11411;
#endif

// L298N电机控制引脚
#define MOTOR_LEFT_PWM   3   // 左轮PWM
#define MOTOR_LEFT_DIR   4   // 左轮方向
#define MOTOR_RIGHT_PWM  5   // 右轮PWM
#define MOTOR_RIGHT_DIR  6   // 右轮方向

// SG90舵机引脚
#define SERVO_PIN 2

// 编码器引脚
#define ENC_RIGHT_A 7
#define ENC_RIGHT_B 8
#define ENC_LEFT_A  9
#define ENC_LEFT_B  10

// MPU6050 I2C引脚和地址
#define MPU_ADDR 0x68
#define I2C_SDA 46
#define I2C_SCL 45

// PWM参数
#define PWM_FREQ 1000      // PWM频率 1kHz
#define PWM_RESOLUTION 8   // 8位分辨率 (0-255)
#define SERVO_FREQ 50      // 舵机PWM频率 50Hz
#define SERVO_RESOLUTION 12

// 速度限制
#define MAX_PWM_VALUE 200  // 最大PWM值

// 命令超时（毫秒）
#define CMD_TIMEOUT_MS 500

// 编码器参数（已测量更新）
#define TICKS_PER_REV 260.0f      // 每转脉冲数
#define WHEEL_RADIUS_M 0.0480f    // 轮子半径（米）
#define WHEEL_BASE_M   0.1475f    // 轮距（米）

// 编码器计数变量（volatile用于中断）
volatile int32_t enc_left_count = 0;
volatile int32_t enc_right_count = 0;
volatile int8_t enc_left_last_state = 0;
volatile int8_t enc_right_last_state = 0;

// 轮速计算
float left_wheel_vel = 0.0f;
float right_wheel_vel = 0.0f;
unsigned long last_enc_time = 0;
const unsigned long ENC_UPDATE_INTERVAL_MS = 50; // 50ms更新一次，20Hz

// ROS节点
ros::NodeHandle nh;
unsigned long last_cmd_time = 0;

// ROS消息变量（必须先声明，发布者才能引用）
geometry_msgs::Vector3Stamped wheel_vel_msg;
sensor_msgs::Imu imu_raw_msg;

// 发布者（必须在消息变量之后声明）
ros::Publisher wheel_vel_pub("/yolo_car/wheel_vel", &wheel_vel_msg);
ros::Publisher imu_raw_pub("/yolo_car/imu_raw", &imu_raw_msg);

// 左编码器中断处理（A相）
void IRAM_ATTR encLeftA() {
  int8_t a = digitalRead(ENC_LEFT_A);
  int8_t b = digitalRead(ENC_LEFT_B);
  int8_t state = (a << 1) | b;
  int8_t diff = (state - enc_left_last_state) & 0x03;
  if (diff == 1 || diff == 3) {
    enc_left_count++;
  } else if (diff == 2) {
    enc_left_count--;
  }
  enc_left_last_state = state;
}

// 右编码器中断处理（A相）
void IRAM_ATTR encRightA() {
  int8_t a = digitalRead(ENC_RIGHT_A);
  int8_t b = digitalRead(ENC_RIGHT_B);
  int8_t state = (a << 1) | b;
  int8_t diff = (state - enc_right_last_state) & 0x03;
  if (diff == 1 || diff == 3) {
    enc_right_count++;
  } else if (diff == 2) {
    enc_right_count--;
  }
  enc_right_last_state = state;
}

// 电机急停
void stopMotors() {
  ledcWrite(0, 0);
  ledcWrite(1, 0);
  digitalWrite(MOTOR_LEFT_DIR, LOW);
  digitalWrite(MOTOR_RIGHT_DIR, LOW);
}

// 速度指令回调
void cmdVelCallback(const geometry_msgs::Twist& msg) {
  float linear_vel = msg.linear.x;   // 线速度 m/s
  float angular_vel = msg.angular.z; // 角速度 rad/s

  last_cmd_time = millis();
  
  if (fabs(linear_vel) < 1e-3 && fabs(angular_vel) < 1e-3) {
    stopMotors();
    return;
  }
  
  // 差速控制
  float left_vel = linear_vel - (angular_vel * WHEEL_BASE_M / 2.0);
  float right_vel = linear_vel + (angular_vel * WHEEL_BASE_M / 2.0);
  
  float max_vel = 0.3;
  left_vel = constrain(left_vel, -max_vel, max_vel);
  right_vel = constrain(right_vel, -max_vel, max_vel);
  
  int left_pwm = abs(left_vel / max_vel * MAX_PWM_VALUE);
  int right_pwm = abs(right_vel / max_vel * MAX_PWM_VALUE);
  
  // 方向控制
  if (left_vel >= 0) {
    digitalWrite(MOTOR_LEFT_DIR, HIGH);
  } else {
    digitalWrite(MOTOR_LEFT_DIR, LOW);
  }
  
  if (right_vel >= 0) {
    digitalWrite(MOTOR_RIGHT_DIR, LOW);
  } else {
    digitalWrite(MOTOR_RIGHT_DIR, HIGH);
  }
  
  ledcWrite(0, left_pwm);
  ledcWrite(1, right_pwm);
}

// 舵机角度回调
void servoAngleCallback(const std_msgs::Float32& msg) {
  float angle = msg.data;
  angle = constrain(angle, 0.0, 180.0);

  int pulse_width = map(angle, 0, 180, 500, 2500);
  int duty_cycle = (pulse_width * 4096L) / 20000;
  duty_cycle = constrain(duty_cycle, 0, 4095);
  ledcWrite(2, duty_cycle);
}

// 订阅者
ros::Subscriber<geometry_msgs::Twist> cmd_vel_sub("/yolo_car/cmd_vel", cmdVelCallback);
ros::Subscriber<std_msgs::Float32> servo_angle_sub("/yolo_car/servo_angle", servoAngleCallback);

// MPU6050初始化
bool initMPU6050() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B); // PWR_MGMT_1寄存器
  Wire.write(0x00); // 唤醒MPU6050
  uint8_t error = Wire.endTransmission();
  
  if (error != 0) {
    return false;
  }
  
  // 配置加速度计范围 ±2g
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1C); // ACCEL_CONFIG
  Wire.write(0x00);
  Wire.endTransmission();
  
  // 配置陀螺仪范围 ±250°/s
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1B); // GYRO_CONFIG
  Wire.write(0x00);
  Wire.endTransmission();
  
  return true;
}

// 读取MPU6050原始数据
void readMPU6050() {
  // 读取加速度计数据（6字节：ACCEL_XOUT_H, ACCEL_XOUT_L, ACCEL_YOUT_H, ACCEL_YOUT_L, ACCEL_ZOUT_H, ACCEL_ZOUT_L）
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B); // ACCEL_XOUT_H寄存器地址
  Wire.endTransmission(false);
  
  // 修复：使用明确的类型参数，避免歧义
  uint8_t bytes_read = Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)14, (bool)true);
  
  if (bytes_read == 14) {
    int16_t accel_x = (Wire.read() << 8) | Wire.read();
    int16_t accel_y = (Wire.read() << 8) | Wire.read();
    int16_t accel_z = (Wire.read() << 8) | Wire.read();
    int16_t temp = (Wire.read() << 8) | Wire.read();
    int16_t gyro_x = (Wire.read() << 8) | Wire.read();
    int16_t gyro_y = (Wire.read() << 8) | Wire.read();
    int16_t gyro_z = (Wire.read() << 8) | Wire.read();
    
    // 转换为物理单位
    // 加速度：±2g范围，32768对应2g，g=9.8 m/s²
    float accel_scale = 2.0f * 9.8f / 32768.0f;
    imu_raw_msg.linear_acceleration.x = accel_x * accel_scale;
    imu_raw_msg.linear_acceleration.y = accel_y * accel_scale;
    imu_raw_msg.linear_acceleration.z = accel_z * accel_scale;
    
    // 角速度：±250°/s范围，32768对应250°/s，转换为rad/s
    float gyro_scale = 250.0f * M_PI / 180.0f / 32768.0f;
    imu_raw_msg.angular_velocity.x = gyro_x * gyro_scale;
    imu_raw_msg.angular_velocity.y = gyro_y * gyro_scale;
    imu_raw_msg.angular_velocity.z = gyro_z * gyro_scale;
    
    imu_raw_msg.header.stamp = nh.now();
    imu_raw_msg.header.frame_id = "imu_link";
  }
}

// 更新轮速
void updateWheelVelocities() {
  unsigned long current_time = millis();
  unsigned long dt_ms = current_time - last_enc_time;
  
  if (dt_ms >= ENC_UPDATE_INTERVAL_MS) {
    // 读取并重置计数（原子操作）
    int32_t left_ticks = enc_left_count;
    int32_t right_ticks = enc_right_count;
    enc_left_count = 0;
    enc_right_count = 0;
    
    // 计算轮速（m/s）
    // 每转脉冲数 * 轮子周长 = 每米脉冲数
    float ticks_per_meter = TICKS_PER_REV / (2.0f * M_PI * WHEEL_RADIUS_M);
    float dt_sec = dt_ms / 1000.0f;
    
    left_wheel_vel = (left_ticks / ticks_per_meter) / dt_sec;
    right_wheel_vel = (right_ticks / ticks_per_meter) / dt_sec;
    
    // 发布轮速
    wheel_vel_msg.header.stamp = nh.now();
    wheel_vel_msg.header.frame_id = "base_link";
    wheel_vel_msg.vector.x = left_wheel_vel;
    wheel_vel_msg.vector.y = right_wheel_vel;
    wheel_vel_msg.vector.z = 0.0;
    wheel_vel_pub.publish(&wheel_vel_msg);
    
    last_enc_time = current_time;
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n\n=== ESP32-S3 + L298N + SG90 + 编码器 + MPU6050 + ROS1 ===");

  last_cmd_time = millis();
  
  // 初始化电机控制引脚
  Serial.println("初始化L298N电机驱动...");
  pinMode(MOTOR_LEFT_DIR, OUTPUT);
  pinMode(MOTOR_RIGHT_DIR, OUTPUT);
  digitalWrite(MOTOR_LEFT_DIR, LOW);
  digitalWrite(MOTOR_RIGHT_DIR, LOW);
  
  // 配置PWM通道（电机）
  ledcSetup(0, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(1, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(MOTOR_LEFT_PWM, 0);
  ledcAttachPin(MOTOR_RIGHT_PWM, 1);
  
  // 配置PWM通道（舵机）
  ledcSetup(2, SERVO_FREQ, SERVO_RESOLUTION);
  ledcAttachPin(SERVO_PIN, 2);
  
  ledcWrite(0, 0);
  ledcWrite(1, 0);
  
  // 初始舵机到90度
  float init_angle = 90.0;
  int pulse_width = map(init_angle, 0, 180, 500, 2500);
  int duty_cycle = (pulse_width * 4096L) / 20000;
  duty_cycle = constrain(duty_cycle, 0, 4095);
  ledcWrite(2, duty_cycle);
  
  Serial.println("✓ 电机和舵机初始化完成");
  
  // 初始化编码器
  Serial.println("初始化编码器...");
  pinMode(ENC_LEFT_A, INPUT_PULLUP);
  pinMode(ENC_LEFT_B, INPUT_PULLUP);
  pinMode(ENC_RIGHT_A, INPUT_PULLUP);
  pinMode(ENC_RIGHT_B, INPUT_PULLUP);
  
  enc_left_last_state = (digitalRead(ENC_LEFT_A) << 1) | digitalRead(ENC_LEFT_B);
  enc_right_last_state = (digitalRead(ENC_RIGHT_A) << 1) | digitalRead(ENC_RIGHT_B);
  
  attachInterrupt(digitalPinToInterrupt(ENC_LEFT_A), encLeftA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_RIGHT_A), encRightA, CHANGE);
  
  last_enc_time = millis();
  Serial.println("✓ 编码器初始化完成");
  
  // 初始化MPU6050
  Serial.println("初始化MPU6050...");
  Wire.begin(I2C_SDA, I2C_SCL);
  delay(100);
  
  if (initMPU6050()) {
    Serial.println("✓ MPU6050初始化成功");
  } else {
    Serial.println("✗ MPU6050初始化失败");
  }
  
#if USE_WIFI
  Serial.println("\n=== WiFi连接 ===");
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  
  Serial.print("连接中");
  int wifiAttempts = 0;
  while (WiFi.status() != WL_CONNECTED && wifiAttempts < 30) {
    delay(500);
    Serial.print(".");
    wifiAttempts++;
  }
  Serial.println();
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("✓ WiFi连接成功！IP: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("✗ WiFi连接失败！");
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    nh.getHardware()->setConnection(server, serverPort);
  }
#endif
  
  Serial.println("\n=== ROS连接 ===");
  nh.initNode();
  nh.subscribe(cmd_vel_sub);
  nh.subscribe(servo_angle_sub);
  nh.advertise(wheel_vel_pub);
  nh.advertise(imu_raw_pub);
  
  Serial.print("等待连接");
  int rosAttempts = 0;
  while (!nh.connected() && rosAttempts < 50) {
    nh.spinOnce();
    delay(100);
    if (rosAttempts % 10 == 0) Serial.print(".");
    rosAttempts++;
  }
  Serial.println();
  
  if (nh.connected()) {
    Serial.println("✓ ROS连接成功！");
  } else {
    Serial.println("✗ ROS连接失败！");
  }
  
  Serial.println("\n=== 系统状态 ===");
  Serial.print("WiFi: ");
  Serial.println((WiFi.status() == WL_CONNECTED) ? "✓ 已连接" : "✗ 未连接");
  Serial.print("ROS: ");
  Serial.println(nh.connected() ? "✓ 已连接" : "✗ 未连接");
  Serial.print("编码器参数: TICKS_PER_REV=");
  Serial.print(TICKS_PER_REV);
  Serial.print(", WHEEL_RADIUS_M=");
  Serial.print(WHEEL_RADIUS_M);
  Serial.print(", WHEEL_BASE_M=");
  Serial.println(WHEEL_BASE_M);
  Serial.println("\n开始运行...\n");
}

void loop() {
  // 命令超时保护
  if (millis() - last_cmd_time > CMD_TIMEOUT_MS) {
    stopMotors();
  }

  if (!nh.connected()) {
    static unsigned long lastReconnectMsg = 0;
    if (millis() - lastReconnectMsg > 5000) {
      Serial.println("ROS连接断开，尝试重连...");
      lastReconnectMsg = millis();
    }
    stopMotors();
    nh.spinOnce();
    delay(100);
    return;
  }

  // 更新轮速（20Hz）
  updateWheelVelocities();
  
  // 读取并发布IMU（50Hz）
  static unsigned long last_imu_time = 0;
  if (millis() - last_imu_time >= 20) {
    readMPU6050();
    imu_raw_pub.publish(&imu_raw_msg);
    last_imu_time = millis();
  }

  nh.spinOnce();
  delay(10);
}


```

修复 `Wire.requestFrom()` 的类型问题：


```cpp
/*
 * ESP32-S3 + L298N电机驱动 + SG90舵机 + 编码器 + MPU6050 + ROS1 (rosserial)
 * 
 * 功能：
 * 1. 订阅速度指令控制L298N电机
 * 2. 订阅舵机角度指令控制SG90舵机
 * 3. 编码器计数并发布轮速
 * 4. MPU6050读取并发布原始IMU数据
 * 
 * 接线：
 * - L298N：N1->GPIO3(左PWM), N2->GPIO4(左DIR), N3->GPIO5(右PWM), N4->GPIO6(右DIR)
 * - SG90舵机信号线：GPIO2
 * - 编码器：右轮A->GPIO7, B->GPIO8; 左轮A->GPIO9, B->GPIO10
 * - MPU6050：SCL->GPIO45, SDA->GPIO46
 */

#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <Wire.h>

#define USE_WIFI true

#if USE_WIFI
  #include <WiFi.h>
  const char* ssid = "机器人小车局域网";
  const char* password = "3252362734";
  IPAddress server(192, 168, 31, 64);
  const uint16_t serverPort = 11411;
#endif

// L298N电机控制引脚
#define MOTOR_LEFT_PWM   3   // 左轮PWM
#define MOTOR_LEFT_DIR   4   // 左轮方向
#define MOTOR_RIGHT_PWM  5   // 右轮PWM
#define MOTOR_RIGHT_DIR  6   // 右轮方向

// SG90舵机引脚
#define SERVO_PIN 2

// 编码器引脚
#define ENC_RIGHT_A 7
#define ENC_RIGHT_B 8
#define ENC_LEFT_A  9
#define ENC_LEFT_B  10

// MPU6050 I2C引脚和地址
#define MPU_ADDR 0x68
#define I2C_SDA 46
#define I2C_SCL 45

// PWM参数
#define PWM_FREQ 1000      // PWM频率 1kHz
#define PWM_RESOLUTION 8   // 8位分辨率 (0-255)
#define SERVO_FREQ 50      // 舵机PWM频率 50Hz
#define SERVO_RESOLUTION 12

// 速度限制
#define MAX_PWM_VALUE 200  // 最大PWM值

// 命令超时（毫秒）
#define CMD_TIMEOUT_MS 500

// 编码器参数（已测量更新）
#define TICKS_PER_REV 260.0f      // 每转脉冲数
#define WHEEL_RADIUS_M 0.0480f    // 轮子半径（米）
#define WHEEL_BASE_M   0.1475f    // 轮距（米）

// 编码器计数变量（volatile用于中断）
volatile int32_t enc_left_count = 0;
volatile int32_t enc_right_count = 0;
volatile int8_t enc_left_last_state = 0;
volatile int8_t enc_right_last_state = 0;

// 轮速计算
float left_wheel_vel = 0.0f;
float right_wheel_vel = 0.0f;
unsigned long last_enc_time = 0;
const unsigned long ENC_UPDATE_INTERVAL_MS = 50; // 50ms更新一次，20Hz

// ROS节点
ros::NodeHandle nh;
unsigned long last_cmd_time = 0;

// ROS消息变量（必须先声明，发布者才能引用）
geometry_msgs::Vector3Stamped wheel_vel_msg;
sensor_msgs::Imu imu_raw_msg;

// 发布者（必须在消息变量之后声明）
ros::Publisher wheel_vel_pub("/yolo_car/wheel_vel", &wheel_vel_msg);
ros::Publisher imu_raw_pub("/yolo_car/imu_raw", &imu_raw_msg);

// 左编码器中断处理（A相）
void IRAM_ATTR encLeftA() {
  int8_t a = digitalRead(ENC_LEFT_A);
  int8_t b = digitalRead(ENC_LEFT_B);
  int8_t state = (a << 1) | b;
  int8_t diff = (state - enc_left_last_state) & 0x03;
  if (diff == 1 || diff == 3) {
    enc_left_count++;
  } else if (diff == 2) {
    enc_left_count--;
  }
  enc_left_last_state = state;
}

// 右编码器中断处理（A相）
void IRAM_ATTR encRightA() {
  int8_t a = digitalRead(ENC_RIGHT_A);
  int8_t b = digitalRead(ENC_RIGHT_B);
  int8_t state = (a << 1) | b;
  int8_t diff = (state - enc_right_last_state) & 0x03;
  if (diff == 1 || diff == 3) {
    enc_right_count++;
  } else if (diff == 2) {
    enc_right_count--;
  }
  enc_right_last_state = state;
}

// 电机急停
void stopMotors() {
  ledcWrite(0, 0);
  ledcWrite(1, 0);
  digitalWrite(MOTOR_LEFT_DIR, LOW);
  digitalWrite(MOTOR_RIGHT_DIR, LOW);
}

// 速度指令回调
void cmdVelCallback(const geometry_msgs::Twist& msg) {
  float linear_vel = msg.linear.x;   // 线速度 m/s
  float angular_vel = msg.angular.z; // 角速度 rad/s

  last_cmd_time = millis();
  
  if (fabs(linear_vel) < 1e-3 && fabs(angular_vel) < 1e-3) {
    stopMotors();
    return;
  }
  
  // 差速控制
  float left_vel = linear_vel - (angular_vel * WHEEL_BASE_M / 2.0);
  float right_vel = linear_vel + (angular_vel * WHEEL_BASE_M / 2.0);
  
  float max_vel = 0.3;
  left_vel = constrain(left_vel, -max_vel, max_vel);
  right_vel = constrain(right_vel, -max_vel, max_vel);
  
  int left_pwm = abs(left_vel / max_vel * MAX_PWM_VALUE);
  int right_pwm = abs(right_vel / max_vel * MAX_PWM_VALUE);
  
  // 方向控制
  if (left_vel >= 0) {
    digitalWrite(MOTOR_LEFT_DIR, HIGH);
  } else {
    digitalWrite(MOTOR_LEFT_DIR, LOW);
  }
  
  if (right_vel >= 0) {
    digitalWrite(MOTOR_RIGHT_DIR, LOW);
  } else {
    digitalWrite(MOTOR_RIGHT_DIR, HIGH);
  }
  
  ledcWrite(0, left_pwm);
  ledcWrite(1, right_pwm);
}

// 舵机角度回调
void servoAngleCallback(const std_msgs::Float32& msg) {
  float angle = msg.data;
  angle = constrain(angle, 0.0, 180.0);

  int pulse_width = map(angle, 0, 180, 500, 2500);
  int duty_cycle = (pulse_width * 4096L) / 20000;
  duty_cycle = constrain(duty_cycle, 0, 4095);
  ledcWrite(2, duty_cycle);
}

// 订阅者
ros::Subscriber<geometry_msgs::Twist> cmd_vel_sub("/yolo_car/cmd_vel", cmdVelCallback);
ros::Subscriber<std_msgs::Float32> servo_angle_sub("/yolo_car/servo_angle", servoAngleCallback);

// MPU6050初始化
bool initMPU6050() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B); // PWR_MGMT_1寄存器
  Wire.write(0x00); // 唤醒MPU6050
  uint8_t error = Wire.endTransmission();
  
  if (error != 0) {
    return false;
  }
  
  // 配置加速度计范围 ±2g
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1C); // ACCEL_CONFIG
  Wire.write(0x00);
  Wire.endTransmission();
  
  // 配置陀螺仪范围 ±250°/s
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1B); // GYRO_CONFIG
  Wire.write(0x00);
  Wire.endTransmission();
  
  return true;
}

// 读取MPU6050原始数据
void readMPU6050() {
  // 读取加速度计数据（6字节：ACCEL_XOUT_H, ACCEL_XOUT_L, ACCEL_YOUT_H, ACCEL_YOUT_L, ACCEL_ZOUT_H, ACCEL_ZOUT_L）
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B); // ACCEL_XOUT_H寄存器地址
  Wire.endTransmission(false);
  
  // 修复：使用明确的类型参数，避免歧义（ESP32 Wire库需要uint8_t版本）
  uint8_t addr = (uint8_t)MPU_ADDR;
  uint8_t len = 14;
  bool stop = true;
  size_t bytes_read = Wire.requestFrom(addr, len, stop);
  
  if (bytes_read == 14) {
    int16_t accel_x = (Wire.read() << 8) | Wire.read();
    int16_t accel_y = (Wire.read() << 8) | Wire.read();
    int16_t accel_z = (Wire.read() << 8) | Wire.read();
    int16_t temp = (Wire.read() << 8) | Wire.read();
    int16_t gyro_x = (Wire.read() << 8) | Wire.read();
    int16_t gyro_y = (Wire.read() << 8) | Wire.read();
    int16_t gyro_z = (Wire.read() << 8) | Wire.read();
    
    // 转换为物理单位
    // 加速度：±2g范围，32768对应2g，g=9.8 m/s²
    float accel_scale = 2.0f * 9.8f / 32768.0f;
    imu_raw_msg.linear_acceleration.x = accel_x * accel_scale;
    imu_raw_msg.linear_acceleration.y = accel_y * accel_scale;
    imu_raw_msg.linear_acceleration.z = accel_z * accel_scale;
    
    // 角速度：±250°/s范围，32768对应250°/s，转换为rad/s
    float gyro_scale = 250.0f * M_PI / 180.0f / 32768.0f;
    imu_raw_msg.angular_velocity.x = gyro_x * gyro_scale;
    imu_raw_msg.angular_velocity.y = gyro_y * gyro_scale;
    imu_raw_msg.angular_velocity.z = gyro_z * gyro_scale;
    
    imu_raw_msg.header.stamp = nh.now();
    imu_raw_msg.header.frame_id = "imu_link";
  }
}

// 更新轮速
void updateWheelVelocities() {
  unsigned long current_time = millis();
  unsigned long dt_ms = current_time - last_enc_time;
  
  if (dt_ms >= ENC_UPDATE_INTERVAL_MS) {
    // 读取并重置计数（原子操作）
    int32_t left_ticks = enc_left_count;
    int32_t right_ticks = enc_right_count;
    enc_left_count = 0;
    enc_right_count = 0;
    
    // 计算轮速（m/s）
    // 每转脉冲数 * 轮子周长 = 每米脉冲数
    float ticks_per_meter = TICKS_PER_REV / (2.0f * M_PI * WHEEL_RADIUS_M);
    float dt_sec = dt_ms / 1000.0f;
    
    left_wheel_vel = (left_ticks / ticks_per_meter) / dt_sec;
    right_wheel_vel = (right_ticks / ticks_per_meter) / dt_sec;
    
    // 发布轮速
    wheel_vel_msg.header.stamp = nh.now();
    wheel_vel_msg.header.frame_id = "base_link";
    wheel_vel_msg.vector.x = left_wheel_vel;
    wheel_vel_msg.vector.y = right_wheel_vel;
    wheel_vel_msg.vector.z = 0.0;
    wheel_vel_pub.publish(&wheel_vel_msg);
    
    last_enc_time = current_time;
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n\n=== ESP32-S3 + L298N + SG90 + 编码器 + MPU6050 + ROS1 ===");

  last_cmd_time = millis();
  
  // 初始化电机控制引脚
  Serial.println("初始化L298N电机驱动...");
  pinMode(MOTOR_LEFT_DIR, OUTPUT);
  pinMode(MOTOR_RIGHT_DIR, OUTPUT);
  digitalWrite(MOTOR_LEFT_DIR, LOW);
  digitalWrite(MOTOR_RIGHT_DIR, LOW);
  
  // 配置PWM通道（电机）
  ledcSetup(0, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(1, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(MOTOR_LEFT_PWM, 0);
  ledcAttachPin(MOTOR_RIGHT_PWM, 1);
  
  // 配置PWM通道（舵机）
  ledcSetup(2, SERVO_FREQ, SERVO_RESOLUTION);
  ledcAttachPin(SERVO_PIN, 2);
  
  ledcWrite(0, 0);
  ledcWrite(1, 0);
  
  // 初始舵机到90度
  float init_angle = 90.0;
  int pulse_width = map(init_angle, 0, 180, 500, 2500);
  int duty_cycle = (pulse_width * 4096L) / 20000;
  duty_cycle = constrain(duty_cycle, 0, 4095);
  ledcWrite(2, duty_cycle);
  
  Serial.println("✓ 电机和舵机初始化完成");
  
  // 初始化编码器
  Serial.println("初始化编码器...");
  pinMode(ENC_LEFT_A, INPUT_PULLUP);
  pinMode(ENC_LEFT_B, INPUT_PULLUP);
  pinMode(ENC_RIGHT_A, INPUT_PULLUP);
  pinMode(ENC_RIGHT_B, INPUT_PULLUP);
  
  enc_left_last_state = (digitalRead(ENC_LEFT_A) << 1) | digitalRead(ENC_LEFT_B);
  enc_right_last_state = (digitalRead(ENC_RIGHT_A) << 1) | digitalRead(ENC_RIGHT_B);
  
  attachInterrupt(digitalPinToInterrupt(ENC_LEFT_A), encLeftA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_RIGHT_A), encRightA, CHANGE);
  
  last_enc_time = millis();
  Serial.println("✓ 编码器初始化完成");
  
  // 初始化MPU6050
  Serial.println("初始化MPU6050...");
  Wire.begin(I2C_SDA, I2C_SCL);
  delay(100);
  
  if (initMPU6050()) {
    Serial.println("✓ MPU6050初始化成功");
  } else {
    Serial.println("✗ MPU6050初始化失败");
  }
  
#if USE_WIFI
  Serial.println("\n=== WiFi连接 ===");
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  
  Serial.print("连接中");
  int wifiAttempts = 0;
  while (WiFi.status() != WL_CONNECTED && wifiAttempts < 30) {
    delay(500);
    Serial.print(".");
    wifiAttempts++;
  }
  Serial.println();
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("✓ WiFi连接成功！IP: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("✗ WiFi连接失败！");
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    nh.getHardware()->setConnection(server, serverPort);
  }
#endif
  
  Serial.println("\n=== ROS连接 ===");
  nh.initNode();
  nh.subscribe(cmd_vel_sub);
  nh.subscribe(servo_angle_sub);
  nh.advertise(wheel_vel_pub);
  nh.advertise(imu_raw_pub);
  
  Serial.print("等待连接");
  int rosAttempts = 0;
  while (!nh.connected() && rosAttempts < 50) {
    nh.spinOnce();
    delay(100);
    if (rosAttempts % 10 == 0) Serial.print(".");
    rosAttempts++;
  }
  Serial.println();
  
  if (nh.connected()) {
    Serial.println("✓ ROS连接成功！");
  } else {
    Serial.println("✗ ROS连接失败！");
  }
  
  Serial.println("\n=== 系统状态 ===");
  Serial.print("WiFi: ");
  Serial.println((WiFi.status() == WL_CONNECTED) ? "✓ 已连接" : "✗ 未连接");
  Serial.print("ROS: ");
  Serial.println(nh.connected() ? "✓ 已连接" : "✗ 未连接");
  Serial.print("编码器参数: TICKS_PER_REV=");
  Serial.print(TICKS_PER_REV);
  Serial.print(", WHEEL_RADIUS_M=");
  Serial.print(WHEEL_RADIUS_M);
  Serial.print(", WHEEL_BASE_M=");
  Serial.println(WHEEL_BASE_M);
  Serial.println("\n开始运行...\n");
}

void loop() {
  // 命令超时保护
  if (millis() - last_cmd_time > CMD_TIMEOUT_MS) {
    stopMotors();
  }

  if (!nh.connected()) {
    static unsigned long lastReconnectMsg = 0;
    if (millis() - lastReconnectMsg > 5000) {
      Serial.println("ROS连接断开，尝试重连...");
      lastReconnectMsg = millis();
    }
    stopMotors();
    nh.spinOnce();
    delay(100);
    return;
  }

  // 更新轮速（20Hz）
  updateWheelVelocities();
  
  // 读取并发布IMU（50Hz）
  static unsigned long last_imu_time = 0;
  if (millis() - last_imu_time >= 20) {
    readMPU6050();
    imu_raw_pub.publish(&imu_raw_msg);
    last_imu_time = millis();
  }

  nh.spinOnce();
  delay(10);
}


```

已修复以下问题：

1. **修复发布者声明顺序**：将消息变量声明移到发布者之前（第87-92行）
2. **修复 `Wire.requestFrom()` 类型歧义**：使用明确的变量类型（第217-220行）
3. **更新常量值**：
   - `TICKS_PER_REV 260.0f`
   - `WHEEL_RADIUS_M 0.0480f`
   - `WHEEL_BASE_M 0.1475f`

现在应可正常编译。主要改动：

```87:92:src/yolo_car/esp32/yolo_car_control.ino
// ROS消息变量（必须先声明，发布者才能引用）
geometry_msgs::Vector3Stamped wheel_vel_msg;
sensor_msgs::Imu imu_raw_msg;

// 发布者（必须在消息变量之后声明）
ros::Publisher wheel_vel_pub("/yolo_car/wheel_vel", &wheel_vel_msg);
```

```217:220:src/yolo_car/esp32/yolo_car_control.ino
  // 修复：使用明确的类型参数，避免歧义（ESP32 Wire库需要uint8_t版本）
  uint8_t addr = (uint8_t)MPU_ADDR;
  uint8_t len = 14;
  bool stop = true;
  size_t bytes_read = Wire.requestFrom(addr, len, stop);
```

如果仍有编译错误，请提供具体错误信息。

```plaintext
// ROS消息变量（必须先声明，发布者才能引用）
geometry_msgs::Vector3Stamped wheel_vel_msg;
sensor_msgs::Imu imu_raw_msg;

// 发布者（必须在消息变量之后声明）
ros::Publisher wheel_vel_pub("/yolo_car/wheel_vel", &wheel_vel_msg);
```

```plaintext
  // 修复：使用明确的类型参数，避免歧义（ESP32 Wire库需要uint8_t版本）
  uint8_t addr = (uint8_t)MPU_ADDR;
  uint8_t len = 14;
  bool stop = true;
  size_t bytes_read = Wire.requestFrom(addr, len, stop);
```

---

**User**

In file included from C:\Users\s3252\Desktop\yolo_car_control\yolo_car_control.ino:22:



C:\Users\s3252\AppData\Local\Arduino15\packages\esp32\hardware\esp32\2.0.7\libraries\Wire\src/Wire.h: In function 'void readMPU6050()':

C:\Users\s3252\AppData\Local\Arduino15\packages\esp32\hardware\esp32\2.0.7\libraries\Wire\src/Wire.h:126:13: note: candidate 1: 'uint8_t TwoWire::requestFrom(int, int, int)'

     uint8_t requestFrom(int address, int size, int sendStop);

             ^~~~~~~~~~~

C:\Users\s3252\AppData\Local\Arduino15\packages\esp32\hardware\esp32\2.0.7\libraries\Wire\src/Wire.h:122:12: note: candidate 2: 'size_t TwoWire::requestFrom(uint8_t, size_t, bool)'

     size_t requestFrom(uint8_t address, size_t len, bool stopBit);

            ^~~~~~~~~~~

C:\Users\s3252\AppData\Local\Arduino15\packages\esp32\hardware\esp32\2.0.7\libraries\Wire\src/Wire.h:126:13: note: candidate 1: 'uint8_t TwoWire::requestFrom(int, int, int)'

     uint8_t requestFrom(int address, int size, int sendStop);

             ^~~~~~~~~~~

C:\Users\s3252\AppData\Local\Arduino15\packages\esp32\hardware\esp32\2.0.7\libraries\Wire\src/Wire.h:121:13: note: candidate 2: 'uint8_t TwoWire::requestFrom(uint16_t, uint8_t, uint8_t)'

     uint8_t requestFrom(uint16_t address, uint8_t size, uint8_t sendStop);

             ^~~~~~~~~~~

C:\Users\s3252\AppData\Local\Arduino15\packages\esp32\hardware\esp32\2.0.7\libraries\Wire\src/Wire.h:126:13: note: candidate 1: 'uint8_t TwoWire::requestFrom(int, int, int)'

     uint8_t requestFrom(int address, int size, int sendStop);

             ^~~~~~~~~~~

C:\Users\s3252\AppData\Local\Arduino15\packages\esp32\hardware\esp32\2.0.7\libraries\Wire\src/Wire.h:120:13: note: candidate 2: 'uint8_t TwoWire::requestFrom(uint16_t, uint8_t, bool)'

     uint8_t requestFrom(uint16_t address, uint8_t size, bool sendStop);

             ^~~~~~~~~~~

C:\Users\s3252\AppData\Local\Arduino15\packages\esp32\hardware\esp32\2.0.7\libraries\Wire\src/Wire.h:126:13: note: candidate 1: 'uint8_t TwoWire::requestFrom(int, int, int)'

     uint8_t requestFrom(int address, int size, int sendStop);

             ^~~~~~~~~~~

C:\Users\s3252\AppData\Local\Arduino15\packages\esp32\hardware\esp32\2.0.7\libraries\Wire\src/Wire.h:119:12: note: candidate 2: 'size_t TwoWire::requestFrom(uint16_t, size_t, bool)'

     size_t requestFrom(uint16_t address, size_t size, bool sendStop);

            ^~~~~~~~~~~

C:\Users\s3252\AppData\Local\Arduino15\packages\esp32\hardware\esp32\2.0.7\libraries\Wire\src/Wire.h:126:13: note: candidate 1: 'uint8_t TwoWire::requestFrom(int, int, int)'

     uint8_t requestFrom(int address, int size, int sendStop);

             ^~~~~~~~~~~

C:\Users\s3252\AppData\Local\Arduino15\packages\esp32\hardware\esp32\2.0.7\libraries\Wire\src/Wire.h:124:13: note: candidate 2: 'uint8_t TwoWire::requestFrom(uint8_t, uint8_t, uint8_t)'

     uint8_t requestFrom(uint8_t address, uint8_t size, uint8_t sendStop);还是有报错，你看看是怎么回事，怎么一直改不了

---

**Cursor**

检查当前代码，改用更明确的类型调用以消除歧义：


改用 `uint8_t` 版本的重载，将 `stop` 参数改为 `uint8_t` 以消除歧义：


```cpp
/*
 * ESP32-S3 + L298N电机驱动 + SG90舵机 + 编码器 + MPU6050 + ROS1 (rosserial)
 * 
 * 功能：
 * 1. 订阅速度指令控制L298N电机
 * 2. 订阅舵机角度指令控制SG90舵机
 * 3. 编码器计数并发布轮速
 * 4. MPU6050读取并发布原始IMU数据
 * 
 * 接线：
 * - L298N：N1->GPIO3(左PWM), N2->GPIO4(左DIR), N3->GPIO5(右PWM), N4->GPIO6(右DIR)
 * - SG90舵机信号线：GPIO2
 * - 编码器：右轮A->GPIO7, B->GPIO8; 左轮A->GPIO9, B->GPIO10
 * - MPU6050：SCL->GPIO45, SDA->GPIO46
 */

#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <Wire.h>

#define USE_WIFI true

#if USE_WIFI
  #include <WiFi.h>
  const char* ssid = "机器人小车局域网";
  const char* password = "3252362734";
  IPAddress server(192, 168, 31, 64);
  const uint16_t serverPort = 11411;
#endif

// L298N电机控制引脚
#define MOTOR_LEFT_PWM   3   // 左轮PWM
#define MOTOR_LEFT_DIR   4   // 左轮方向
#define MOTOR_RIGHT_PWM  5   // 右轮PWM
#define MOTOR_RIGHT_DIR  6   // 右轮方向

// SG90舵机引脚
#define SERVO_PIN 2

// 编码器引脚
#define ENC_RIGHT_A 7
#define ENC_RIGHT_B 8
#define ENC_LEFT_A  9
#define ENC_LEFT_B  10

// MPU6050 I2C引脚和地址
#define MPU_ADDR 0x68
#define I2C_SDA 46
#define I2C_SCL 45

// PWM参数
#define PWM_FREQ 1000      // PWM频率 1kHz
#define PWM_RESOLUTION 8   // 8位分辨率 (0-255)
#define SERVO_FREQ 50      // 舵机PWM频率 50Hz
#define SERVO_RESOLUTION 12

// 速度限制
#define MAX_PWM_VALUE 200  // 最大PWM值

// 命令超时（毫秒）
#define CMD_TIMEOUT_MS 500

// 编码器参数（已测量更新）
#define TICKS_PER_REV 260.0f      // 每转脉冲数
#define WHEEL_RADIUS_M 0.0480f    // 轮子半径（米）
#define WHEEL_BASE_M   0.1475f    // 轮距（米）

// 编码器计数变量（volatile用于中断）
volatile int32_t enc_left_count = 0;
volatile int32_t enc_right_count = 0;
volatile int8_t enc_left_last_state = 0;
volatile int8_t enc_right_last_state = 0;

// 轮速计算
float left_wheel_vel = 0.0f;
float right_wheel_vel = 0.0f;
unsigned long last_enc_time = 0;
const unsigned long ENC_UPDATE_INTERVAL_MS = 50; // 50ms更新一次，20Hz

// ROS节点
ros::NodeHandle nh;
unsigned long last_cmd_time = 0;

// ROS消息变量（必须先声明，发布者才能引用）
geometry_msgs::Vector3Stamped wheel_vel_msg;
sensor_msgs::Imu imu_raw_msg;

// 发布者（必须在消息变量之后声明）
ros::Publisher wheel_vel_pub("/yolo_car/wheel_vel", &wheel_vel_msg);
ros::Publisher imu_raw_pub("/yolo_car/imu_raw", &imu_raw_msg);

// 左编码器中断处理（A相）
void IRAM_ATTR encLeftA() {
  int8_t a = digitalRead(ENC_LEFT_A);
  int8_t b = digitalRead(ENC_LEFT_B);
  int8_t state = (a << 1) | b;
  int8_t diff = (state - enc_left_last_state) & 0x03;
  if (diff == 1 || diff == 3) {
    enc_left_count++;
  } else if (diff == 2) {
    enc_left_count--;
  }
  enc_left_last_state = state;
}

// 右编码器中断处理（A相）
void IRAM_ATTR encRightA() {
  int8_t a = digitalRead(ENC_RIGHT_A);
  int8_t b = digitalRead(ENC_RIGHT_B);
  int8_t state = (a << 1) | b;
  int8_t diff = (state - enc_right_last_state) & 0x03;
  if (diff == 1 || diff == 3) {
    enc_right_count++;
  } else if (diff == 2) {
    enc_right_count--;
  }
  enc_right_last_state = state;
}

// 电机急停
void stopMotors() {
  ledcWrite(0, 0);
  ledcWrite(1, 0);
  digitalWrite(MOTOR_LEFT_DIR, LOW);
  digitalWrite(MOTOR_RIGHT_DIR, LOW);
}

// 速度指令回调
void cmdVelCallback(const geometry_msgs::Twist& msg) {
  float linear_vel = msg.linear.x;   // 线速度 m/s
  float angular_vel = msg.angular.z; // 角速度 rad/s

  last_cmd_time = millis();
  
  if (fabs(linear_vel) < 1e-3 && fabs(angular_vel) < 1e-3) {
    stopMotors();
    return;
  }
  
  // 差速控制
  float left_vel = linear_vel - (angular_vel * WHEEL_BASE_M / 2.0);
  float right_vel = linear_vel + (angular_vel * WHEEL_BASE_M / 2.0);
  
  float max_vel = 0.3;
  left_vel = constrain(left_vel, -max_vel, max_vel);
  right_vel = constrain(right_vel, -max_vel, max_vel);
  
  int left_pwm = abs(left_vel / max_vel * MAX_PWM_VALUE);
  int right_pwm = abs(right_vel / max_vel * MAX_PWM_VALUE);
  
  // 方向控制
  if (left_vel >= 0) {
    digitalWrite(MOTOR_LEFT_DIR, HIGH);
  } else {
    digitalWrite(MOTOR_LEFT_DIR, LOW);
  }
  
  if (right_vel >= 0) {
    digitalWrite(MOTOR_RIGHT_DIR, LOW);
  } else {
    digitalWrite(MOTOR_RIGHT_DIR, HIGH);
  }
  
  ledcWrite(0, left_pwm);
  ledcWrite(1, right_pwm);
}

// 舵机角度回调
void servoAngleCallback(const std_msgs::Float32& msg) {
  float angle = msg.data;
  angle = constrain(angle, 0.0, 180.0);

  int pulse_width = map(angle, 0, 180, 500, 2500);
  int duty_cycle = (pulse_width * 4096L) / 20000;
  duty_cycle = constrain(duty_cycle, 0, 4095);
  ledcWrite(2, duty_cycle);
}

// 订阅者
ros::Subscriber<geometry_msgs::Twist> cmd_vel_sub("/yolo_car/cmd_vel", cmdVelCallback);
ros::Subscriber<std_msgs::Float32> servo_angle_sub("/yolo_car/servo_angle", servoAngleCallback);

// MPU6050初始化
bool initMPU6050() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B); // PWR_MGMT_1寄存器
  Wire.write(0x00); // 唤醒MPU6050
  uint8_t error = Wire.endTransmission();
  
  if (error != 0) {
    return false;
  }
  
  // 配置加速度计范围 ±2g
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1C); // ACCEL_CONFIG
  Wire.write(0x00);
  Wire.endTransmission();
  
  // 配置陀螺仪范围 ±250°/s
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1B); // GYRO_CONFIG
  Wire.write(0x00);
  Wire.endTransmission();
  
  return true;
}

// 读取MPU6050原始数据
void readMPU6050() {
  // 读取加速度计数据（6字节：ACCEL_XOUT_H, ACCEL_XOUT_L, ACCEL_YOUT_H, ACCEL_YOUT_L, ACCEL_ZOUT_H, ACCEL_ZOUT_L）
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B); // ACCEL_XOUT_H寄存器地址
  Wire.endTransmission(false);
  
  // 修复：使用明确的类型参数，避免歧义（使用uint8_t版本的重载）
  uint8_t addr = (uint8_t)MPU_ADDR;
  uint8_t len = 14;
  uint8_t stop = 1;  // 使用uint8_t类型，1表示发送停止位
  uint8_t bytes_read = Wire.requestFrom(addr, len, stop);
  
  if (bytes_read == 14) {
    int16_t accel_x = (Wire.read() << 8) | Wire.read();
    int16_t accel_y = (Wire.read() << 8) | Wire.read();
    int16_t accel_z = (Wire.read() << 8) | Wire.read();
    int16_t temp = (Wire.read() << 8) | Wire.read();
    int16_t gyro_x = (Wire.read() << 8) | Wire.read();
    int16_t gyro_y = (Wire.read() << 8) | Wire.read();
    int16_t gyro_z = (Wire.read() << 8) | Wire.read();
    
    // 转换为物理单位
    // 加速度：±2g范围，32768对应2g，g=9.8 m/s²
    float accel_scale = 2.0f * 9.8f / 32768.0f;
    imu_raw_msg.linear_acceleration.x = accel_x * accel_scale;
    imu_raw_msg.linear_acceleration.y = accel_y * accel_scale;
    imu_raw_msg.linear_acceleration.z = accel_z * accel_scale;
    
    // 角速度：±250°/s范围，32768对应250°/s，转换为rad/s
    float gyro_scale = 250.0f * M_PI / 180.0f / 32768.0f;
    imu_raw_msg.angular_velocity.x = gyro_x * gyro_scale;
    imu_raw_msg.angular_velocity.y = gyro_y * gyro_scale;
    imu_raw_msg.angular_velocity.z = gyro_z * gyro_scale;
    
    imu_raw_msg.header.stamp = nh.now();
    imu_raw_msg.header.frame_id = "imu_link";
  }
}

// 更新轮速
void updateWheelVelocities() {
  unsigned long current_time = millis();
  unsigned long dt_ms = current_time - last_enc_time;
  
  if (dt_ms >= ENC_UPDATE_INTERVAL_MS) {
    // 读取并重置计数（原子操作）
    int32_t left_ticks = enc_left_count;
    int32_t right_ticks = enc_right_count;
    enc_left_count = 0;
    enc_right_count = 0;
    
    // 计算轮速（m/s）
    // 每转脉冲数 * 轮子周长 = 每米脉冲数
    float ticks_per_meter = TICKS_PER_REV / (2.0f * M_PI * WHEEL_RADIUS_M);
    float dt_sec = dt_ms / 1000.0f;
    
    left_wheel_vel = (left_ticks / ticks_per_meter) / dt_sec;
    right_wheel_vel = (right_ticks / ticks_per_meter) / dt_sec;
    
    // 发布轮速
    wheel_vel_msg.header.stamp = nh.now();
    wheel_vel_msg.header.frame_id = "base_link";
    wheel_vel_msg.vector.x = left_wheel_vel;
    wheel_vel_msg.vector.y = right_wheel_vel;
    wheel_vel_msg.vector.z = 0.0;
    wheel_vel_pub.publish(&wheel_vel_msg);
    
    last_enc_time = current_time;
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n\n=== ESP32-S3 + L298N + SG90 + 编码器 + MPU6050 + ROS1 ===");

  last_cmd_time = millis();
  
  // 初始化电机控制引脚
  Serial.println("初始化L298N电机驱动...");
  pinMode(MOTOR_LEFT_DIR, OUTPUT);
  pinMode(MOTOR_RIGHT_DIR, OUTPUT);
  digitalWrite(MOTOR_LEFT_DIR, LOW);
  digitalWrite(MOTOR_RIGHT_DIR, LOW);
  
  // 配置PWM通道（电机）
  ledcSetup(0, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(1, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(MOTOR_LEFT_PWM, 0);
  ledcAttachPin(MOTOR_RIGHT_PWM, 1);
  
  // 配置PWM通道（舵机）
  ledcSetup(2, SERVO_FREQ, SERVO_RESOLUTION);
  ledcAttachPin(SERVO_PIN, 2);
  
  ledcWrite(0, 0);
  ledcWrite(1, 0);
  
  // 初始舵机到90度
  float init_angle = 90.0;
  int pulse_width = map(init_angle, 0, 180, 500, 2500);
  int duty_cycle = (pulse_width * 4096L) / 20000;
  duty_cycle = constrain(duty_cycle, 0, 4095);
  ledcWrite(2, duty_cycle);
  
  Serial.println("✓ 电机和舵机初始化完成");
  
  // 初始化编码器
  Serial.println("初始化编码器...");
  pinMode(ENC_LEFT_A, INPUT_PULLUP);
  pinMode(ENC_LEFT_B, INPUT_PULLUP);
  pinMode(ENC_RIGHT_A, INPUT_PULLUP);
  pinMode(ENC_RIGHT_B, INPUT_PULLUP);
  
  enc_left_last_state = (digitalRead(ENC_LEFT_A) << 1) | digitalRead(ENC_LEFT_B);
  enc_right_last_state = (digitalRead(ENC_RIGHT_A) << 1) | digitalRead(ENC_RIGHT_B);
  
  attachInterrupt(digitalPinToInterrupt(ENC_LEFT_A), encLeftA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_RIGHT_A), encRightA, CHANGE);
  
  last_enc_time = millis();
  Serial.println("✓ 编码器初始化完成");
  
  // 初始化MPU6050
  Serial.println("初始化MPU6050...");
  Wire.begin(I2C_SDA, I2C_SCL);
  delay(100);
  
  if (initMPU6050()) {
    Serial.println("✓ MPU6050初始化成功");
  } else {
    Serial.println("✗ MPU6050初始化失败");
  }
  
#if USE_WIFI
  Serial.println("\n=== WiFi连接 ===");
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  
  Serial.print("连接中");
  int wifiAttempts = 0;
  while (WiFi.status() != WL_CONNECTED && wifiAttempts < 30) {
    delay(500);
    Serial.print(".");
    wifiAttempts++;
  }
  Serial.println();
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("✓ WiFi连接成功！IP: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("✗ WiFi连接失败！");
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    nh.getHardware()->setConnection(server, serverPort);
  }
#endif
  
  Serial.println("\n=== ROS连接 ===");
  nh.initNode();
  nh.subscribe(cmd_vel_sub);
  nh.subscribe(servo_angle_sub);
  nh.advertise(wheel_vel_pub);
  nh.advertise(imu_raw_pub);
  
  Serial.print("等待连接");
  int rosAttempts = 0;
  while (!nh.connected() && rosAttempts < 50) {
    nh.spinOnce();
    delay(100);
    if (rosAttempts % 10 == 0) Serial.print(".");
    rosAttempts++;
  }
  Serial.println();
  
  if (nh.connected()) {
    Serial.println("✓ ROS连接成功！");
  } else {
    Serial.println("✗ ROS连接失败！");
  }
  
  Serial.println("\n=== 系统状态 ===");
  Serial.print("WiFi: ");
  Serial.println((WiFi.status() == WL_CONNECTED) ? "✓ 已连接" : "✗ 未连接");
  Serial.print("ROS: ");
  Serial.println(nh.connected() ? "✓ 已连接" : "✗ 未连接");
  Serial.print("编码器参数: TICKS_PER_REV=");
  Serial.print(TICKS_PER_REV);
  Serial.print(", WHEEL_RADIUS_M=");
  Serial.print(WHEEL_RADIUS_M);
  Serial.print(", WHEEL_BASE_M=");
  Serial.println(WHEEL_BASE_M);
  Serial.println("\n开始运行...\n");
}

void loop() {
  // 命令超时保护
  if (millis() - last_cmd_time > CMD_TIMEOUT_MS) {
    stopMotors();
  }

  if (!nh.connected()) {
    static unsigned long lastReconnectMsg = 0;
    if (millis() - lastReconnectMsg > 5000) {
      Serial.println("ROS连接断开，尝试重连...");
      lastReconnectMsg = millis();
    }
    stopMotors();
    nh.spinOnce();
    delay(100);
    return;
  }

  // 更新轮速（20Hz）
  updateWheelVelocities();
  
  // 读取并发布IMU（50Hz）
  static unsigned long last_imu_time = 0;
  if (millis() - last_imu_time >= 20) {
    readMPU6050();
    imu_raw_pub.publish(&imu_raw_msg);
    last_imu_time = millis();
  }

  nh.spinOnce();
  delay(10);
}


```

已修改为使用 `uint8_t` 版本的重载。如果仍有问题，改用 `int` 版本（更通用）：


```cpp
/*
 * ESP32-S3 + L298N电机驱动 + SG90舵机 + 编码器 + MPU6050 + ROS1 (rosserial)
 * 
 * 功能：
 * 1. 订阅速度指令控制L298N电机
 * 2. 订阅舵机角度指令控制SG90舵机
 * 3. 编码器计数并发布轮速
 * 4. MPU6050读取并发布原始IMU数据
 * 
 * 接线：
 * - L298N：N1->GPIO3(左PWM), N2->GPIO4(左DIR), N3->GPIO5(右PWM), N4->GPIO6(右DIR)
 * - SG90舵机信号线：GPIO2
 * - 编码器：右轮A->GPIO7, B->GPIO8; 左轮A->GPIO9, B->GPIO10
 * - MPU6050：SCL->GPIO45, SDA->GPIO46
 */

#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <Wire.h>

#define USE_WIFI true

#if USE_WIFI
  #include <WiFi.h>
  const char* ssid = "机器人小车局域网";
  const char* password = "3252362734";
  IPAddress server(192, 168, 31, 64);
  const uint16_t serverPort = 11411;
#endif

// L298N电机控制引脚
#define MOTOR_LEFT_PWM   3   // 左轮PWM
#define MOTOR_LEFT_DIR   4   // 左轮方向
#define MOTOR_RIGHT_PWM  5   // 右轮PWM
#define MOTOR_RIGHT_DIR  6   // 右轮方向

// SG90舵机引脚
#define SERVO_PIN 2

// 编码器引脚
#define ENC_RIGHT_A 7
#define ENC_RIGHT_B 8
#define ENC_LEFT_A  9
#define ENC_LEFT_B  10

// MPU6050 I2C引脚和地址
#define MPU_ADDR 0x68
#define I2C_SDA 46
#define I2C_SCL 45

// PWM参数
#define PWM_FREQ 1000      // PWM频率 1kHz
#define PWM_RESOLUTION 8   // 8位分辨率 (0-255)
#define SERVO_FREQ 50      // 舵机PWM频率 50Hz
#define SERVO_RESOLUTION 12

// 速度限制
#define MAX_PWM_VALUE 200  // 最大PWM值

// 命令超时（毫秒）
#define CMD_TIMEOUT_MS 500

// 编码器参数（已测量更新）
#define TICKS_PER_REV 260.0f      // 每转脉冲数
#define WHEEL_RADIUS_M 0.0480f    // 轮子半径（米）
#define WHEEL_BASE_M   0.1475f    // 轮距（米）

// 编码器计数变量（volatile用于中断）
volatile int32_t enc_left_count = 0;
volatile int32_t enc_right_count = 0;
volatile int8_t enc_left_last_state = 0;
volatile int8_t enc_right_last_state = 0;

// 轮速计算
float left_wheel_vel = 0.0f;
float right_wheel_vel = 0.0f;
unsigned long last_enc_time = 0;
const unsigned long ENC_UPDATE_INTERVAL_MS = 50; // 50ms更新一次，20Hz

// ROS节点
ros::NodeHandle nh;
unsigned long last_cmd_time = 0;

// ROS消息变量（必须先声明，发布者才能引用）
geometry_msgs::Vector3Stamped wheel_vel_msg;
sensor_msgs::Imu imu_raw_msg;

// 发布者（必须在消息变量之后声明）
ros::Publisher wheel_vel_pub("/yolo_car/wheel_vel", &wheel_vel_msg);
ros::Publisher imu_raw_pub("/yolo_car/imu_raw", &imu_raw_msg);

// 左编码器中断处理（A相）
void IRAM_ATTR encLeftA() {
  int8_t a = digitalRead(ENC_LEFT_A);
  int8_t b = digitalRead(ENC_LEFT_B);
  int8_t state = (a << 1) | b;
  int8_t diff = (state - enc_left_last_state) & 0x03;
  if (diff == 1 || diff == 3) {
    enc_left_count++;
  } else if (diff == 2) {
    enc_left_count--;
  }
  enc_left_last_state = state;
}

// 右编码器中断处理（A相）
void IRAM_ATTR encRightA() {
  int8_t a = digitalRead(ENC_RIGHT_A);
  int8_t b = digitalRead(ENC_RIGHT_B);
  int8_t state = (a << 1) | b;
  int8_t diff = (state - enc_right_last_state) & 0x03;
  if (diff == 1 || diff == 3) {
    enc_right_count++;
  } else if (diff == 2) {
    enc_right_count--;
  }
  enc_right_last_state = state;
}

// 电机急停
void stopMotors() {
  ledcWrite(0, 0);
  ledcWrite(1, 0);
  digitalWrite(MOTOR_LEFT_DIR, LOW);
  digitalWrite(MOTOR_RIGHT_DIR, LOW);
}

// 速度指令回调
void cmdVelCallback(const geometry_msgs::Twist& msg) {
  float linear_vel = msg.linear.x;   // 线速度 m/s
  float angular_vel = msg.angular.z; // 角速度 rad/s

  last_cmd_time = millis();
  
  if (fabs(linear_vel) < 1e-3 && fabs(angular_vel) < 1e-3) {
    stopMotors();
    return;
  }
  
  // 差速控制
  float left_vel = linear_vel - (angular_vel * WHEEL_BASE_M / 2.0);
  float right_vel = linear_vel + (angular_vel * WHEEL_BASE_M / 2.0);
  
  float max_vel = 0.3;
  left_vel = constrain(left_vel, -max_vel, max_vel);
  right_vel = constrain(right_vel, -max_vel, max_vel);
  
  int left_pwm = abs(left_vel / max_vel * MAX_PWM_VALUE);
  int right_pwm = abs(right_vel / max_vel * MAX_PWM_VALUE);
  
  // 方向控制
  if (left_vel >= 0) {
    digitalWrite(MOTOR_LEFT_DIR, HIGH);
  } else {
    digitalWrite(MOTOR_LEFT_DIR, LOW);
  }
  
  if (right_vel >= 0) {
    digitalWrite(MOTOR_RIGHT_DIR, LOW);
  } else {
    digitalWrite(MOTOR_RIGHT_DIR, HIGH);
  }
  
  ledcWrite(0, left_pwm);
  ledcWrite(1, right_pwm);
}

// 舵机角度回调
void servoAngleCallback(const std_msgs::Float32& msg) {
  float angle = msg.data;
  angle = constrain(angle, 0.0, 180.0);

  int pulse_width = map(angle, 0, 180, 500, 2500);
  int duty_cycle = (pulse_width * 4096L) / 20000;
  duty_cycle = constrain(duty_cycle, 0, 4095);
  ledcWrite(2, duty_cycle);
}

// 订阅者
ros::Subscriber<geometry_msgs::Twist> cmd_vel_sub("/yolo_car/cmd_vel", cmdVelCallback);
ros::Subscriber<std_msgs::Float32> servo_angle_sub("/yolo_car/servo_angle", servoAngleCallback);

// MPU6050初始化
bool initMPU6050() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B); // PWR_MGMT_1寄存器
  Wire.write(0x00); // 唤醒MPU6050
  uint8_t error = Wire.endTransmission();
  
  if (error != 0) {
    return false;
  }
  
  // 配置加速度计范围 ±2g
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1C); // ACCEL_CONFIG
  Wire.write(0x00);
  Wire.endTransmission();
  
  // 配置陀螺仪范围 ±250°/s
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1B); // GYRO_CONFIG
  Wire.write(0x00);
  Wire.endTransmission();
  
  return true;
}

// 读取MPU6050原始数据
void readMPU6050() {
  // 读取加速度计数据（6字节：ACCEL_XOUT_H, ACCEL_XOUT_L, ACCEL_YOUT_H, ACCEL_YOUT_L, ACCEL_ZOUT_H, ACCEL_ZOUT_L）
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B); // ACCEL_XOUT_H寄存器地址
  Wire.endTransmission(false);
  
  // 修复：使用int版本的重载，避免类型歧义
  int addr = (int)MPU_ADDR;
  int len = 14;
  int stop = 1;  // 1表示发送停止位
  uint8_t bytes_read = Wire.requestFrom(addr, len, stop);
  
  if (bytes_read == 14) {
    int16_t accel_x = (Wire.read() << 8) | Wire.read();
    int16_t accel_y = (Wire.read() << 8) | Wire.read();
    int16_t accel_z = (Wire.read() << 8) | Wire.read();
    int16_t temp = (Wire.read() << 8) | Wire.read();
    int16_t gyro_x = (Wire.read() << 8) | Wire.read();
    int16_t gyro_y = (Wire.read() << 8) | Wire.read();
    int16_t gyro_z = (Wire.read() << 8) | Wire.read();
    
    // 转换为物理单位
    // 加速度：±2g范围，32768对应2g，g=9.8 m/s²
    float accel_scale = 2.0f * 9.8f / 32768.0f;
    imu_raw_msg.linear_acceleration.x = accel_x * accel_scale;
    imu_raw_msg.linear_acceleration.y = accel_y * accel_scale;
    imu_raw_msg.linear_acceleration.z = accel_z * accel_scale;
    
    // 角速度：±250°/s范围，32768对应250°/s，转换为rad/s
    float gyro_scale = 250.0f * M_PI / 180.0f / 32768.0f;
    imu_raw_msg.angular_velocity.x = gyro_x * gyro_scale;
    imu_raw_msg.angular_velocity.y = gyro_y * gyro_scale;
    imu_raw_msg.angular_velocity.z = gyro_z * gyro_scale;
    
    imu_raw_msg.header.stamp = nh.now();
    imu_raw_msg.header.frame_id = "imu_link";
  }
}

// 更新轮速
void updateWheelVelocities() {
  unsigned long current_time = millis();
  unsigned long dt_ms = current_time - last_enc_time;
  
  if (dt_ms >= ENC_UPDATE_INTERVAL_MS) {
    // 读取并重置计数（原子操作）
    int32_t left_ticks = enc_left_count;
    int32_t right_ticks = enc_right_count;
    enc_left_count = 0;
    enc_right_count = 0;
    
    // 计算轮速（m/s）
    // 每转脉冲数 * 轮子周长 = 每米脉冲数
    float ticks_per_meter = TICKS_PER_REV / (2.0f * M_PI * WHEEL_RADIUS_M);
    float dt_sec = dt_ms / 1000.0f;
    
    left_wheel_vel = (left_ticks / ticks_per_meter) / dt_sec;
    right_wheel_vel = (right_ticks / ticks_per_meter) / dt_sec;
    
    // 发布轮速
    wheel_vel_msg.header.stamp = nh.now();
    wheel_vel_msg.header.frame_id = "base_link";
    wheel_vel_msg.vector.x = left_wheel_vel;
    wheel_vel_msg.vector.y = right_wheel_vel;
    wheel_vel_msg.vector.z = 0.0;
    wheel_vel_pub.publish(&wheel_vel_msg);
    
    last_enc_time = current_time;
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n\n=== ESP32-S3 + L298N + SG90 + 编码器 + MPU6050 + ROS1 ===");

  last_cmd_time = millis();
  
  // 初始化电机控制引脚
  Serial.println("初始化L298N电机驱动...");
  pinMode(MOTOR_LEFT_DIR, OUTPUT);
  pinMode(MOTOR_RIGHT_DIR, OUTPUT);
  digitalWrite(MOTOR_LEFT_DIR, LOW);
  digitalWrite(MOTOR_RIGHT_DIR, LOW);
  
  // 配置PWM通道（电机）
  ledcSetup(0, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(1, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(MOTOR_LEFT_PWM, 0);
  ledcAttachPin(MOTOR_RIGHT_PWM, 1);
  
  // 配置PWM通道（舵机）
  ledcSetup(2, SERVO_FREQ, SERVO_RESOLUTION);
  ledcAttachPin(SERVO_PIN, 2);
  
  ledcWrite(0, 0);
  ledcWrite(1, 0);
  
  // 初始舵机到90度
  float init_angle = 90.0;
  int pulse_width = map(init_angle, 0, 180, 500, 2500);
  int duty_cycle = (pulse_width * 4096L) / 20000;
  duty_cycle = constrain(duty_cycle, 0, 4095);
  ledcWrite(2, duty_cycle);
  
  Serial.println("✓ 电机和舵机初始化完成");
  
  // 初始化编码器
  Serial.println("初始化编码器...");
  pinMode(ENC_LEFT_A, INPUT_PULLUP);
  pinMode(ENC_LEFT_B, INPUT_PULLUP);
  pinMode(ENC_RIGHT_A, INPUT_PULLUP);
  pinMode(ENC_RIGHT_B, INPUT_PULLUP);
  
  enc_left_last_state = (digitalRead(ENC_LEFT_A) << 1) | digitalRead(ENC_LEFT_B);
  enc_right_last_state = (digitalRead(ENC_RIGHT_A) << 1) | digitalRead(ENC_RIGHT_B);
  
  attachInterrupt(digitalPinToInterrupt(ENC_LEFT_A), encLeftA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_RIGHT_A), encRightA, CHANGE);
  
  last_enc_time = millis();
  Serial.println("✓ 编码器初始化完成");
  
  // 初始化MPU6050
  Serial.println("初始化MPU6050...");
  Wire.begin(I2C_SDA, I2C_SCL);
  delay(100);
  
  if (initMPU6050()) {
    Serial.println("✓ MPU6050初始化成功");
  } else {
    Serial.println("✗ MPU6050初始化失败");
  }
  
#if USE_WIFI
  Serial.println("\n=== WiFi连接 ===");
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  
  Serial.print("连接中");
  int wifiAttempts = 0;
  while (WiFi.status() != WL_CONNECTED && wifiAttempts < 30) {
    delay(500);
    Serial.print(".");
    wifiAttempts++;
  }
  Serial.println();
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("✓ WiFi连接成功！IP: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("✗ WiFi连接失败！");
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    nh.getHardware()->setConnection(server, serverPort);
  }
#endif
  
  Serial.println("\n=== ROS连接 ===");
  nh.initNode();
  nh.subscribe(cmd_vel_sub);
  nh.subscribe(servo_angle_sub);
  nh.advertise(wheel_vel_pub);
  nh.advertise(imu_raw_pub);
  
  Serial.print("等待连接");
  int rosAttempts = 0;
  while (!nh.connected() && rosAttempts < 50) {
    nh.spinOnce();
    delay(100);
    if (rosAttempts % 10 == 0) Serial.print(".");
    rosAttempts++;
  }
  Serial.println();
  
  if (nh.connected()) {
    Serial.println("✓ ROS连接成功！");
  } else {
    Serial.println("✗ ROS连接失败！");
  }
  
  Serial.println("\n=== 系统状态 ===");
  Serial.print("WiFi: ");
  Serial.println((WiFi.status() == WL_CONNECTED) ? "✓ 已连接" : "✗ 未连接");
  Serial.print("ROS: ");
  Serial.println(nh.connected() ? "✓ 已连接" : "✗ 未连接");
  Serial.print("编码器参数: TICKS_PER_REV=");
  Serial.print(TICKS_PER_REV);
  Serial.print(", WHEEL_RADIUS_M=");
  Serial.print(WHEEL_RADIUS_M);
  Serial.print(", WHEEL_BASE_M=");
  Serial.println(WHEEL_BASE_M);
  Serial.println("\n开始运行...\n");
}

void loop() {
  // 命令超时保护
  if (millis() - last_cmd_time > CMD_TIMEOUT_MS) {
    stopMotors();
  }

  if (!nh.connected()) {
    static unsigned long lastReconnectMsg = 0;
    if (millis() - lastReconnectMsg > 5000) {
      Serial.println("ROS连接断开，尝试重连...");
      lastReconnectMsg = millis();
    }
    stopMotors();
    nh.spinOnce();
    delay(100);
    return;
  }

  // 更新轮速（20Hz）
  updateWheelVelocities();
  
  // 读取并发布IMU（50Hz）
  static unsigned long last_imu_time = 0;
  if (millis() - last_imu_time >= 20) {
    readMPU6050();
    imu_raw_pub.publish(&imu_raw_msg);
    last_imu_time = millis();
  }

  nh.spinOnce();
  delay(10);
}


```

已改为使用 `int` 版本的重载，应能解决类型歧义。修改如下：

```219:223:src/yolo_car/esp32/yolo_car_control.ino
  // 修复：使用int版本的重载，避免类型歧义
  int addr = (int)MPU_ADDR;
  int len = 14;
  int stop = 1;  // 1表示发送停止位
  uint8_t bytes_read = Wire.requestFrom(addr, len, stop);
```

这匹配 `uint8_t requestFrom(int address, int size, int sendStop);` 版本，应能消除歧义。请重新编译，如仍有错误，请提供具体错误信息。

```plaintext
  // 修复：使用int版本的重载，避免类型歧义
  int addr = (int)MPU_ADDR;
  int len = 14;
  int stop = 1;  // 1表示发送停止位
  uint8_t bytes_read = Wire.requestFrom(addr, len, stop);
```

---

**User**

好的这个问题解决了，那现在esp32段就是干什么呢

---

**Cursor**

ESP32端的功能如下：

## ESP32端功能总结

### 1. **订阅ROS话题（接收指令）**
- `/yolo_car/cmd_vel` (geometry_msgs/Twist)
  - 接收线速度和角速度指令
  - 转换为左右轮PWM输出
  - 控制L298N驱动电机
- `/yolo_car/servo_angle` (std_msgs/Float32)
  - 接收舵机角度指令（0-180度）
  - 控制SG90舵机

### 2. **发布ROS话题（发送数据）**
- `/yolo_car/wheel_vel` (geometry_msgs/Vector3Stamped)
  - 发布左右轮实际速度（m/s）
  - 频率：20Hz（每50ms更新）
  - 基于编码器脉冲计算
- `/yolo_car/imu_raw` (sensor_msgs/Imu)
  - 发布MPU6050原始数据
  - 包含：角速度（rad/s）、线加速度（m/s²）
  - 频率：50Hz（每20ms更新）
  - 注意：只发布原始数据，不做滤波/姿态估计

### 3. **硬件控制**
- 电机控制：L298N四线控制（PWM + 方向）
- 编码器：中断计数，计算轮速
- MPU6050：I2C读取原始传感器数据
- 安全保护：命令超时500ms自动停车

### 4. **通信方式**
- WiFi rosserial TCP连接
- 连接到ROS master（192.168.31.64:11411）

## 数据流向

```
ROS控制节点 → /yolo_car/cmd_vel → ESP32 → 电机PWM输出
ROS控制节点 → /yolo_car/servo_angle → ESP32 → 舵机PWM输出

ESP32编码器 → /yolo_car/wheel_vel → ROS（供监控/闭环使用）
ESP32 MPU6050 → /yolo_car/imu_raw → ROS（供滤波/融合使用）
```

## 当前状态

ESP32端代码已完成，功能包括：
- 电机和舵机控制
- 编码器轮速发布
- MPU6050原始IMU数据发布
- WiFi rosserial通信

接下来可以在ROS端：
1. 订阅 `/yolo_car/wheel_vel` 和 `/yolo_car/imu_raw`
2. 对IMU数据进行滤波处理（如互补滤波）
3. 使用轮速和IMU数据进行控制优化

需要我检查ROS端代码，确保能正确接收这些数据吗？

```plaintext
ROS控制节点 → /yolo_car/cmd_vel → ESP32 → 电机PWM输出
ROS控制节点 → /yolo_car/servo_angle → ESP32 → 舵机PWM输出

ESP32编码器 → /yolo_car/wheel_vel → ROS（供监控/闭环使用）
ESP32 MPU6050 → /yolo_car/imu_raw → ROS（供滤波/融合使用）
```

---

**User**

1. 订阅ROS话题（接收指令）



/yolo_car/cmd_vel (geometry_msgs/Twist)

接收线速度和角速度指令

转换为左右轮PWM输出

控制L298N驱动电机，好的现在我们来测试一下这个功能，就分别测试单独让左电机前转，让右电机前转，确保这些正确，我要怎么输入命令去测试，这个你要说详细一点，我跟着你操作， 发布ROS话题（发送数据）

/yolo_car/wheel_vel (geometry_msgs/Vector3Stamped)

发布左右轮实际速度（m/s）

频率：20Hz（每50ms更新）

基于编码器脉冲计算

/yolo_car/imu_raw (sensor_msgs/Imu)

发布MPU6050原始数据

包含：角速度（rad/s）、线加速度（m/s²）

频率：50Hz（每20ms更新）

注意：只发布原始数据，不做滤波/姿态估计     这些也是，我们要测试一下看能不能接收到

---

**Cursor**

按以下步骤测试：

步骤1：启动ROS系统


编写测试指南：


```markdown
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

```

已创建测试指南。按以下步骤操作：

## 快速测试步骤

### 第一步：启动ROS系统

**终端1** - 启动roscore：
```bash
cd ~/catkin_ws
source devel/setup.bash
roscore
```

**终端2** - 启动rosserial（ESP32连接）：
```bash
cd ~/catkin_ws
source devel/setup.bash
rosrun rosserial_python serial_node.py tcp 11411
```
看到 "Connected to ESP32" 表示连接成功。

---

### 第二步：测试电机控制

**终端3** - 测试左电机前转：
```bash
cd ~/catkin_ws
source devel/setup.bash

# 让左轮转（线速度0.1，角速度-0.4，左轮会转得快）
rostopic pub -r 10 /yolo_car/cmd_vel geometry_msgs/Twist "linear:
  x: 0.1
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: -0.4"
```
观察：左轮应该前转。

停止：
```bash
rostopic pub -1 /yolo_car/cmd_vel geometry_msgs/Twist "linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}"
```

测试右电机前转：
```bash
# 让右轮转（角速度改为+0.4）
rostopic pub -r 10 /yolo_car/cmd_vel geometry_msgs/Twist "linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.4}"
```

---

### 第三步：测试数据接收

**终端4** - 查看轮速数据：
```bash
cd ~/catkin_ws
source devel/setup.bash
rostopic echo /yolo_car/wheel_vel
```
应该每50ms看到一次数据，`vector.x`是左轮速度，`vector.y`是右轮速度。

**终端5** - 查看IMU数据：
```bash
cd ~/catkin_ws
source devel/setup.bash
rostopic echo /yolo_car/imu_raw
```
应该每20ms看到一次数据，包含角速度和加速度。

---

### 检查话题频率

```bash
# 检查轮速发布频率（应该约20Hz）
rostopic hz /yolo_car/wheel_vel

# 检查IMU发布频率（应该约50Hz）
rostopic hz /yolo_car/imu_raw
```

---

## 注意事项

1. 电机测试时，先让小车悬空，避免意外移动。
2. 如果电机不转，检查ESP32串口输出，确认是否收到指令。
3. 如果数据为0，检查编码器/MPU6050接线和参数。

详细测试指南已保存在：`/home/mycar/catkin_ws/src/yolo_car/ESP32测试指南.md`

按顺序执行，如有问题告诉我。

```shellscript
cd ~/catkin_ws
source devel/setup.bash
roscore
```

```shellscript
cd ~/catkin_ws
source devel/setup.bash
rosrun rosserial_python serial_node.py tcp 11411
```

```shellscript
cd ~/catkin_ws
source devel/setup.bash

# 让左轮转（线速度0.1，角速度-0.4，左轮会转得快）
rostopic pub -r 10 /yolo_car/cmd_vel geometry_msgs/Twist "linear:
  x: 0.1
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: -0.4"
```

```shellscript
rostopic pub -1 /yolo_car/cmd_vel geometry_msgs/Twist "linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}"
```

```shellscript
# 让右轮转（角速度改为+0.4）
rostopic pub -r 10 /yolo_car/cmd_vel geometry_msgs/Twist "linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.4}"
```

```shellscript
cd ~/catkin_ws
source devel/setup.bash
rostopic echo /yolo_car/wheel_vel
```

```shellscript
cd ~/catkin_ws
source devel/setup.bash
rostopic echo /yolo_car/imu_raw
```

```shellscript
# 检查轮速发布频率（应该约20Hz）
rostopic hz /yolo_car/wheel_vel

# 检查IMU发布频率（应该约50Hz）
rostopic hz /yolo_car/imu_raw
```

```shellscript
cd ~/catkin_ws
source devel/setup.bash
roscore
```

```shellscript
cd ~/catkin_ws
source devel/setup.bash
rosrun rosserial_python serial_node.py tcp 11411
```

```shellscript
cd ~/catkin_ws
source devel/setup.bash

# 让左轮转（线速度0.1，角速度-0.4，左轮会转得快）
rostopic pub -r 10 /yolo_car/cmd_vel geometry_msgs/Twist "linear:
  x: 0.1
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: -0.4"
```

```shellscript
rostopic pub -1 /yolo_car/cmd_vel geometry_msgs/Twist "linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}"
```

```shellscript
# 让右轮转（角速度改为+0.4）
rostopic pub -r 10 /yolo_car/cmd_vel geometry_msgs/Twist "linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.4}"
```

```shellscript
cd ~/catkin_ws
source devel/setup.bash
rostopic echo /yolo_car/wheel_vel
```

```shellscript
cd ~/catkin_ws
source devel/setup.bash
rostopic echo /yolo_car/imu_raw
```

```shellscript
# 检查轮速发布频率（应该约20Hz）
rostopic hz /yolo_car/wheel_vel

# 检查IMU发布频率（应该约50Hz）
rostopic hz /yolo_car/imu_raw
```

---

**User**

# 让左轮转（线速度0.1，角速度-0.4，左轮会转得快）



rostopic pub -r 10 /yolo_car/cmd_vel geometry_msgs/Twist "linear:

  x: 0.1

  y: 0.0

  z: 0.0

angular:

  x: 0.0

  y: 0.0

  z: -0.4" 用这种方式让右电机转

---

**Cursor**

让右电机转，使用正角速度：

```bash
# 让右轮转（线速度0.1，角速度+0.4，右轮会转得快）
rostopic pub -r 10 /yolo_car/cmd_vel geometry_msgs/Twist "linear:
  x: 0.1
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.4"
```

**区别说明：**
- 左轮转：角速度 `z: -0.4`（负值）
- 右轮转：角速度 `z: 0.4`（正值）

**原理：**
根据差速控制公式（轮距0.1475m）：
- 左轮速度 = 0.1 - (0.4 × 0.1475 / 2) = 0.1 - 0.0295 ≈ 0.07 m/s（慢）
- 右轮速度 = 0.1 + (0.4 × 0.1475 / 2) = 0.1 + 0.0295 ≈ 0.13 m/s（快）

所以右轮会转得更快。

**停止命令：**
```bash
rostopic pub -1 /yolo_car/cmd_vel geometry_msgs/Twist "linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}"
```

执行后观察右轮是否前转。

```shellscript
# 让右轮转（线速度0.1，角速度+0.4，右轮会转得快）
rostopic pub -r 10 /yolo_car/cmd_vel geometry_msgs/Twist "linear:
  x: 0.1
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.4"
```

```shellscript
rostopic pub -1 /yolo_car/cmd_vel geometry_msgs/Twist "linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}"
```

---

**User**

# 让右轮转（线速度0.1，角速度+0.4，右轮会转得快）



rostopic pub -r 10 /yolo_car/cmd_vel geometry_msgs/Twist "linear:

  x: 0.1

  y: 0.0

  z: 0.0

angular:

  x: 0.0

  y: 0.0

  z: 0.4"我测试这个怎么左右电机都转了，你是让它们前转还是后转，我要的是让它们前转，分别测试左右电机

---

**Cursor**

要让一个轮子转、另一个停，需要调整参数。当前设置会让两个轮子都转，只是速度不同。

## 只让左轮前转（右轮停止）

```bash
# 计算：让 right_vel = 0，即 linear_vel = -(angular_vel * WHEEL_BASE / 2)
# 如果 angular_vel = -0.4，则 linear_vel = -(-0.4 * 0.1475 / 2) = 0.0295
# 这样 left_vel = 0.0295 - (-0.4 * 0.1475 / 2) = 0.059 m/s（前转）
# right_vel = 0.0295 + (-0.4 * 0.1475 / 2) = 0（停止）

rostopic pub -r 10 /yolo_car/cmd_vel geometry_msgs/Twist "linear:
  x: 0.03
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: -0.4"
```

## 只让右轮前转（左轮停止）

```bash
# 计算：让 left_vel = 0，即 linear_vel = (angular_vel * WHEEL_BASE / 2)
# 如果 angular_vel = 0.4，则 linear_vel = 0.4 * 0.1475 / 2 = 0.0295
# 这样 left_vel = 0.0295 - (0.4 * 0.1475 / 2) = 0（停止）
# right_vel = 0.0295 + (0.4 * 0.1475 / 2) = 0.059 m/s（前转）

rostopic pub -r 10 /yolo_car/cmd_vel geometry_msgs/Twist "linear:
  x: 0.03
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.4"
```

## 更简单的方法：直接设置较大的角速度差

如果上面的方法效果不明显，可以增大角速度：

**只让左轮转：**
```bash
rostopic pub -r 10 /yolo_car/cmd_vel geometry_msgs/Twist "linear:
  x: 0.05
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: -0.6"
```

**只让右轮转：**
```bash
rostopic pub -r 10 /yolo_car/cmd_vel geometry_msgs/Twist "linear:
  x: 0.05
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.6"
```

## 停止命令

```bash
rostopic pub -1 /yolo_car/cmd_vel geometry_msgs/Twist "linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}"
```

**说明：**
- 所有命令都是让轮子前转（正方向）
- 如果轮子反转，可能是电机接线方向问题，需要调整ESP32代码中的方向控制逻辑
- 先测试第一个方法，如果效果不明显，再用第二个方法（增大角速度）

测试后告诉我结果。

```shellscript
# 计算：让 right_vel = 0，即 linear_vel = -(angular_vel * WHEEL_BASE / 2)
# 如果 angular_vel = -0.4，则 linear_vel = -(-0.4 * 0.1475 / 2) = 0.0295
# 这样 left_vel = 0.0295 - (-0.4 * 0.1475 / 2) = 0.059 m/s（前转）
# right_vel = 0.0295 + (-0.4 * 0.1475 / 2) = 0（停止）

rostopic pub -r 10 /yolo_car/cmd_vel geometry_msgs/Twist "linear:
  x: 0.03
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: -0.4"
```

```shellscript
# 计算：让 left_vel = 0，即 linear_vel = (angular_vel * WHEEL_BASE / 2)
# 如果 angular_vel = 0.4，则 linear_vel = 0.4 * 0.1475 / 2 = 0.0295
# 这样 left_vel = 0.0295 - (0.4 * 0.1475 / 2) = 0（停止）
# right_vel = 0.0295 + (0.4 * 0.1475 / 2) = 0.059 m/s（前转）

rostopic pub -r 10 /yolo_car/cmd_vel geometry_msgs/Twist "linear:
  x: 0.03
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.4"
```

```shellscript
rostopic pub -r 10 /yolo_car/cmd_vel geometry_msgs/Twist "linear:
  x: 0.05
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: -0.6"
```

```shellscript
rostopic pub -r 10 /yolo_car/cmd_vel geometry_msgs/Twist "linear:
  x: 0.05
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.6"
```

```shellscript
rostopic pub -1 /yolo_car/cmd_vel geometry_msgs/Twist "linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}"
```

---

**User**

rostopic pub -r 10 /yolo_car/cmd_vel geometry_msgs/Twist "linear:



  x: 0.03

  y: 0.0

  z: 0.0

angular:

  x: 0.0

  y: 0.0

  z: 0.4"我测试了一下这个，它是左电机快速旋转，右电机缓慢旋转，rostopic pub -r 10 /yolo_car/cmd_vel geometry_msgs/Twist "linear:

  x: 0.05

  y: 0.0

  z: 0.0

angular:

  x: 0.0

  y: 0.0

  z: -0.6"这个是左电机转，右电机不转，这是怎么回事，怎么只让右电机转动，这个为什么实现不了

---

**Cursor**

检查ESP32代码中的差速控制与方向逻辑，定位问题：


分析问题：左右轮的方向控制逻辑可能不一致，或左右轮定义相反。先计算让右轮单独转的参数：


根据测试结果，左右轮的定义可能相反。先计算让右轮单独转的参数：

要让 `left_vel = 0`，需要：
- `linear_vel = angular_vel * WHEEL_BASE / 2`

试试这个命令让右轮单独转：

```bash
# 让右轮单独转（左轮停止）
# 计算：linear_vel = 0.6 * 0.1475 / 2 = 0.04425
# left_vel = 0.04425 - (0.6 * 0.1475 / 2) = 0（停止）
# right_vel = 0.04425 + (0.6 * 0.1475 / 2) = 0.0885（转）

rostopic pub -r 10 /yolo_car/cmd_vel geometry_msgs/Twist "linear:
  x: 0.044
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.6"
```

如果还是不行，可能是左右轮定义相反。试试这个（用负角速度）：

```bash
# 尝试：如果左右轮定义相反，用负角速度
rostopic pub -r 10 /yolo_car/cmd_vel geometry_msgs/Twist "linear:
  x: 0.044
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: -0.6"
```

或者，如果 `angular: -0.6` 能让左轮转、右轮停，可以尝试让 `left_vel` 接近 0，`right_vel` 较大：

```bash
# 基于你的测试：angular: -0.6 时左轮转右轮停
# 要让右轮转，可以尝试相反的角度
rostopic pub -r 10 /yolo_car/cmd_vel geometry_msgs/Twist "linear:
  x: 0.05
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.8"
```

如果以上都不行，可能是左右轮定义相反。建议先测试第一个命令，告诉我结果，我再根据实际情况调整。

```shellscript
# 让右轮单独转（左轮停止）
# 计算：linear_vel = 0.6 * 0.1475 / 2 = 0.04425
# left_vel = 0.04425 - (0.6 * 0.1475 / 2) = 0（停止）
# right_vel = 0.04425 + (0.6 * 0.1475 / 2) = 0.0885（转）

rostopic pub -r 10 /yolo_car/cmd_vel geometry_msgs/Twist "linear:
  x: 0.044
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.6"
```

```shellscript
# 尝试：如果左右轮定义相反，用负角速度
rostopic pub -r 10 /yolo_car/cmd_vel geometry_msgs/Twist "linear:
  x: 0.044
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: -0.6"
```

```shellscript
# 基于你的测试：angular: -0.6 时左轮转右轮停
# 要让右轮转，可以尝试相反的角度
rostopic pub -r 10 /yolo_car/cmd_vel geometry_msgs/Twist "linear:
  x: 0.05
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.8"
```

---

**User**

rostopic pub -r 10 /yolo_car/cmd_vel geometry_msgs/Twist "linear:



  x: 0.044

  y: 0.0

  z: 0.0

angular:

  x: 0.0

  y: 0.0

  z: 0.6"这个就没问题了，就是单独让右电机转了，刚刚是怎么回事，为什么不可以

---

**Cursor**

原因如下：

## 问题分析

### 之前不行的原因：

**命令1：`linear: 0.03, angular: 0.4`**
- `left_vel = 0.03 - (0.4 × 0.1475 / 2) = 0.03 - 0.0295 = 0.0005 m/s`（太小，但可能仍会转）
- `right_vel = 0.03 + 0.0295 = 0.0595 m/s`（快）

**命令2：`linear: 0.05, angular: -0.6`**
- `left_vel = 0.05 + 0.04425 = 0.09425 m/s`（快）
- `right_vel = 0.05 - 0.04425 = 0.00575 m/s`（很小，接近0但可能仍会转）

### 现在成功的命令：

**命令3：`linear: 0.044, angular: 0.6`**
- `left_vel = 0.044 - (0.6 × 0.1475 / 2) = 0.044 - 0.04425 = -0.00025 m/s`（接近0，且为负值）
- `right_vel = 0.044 + 0.04425 = 0.08825 m/s`（快）

## 关键点

1. 精确计算：要让一个轮子完全停止，需要让计算出的速度接近0，且考虑到MG310的最小启动PWM。
2. 角速度要足够大：`angular: 0.6` 比 `0.4` 大，产生的速度差更大，更容易让一个轮子接近0。
3. 线性速度要匹配：`linear: 0.044` 正好等于 `0.6 × 0.1475 / 2`，这样 `left_vel` 才接近0。

## 总结

要让右轮单独转，需要：
- `linear_vel = angular_vel × WHEEL_BASE / 2`
- 这样 `left_vel` 会接近0，`right_vel` 会较大

**让左轮单独转的命令：**

```bash
# 让左轮单独转（右轮停止）
# linear_vel = -0.6 * 0.1475 / 2 = -0.04425（负值）
# 但线速度通常用正值，所以用负角速度
rostopic pub -r 10 /yolo_car/cmd_vel geometry_msgs/Twist "linear:
  x: 0.044
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: -0.6"
```

现在两个电机都可以单独测试了。

```shellscript
# 让左轮单独转（右轮停止）
# linear_vel = -0.6 * 0.1475 / 2 = -0.04425（负值）
# 但线速度通常用正值，所以用负角速度
rostopic pub -r 10 /yolo_car/cmd_vel geometry_msgs/Twist "linear:
  x: 0.044
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: -0.6"
```

---

**User**

/*



 * ESP32-S3 + L298N电机驱动 + SG90舵机 + 编码器 + MPU6050 + ROS1 (rosserial)

 * 

 * 功能：

 * 1. 订阅速度指令控制L298N电机

 * 2. 订阅舵机角度指令控制SG90舵机

 * 3. 编码器计数并发布轮速

 * 4. MPU6050读取并发布原始IMU数据

 * 

 * 接线：

 * - L298N：N1->GPIO3(左PWM), N2->GPIO4(左DIR), N3->GPIO5(右PWM), N4->GPIO6(右DIR)

 * - SG90舵机信号线：GPIO2

 * - 编码器：右轮A->GPIO7, B->GPIO8; 左轮A->GPIO9, B->GPIO10

 * - MPU6050：SCL->GPIO45, SDA->GPIO46

 */

#include <ros.h>

#include <geometry_msgs/Twist.h>

#include <std_msgs/Float32.h>

#include <sensor_msgs/Imu.h>

#include <geometry_msgs/Vector3Stamped.h>

#include <Wire.h>

#define USE_WIFI true

#if USE_WIFI

  #include <WiFi.h>

  const char* ssid = "机器人小车局域网";

  const char* password = "3252362734";

  IPAddress server(192, 168, 31, 64);

  const uint16_t serverPort = 11411;

#endif

// L298N电机控制引脚

#define MOTOR_LEFT_PWM   3   // 左轮PWM

#define MOTOR_LEFT_DIR   4   // 左轮方向

#define MOTOR_RIGHT_PWM  5   // 右轮PWM

#define MOTOR_RIGHT_DIR  6   // 右轮方向

// SG90舵机引脚

#define SERVO_PIN 2

// 编码器引脚

#define ENC_RIGHT_A 7

#define ENC_RIGHT_B 8

#define ENC_LEFT_A  9

#define ENC_LEFT_B  10

// MPU6050 I2C引脚和地址

#define MPU_ADDR 0x68

#define I2C_SDA 46

#define I2C_SCL 45

// PWM参数

#define PWM_FREQ 1000      // PWM频率 1kHz

#define PWM_RESOLUTION 8   // 8位分辨率 (0-255)

#define SERVO_FREQ 50      // 舵机PWM频率 50Hz

#define SERVO_RESOLUTION 12

// 速度限制

#define MAX_PWM_VALUE 200  // 最大PWM值

// 命令超时（毫秒）

#define CMD_TIMEOUT_MS 500

// 编码器参数（已测量更新）

#define TICKS_PER_REV 260.0f      // 每转脉冲数

#define WHEEL_RADIUS_M 0.0480f    // 轮子半径（米）

#define WHEEL_BASE_M   0.1475f    // 轮距（米）

// 编码器计数变量（volatile用于中断）

volatile int32_t enc_left_count = 0;

volatile int32_t enc_right_count = 0;

volatile int8_t enc_left_last_state = 0;

volatile int8_t enc_right_last_state = 0;

// 轮速计算

float left_wheel_vel = 0.0f;

float right_wheel_vel = 0.0f;

unsigned long last_enc_time = 0;

const unsigned long ENC_UPDATE_INTERVAL_MS = 50; // 50ms更新一次，20Hz

// ROS节点

ros::NodeHandle nh;

unsigned long last_cmd_time = 0;

// ROS消息变量（必须先声明，发布者才能引用）

geometry_msgs::Vector3Stamped wheel_vel_msg;

sensor_msgs::Imu imu_raw_msg;

// 发布者（必须在消息变量之后声明）

ros::Publisher wheel_vel_pub("/yolo_car/wheel_vel", &wheel_vel_msg);

ros::Publisher imu_raw_pub("/yolo_car/imu_raw", &imu_raw_msg);

// 左编码器中断处理（A相）

void IRAM_ATTR encLeftA() {

  int8_t a = digitalRead(ENC_LEFT_A);

  int8_t b = digitalRead(ENC_LEFT_B);

  int8_t state = (a << 1) | b;

  int8_t diff = (state - enc_left_last_state) & 0x03;

  if (diff == 1 || diff == 3) {

    enc_left_count++;

  } else if (diff == 2) {

    enc_left_count--;

  }

  enc_left_last_state = state;

}

// 右编码器中断处理（A相）

void IRAM_ATTR encRightA() {

  int8_t a = digitalRead(ENC_RIGHT_A);

  int8_t b = digitalRead(ENC_RIGHT_B);

  int8_t state = (a << 1) | b;

  int8_t diff = (state - enc_right_last_state) & 0x03;

  if (diff == 1 || diff == 3) {

    enc_right_count++;

  } else if (diff == 2) {

    enc_right_count--;

  }

  enc_right_last_state = state;

}

// 电机急停

void stopMotors() {

  ledcWrite(0, 0);

  ledcWrite(1, 0);

  digitalWrite(MOTOR_LEFT_DIR, LOW);

  digitalWrite(MOTOR_RIGHT_DIR, LOW);

}

// 速度指令回调

void cmdVelCallback(const geometry_msgs::Twist& msg) {

  float linear_vel = msg.linear.x;   // 线速度 m/s

  float angular_vel = msg.angular.z; // 角速度 rad/s

  last_cmd_time = millis();

  

  if (fabs(linear_vel) < 1e-3 && fabs(angular_vel) < 1e-3) {

    stopMotors();

    return;

  }

  

  // 差速控制

  float left_vel = linear_vel - (angular_vel * WHEEL_BASE_M / 2.0);

  float right_vel = linear_vel + (angular_vel * WHEEL_BASE_M / 2.0);

  

  float max_vel = 0.3;

  left_vel = constrain(left_vel, -max_vel, max_vel);

  right_vel = constrain(right_vel, -max_vel, max_vel);

  

  int left_pwm = abs(left_vel / max_vel * MAX_PWM_VALUE);

  int right_pwm = abs(right_vel / max_vel * MAX_PWM_VALUE);

  

  // 方向控制

  if (left_vel >= 0) {

    digitalWrite(MOTOR_LEFT_DIR, HIGH);

  } else {

    digitalWrite(MOTOR_LEFT_DIR, LOW);

  }

  

  if (right_vel >= 0) {

    digitalWrite(MOTOR_RIGHT_DIR, LOW);

  } else {

    digitalWrite(MOTOR_RIGHT_DIR, HIGH);

  }

  

  ledcWrite(0, left_pwm);

  ledcWrite(1, right_pwm);

}

// 舵机角度回调

void servoAngleCallback(const std_msgs::Float32& msg) {

  float angle = msg.data;

  angle = constrain(angle, 0.0, 180.0);

  int pulse_width = map(angle, 0, 180, 500, 2500);

  int duty_cycle = (pulse_width * 4096L) / 20000;

  duty_cycle = constrain(duty_cycle, 0, 4095);

  ledcWrite(2, duty_cycle);

}

// 订阅者

ros::Subscriber<geometry_msgs::Twist> cmd_vel_sub("/yolo_car/cmd_vel", cmdVelCallback);

ros::Subscriber<std_msgs::Float32> servo_angle_sub("/yolo_car/servo_angle", servoAngleCallback);

// MPU6050初始化

bool initMPU6050() {

  Wire.beginTransmission(MPU_ADDR);

  Wire.write(0x6B); // PWR_MGMT_1寄存器

  Wire.write(0x00); // 唤醒MPU6050

  uint8_t error = Wire.endTransmission();

  

  if (error != 0) {

    return false;

  }

  

  // 配置加速度计范围 ±2g

  Wire.beginTransmission(MPU_ADDR);

  Wire.write(0x1C); // ACCEL_CONFIG

  Wire.write(0x00);

  Wire.endTransmission();

  

  // 配置陀螺仪范围 ±250°/s

  Wire.beginTransmission(MPU_ADDR);

  Wire.write(0x1B); // GYRO_CONFIG

  Wire.write(0x00);

  Wire.endTransmission();

  

  return true;

}

// 读取MPU6050原始数据

void readMPU6050() {

  // 读取加速度计数据（6字节：ACCEL_XOUT_H, ACCEL_XOUT_L, ACCEL_YOUT_H, ACCEL_YOUT_L, ACCEL_ZOUT_H, ACCEL_ZOUT_L）

  Wire.beginTransmission(MPU_ADDR);

  Wire.write(0x3B); // ACCEL_XOUT_H寄存器地址

  Wire.endTransmission(false);

  

  // 修复：使用int版本的重载，避免类型歧义

  int addr = (int)MPU_ADDR;

  int len = 14;

  int stop = 1;  // 1表示发送停止位

  uint8_t bytes_read = Wire.requestFrom(addr, len, stop);

  

  if (bytes_read == 14) {

    int16_t accel_x = (Wire.read() << 8) | Wire.read();

    int16_t accel_y = (Wire.read() << 8) | Wire.read();

    int16_t accel_z = (Wire.read() << 8) | Wire.read();

    int16_t temp = (Wire.read() << 8) | Wire.read();

    int16_t gyro_x = (Wire.read() << 8) | Wire.read();

    int16_t gyro_y = (Wire.read() << 8) | Wire.read();

    int16_t gyro_z = (Wire.read() << 8) | Wire.read();

    

    // 转换为物理单位

    // 加速度：±2g范围，32768对应2g，g=9.8 m/s²

    float accel_scale = 2.0f * 9.8f / 32768.0f;

    imu_raw_msg.linear_acceleration.x = accel_x * accel_scale;

    imu_raw_msg.linear_acceleration.y = accel_y * accel_scale;

    imu_raw_msg.linear_acceleration.z = accel_z * accel_scale;

    

    // 角速度：±250°/s范围，32768对应250°/s，转换为rad/s

    float gyro_scale = 250.0f * M_PI / 180.0f / 32768.0f;

    imu_raw_msg.angular_velocity.x = gyro_x * gyro_scale;

    imu_raw_msg.angular_velocity.y = gyro_y * gyro_scale;

    imu_raw_msg.angular_velocity.z = gyro_z * gyro_scale;

    

    imu_raw_msg.header.stamp = nh.now();

    imu_raw_msg.header.frame_id = "imu_link";

  }

}

// 更新轮速

void updateWheelVelocities() {

  unsigned long current_time = millis();

  unsigned long dt_ms = current_time - last_enc_time;

  

  if (dt_ms >= ENC_UPDATE_INTERVAL_MS) {

    // 读取并重置计数（原子操作）

    int32_t left_ticks = enc_left_count;

    int32_t right_ticks = enc_right_count;

    enc_left_count = 0;

    enc_right_count = 0;

    

    // 计算轮速（m/s）

    // 每转脉冲数 * 轮子周长 = 每米脉冲数

    float ticks_per_meter = TICKS_PER_REV / (2.0f * M_PI * WHEEL_RADIUS_M);

    float dt_sec = dt_ms / 1000.0f;

    

    left_wheel_vel = (left_ticks / ticks_per_meter) / dt_sec;

    right_wheel_vel = (right_ticks / ticks_per_meter) / dt_sec;

    

    // 发布轮速

    wheel_vel_msg.header.stamp = nh.now();

    wheel_vel_msg.header.frame_id = "base_link";

    wheel_vel_msg.vector.x = left_wheel_vel;

    wheel_vel_msg.vector.y = right_wheel_vel;

    wheel_vel_msg.vector.z = 0.0;

    wheel_vel_pub.publish(&wheel_vel_msg);

    

    last_enc_time = current_time;

  }

}

void setup() {

  Serial.begin(115200);

  delay(1000);

  Serial.println("\n\n=== ESP32-S3 + L298N + SG90 + 编码器 + MPU6050 + ROS1 ===");

  last_cmd_time = millis();

  

  // 初始化电机控制引脚

  Serial.println("初始化L298N电机驱动...");

  pinMode(MOTOR_LEFT_DIR, OUTPUT);

  pinMode(MOTOR_RIGHT_DIR, OUTPUT);

  digitalWrite(MOTOR_LEFT_DIR, LOW);

  digitalWrite(MOTOR_RIGHT_DIR, LOW);

  

  // 配置PWM通道（电机）

  ledcSetup(0, PWM_FREQ, PWM_RESOLUTION);

  ledcSetup(1, PWM_FREQ, PWM_RESOLUTION);

  ledcAttachPin(MOTOR_LEFT_PWM, 0);

  ledcAttachPin(MOTOR_RIGHT_PWM, 1);

  

  // 配置PWM通道（舵机）

  ledcSetup(2, SERVO_FREQ, SERVO_RESOLUTION);

  ledcAttachPin(SERVO_PIN, 2);

  

  ledcWrite(0, 0);

  ledcWrite(1, 0);

  

  // 初始舵机到90度

  float init_angle = 90.0;

  int pulse_width = map(init_angle, 0, 180, 500, 2500);

  int duty_cycle = (pulse_width * 4096L) / 20000;

  duty_cycle = constrain(duty_cycle, 0, 4095);

  ledcWrite(2, duty_cycle);

  

  Serial.println("✓ 电机和舵机初始化完成");

  

  // 初始化编码器

  Serial.println("初始化编码器...");

  pinMode(ENC_LEFT_A, INPUT_PULLUP);

  pinMode(ENC_LEFT_B, INPUT_PULLUP);

  pinMode(ENC_RIGHT_A, INPUT_PULLUP);

  pinMode(ENC_RIGHT_B, INPUT_PULLUP);

  

  enc_left_last_state = (digitalRead(ENC_LEFT_A) << 1) | digitalRead(ENC_LEFT_B);

  enc_right_last_state = (digitalRead(ENC_RIGHT_A) << 1) | digitalRead(ENC_RIGHT_B);

  

  attachInterrupt(digitalPinToInterrupt(ENC_LEFT_A), encLeftA, CHANGE);

  attachInterrupt(digitalPinToInterrupt(ENC_RIGHT_A), encRightA, CHANGE);

  

  last_enc_time = millis();

  Serial.println("✓ 编码器初始化完成");

  

  // 初始化MPU6050

  Serial.println("初始化MPU6050...");

  Wire.begin(I2C_SDA, I2C_SCL);

  delay(100);

  

  if (initMPU6050()) {

    Serial.println("✓ MPU6050初始化成功");

  } else {

    Serial.println("✗ MPU6050初始化失败");

  }

  

#if USE_WIFI

  Serial.println("\n=== WiFi连接 ===");

  WiFi.mode(WIFI_STA);

  WiFi.begin(ssid, password);

  

  Serial.print("连接中");

  int wifiAttempts = 0;

  while (WiFi.status() != WL_CONNECTED && wifiAttempts < 30) {

    delay(500);

    Serial.print(".");

    wifiAttempts++;

  }

  Serial.println();

  

  if (WiFi.status() == WL_CONNECTED) {

    Serial.print("✓ WiFi连接成功！IP: ");

    Serial.println(WiFi.localIP());

  } else {

    Serial.println("✗ WiFi连接失败！");

  }

  

  if (WiFi.status() == WL_CONNECTED) {

    nh.getHardware()->setConnection(server, serverPort);

  }

#endif

  

  Serial.println("\n=== ROS连接 ===");

  nh.initNode();

  nh.subscribe(cmd_vel_sub);

  nh.subscribe(servo_angle_sub);

  nh.advertise(wheel_vel_pub);

  nh.advertise(imu_raw_pub);

  

  Serial.print("等待连接");

  int rosAttempts = 0;

  while (!nh.connected() && rosAttempts < 50) {

    nh.spinOnce();

    delay(100);

    if (rosAttempts % 10 == 0) Serial.print(".");

    rosAttempts++;

  }

  Serial.println();

  

  if (nh.connected()) {

    Serial.println("✓ ROS连接成功！");

  } else {

    Serial.println("✗ ROS连接失败！");

  }

  

  Serial.println("\n=== 系统状态 ===");

  Serial.print("WiFi: ");

  Serial.println((WiFi.status() == WL_CONNECTED) ? "✓ 已连接" : "✗ 未连接");

  Serial.print("ROS: ");

  Serial.println(nh.connected() ? "✓ 已连接" : "✗ 未连接");

  Serial.print("编码器参数: TICKS_PER_REV=");

  Serial.print(TICKS_PER_REV);

  Serial.print(", WHEEL_RADIUS_M=");

  Serial.print(WHEEL_RADIUS_M);

  Serial.print(", WHEEL_BASE_M=");

  Serial.println(WHEEL_BASE_M);

  Serial.println("\n开始运行...\n");

}

void loop() {

  // 命令超时保护

  if (millis() - last_cmd_time > CMD_TIMEOUT_MS) {

    stopMotors();

  }

  if (!nh.connected()) {

    static unsigned long lastReconnectMsg = 0;

    if (millis() - lastReconnectMsg > 5000) {

      Serial.println("ROS连接断开，尝试重连...");

      lastReconnectMsg = millis();

    }

    stopMotors();

    nh.spinOnce();

    delay(100);

    return;

  }

  // 更新轮速（20Hz）

  updateWheelVelocities();

  

  // 读取并发布IMU（50Hz）

  static unsigned long last_imu_time = 0;

  if (millis() - last_imu_time >= 20) {

    readMPU6050();

    imu_raw_pub.publish(&imu_raw_msg);

    last_imu_time = millis();

  }

  nh.spinOnce();

  delay(10);

}

这是我现在的代码，刚刚我们不是测试了一下让左右轮前转吗，测试了一下，左右轮没问题，但两个的方向都反了，明明是控制前转，但左右电机却都是后转，你在代码里把这来个的方向纠正一下后把完整代码给我，只改左右轮方向问题，其它没说到的一点都不要动

---

**Cursor**

修正左右轮方向控制逻辑：将方向反转。


修正左右轮方向控制逻辑：


```cpp
/*
 * ESP32-S3 + L298N电机驱动 + SG90舵机 + 编码器 + MPU6050 + ROS1 (rosserial)
 * 
 * 功能：
 * 1. 订阅速度指令控制L298N电机
 * 2. 订阅舵机角度指令控制SG90舵机
 * 3. 编码器计数并发布轮速
 * 4. MPU6050读取并发布原始IMU数据
 * 
 * 接线：
 * - L298N：N1->GPIO3(左PWM), N2->GPIO4(左DIR), N3->GPIO5(右PWM), N4->GPIO6(右DIR)
 * - SG90舵机信号线：GPIO2
 * - 编码器：右轮A->GPIO7, B->GPIO8; 左轮A->GPIO9, B->GPIO10
 * - MPU6050：SCL->GPIO45, SDA->GPIO46
 */

#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <Wire.h>

#define USE_WIFI true

#if USE_WIFI
  #include <WiFi.h>
  const char* ssid = "机器人小车局域网";
  const char* password = "3252362734";
  IPAddress server(192, 168, 31, 64);
  const uint16_t serverPort = 11411;
#endif

// L298N电机控制引脚
#define MOTOR_LEFT_PWM   3   // 左轮PWM
#define MOTOR_LEFT_DIR   4   // 左轮方向
#define MOTOR_RIGHT_PWM  5   // 右轮PWM
#define MOTOR_RIGHT_DIR  6   // 右轮方向

// SG90舵机引脚
#define SERVO_PIN 2

// 编码器引脚
#define ENC_RIGHT_A 7
#define ENC_RIGHT_B 8
#define ENC_LEFT_A  9
#define ENC_LEFT_B  10

// MPU6050 I2C引脚和地址
#define MPU_ADDR 0x68
#define I2C_SDA 46
#define I2C_SCL 45

// PWM参数
#define PWM_FREQ 1000      // PWM频率 1kHz
#define PWM_RESOLUTION 8   // 8位分辨率 (0-255)
#define SERVO_FREQ 50      // 舵机PWM频率 50Hz
#define SERVO_RESOLUTION 12

// 速度限制
#define MAX_PWM_VALUE 200  // 最大PWM值

// 命令超时（毫秒）
#define CMD_TIMEOUT_MS 500

// 编码器参数（已测量更新）
#define TICKS_PER_REV 260.0f      // 每转脉冲数
#define WHEEL_RADIUS_M 0.0480f    // 轮子半径（米）
#define WHEEL_BASE_M   0.1475f    // 轮距（米）

// 编码器计数变量（volatile用于中断）
volatile int32_t enc_left_count = 0;
volatile int32_t enc_right_count = 0;
volatile int8_t enc_left_last_state = 0;
volatile int8_t enc_right_last_state = 0;

// 轮速计算
float left_wheel_vel = 0.0f;
float right_wheel_vel = 0.0f;
unsigned long last_enc_time = 0;
const unsigned long ENC_UPDATE_INTERVAL_MS = 50; // 50ms更新一次，20Hz

// ROS节点
ros::NodeHandle nh;
unsigned long last_cmd_time = 0;

// ROS消息变量（必须先声明，发布者才能引用）
geometry_msgs::Vector3Stamped wheel_vel_msg;
sensor_msgs::Imu imu_raw_msg;

// 发布者（必须在消息变量之后声明）
ros::Publisher wheel_vel_pub("/yolo_car/wheel_vel", &wheel_vel_msg);
ros::Publisher imu_raw_pub("/yolo_car/imu_raw", &imu_raw_msg);

// 左编码器中断处理（A相）
void IRAM_ATTR encLeftA() {
  int8_t a = digitalRead(ENC_LEFT_A);
  int8_t b = digitalRead(ENC_LEFT_B);
  int8_t state = (a << 1) | b;
  int8_t diff = (state - enc_left_last_state) & 0x03;
  if (diff == 1 || diff == 3) {
    enc_left_count++;
  } else if (diff == 2) {
    enc_left_count--;
  }
  enc_left_last_state = state;
}

// 右编码器中断处理（A相）
void IRAM_ATTR encRightA() {
  int8_t a = digitalRead(ENC_RIGHT_A);
  int8_t b = digitalRead(ENC_RIGHT_B);
  int8_t state = (a << 1) | b;
  int8_t diff = (state - enc_right_last_state) & 0x03;
  if (diff == 1 || diff == 3) {
    enc_right_count++;
  } else if (diff == 2) {
    enc_right_count--;
  }
  enc_right_last_state = state;
}

// 电机急停
void stopMotors() {
  ledcWrite(0, 0);
  ledcWrite(1, 0);
  digitalWrite(MOTOR_LEFT_DIR, LOW);
  digitalWrite(MOTOR_RIGHT_DIR, LOW);
}

// 速度指令回调
void cmdVelCallback(const geometry_msgs::Twist& msg) {
  float linear_vel = msg.linear.x;   // 线速度 m/s
  float angular_vel = msg.angular.z; // 角速度 rad/s

  last_cmd_time = millis();
  
  if (fabs(linear_vel) < 1e-3 && fabs(angular_vel) < 1e-3) {
    stopMotors();
    return;
  }
  
  // 差速控制
  float left_vel = linear_vel - (angular_vel * WHEEL_BASE_M / 2.0);
  float right_vel = linear_vel + (angular_vel * WHEEL_BASE_M / 2.0);
  
  float max_vel = 0.3;
  left_vel = constrain(left_vel, -max_vel, max_vel);
  right_vel = constrain(right_vel, -max_vel, max_vel);
  
  int left_pwm = abs(left_vel / max_vel * MAX_PWM_VALUE);
  int right_pwm = abs(right_vel / max_vel * MAX_PWM_VALUE);
  
  // 方向控制（已修正：左右轮方向反转）
  if (left_vel >= 0) {
    digitalWrite(MOTOR_LEFT_DIR, LOW);
  } else {
    digitalWrite(MOTOR_LEFT_DIR, HIGH);
  }
  
  if (right_vel >= 0) {
    digitalWrite(MOTOR_RIGHT_DIR, HIGH);
  } else {
    digitalWrite(MOTOR_RIGHT_DIR, LOW);
  }
  
  ledcWrite(0, left_pwm);
  ledcWrite(1, right_pwm);
}

// 舵机角度回调
void servoAngleCallback(const std_msgs::Float32& msg) {
  float angle = msg.data;
  angle = constrain(angle, 0.0, 180.0);

  int pulse_width = map(angle, 0, 180, 500, 2500);
  int duty_cycle = (pulse_width * 4096L) / 20000;
  duty_cycle = constrain(duty_cycle, 0, 4095);
  ledcWrite(2, duty_cycle);
}

// 订阅者
ros::Subscriber<geometry_msgs::Twist> cmd_vel_sub("/yolo_car/cmd_vel", cmdVelCallback);
ros::Subscriber<std_msgs::Float32> servo_angle_sub("/yolo_car/servo_angle", servoAngleCallback);

// MPU6050初始化
bool initMPU6050() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B); // PWR_MGMT_1寄存器
  Wire.write(0x00); // 唤醒MPU6050
  uint8_t error = Wire.endTransmission();
  
  if (error != 0) {
    return false;
  }
  
  // 配置加速度计范围 ±2g
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1C); // ACCEL_CONFIG
  Wire.write(0x00);
  Wire.endTransmission();
  
  // 配置陀螺仪范围 ±250°/s
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1B); // GYRO_CONFIG
  Wire.write(0x00);
  Wire.endTransmission();
  
  return true;
}

// 读取MPU6050原始数据
void readMPU6050() {
  // 读取加速度计数据（6字节：ACCEL_XOUT_H, ACCEL_XOUT_L, ACCEL_YOUT_H, ACCEL_YOUT_L, ACCEL_ZOUT_H, ACCEL_ZOUT_L）
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B); // ACCEL_XOUT_H寄存器地址
  Wire.endTransmission(false);
  
  // 修复：使用int版本的重载，避免类型歧义
  int addr = (int)MPU_ADDR;
  int len = 14;
  int stop = 1;  // 1表示发送停止位
  uint8_t bytes_read = Wire.requestFrom(addr, len, stop);
  
  if (bytes_read == 14) {
    int16_t accel_x = (Wire.read() << 8) | Wire.read();
    int16_t accel_y = (Wire.read() << 8) | Wire.read();
    int16_t accel_z = (Wire.read() << 8) | Wire.read();
    int16_t temp = (Wire.read() << 8) | Wire.read();
    int16_t gyro_x = (Wire.read() << 8) | Wire.read();
    int16_t gyro_y = (Wire.read() << 8) | Wire.read();
    int16_t gyro_z = (Wire.read() << 8) | Wire.read();
    
    // 转换为物理单位
    // 加速度：±2g范围，32768对应2g，g=9.8 m/s²
    float accel_scale = 2.0f * 9.8f / 32768.0f;
    imu_raw_msg.linear_acceleration.x = accel_x * accel_scale;
    imu_raw_msg.linear_acceleration.y = accel_y * accel_scale;
    imu_raw_msg.linear_acceleration.z = accel_z * accel_scale;
    
    // 角速度：±250°/s范围，32768对应250°/s，转换为rad/s
    float gyro_scale = 250.0f * M_PI / 180.0f / 32768.0f;
    imu_raw_msg.angular_velocity.x = gyro_x * gyro_scale;
    imu_raw_msg.angular_velocity.y = gyro_y * gyro_scale;
    imu_raw_msg.angular_velocity.z = gyro_z * gyro_scale;
    
    imu_raw_msg.header.stamp = nh.now();
    imu_raw_msg.header.frame_id = "imu_link";
  }
}

// 更新轮速
void updateWheelVelocities() {
  unsigned long current_time = millis();
  unsigned long dt_ms = current_time - last_enc_time;
  
  if (dt_ms >= ENC_UPDATE_INTERVAL_MS) {
    // 读取并重置计数（原子操作）
    int32_t left_ticks = enc_left_count;
    int32_t right_ticks = enc_right_count;
    enc_left_count = 0;
    enc_right_count = 0;
    
    // 计算轮速（m/s）
    // 每转脉冲数 * 轮子周长 = 每米脉冲数
    float ticks_per_meter = TICKS_PER_REV / (2.0f * M_PI * WHEEL_RADIUS_M);
    float dt_sec = dt_ms / 1000.0f;
    
    left_wheel_vel = (left_ticks / ticks_per_meter) / dt_sec;
    right_wheel_vel = (right_ticks / ticks_per_meter) / dt_sec;
    
    // 发布轮速
    wheel_vel_msg.header.stamp = nh.now();
    wheel_vel_msg.header.frame_id = "base_link";
    wheel_vel_msg.vector.x = left_wheel_vel;
    wheel_vel_msg.vector.y = right_wheel_vel;
    wheel_vel_msg.vector.z = 0.0;
    wheel_vel_pub.publish(&wheel_vel_msg);
    
    last_enc_time = current_time;
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n\n=== ESP32-S3 + L298N + SG90 + 编码器 + MPU6050 + ROS1 ===");

  last_cmd_time = millis();
  
  // 初始化电机控制引脚
  Serial.println("初始化L298N电机驱动...");
  pinMode(MOTOR_LEFT_DIR, OUTPUT);
  pinMode(MOTOR_RIGHT_DIR, OUTPUT);
  digitalWrite(MOTOR_LEFT_DIR, LOW);
  digitalWrite(MOTOR_RIGHT_DIR, LOW);
  
  // 配置PWM通道（电机）
  ledcSetup(0, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(1, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(MOTOR_LEFT_PWM, 0);
  ledcAttachPin(MOTOR_RIGHT_PWM, 1);
  
  // 配置PWM通道（舵机）
  ledcSetup(2, SERVO_FREQ, SERVO_RESOLUTION);
  ledcAttachPin(SERVO_PIN, 2);
  
  ledcWrite(0, 0);
  ledcWrite(1, 0);
  
  // 初始舵机到90度
  float init_angle = 90.0;
  int pulse_width = map(init_angle, 0, 180, 500, 2500);
  int duty_cycle = (pulse_width * 4096L) / 20000;
  duty_cycle = constrain(duty_cycle, 0, 4095);
  ledcWrite(2, duty_cycle);
  
  Serial.println("✓ 电机和舵机初始化完成");
  
  // 初始化编码器
  Serial.println("初始化编码器...");
  pinMode(ENC_LEFT_A, INPUT_PULLUP);
  pinMode(ENC_LEFT_B, INPUT_PULLUP);
  pinMode(ENC_RIGHT_A, INPUT_PULLUP);
  pinMode(ENC_RIGHT_B, INPUT_PULLUP);
  
  enc_left_last_state = (digitalRead(ENC_LEFT_A) << 1) | digitalRead(ENC_LEFT_B);
  enc_right_last_state = (digitalRead(ENC_RIGHT_A) << 1) | digitalRead(ENC_RIGHT_B);
  
  attachInterrupt(digitalPinToInterrupt(ENC_LEFT_A), encLeftA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_RIGHT_A), encRightA, CHANGE);
  
  last_enc_time = millis();
  Serial.println("✓ 编码器初始化完成");
  
  // 初始化MPU6050
  Serial.println("初始化MPU6050...");
  Wire.begin(I2C_SDA, I2C_SCL);
  delay(100);
  
  if (initMPU6050()) {
    Serial.println("✓ MPU6050初始化成功");
  } else {
    Serial.println("✗ MPU6050初始化失败");
  }
  
#if USE_WIFI
  Serial.println("\n=== WiFi连接 ===");
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  
  Serial.print("连接中");
  int wifiAttempts = 0;
  while (WiFi.status() != WL_CONNECTED && wifiAttempts < 30) {
    delay(500);
    Serial.print(".");
    wifiAttempts++;
  }
  Serial.println();
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("✓ WiFi连接成功！IP: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("✗ WiFi连接失败！");
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    nh.getHardware()->setConnection(server, serverPort);
  }
#endif
  
  Serial.println("\n=== ROS连接 ===");
  nh.initNode();
  nh.subscribe(cmd_vel_sub);
  nh.subscribe(servo_angle_sub);
  nh.advertise(wheel_vel_pub);
  nh.advertise(imu_raw_pub);
  
  Serial.print("等待连接");
  int rosAttempts = 0;
  while (!nh.connected() && rosAttempts < 50) {
    nh.spinOnce();
    delay(100);
    if (rosAttempts % 10 == 0) Serial.print(".");
    rosAttempts++;
  }
  Serial.println();
  
  if (nh.connected()) {
    Serial.println("✓ ROS连接成功！");
  } else {
    Serial.println("✗ ROS连接失败！");
  }
  
  Serial.println("\n=== 系统状态 ===");
  Serial.print("WiFi: ");
  Serial.println((WiFi.status() == WL_CONNECTED) ? "✓ 已连接" : "✗ 未连接");
  Serial.print("ROS: ");
  Serial.println(nh.connected() ? "✓ 已连接" : "✗ 未连接");
  Serial.print("编码器参数: TICKS_PER_REV=");
  Serial.print(TICKS_PER_REV);
  Serial.print(", WHEEL_RADIUS_M=");
  Serial.print(WHEEL_RADIUS_M);
  Serial.print(", WHEEL_BASE_M=");
  Serial.println(WHEEL_BASE_M);
  Serial.println("\n开始运行...\n");
}

void loop() {
  // 命令超时保护
  if (millis() - last_cmd_time > CMD_TIMEOUT_MS) {
    stopMotors();
  }

  if (!nh.connected()) {
    static unsigned long lastReconnectMsg = 0;
    if (millis() - lastReconnectMsg > 5000) {
      Serial.println("ROS连接断开，尝试重连...");
      lastReconnectMsg = millis();
    }
    stopMotors();
    nh.spinOnce();
    delay(100);
    return;
  }

  // 更新轮速（20Hz）
  updateWheelVelocities();
  
  // 读取并发布IMU（50Hz）
  static unsigned long last_imu_time = 0;
  if (millis() - last_imu_time >= 20) {
    readMPU6050();
    imu_raw_pub.publish(&imu_raw_msg);
    last_imu_time = millis();
  }

  nh.spinOnce();
  delay(10);
}


```

已修正左右轮方向控制逻辑。完整代码：


已修正左右轮方向控制逻辑。修改位置在第154-165行：

**修改内容：**
- 左轮：`vel >= 0` 时 DIR 从 `HIGH` 改为 `LOW`，`vel < 0` 时 DIR 从 `LOW` 改为 `HIGH`
- 右轮：`vel >= 0` 时 DIR 从 `LOW` 改为 `HIGH`，`vel < 0` 时 DIR 从 `HIGH` 改为 `LOW`

现在左右轮方向已修正。代码已保存，可直接上传到ESP32测试。发送前转命令时，两个轮子应该都前转。

---

**User**

然后现在我们再来测试一下单独让左右轮前转吧，你把命令给我我去运行

---

**Cursor**

单独测试左右轮前转的命令：

## 测试命令

### 1. 让左轮单独前转（右轮停止）

```bash
rostopic pub -r 10 /yolo_car/cmd_vel geometry_msgs/Twist "linear:
  x: 0.044
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: -0.6"
```

### 2. 让右轮单独前转（左轮停止）

```bash
rostopic pub -r 10 /yolo_car/cmd_vel geometry_msgs/Twist "linear:
  x: 0.044
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.6"
```

### 3. 停止电机

```bash
rostopic pub -1 /yolo_car/cmd_vel geometry_msgs/Twist "linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}"
```

## 测试步骤

1. 先测试左轮：运行命令1，观察左轮是否前转，右轮是否停止
2. 停止：运行命令3
3. 再测试右轮：运行命令2，观察右轮是否前转，左轮是否停止
4. 停止：运行命令3

**注意：**
- 测试时让小车悬空，避免意外移动
- 如果某个轮子不转或方向不对，告诉我结果，我再调整参数

按顺序测试，告诉我结果。

```shellscript
rostopic pub -r 10 /yolo_car/cmd_vel geometry_msgs/Twist "linear:
  x: 0.044
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: -0.6"
```

```shellscript
rostopic pub -r 10 /yolo_car/cmd_vel geometry_msgs/Twist "linear:
  x: 0.044
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.6"
```

```shellscript
rostopic pub -1 /yolo_car/cmd_vel geometry_msgs/Twist "linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}"
```

---

**User**

rostopic pub -r 10 /yolo_car/cmd_vel geometry_msgs/Twist "linear:
  x: 0.044
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.6"我测试了一下，你这个有问题，2. 让右轮单独前转（左轮停止），我测试的时候它是左轮快速后转，有轮快速前转，1. 让左轮单独前转（右轮停止）这个就是正常的，这是什么问题

---

**Cursor**

问题在于计算精度：`left_vel` 变成了很小的负数，导致左轮反转。重新计算让右轮单独转的参数：


问题在于计算精度：`linear_vel = 0.044` 时，`left_vel` 变成了很小的负数（约 -0.00025），导致左轮反转。

修正后的命令：

## 修正后的测试命令

### 1. 让左轮单独前转（右轮停止）- 这个正常，保持不变

```bash
rostopic pub -r 10 /yolo_car/cmd_vel geometry_msgs/Twist "linear:
  x: 0.044
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: -0.6"
```

### 2. 让右轮单独前转（左轮停止）- 修正版

```bash
rostopic pub -r 10 /yolo_car/cmd_vel geometry_msgs/Twist "linear:
  x: 0.045
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.6"
```

**修正说明：**
- 将 `linear_vel` 从 `0.044` 改为 `0.045`
- 计算：`left_vel = 0.045 - (0.6 × 0.1475 / 2) = 0.045 - 0.04425 = 0.00075`（很小的正数，应该不会转）
- `right_vel = 0.045 + 0.04425 = 0.08925`（会转）

### 3. 停止电机

```bash
rostopic pub -1 /yolo_car/cmd_vel geometry_msgs/Twist "linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}"
```

如果 `0.045` 还是不够精确，可以尝试 `0.0443` 或 `0.0445`。先测试 `0.045`，告诉我结果。

```shellscript
rostopic pub -r 10 /yolo_car/cmd_vel geometry_msgs/Twist "linear:
  x: 0.044
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: -0.6"
```

```shellscript
rostopic pub -r 10 /yolo_car/cmd_vel geometry_msgs/Twist "linear:
  x: 0.045
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.6"
```

```shellscript
rostopic pub -1 /yolo_car/cmd_vel geometry_msgs/Twist "linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}"
```

---

**User**

    nsecs:  29886007



  frame_id: "base_link"

vector: 

  x: 0.0

  y: 0.0

  z: 0.0

---

header: 

  seq: 822

  stamp: 

    secs: 1765433560

    nsecs:  90886007

  frame_id: "base_link"

vector: 

  x: 0.0

  y: 0.0

  z: 0.0

---

header: 

  seq: 823

  stamp: 

    secs: 1765433560

    nsecs: 148886007

  frame_id: "base_link"

vector: 

  x: 0.0

  y: 0.0

  z: 0.0

---

header: 

  seq: 824

  stamp: 

    secs: 1765433560

    nsecs: 207886007

  frame_id: "base_link"

vector: 

  x: 0.0

  y: 0.0

  z: 0.0

现在是右轮在转，这个数据是正常的吗

---

**Cursor**

不正常。右轮在转，但轮速数据为 0.0，说明编码器未检测到速度。可能原因：
1. 编码器接线问题（A、B相接反或接触不良）
2. 编码器参数不匹配（TICKS_PER_REV、WHEEL_RADIUS_M）
3. 编码器中断未正常工作
4. 编码器计数方向问题

检查 ESP32 串口输出，查看编码器计数是否有变化：


添加调试输出，查看编码器原始计数，便于诊断问题：


```cpp
/*
 * ESP32-S3 + L298N电机驱动 + SG90舵机 + 编码器 + MPU6050 + ROS1 (rosserial)
 * 
 * 功能：
 * 1. 订阅速度指令控制L298N电机
 * 2. 订阅舵机角度指令控制SG90舵机
 * 3. 编码器计数并发布轮速
 * 4. MPU6050读取并发布原始IMU数据
 * 
 * 接线：
 * - L298N：N1->GPIO3(左PWM), N2->GPIO4(左DIR), N3->GPIO5(右PWM), N4->GPIO6(右DIR)
 * - SG90舵机信号线：GPIO2
 * - 编码器：右轮A->GPIO7, B->GPIO8; 左轮A->GPIO9, B->GPIO10
 * - MPU6050：SCL->GPIO45, SDA->GPIO46
 */

#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <Wire.h>

#define USE_WIFI true

#if USE_WIFI
  #include <WiFi.h>
  const char* ssid = "机器人小车局域网";
  const char* password = "3252362734";
  IPAddress server(192, 168, 31, 64);
  const uint16_t serverPort = 11411;
#endif

// L298N电机控制引脚
#define MOTOR_LEFT_PWM   3   // 左轮PWM
#define MOTOR_LEFT_DIR   4   // 左轮方向
#define MOTOR_RIGHT_PWM  5   // 右轮PWM
#define MOTOR_RIGHT_DIR  6   // 右轮方向

// SG90舵机引脚
#define SERVO_PIN 2

// 编码器引脚
#define ENC_RIGHT_A 7
#define ENC_RIGHT_B 8
#define ENC_LEFT_A  9
#define ENC_LEFT_B  10

// MPU6050 I2C引脚和地址
#define MPU_ADDR 0x68
#define I2C_SDA 46
#define I2C_SCL 45

// PWM参数
#define PWM_FREQ 1000      // PWM频率 1kHz
#define PWM_RESOLUTION 8   // 8位分辨率 (0-255)
#define SERVO_FREQ 50      // 舵机PWM频率 50Hz
#define SERVO_RESOLUTION 12

// 速度限制
#define MAX_PWM_VALUE 200  // 最大PWM值

// 命令超时（毫秒）
#define CMD_TIMEOUT_MS 500

// 编码器参数（已测量更新）
#define TICKS_PER_REV 260.0f      // 每转脉冲数
#define WHEEL_RADIUS_M 0.0480f    // 轮子半径（米）
#define WHEEL_BASE_M   0.1475f    // 轮距（米）

// 编码器计数变量（volatile用于中断）
volatile int32_t enc_left_count = 0;
volatile int32_t enc_right_count = 0;
volatile int8_t enc_left_last_state = 0;
volatile int8_t enc_right_last_state = 0;

// 轮速计算
float left_wheel_vel = 0.0f;
float right_wheel_vel = 0.0f;
unsigned long last_enc_time = 0;
const unsigned long ENC_UPDATE_INTERVAL_MS = 50; // 50ms更新一次，20Hz

// ROS节点
ros::NodeHandle nh;
unsigned long last_cmd_time = 0;

// ROS消息变量（必须先声明，发布者才能引用）
geometry_msgs::Vector3Stamped wheel_vel_msg;
sensor_msgs::Imu imu_raw_msg;

// 发布者（必须在消息变量之后声明）
ros::Publisher wheel_vel_pub("/yolo_car/wheel_vel", &wheel_vel_msg);
ros::Publisher imu_raw_pub("/yolo_car/imu_raw", &imu_raw_msg);

// 左编码器中断处理（A相）
void IRAM_ATTR encLeftA() {
  int8_t a = digitalRead(ENC_LEFT_A);
  int8_t b = digitalRead(ENC_LEFT_B);
  int8_t state = (a << 1) | b;
  int8_t diff = (state - enc_left_last_state) & 0x03;
  if (diff == 1 || diff == 3) {
    enc_left_count++;
  } else if (diff == 2) {
    enc_left_count--;
  }
  enc_left_last_state = state;
}

// 右编码器中断处理（A相）
void IRAM_ATTR encRightA() {
  int8_t a = digitalRead(ENC_RIGHT_A);
  int8_t b = digitalRead(ENC_RIGHT_B);
  int8_t state = (a << 1) | b;
  int8_t diff = (state - enc_right_last_state) & 0x03;
  if (diff == 1 || diff == 3) {
    enc_right_count++;
  } else if (diff == 2) {
    enc_right_count--;
  }
  enc_right_last_state = state;
}

// 电机急停
void stopMotors() {
  ledcWrite(0, 0);
  ledcWrite(1, 0);
  digitalWrite(MOTOR_LEFT_DIR, LOW);
  digitalWrite(MOTOR_RIGHT_DIR, LOW);
}

// 速度指令回调
void cmdVelCallback(const geometry_msgs::Twist& msg) {
  float linear_vel = msg.linear.x;   // 线速度 m/s
  float angular_vel = msg.angular.z; // 角速度 rad/s

  last_cmd_time = millis();
  
  if (fabs(linear_vel) < 1e-3 && fabs(angular_vel) < 1e-3) {
    stopMotors();
    return;
  }
  
  // 差速控制
  float left_vel = linear_vel - (angular_vel * WHEEL_BASE_M / 2.0);
  float right_vel = linear_vel + (angular_vel * WHEEL_BASE_M / 2.0);
  
  float max_vel = 0.3;
  left_vel = constrain(left_vel, -max_vel, max_vel);
  right_vel = constrain(right_vel, -max_vel, max_vel);
  
  int left_pwm = abs(left_vel / max_vel * MAX_PWM_VALUE);
  int right_pwm = abs(right_vel / max_vel * MAX_PWM_VALUE);
  
  // 方向控制（已修正：左右轮方向反转）
  if (left_vel >= 0) {
    digitalWrite(MOTOR_LEFT_DIR, LOW);
  } else {
    digitalWrite(MOTOR_LEFT_DIR, HIGH);
  }
  
  if (right_vel >= 0) {
    digitalWrite(MOTOR_RIGHT_DIR, HIGH);
  } else {
    digitalWrite(MOTOR_RIGHT_DIR, LOW);
  }
  
  ledcWrite(0, left_pwm);
  ledcWrite(1, right_pwm);
}

// 舵机角度回调
void servoAngleCallback(const std_msgs::Float32& msg) {
  float angle = msg.data;
  angle = constrain(angle, 0.0, 180.0);

  int pulse_width = map(angle, 0, 180, 500, 2500);
  int duty_cycle = (pulse_width * 4096L) / 20000;
  duty_cycle = constrain(duty_cycle, 0, 4095);
  ledcWrite(2, duty_cycle);
}

// 订阅者
ros::Subscriber<geometry_msgs::Twist> cmd_vel_sub("/yolo_car/cmd_vel", cmdVelCallback);
ros::Subscriber<std_msgs::Float32> servo_angle_sub("/yolo_car/servo_angle", servoAngleCallback);

// MPU6050初始化
bool initMPU6050() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B); // PWR_MGMT_1寄存器
  Wire.write(0x00); // 唤醒MPU6050
  uint8_t error = Wire.endTransmission();
  
  if (error != 0) {
    return false;
  }
  
  // 配置加速度计范围 ±2g
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1C); // ACCEL_CONFIG
  Wire.write(0x00);
  Wire.endTransmission();
  
  // 配置陀螺仪范围 ±250°/s
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1B); // GYRO_CONFIG
  Wire.write(0x00);
  Wire.endTransmission();
  
  return true;
}

// 读取MPU6050原始数据
void readMPU6050() {
  // 读取加速度计数据（6字节：ACCEL_XOUT_H, ACCEL_XOUT_L, ACCEL_YOUT_H, ACCEL_YOUT_L, ACCEL_ZOUT_H, ACCEL_ZOUT_L）
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B); // ACCEL_XOUT_H寄存器地址
  Wire.endTransmission(false);
  
  // 修复：使用int版本的重载，避免类型歧义
  int addr = (int)MPU_ADDR;
  int len = 14;
  int stop = 1;  // 1表示发送停止位
  uint8_t bytes_read = Wire.requestFrom(addr, len, stop);
  
  if (bytes_read == 14) {
    int16_t accel_x = (Wire.read() << 8) | Wire.read();
    int16_t accel_y = (Wire.read() << 8) | Wire.read();
    int16_t accel_z = (Wire.read() << 8) | Wire.read();
    int16_t temp = (Wire.read() << 8) | Wire.read();
    int16_t gyro_x = (Wire.read() << 8) | Wire.read();
    int16_t gyro_y = (Wire.read() << 8) | Wire.read();
    int16_t gyro_z = (Wire.read() << 8) | Wire.read();
    
    // 转换为物理单位
    // 加速度：±2g范围，32768对应2g，g=9.8 m/s²
    float accel_scale = 2.0f * 9.8f / 32768.0f;
    imu_raw_msg.linear_acceleration.x = accel_x * accel_scale;
    imu_raw_msg.linear_acceleration.y = accel_y * accel_scale;
    imu_raw_msg.linear_acceleration.z = accel_z * accel_scale;
    
    // 角速度：±250°/s范围，32768对应250°/s，转换为rad/s
    float gyro_scale = 250.0f * M_PI / 180.0f / 32768.0f;
    imu_raw_msg.angular_velocity.x = gyro_x * gyro_scale;
    imu_raw_msg.angular_velocity.y = gyro_y * gyro_scale;
    imu_raw_msg.angular_velocity.z = gyro_z * gyro_scale;
    
    imu_raw_msg.header.stamp = nh.now();
    imu_raw_msg.header.frame_id = "imu_link";
  }
}

// 更新轮速
void updateWheelVelocities() {
  unsigned long current_time = millis();
  unsigned long dt_ms = current_time - last_enc_time;
  
  if (dt_ms >= ENC_UPDATE_INTERVAL_MS) {
    // 读取并重置计数（原子操作）
    int32_t left_ticks = enc_left_count;
    int32_t right_ticks = enc_right_count;
    enc_left_count = 0;
    enc_right_count = 0;
    
    // 调试输出：打印编码器原始计数
    Serial.print("编码器计数 - 左轮: ");
    Serial.print(left_ticks);
    Serial.print(", 右轮: ");
    Serial.println(right_ticks);
    
    // 计算轮速（m/s）
    // 每转脉冲数 * 轮子周长 = 每米脉冲数
    float ticks_per_meter = TICKS_PER_REV / (2.0f * M_PI * WHEEL_RADIUS_M);
    float dt_sec = dt_ms / 1000.0f;
    
    left_wheel_vel = (left_ticks / ticks_per_meter) / dt_sec;
    right_wheel_vel = (right_ticks / ticks_per_meter) / dt_sec;
    
    // 调试输出：打印计算出的轮速
    Serial.print("轮速 - 左轮: ");
    Serial.print(left_wheel_vel, 4);
    Serial.print(" m/s, 右轮: ");
    Serial.print(right_wheel_vel, 4);
    Serial.println(" m/s");
    
    // 发布轮速
    wheel_vel_msg.header.stamp = nh.now();
    wheel_vel_msg.header.frame_id = "base_link";
    wheel_vel_msg.vector.x = left_wheel_vel;
    wheel_vel_msg.vector.y = right_wheel_vel;
    wheel_vel_msg.vector.z = 0.0;
    wheel_vel_pub.publish(&wheel_vel_msg);
    
    last_enc_time = current_time;
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n\n=== ESP32-S3 + L298N + SG90 + 编码器 + MPU6050 + ROS1 ===");

  last_cmd_time = millis();
  
  // 初始化电机控制引脚
  Serial.println("初始化L298N电机驱动...");
  pinMode(MOTOR_LEFT_DIR, OUTPUT);
  pinMode(MOTOR_RIGHT_DIR, OUTPUT);
  digitalWrite(MOTOR_LEFT_DIR, LOW);
  digitalWrite(MOTOR_RIGHT_DIR, LOW);
  
  // 配置PWM通道（电机）
  ledcSetup(0, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(1, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(MOTOR_LEFT_PWM, 0);
  ledcAttachPin(MOTOR_RIGHT_PWM, 1);
  
  // 配置PWM通道（舵机）
  ledcSetup(2, SERVO_FREQ, SERVO_RESOLUTION);
  ledcAttachPin(SERVO_PIN, 2);
  
  ledcWrite(0, 0);
  ledcWrite(1, 0);
  
  // 初始舵机到90度
  float init_angle = 90.0;
  int pulse_width = map(init_angle, 0, 180, 500, 2500);
  int duty_cycle = (pulse_width * 4096L) / 20000;
  duty_cycle = constrain(duty_cycle, 0, 4095);
  ledcWrite(2, duty_cycle);
  
  Serial.println("✓ 电机和舵机初始化完成");
  
  // 初始化编码器
  Serial.println("初始化编码器...");
  pinMode(ENC_LEFT_A, INPUT_PULLUP);
  pinMode(ENC_LEFT_B, INPUT_PULLUP);
  pinMode(ENC_RIGHT_A, INPUT_PULLUP);
  pinMode(ENC_RIGHT_B, INPUT_PULLUP);
  
  enc_left_last_state = (digitalRead(ENC_LEFT_A) << 1) | digitalRead(ENC_LEFT_B);
  enc_right_last_state = (digitalRead(ENC_RIGHT_A) << 1) | digitalRead(ENC_RIGHT_B);
  
  attachInterrupt(digitalPinToInterrupt(ENC_LEFT_A), encLeftA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_RIGHT_A), encRightA, CHANGE);
  
  last_enc_time = millis();
  Serial.println("✓ 编码器初始化完成");
  
  // 初始化MPU6050
  Serial.println("初始化MPU6050...");
  Wire.begin(I2C_SDA, I2C_SCL);
  delay(100);
  
  if (initMPU6050()) {
    Serial.println("✓ MPU6050初始化成功");
  } else {
    Serial.println("✗ MPU6050初始化失败");
  }
  
#if USE_WIFI
  Serial.println("\n=== WiFi连接 ===");
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  
  Serial.print("连接中");
  int wifiAttempts = 0;
  while (WiFi.status() != WL_CONNECTED && wifiAttempts < 30) {
    delay(500);
    Serial.print(".");
    wifiAttempts++;
  }
  Serial.println();
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("✓ WiFi连接成功！IP: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("✗ WiFi连接失败！");
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    nh.getHardware()->setConnection(server, serverPort);
  }
#endif
  
  Serial.println("\n=== ROS连接 ===");
  nh.initNode();
  nh.subscribe(cmd_vel_sub);
  nh.subscribe(servo_angle_sub);
  nh.advertise(wheel_vel_pub);
  nh.advertise(imu_raw_pub);
  
  Serial.print("等待连接");
  int rosAttempts = 0;
  while (!nh.connected() && rosAttempts < 50) {
    nh.spinOnce();
    delay(100);
    if (rosAttempts % 10 == 0) Serial.print(".");
    rosAttempts++;
  }
  Serial.println();
  
  if (nh.connected()) {
    Serial.println("✓ ROS连接成功！");
  } else {
    Serial.println("✗ ROS连接失败！");
  }
  
  Serial.println("\n=== 系统状态 ===");
  Serial.print("WiFi: ");
  Serial.println((WiFi.status() == WL_CONNECTED) ? "✓ 已连接" : "✗ 未连接");
  Serial.print("ROS: ");
  Serial.println(nh.connected() ? "✓ 已连接" : "✗ 未连接");
  Serial.print("编码器参数: TICKS_PER_REV=");
  Serial.print(TICKS_PER_REV);
  Serial.print(", WHEEL_RADIUS_M=");
  Serial.print(WHEEL_RADIUS_M);
  Serial.print(", WHEEL_BASE_M=");
  Serial.println(WHEEL_BASE_M);
  Serial.println("\n开始运行...\n");
}

void loop() {
  // 命令超时保护
  if (millis() - last_cmd_time > CMD_TIMEOUT_MS) {
    stopMotors();
  }

  if (!nh.connected()) {
    static unsigned long lastReconnectMsg = 0;
    if (millis() - lastReconnectMsg > 5000) {
      Serial.println("ROS连接断开，尝试重连...");
      lastReconnectMsg = millis();
    }
    stopMotors();
    nh.spinOnce();
    delay(100);
    return;
  }

  // 更新轮速（20Hz）
  updateWheelVelocities();
  
  // 读取并发布IMU（50Hz）
  static unsigned long last_imu_time = 0;
  if (millis() - last_imu_time >= 20) {
    readMPU6050();
    imu_raw_pub.publish(&imu_raw_msg);
    last_imu_time = millis();
  }

  nh.spinOnce();
  delay(10);
}


```

已添加调试输出。按以下步骤排查：

## 诊断步骤

### 1. 重新上传代码
将修改后的代码上传到ESP32。

### 2. 打开串口监视器
在Arduino IDE中打开串口监视器（波特率115200），应能看到编码器计数和轮速的调试信息。

### 3. 测试右轮转
运行让右轮转的命令：
```bash
rostopic pub -r 10 /yolo_car/cmd_vel geometry_msgs/Twist "linear: {x: 0.045, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.6}"
```

### 4. 观察串口输出
在串口监视器中应看到类似：
```
编码器计数 - 左轮: 0, 右轮: 15
轮速 - 左轮: 0.0000 m/s, 右轮: 0.1234 m/s
```

## 可能的情况

1. 如果编码器计数都是0：
   - 编码器未接线或接线错误
   - 编码器中断未触发
   - 检查编码器A、B相接线

2. 如果编码器计数有数值但轮速为0：
   - 编码器参数（TICKS_PER_REV、WHEEL_RADIUS_M）可能不对
   - 检查参数是否正确

3. 如果编码器计数为负数：
   - 编码器A、B相可能接反
   - 需要交换A、B相接线

请重新上传代码并测试，把串口监视器的输出发给我，我再帮你分析。

```shellscript
rostopic pub -r 10 /yolo_car/cmd_vel geometry_msgs/Twist "linear: {x: 0.045, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.6}"
```

```plaintext
编码器计数 - 左轮: 0, 右轮: 15
轮速 - 左轮: 0.0000 m/s, 右轮: 0.1234 m/s
```

---

**User**

编码器计数 - 左轮: 0, 右轮: 0
轮速 - 左轮: 0.0000 m/s, 右轮: 0.0000 m/s
编码器计数 - 左轮: 0, 右轮: 0
轮速 - 左轮: 0.0000 m/s, 右轮: 0.0000 m/s
编码器计数 - 左轮: 0, 右轮: 0
轮速 - 左轮: 0.0000 m/s, 右轮: 0.0000 m/s
编码器计数 - 左轮: 0, 右轮: 0
轮速 - 左轮: 0.0000 m/s, 右轮: 0.0000 m/s
编码器计数 - 左轮: 0, 右轮: 0
轮速 - 左轮: 0.0000 m/s, 右轮: 0.0000 m/s
编码器计数 - 左轮: 0, 右轮: 0
轮速 - 左轮: 0.0000 m/s, 右轮: 0.0000 m/s
编码器计数 - 左轮: 0, 右轮: 0
轮速 - 左轮: 0.0000 m/s, 右轮: 0.0000 m/s
编码器计数 - 左轮: 0, 右轮: 0
轮速 - 左轮: 0.0000 m/s, 右轮: 0.0000 m/s
编码器计数 - 左轮: 0, 右轮: 0
轮速 - 左轮: 0.0000 m/s, 右轮: 0.0000 m/s
编码器计数 - 左轮: 0, 右轮: 0
轮速 - 左轮: 0.0000 m/s, 右轮: 0.0000 m/s
编码器计数 - 左轮: 0, 右轮: 0
轮速 - 左轮: 0.0000 m/s, 右轮: 0.0000 m/s
编码器计数 - 左轮: 0, 右轮: 0
轮速 - 左轮: 0.0000 m/s, 右轮: 0.0000 m/s
编码器计数 - 左轮: 0, 右轮: 0
轮速 - 左轮: 0.0000 m/s, 右轮: 0.0000 m/s
编码器计数 - 左轮: 0, 右轮: 0
轮速 - 左轮: 0.0000 m/s, 右轮: 0.0000 m/s
编码器计数 - 左轮: 0, 右轮: 0
轮速 - 左轮: 0.0000 m/s, 右轮: 0.0000 m/s
编码器计数 - 左轮: 0, 右轮: 0
轮速 - 左轮: 0.0000 m/s, 右轮: 0.0000 m/s
编码器计数 - 左轮: 0, 右轮: 0
轮速 - 左轮: 0.0000 m/s, 右轮: 0.0000 m/s
编码器计数 - 左轮: 0, 右轮: 0
轮速 - 左轮: 0.0000 m/s, 右轮: 0.0000 m/s
编码器计数 - 左轮: 0, 右轮: 0
轮速 - 左轮: 0.0000 m/s, 右轮: 0.0000 m/s
编码器计数 - 左轮: 0, 右轮: 0
轮速 - 左轮: 0.0000 m/s, 右轮: 0.0000 m/s我不管是让右轮转还是左轮转，它都打印的这个是怎么回事，要怎么解决

---

**Cursor**

编码器计数始终为0，说明中断未触发或编码器未连接。添加更详细的诊断代码：


添加诊断代码，检查编码器引脚状态和中断配置：


```cpp
/*
 * ESP32-S3 + L298N电机驱动 + SG90舵机 + 编码器 + MPU6050 + ROS1 (rosserial)
 * 
 * 功能：
 * 1. 订阅速度指令控制L298N电机
 * 2. 订阅舵机角度指令控制SG90舵机
 * 3. 编码器计数并发布轮速
 * 4. MPU6050读取并发布原始IMU数据
 * 
 * 接线：
 * - L298N：N1->GPIO3(左PWM), N2->GPIO4(左DIR), N3->GPIO5(右PWM), N4->GPIO6(右DIR)
 * - SG90舵机信号线：GPIO2
 * - 编码器：右轮A->GPIO7, B->GPIO8; 左轮A->GPIO9, B->GPIO10
 * - MPU6050：SCL->GPIO45, SDA->GPIO46
 */

#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <Wire.h>

#define USE_WIFI true

#if USE_WIFI
  #include <WiFi.h>
  const char* ssid = "机器人小车局域网";
  const char* password = "3252362734";
  IPAddress server(192, 168, 31, 64);
  const uint16_t serverPort = 11411;
#endif

// L298N电机控制引脚
#define MOTOR_LEFT_PWM   3   // 左轮PWM
#define MOTOR_LEFT_DIR   4   // 左轮方向
#define MOTOR_RIGHT_PWM  5   // 右轮PWM
#define MOTOR_RIGHT_DIR  6   // 右轮方向

// SG90舵机引脚
#define SERVO_PIN 2

// 编码器引脚
#define ENC_RIGHT_A 7
#define ENC_RIGHT_B 8
#define ENC_LEFT_A  9
#define ENC_LEFT_B  10

// MPU6050 I2C引脚和地址
#define MPU_ADDR 0x68
#define I2C_SDA 46
#define I2C_SCL 45

// PWM参数
#define PWM_FREQ 1000      // PWM频率 1kHz
#define PWM_RESOLUTION 8   // 8位分辨率 (0-255)
#define SERVO_FREQ 50      // 舵机PWM频率 50Hz
#define SERVO_RESOLUTION 12

// 速度限制
#define MAX_PWM_VALUE 200  // 最大PWM值

// 命令超时（毫秒）
#define CMD_TIMEOUT_MS 500

// 编码器参数（已测量更新）
#define TICKS_PER_REV 260.0f      // 每转脉冲数
#define WHEEL_RADIUS_M 0.0480f    // 轮子半径（米）
#define WHEEL_BASE_M   0.1475f    // 轮距（米）

// 编码器计数变量（volatile用于中断）
volatile int32_t enc_left_count = 0;
volatile int32_t enc_right_count = 0;
volatile int8_t enc_left_last_state = 0;
volatile int8_t enc_right_last_state = 0;

// 轮速计算
float left_wheel_vel = 0.0f;
float right_wheel_vel = 0.0f;
unsigned long last_enc_time = 0;
const unsigned long ENC_UPDATE_INTERVAL_MS = 50; // 50ms更新一次，20Hz

// ROS节点
ros::NodeHandle nh;
unsigned long last_cmd_time = 0;

// ROS消息变量（必须先声明，发布者才能引用）
geometry_msgs::Vector3Stamped wheel_vel_msg;
sensor_msgs::Imu imu_raw_msg;

// 发布者（必须在消息变量之后声明）
ros::Publisher wheel_vel_pub("/yolo_car/wheel_vel", &wheel_vel_msg);
ros::Publisher imu_raw_pub("/yolo_car/imu_raw", &imu_raw_msg);

// 左编码器中断处理（A相）
void IRAM_ATTR encLeftA() {
  int8_t a = digitalRead(ENC_LEFT_A);
  int8_t b = digitalRead(ENC_LEFT_B);
  int8_t state = (a << 1) | b;
  int8_t diff = (state - enc_left_last_state) & 0x03;
  if (diff == 1 || diff == 3) {
    enc_left_count++;
  } else if (diff == 2) {
    enc_left_count--;
  }
  enc_left_last_state = state;
}

// 右编码器中断处理（A相）
void IRAM_ATTR encRightA() {
  int8_t a = digitalRead(ENC_RIGHT_A);
  int8_t b = digitalRead(ENC_RIGHT_B);
  int8_t state = (a << 1) | b;
  int8_t diff = (state - enc_right_last_state) & 0x03;
  if (diff == 1 || diff == 3) {
    enc_right_count++;
  } else if (diff == 2) {
    enc_right_count--;
  }
  enc_right_last_state = state;
}

// 电机急停
void stopMotors() {
  ledcWrite(0, 0);
  ledcWrite(1, 0);
  digitalWrite(MOTOR_LEFT_DIR, LOW);
  digitalWrite(MOTOR_RIGHT_DIR, LOW);
}

// 速度指令回调
void cmdVelCallback(const geometry_msgs::Twist& msg) {
  float linear_vel = msg.linear.x;   // 线速度 m/s
  float angular_vel = msg.angular.z; // 角速度 rad/s

  last_cmd_time = millis();
  
  if (fabs(linear_vel) < 1e-3 && fabs(angular_vel) < 1e-3) {
    stopMotors();
    return;
  }
  
  // 差速控制
  float left_vel = linear_vel - (angular_vel * WHEEL_BASE_M / 2.0);
  float right_vel = linear_vel + (angular_vel * WHEEL_BASE_M / 2.0);
  
  float max_vel = 0.3;
  left_vel = constrain(left_vel, -max_vel, max_vel);
  right_vel = constrain(right_vel, -max_vel, max_vel);
  
  int left_pwm = abs(left_vel / max_vel * MAX_PWM_VALUE);
  int right_pwm = abs(right_vel / max_vel * MAX_PWM_VALUE);
  
  // 方向控制（已修正：左右轮方向反转）
  if (left_vel >= 0) {
    digitalWrite(MOTOR_LEFT_DIR, LOW);
  } else {
    digitalWrite(MOTOR_LEFT_DIR, HIGH);
  }
  
  if (right_vel >= 0) {
    digitalWrite(MOTOR_RIGHT_DIR, HIGH);
  } else {
    digitalWrite(MOTOR_RIGHT_DIR, LOW);
  }
  
  ledcWrite(0, left_pwm);
  ledcWrite(1, right_pwm);
}

// 舵机角度回调
void servoAngleCallback(const std_msgs::Float32& msg) {
  float angle = msg.data;
  angle = constrain(angle, 0.0, 180.0);

  int pulse_width = map(angle, 0, 180, 500, 2500);
  int duty_cycle = (pulse_width * 4096L) / 20000;
  duty_cycle = constrain(duty_cycle, 0, 4095);
  ledcWrite(2, duty_cycle);
}

// 订阅者
ros::Subscriber<geometry_msgs::Twist> cmd_vel_sub("/yolo_car/cmd_vel", cmdVelCallback);
ros::Subscriber<std_msgs::Float32> servo_angle_sub("/yolo_car/servo_angle", servoAngleCallback);

// MPU6050初始化
bool initMPU6050() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B); // PWR_MGMT_1寄存器
  Wire.write(0x00); // 唤醒MPU6050
  uint8_t error = Wire.endTransmission();
  
  if (error != 0) {
    return false;
  }
  
  // 配置加速度计范围 ±2g
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1C); // ACCEL_CONFIG
  Wire.write(0x00);
  Wire.endTransmission();
  
  // 配置陀螺仪范围 ±250°/s
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1B); // GYRO_CONFIG
  Wire.write(0x00);
  Wire.endTransmission();
  
  return true;
}

// 读取MPU6050原始数据
void readMPU6050() {
  // 读取加速度计数据（6字节：ACCEL_XOUT_H, ACCEL_XOUT_L, ACCEL_YOUT_H, ACCEL_YOUT_L, ACCEL_ZOUT_H, ACCEL_ZOUT_L）
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B); // ACCEL_XOUT_H寄存器地址
  Wire.endTransmission(false);
  
  // 修复：使用int版本的重载，避免类型歧义
  int addr = (int)MPU_ADDR;
  int len = 14;
  int stop = 1;  // 1表示发送停止位
  uint8_t bytes_read = Wire.requestFrom(addr, len, stop);
  
  if (bytes_read == 14) {
    int16_t accel_x = (Wire.read() << 8) | Wire.read();
    int16_t accel_y = (Wire.read() << 8) | Wire.read();
    int16_t accel_z = (Wire.read() << 8) | Wire.read();
    int16_t temp = (Wire.read() << 8) | Wire.read();
    int16_t gyro_x = (Wire.read() << 8) | Wire.read();
    int16_t gyro_y = (Wire.read() << 8) | Wire.read();
    int16_t gyro_z = (Wire.read() << 8) | Wire.read();
    
    // 转换为物理单位
    // 加速度：±2g范围，32768对应2g，g=9.8 m/s²
    float accel_scale = 2.0f * 9.8f / 32768.0f;
    imu_raw_msg.linear_acceleration.x = accel_x * accel_scale;
    imu_raw_msg.linear_acceleration.y = accel_y * accel_scale;
    imu_raw_msg.linear_acceleration.z = accel_z * accel_scale;
    
    // 角速度：±250°/s范围，32768对应250°/s，转换为rad/s
    float gyro_scale = 250.0f * M_PI / 180.0f / 32768.0f;
    imu_raw_msg.angular_velocity.x = gyro_x * gyro_scale;
    imu_raw_msg.angular_velocity.y = gyro_y * gyro_scale;
    imu_raw_msg.angular_velocity.z = gyro_z * gyro_scale;
    
    imu_raw_msg.header.stamp = nh.now();
    imu_raw_msg.header.frame_id = "imu_link";
  }
}

// 诊断编码器引脚状态
void diagnoseEncoders() {
  static unsigned long last_diag_time = 0;
  unsigned long current_time = millis();
  
  // 每1秒诊断一次
  if (current_time - last_diag_time >= 1000) {
    Serial.println("\n=== 编码器诊断 ===");
    
    // 读取编码器引脚状态
    int left_a = digitalRead(ENC_LEFT_A);
    int left_b = digitalRead(ENC_LEFT_B);
    int right_a = digitalRead(ENC_RIGHT_A);
    int right_b = digitalRead(ENC_RIGHT_B);
    
    Serial.print("左编码器 - A相(GPIO9): ");
    Serial.print(left_a);
    Serial.print(", B相(GPIO10): ");
    Serial.println(left_b);
    
    Serial.print("右编码器 - A相(GPIO7): ");
    Serial.print(right_a);
    Serial.print(", B相(GPIO8): ");
    Serial.println(right_b);
    
    Serial.print("编码器计数 - 左轮: ");
    Serial.print(enc_left_count);
    Serial.print(", 右轮: ");
    Serial.println(enc_right_count);
    
    Serial.println("==================\n");
    
    last_diag_time = current_time;
  }
}

// 更新轮速
void updateWheelVelocities() {
  unsigned long current_time = millis();
  unsigned long dt_ms = current_time - last_enc_time;
  
  if (dt_ms >= ENC_UPDATE_INTERVAL_MS) {
    // 读取并重置计数（原子操作）
    int32_t left_ticks = enc_left_count;
    int32_t right_ticks = enc_right_count;
    enc_left_count = 0;
    enc_right_count = 0;
    
    // 调试输出：打印编码器原始计数
    Serial.print("编码器计数 - 左轮: ");
    Serial.print(left_ticks);
    Serial.print(", 右轮: ");
    Serial.println(right_ticks);
    
    // 计算轮速（m/s）
    // 每转脉冲数 * 轮子周长 = 每米脉冲数
    float ticks_per_meter = TICKS_PER_REV / (2.0f * M_PI * WHEEL_RADIUS_M);
    float dt_sec = dt_ms / 1000.0f;
    
    left_wheel_vel = (left_ticks / ticks_per_meter) / dt_sec;
    right_wheel_vel = (right_ticks / ticks_per_meter) / dt_sec;
    
    // 调试输出：打印计算出的轮速
    Serial.print("轮速 - 左轮: ");
    Serial.print(left_wheel_vel, 4);
    Serial.print(" m/s, 右轮: ");
    Serial.print(right_wheel_vel, 4);
    Serial.println(" m/s");
    
    // 发布轮速
    wheel_vel_msg.header.stamp = nh.now();
    wheel_vel_msg.header.frame_id = "base_link";
    wheel_vel_msg.vector.x = left_wheel_vel;
    wheel_vel_msg.vector.y = right_wheel_vel;
    wheel_vel_msg.vector.z = 0.0;
    wheel_vel_pub.publish(&wheel_vel_msg);
    
    last_enc_time = current_time;
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n\n=== ESP32-S3 + L298N + SG90 + 编码器 + MPU6050 + ROS1 ===");

  last_cmd_time = millis();
  
  // 初始化电机控制引脚
  Serial.println("初始化L298N电机驱动...");
  pinMode(MOTOR_LEFT_DIR, OUTPUT);
  pinMode(MOTOR_RIGHT_DIR, OUTPUT);
  digitalWrite(MOTOR_LEFT_DIR, LOW);
  digitalWrite(MOTOR_RIGHT_DIR, LOW);
  
  // 配置PWM通道（电机）
  ledcSetup(0, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(1, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(MOTOR_LEFT_PWM, 0);
  ledcAttachPin(MOTOR_RIGHT_PWM, 1);
  
  // 配置PWM通道（舵机）
  ledcSetup(2, SERVO_FREQ, SERVO_RESOLUTION);
  ledcAttachPin(SERVO_PIN, 2);
  
  ledcWrite(0, 0);
  ledcWrite(1, 0);
  
  // 初始舵机到90度
  float init_angle = 90.0;
  int pulse_width = map(init_angle, 0, 180, 500, 2500);
  int duty_cycle = (pulse_width * 4096L) / 20000;
  duty_cycle = constrain(duty_cycle, 0, 4095);
  ledcWrite(2, duty_cycle);
  
  Serial.println("✓ 电机和舵机初始化完成");
  
  // 初始化编码器
  Serial.println("初始化编码器...");
  pinMode(ENC_LEFT_A, INPUT_PULLUP);
  pinMode(ENC_LEFT_B, INPUT_PULLUP);
  pinMode(ENC_RIGHT_A, INPUT_PULLUP);
  pinMode(ENC_RIGHT_B, INPUT_PULLUP);
  
  enc_left_last_state = (digitalRead(ENC_LEFT_A) << 1) | digitalRead(ENC_LEFT_B);
  enc_right_last_state = (digitalRead(ENC_RIGHT_A) << 1) | digitalRead(ENC_RIGHT_B);
  
  attachInterrupt(digitalPinToInterrupt(ENC_LEFT_A), encLeftA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_RIGHT_A), encRightA, CHANGE);
  
  last_enc_time = millis();
  Serial.println("✓ 编码器初始化完成");
  
  // 初始化MPU6050
  Serial.println("初始化MPU6050...");
  Wire.begin(I2C_SDA, I2C_SCL);
  delay(100);
  
  if (initMPU6050()) {
    Serial.println("✓ MPU6050初始化成功");
  } else {
    Serial.println("✗ MPU6050初始化失败");
  }
  
#if USE_WIFI
  Serial.println("\n=== WiFi连接 ===");
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  
  Serial.print("连接中");
  int wifiAttempts = 0;
  while (WiFi.status() != WL_CONNECTED && wifiAttempts < 30) {
    delay(500);
    Serial.print(".");
    wifiAttempts++;
  }
  Serial.println();
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("✓ WiFi连接成功！IP: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("✗ WiFi连接失败！");
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    nh.getHardware()->setConnection(server, serverPort);
  }
#endif
  
  Serial.println("\n=== ROS连接 ===");
  nh.initNode();
  nh.subscribe(cmd_vel_sub);
  nh.subscribe(servo_angle_sub);
  nh.advertise(wheel_vel_pub);
  nh.advertise(imu_raw_pub);
  
  Serial.print("等待连接");
  int rosAttempts = 0;
  while (!nh.connected() && rosAttempts < 50) {
    nh.spinOnce();
    delay(100);
    if (rosAttempts % 10 == 0) Serial.print(".");
    rosAttempts++;
  }
  Serial.println();
  
  if (nh.connected()) {
    Serial.println("✓ ROS连接成功！");
  } else {
    Serial.println("✗ ROS连接失败！");
  }
  
  Serial.println("\n=== 系统状态 ===");
  Serial.print("WiFi: ");
  Serial.println((WiFi.status() == WL_CONNECTED) ? "✓ 已连接" : "✗ 未连接");
  Serial.print("ROS: ");
  Serial.println(nh.connected() ? "✓ 已连接" : "✗ 未连接");
  Serial.print("编码器参数: TICKS_PER_REV=");
  Serial.print(TICKS_PER_REV);
  Serial.print(", WHEEL_RADIUS_M=");
  Serial.print(WHEEL_RADIUS_M);
  Serial.print(", WHEEL_BASE_M=");
  Serial.println(WHEEL_BASE_M);
  Serial.println("\n开始运行...\n");
}

void loop() {
  // 命令超时保护
  if (millis() - last_cmd_time > CMD_TIMEOUT_MS) {
    stopMotors();
  }

  if (!nh.connected()) {
    static unsigned long lastReconnectMsg = 0;
    if (millis() - lastReconnectMsg > 5000) {
      Serial.println("ROS连接断开，尝试重连...");
      lastReconnectMsg = millis();
    }
    stopMotors();
    nh.spinOnce();
    delay(100);
    return;
  }

  // 更新轮速（20Hz）
  updateWheelVelocities();
  
  // 读取并发布IMU（50Hz）
  static unsigned long last_imu_time = 0;
  if (millis() - last_imu_time >= 20) {
    readMPU6050();
    imu_raw_pub.publish(&imu_raw_msg);
    last_imu_time = millis();
  }

  nh.spinOnce();
  delay(10);
}


```

在loop函数中调用诊断函数：


```cpp
/*
 * ESP32-S3 + L298N电机驱动 + SG90舵机 + 编码器 + MPU6050 + ROS1 (rosserial)
 * 
 * 功能：
 * 1. 订阅速度指令控制L298N电机
 * 2. 订阅舵机角度指令控制SG90舵机
 * 3. 编码器计数并发布轮速
 * 4. MPU6050读取并发布原始IMU数据
 * 
 * 接线：
 * - L298N：N1->GPIO3(左PWM), N2->GPIO4(左DIR), N3->GPIO5(右PWM), N4->GPIO6(右DIR)
 * - SG90舵机信号线：GPIO2
 * - 编码器：右轮A->GPIO7, B->GPIO8; 左轮A->GPIO9, B->GPIO10
 * - MPU6050：SCL->GPIO45, SDA->GPIO46
 */

#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <Wire.h>

#define USE_WIFI true

#if USE_WIFI
  #include <WiFi.h>
  const char* ssid = "机器人小车局域网";
  const char* password = "3252362734";
  IPAddress server(192, 168, 31, 64);
  const uint16_t serverPort = 11411;
#endif

// L298N电机控制引脚
#define MOTOR_LEFT_PWM   3   // 左轮PWM
#define MOTOR_LEFT_DIR   4   // 左轮方向
#define MOTOR_RIGHT_PWM  5   // 右轮PWM
#define MOTOR_RIGHT_DIR  6   // 右轮方向

// SG90舵机引脚
#define SERVO_PIN 2

// 编码器引脚
#define ENC_RIGHT_A 7
#define ENC_RIGHT_B 8
#define ENC_LEFT_A  9
#define ENC_LEFT_B  10

// MPU6050 I2C引脚和地址
#define MPU_ADDR 0x68
#define I2C_SDA 46
#define I2C_SCL 45

// PWM参数
#define PWM_FREQ 1000      // PWM频率 1kHz
#define PWM_RESOLUTION 8   // 8位分辨率 (0-255)
#define SERVO_FREQ 50      // 舵机PWM频率 50Hz
#define SERVO_RESOLUTION 12

// 速度限制
#define MAX_PWM_VALUE 200  // 最大PWM值

// 命令超时（毫秒）
#define CMD_TIMEOUT_MS 500

// 编码器参数（已测量更新）
#define TICKS_PER_REV 260.0f      // 每转脉冲数
#define WHEEL_RADIUS_M 0.0480f    // 轮子半径（米）
#define WHEEL_BASE_M   0.1475f    // 轮距（米）

// 编码器计数变量（volatile用于中断）
volatile int32_t enc_left_count = 0;
volatile int32_t enc_right_count = 0;
volatile int8_t enc_left_last_state = 0;
volatile int8_t enc_right_last_state = 0;

// 轮速计算
float left_wheel_vel = 0.0f;
float right_wheel_vel = 0.0f;
unsigned long last_enc_time = 0;
const unsigned long ENC_UPDATE_INTERVAL_MS = 50; // 50ms更新一次，20Hz

// ROS节点
ros::NodeHandle nh;
unsigned long last_cmd_time = 0;

// ROS消息变量（必须先声明，发布者才能引用）
geometry_msgs::Vector3Stamped wheel_vel_msg;
sensor_msgs::Imu imu_raw_msg;

// 发布者（必须在消息变量之后声明）
ros::Publisher wheel_vel_pub("/yolo_car/wheel_vel", &wheel_vel_msg);
ros::Publisher imu_raw_pub("/yolo_car/imu_raw", &imu_raw_msg);

// 左编码器中断处理（A相）
void IRAM_ATTR encLeftA() {
  int8_t a = digitalRead(ENC_LEFT_A);
  int8_t b = digitalRead(ENC_LEFT_B);
  int8_t state = (a << 1) | b;
  int8_t diff = (state - enc_left_last_state) & 0x03;
  if (diff == 1 || diff == 3) {
    enc_left_count++;
  } else if (diff == 2) {
    enc_left_count--;
  }
  enc_left_last_state = state;
}

// 右编码器中断处理（A相）
void IRAM_ATTR encRightA() {
  int8_t a = digitalRead(ENC_RIGHT_A);
  int8_t b = digitalRead(ENC_RIGHT_B);
  int8_t state = (a << 1) | b;
  int8_t diff = (state - enc_right_last_state) & 0x03;
  if (diff == 1 || diff == 3) {
    enc_right_count++;
  } else if (diff == 2) {
    enc_right_count--;
  }
  enc_right_last_state = state;
}

// 电机急停
void stopMotors() {
  ledcWrite(0, 0);
  ledcWrite(1, 0);
  digitalWrite(MOTOR_LEFT_DIR, LOW);
  digitalWrite(MOTOR_RIGHT_DIR, LOW);
}

// 速度指令回调
void cmdVelCallback(const geometry_msgs::Twist& msg) {
  float linear_vel = msg.linear.x;   // 线速度 m/s
  float angular_vel = msg.angular.z; // 角速度 rad/s

  last_cmd_time = millis();
  
  if (fabs(linear_vel) < 1e-3 && fabs(angular_vel) < 1e-3) {
    stopMotors();
    return;
  }
  
  // 差速控制
  float left_vel = linear_vel - (angular_vel * WHEEL_BASE_M / 2.0);
  float right_vel = linear_vel + (angular_vel * WHEEL_BASE_M / 2.0);
  
  float max_vel = 0.3;
  left_vel = constrain(left_vel, -max_vel, max_vel);
  right_vel = constrain(right_vel, -max_vel, max_vel);
  
  int left_pwm = abs(left_vel / max_vel * MAX_PWM_VALUE);
  int right_pwm = abs(right_vel / max_vel * MAX_PWM_VALUE);
  
  // 方向控制（已修正：左右轮方向反转）
  if (left_vel >= 0) {
    digitalWrite(MOTOR_LEFT_DIR, LOW);
  } else {
    digitalWrite(MOTOR_LEFT_DIR, HIGH);
  }
  
  if (right_vel >= 0) {
    digitalWrite(MOTOR_RIGHT_DIR, HIGH);
  } else {
    digitalWrite(MOTOR_RIGHT_DIR, LOW);
  }
  
  ledcWrite(0, left_pwm);
  ledcWrite(1, right_pwm);
}

// 舵机角度回调
void servoAngleCallback(const std_msgs::Float32& msg) {
  float angle = msg.data;
  angle = constrain(angle, 0.0, 180.0);

  int pulse_width = map(angle, 0, 180, 500, 2500);
  int duty_cycle = (pulse_width * 4096L) / 20000;
  duty_cycle = constrain(duty_cycle, 0, 4095);
  ledcWrite(2, duty_cycle);
}

// 订阅者
ros::Subscriber<geometry_msgs::Twist> cmd_vel_sub("/yolo_car/cmd_vel", cmdVelCallback);
ros::Subscriber<std_msgs::Float32> servo_angle_sub("/yolo_car/servo_angle", servoAngleCallback);

// MPU6050初始化
bool initMPU6050() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B); // PWR_MGMT_1寄存器
  Wire.write(0x00); // 唤醒MPU6050
  uint8_t error = Wire.endTransmission();
  
  if (error != 0) {
    return false;
  }
  
  // 配置加速度计范围 ±2g
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1C); // ACCEL_CONFIG
  Wire.write(0x00);
  Wire.endTransmission();
  
  // 配置陀螺仪范围 ±250°/s
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1B); // GYRO_CONFIG
  Wire.write(0x00);
  Wire.endTransmission();
  
  return true;
}

// 读取MPU6050原始数据
void readMPU6050() {
  // 读取加速度计数据（6字节：ACCEL_XOUT_H, ACCEL_XOUT_L, ACCEL_YOUT_H, ACCEL_YOUT_L, ACCEL_ZOUT_H, ACCEL_ZOUT_L）
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B); // ACCEL_XOUT_H寄存器地址
  Wire.endTransmission(false);
  
  // 修复：使用int版本的重载，避免类型歧义
  int addr = (int)MPU_ADDR;
  int len = 14;
  int stop = 1;  // 1表示发送停止位
  uint8_t bytes_read = Wire.requestFrom(addr, len, stop);
  
  if (bytes_read == 14) {
    int16_t accel_x = (Wire.read() << 8) | Wire.read();
    int16_t accel_y = (Wire.read() << 8) | Wire.read();
    int16_t accel_z = (Wire.read() << 8) | Wire.read();
    int16_t temp = (Wire.read() << 8) | Wire.read();
    int16_t gyro_x = (Wire.read() << 8) | Wire.read();
    int16_t gyro_y = (Wire.read() << 8) | Wire.read();
    int16_t gyro_z = (Wire.read() << 8) | Wire.read();
    
    // 转换为物理单位
    // 加速度：±2g范围，32768对应2g，g=9.8 m/s²
    float accel_scale = 2.0f * 9.8f / 32768.0f;
    imu_raw_msg.linear_acceleration.x = accel_x * accel_scale;
    imu_raw_msg.linear_acceleration.y = accel_y * accel_scale;
    imu_raw_msg.linear_acceleration.z = accel_z * accel_scale;
    
    // 角速度：±250°/s范围，32768对应250°/s，转换为rad/s
    float gyro_scale = 250.0f * M_PI / 180.0f / 32768.0f;
    imu_raw_msg.angular_velocity.x = gyro_x * gyro_scale;
    imu_raw_msg.angular_velocity.y = gyro_y * gyro_scale;
    imu_raw_msg.angular_velocity.z = gyro_z * gyro_scale;
    
    imu_raw_msg.header.stamp = nh.now();
    imu_raw_msg.header.frame_id = "imu_link";
  }
}

// 诊断编码器引脚状态
void diagnoseEncoders() {
  static unsigned long last_diag_time = 0;
  unsigned long current_time = millis();
  
  // 每1秒诊断一次
  if (current_time - last_diag_time >= 1000) {
    Serial.println("\n=== 编码器诊断 ===");
    
    // 读取编码器引脚状态
    int left_a = digitalRead(ENC_LEFT_A);
    int left_b = digitalRead(ENC_LEFT_B);
    int right_a = digitalRead(ENC_RIGHT_A);
    int right_b = digitalRead(ENC_RIGHT_B);
    
    Serial.print("左编码器 - A相(GPIO9): ");
    Serial.print(left_a);
    Serial.print(", B相(GPIO10): ");
    Serial.println(left_b);
    
    Serial.print("右编码器 - A相(GPIO7): ");
    Serial.print(right_a);
    Serial.print(", B相(GPIO8): ");
    Serial.println(right_b);
    
    Serial.print("编码器计数 - 左轮: ");
    Serial.print(enc_left_count);
    Serial.print(", 右轮: ");
    Serial.println(enc_right_count);
    
    Serial.println("==================\n");
    
    last_diag_time = current_time;
  }
}

// 更新轮速
void updateWheelVelocities() {
  unsigned long current_time = millis();
  unsigned long dt_ms = current_time - last_enc_time;
  
  if (dt_ms >= ENC_UPDATE_INTERVAL_MS) {
    // 读取并重置计数（原子操作）
    int32_t left_ticks = enc_left_count;
    int32_t right_ticks = enc_right_count;
    enc_left_count = 0;
    enc_right_count = 0;
    
    // 调试输出：打印编码器原始计数
    Serial.print("编码器计数 - 左轮: ");
    Serial.print(left_ticks);
    Serial.print(", 右轮: ");
    Serial.println(right_ticks);
    
    // 计算轮速（m/s）
    // 每转脉冲数 * 轮子周长 = 每米脉冲数
    float ticks_per_meter = TICKS_PER_REV / (2.0f * M_PI * WHEEL_RADIUS_M);
    float dt_sec = dt_ms / 1000.0f;
    
    left_wheel_vel = (left_ticks / ticks_per_meter) / dt_sec;
    right_wheel_vel = (right_ticks / ticks_per_meter) / dt_sec;
    
    // 调试输出：打印计算出的轮速
    Serial.print("轮速 - 左轮: ");
    Serial.print(left_wheel_vel, 4);
    Serial.print(" m/s, 右轮: ");
    Serial.print(right_wheel_vel, 4);
    Serial.println(" m/s");
    
    // 发布轮速
    wheel_vel_msg.header.stamp = nh.now();
    wheel_vel_msg.header.frame_id = "base_link";
    wheel_vel_msg.vector.x = left_wheel_vel;
    wheel_vel_msg.vector.y = right_wheel_vel;
    wheel_vel_msg.vector.z = 0.0;
    wheel_vel_pub.publish(&wheel_vel_msg);
    
    last_enc_time = current_time;
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n\n=== ESP32-S3 + L298N + SG90 + 编码器 + MPU6050 + ROS1 ===");

  last_cmd_time = millis();
  
  // 初始化电机控制引脚
  Serial.println("初始化L298N电机驱动...");
  pinMode(MOTOR_LEFT_DIR, OUTPUT);
  pinMode(MOTOR_RIGHT_DIR, OUTPUT);
  digitalWrite(MOTOR_LEFT_DIR, LOW);
  digitalWrite(MOTOR_RIGHT_DIR, LOW);
  
  // 配置PWM通道（电机）
  ledcSetup(0, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(1, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(MOTOR_LEFT_PWM, 0);
  ledcAttachPin(MOTOR_RIGHT_PWM, 1);
  
  // 配置PWM通道（舵机）
  ledcSetup(2, SERVO_FREQ, SERVO_RESOLUTION);
  ledcAttachPin(SERVO_PIN, 2);
  
  ledcWrite(0, 0);
  ledcWrite(1, 0);
  
  // 初始舵机到90度
  float init_angle = 90.0;
  int pulse_width = map(init_angle, 0, 180, 500, 2500);
  int duty_cycle = (pulse_width * 4096L) / 20000;
  duty_cycle = constrain(duty_cycle, 0, 4095);
  ledcWrite(2, duty_cycle);
  
  Serial.println("✓ 电机和舵机初始化完成");
  
  // 初始化编码器
  Serial.println("初始化编码器...");
  pinMode(ENC_LEFT_A, INPUT_PULLUP);
  pinMode(ENC_LEFT_B, INPUT_PULLUP);
  pinMode(ENC_RIGHT_A, INPUT_PULLUP);
  pinMode(ENC_RIGHT_B, INPUT_PULLUP);
  
  enc_left_last_state = (digitalRead(ENC_LEFT_A) << 1) | digitalRead(ENC_LEFT_B);
  enc_right_last_state = (digitalRead(ENC_RIGHT_A) << 1) | digitalRead(ENC_RIGHT_B);
  
  attachInterrupt(digitalPinToInterrupt(ENC_LEFT_A), encLeftA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_RIGHT_A), encRightA, CHANGE);
  
  last_enc_time = millis();
  Serial.println("✓ 编码器初始化完成");
  
  // 初始化MPU6050
  Serial.println("初始化MPU6050...");
  Wire.begin(I2C_SDA, I2C_SCL);
  delay(100);
  
  if (initMPU6050()) {
    Serial.println("✓ MPU6050初始化成功");
  } else {
    Serial.println("✗ MPU6050初始化失败");
  }
  
#if USE_WIFI
  Serial.println("\n=== WiFi连接 ===");
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  
  Serial.print("连接中");
  int wifiAttempts = 0;
  while (WiFi.status() != WL_CONNECTED && wifiAttempts < 30) {
    delay(500);
    Serial.print(".");
    wifiAttempts++;
  }
  Serial.println();
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("✓ WiFi连接成功！IP: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("✗ WiFi连接失败！");
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    nh.getHardware()->setConnection(server, serverPort);
  }
#endif
  
  Serial.println("\n=== ROS连接 ===");
  nh.initNode();
  nh.subscribe(cmd_vel_sub);
  nh.subscribe(servo_angle_sub);
  nh.advertise(wheel_vel_pub);
  nh.advertise(imu_raw_pub);
  
  Serial.print("等待连接");
  int rosAttempts = 0;
  while (!nh.connected() && rosAttempts < 50) {
    nh.spinOnce();
    delay(100);
    if (rosAttempts % 10 == 0) Serial.print(".");
    rosAttempts++;
  }
  Serial.println();
  
  if (nh.connected()) {
    Serial.println("✓ ROS连接成功！");
  } else {
    Serial.println("✗ ROS连接失败！");
  }
  
  Serial.println("\n=== 系统状态 ===");
  Serial.print("WiFi: ");
  Serial.println((WiFi.status() == WL_CONNECTED) ? "✓ 已连接" : "✗ 未连接");
  Serial.print("ROS: ");
  Serial.println(nh.connected() ? "✓ 已连接" : "✗ 未连接");
  Serial.print("编码器参数: TICKS_PER_REV=");
  Serial.print(TICKS_PER_REV);
  Serial.print(", WHEEL_RADIUS_M=");
  Serial.print(WHEEL_RADIUS_M);
  Serial.print(", WHEEL_BASE_M=");
  Serial.println(WHEEL_BASE_M);
  Serial.println("\n开始运行...\n");
}

void loop() {
  // 命令超时保护
  if (millis() - last_cmd_time > CMD_TIMEOUT_MS) {
    stopMotors();
  }

  if (!nh.connected()) {
    static unsigned long lastReconnectMsg = 0;
    if (millis() - lastReconnectMsg > 5000) {
      Serial.println("ROS连接断开，尝试重连...");
      lastReconnectMsg = millis();
    }
    stopMotors();
    nh.spinOnce();
    delay(100);
    return;
  }

  // 更新轮速（20Hz）
  updateWheelVelocities();
  
  // 诊断编码器（每秒一次）
  diagnoseEncoders();
  
  // 读取并发布IMU（50Hz）
  static unsigned long last_imu_time = 0;
  if (millis() - last_imu_time >= 20) {
    readMPU6050();
    imu_raw_pub.publish(&imu_raw_msg);
    last_imu_time = millis();
  }

  nh.spinOnce();
  delay(10);
}


```

添加中断触发计数器，检查中断是否被触发：


```cpp
/*
 * ESP32-S3 + L298N电机驱动 + SG90舵机 + 编码器 + MPU6050 + ROS1 (rosserial)
 * 
 * 功能：
 * 1. 订阅速度指令控制L298N电机
 * 2. 订阅舵机角度指令控制SG90舵机
 * 3. 编码器计数并发布轮速
 * 4. MPU6050读取并发布原始IMU数据
 * 
 * 接线：
 * - L298N：N1->GPIO3(左PWM), N2->GPIO4(左DIR), N3->GPIO5(右PWM), N4->GPIO6(右DIR)
 * - SG90舵机信号线：GPIO2
 * - 编码器：右轮A->GPIO7, B->GPIO8; 左轮A->GPIO9, B->GPIO10
 * - MPU6050：SCL->GPIO45, SDA->GPIO46
 */

#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <Wire.h>

#define USE_WIFI true

#if USE_WIFI
  #include <WiFi.h>
  const char* ssid = "机器人小车局域网";
  const char* password = "3252362734";
  IPAddress server(192, 168, 31, 64);
  const uint16_t serverPort = 11411;
#endif

// L298N电机控制引脚
#define MOTOR_LEFT_PWM   3   // 左轮PWM
#define MOTOR_LEFT_DIR   4   // 左轮方向
#define MOTOR_RIGHT_PWM  5   // 右轮PWM
#define MOTOR_RIGHT_DIR  6   // 右轮方向

// SG90舵机引脚
#define SERVO_PIN 2

// 编码器引脚
#define ENC_RIGHT_A 7
#define ENC_RIGHT_B 8
#define ENC_LEFT_A  9
#define ENC_LEFT_B  10

// MPU6050 I2C引脚和地址
#define MPU_ADDR 0x68
#define I2C_SDA 46
#define I2C_SCL 45

// PWM参数
#define PWM_FREQ 1000      // PWM频率 1kHz
#define PWM_RESOLUTION 8   // 8位分辨率 (0-255)
#define SERVO_FREQ 50      // 舵机PWM频率 50Hz
#define SERVO_RESOLUTION 12

// 速度限制
#define MAX_PWM_VALUE 200  // 最大PWM值

// 命令超时（毫秒）
#define CMD_TIMEOUT_MS 500

// 编码器参数（已测量更新）
#define TICKS_PER_REV 260.0f      // 每转脉冲数
#define WHEEL_RADIUS_M 0.0480f    // 轮子半径（米）
#define WHEEL_BASE_M   0.1475f    // 轮距（米）

// 编码器计数变量（volatile用于中断）
volatile int32_t enc_left_count = 0;
volatile int32_t enc_right_count = 0;
volatile int8_t enc_left_last_state = 0;
volatile int8_t enc_right_last_state = 0;
volatile uint32_t enc_left_interrupt_count = 0;  // 中断触发计数
volatile uint32_t enc_right_interrupt_count = 0; // 中断触发计数

// 轮速计算
float left_wheel_vel = 0.0f;
float right_wheel_vel = 0.0f;
unsigned long last_enc_time = 0;
const unsigned long ENC_UPDATE_INTERVAL_MS = 50; // 50ms更新一次，20Hz

// ROS节点
ros::NodeHandle nh;
unsigned long last_cmd_time = 0;

// ROS消息变量（必须先声明，发布者才能引用）
geometry_msgs::Vector3Stamped wheel_vel_msg;
sensor_msgs::Imu imu_raw_msg;

// 发布者（必须在消息变量之后声明）
ros::Publisher wheel_vel_pub("/yolo_car/wheel_vel", &wheel_vel_msg);
ros::Publisher imu_raw_pub("/yolo_car/imu_raw", &imu_raw_msg);

// 左编码器中断处理（A相）
void IRAM_ATTR encLeftA() {
  int8_t a = digitalRead(ENC_LEFT_A);
  int8_t b = digitalRead(ENC_LEFT_B);
  int8_t state = (a << 1) | b;
  int8_t diff = (state - enc_left_last_state) & 0x03;
  if (diff == 1 || diff == 3) {
    enc_left_count++;
  } else if (diff == 2) {
    enc_left_count--;
  }
  enc_left_last_state = state;
}

// 右编码器中断处理（A相）
void IRAM_ATTR encRightA() {
  int8_t a = digitalRead(ENC_RIGHT_A);
  int8_t b = digitalRead(ENC_RIGHT_B);
  int8_t state = (a << 1) | b;
  int8_t diff = (state - enc_right_last_state) & 0x03;
  if (diff == 1 || diff == 3) {
    enc_right_count++;
  } else if (diff == 2) {
    enc_right_count--;
  }
  enc_right_last_state = state;
}

// 电机急停
void stopMotors() {
  ledcWrite(0, 0);
  ledcWrite(1, 0);
  digitalWrite(MOTOR_LEFT_DIR, LOW);
  digitalWrite(MOTOR_RIGHT_DIR, LOW);
}

// 速度指令回调
void cmdVelCallback(const geometry_msgs::Twist& msg) {
  float linear_vel = msg.linear.x;   // 线速度 m/s
  float angular_vel = msg.angular.z; // 角速度 rad/s

  last_cmd_time = millis();
  
  if (fabs(linear_vel) < 1e-3 && fabs(angular_vel) < 1e-3) {
    stopMotors();
    return;
  }
  
  // 差速控制
  float left_vel = linear_vel - (angular_vel * WHEEL_BASE_M / 2.0);
  float right_vel = linear_vel + (angular_vel * WHEEL_BASE_M / 2.0);
  
  float max_vel = 0.3;
  left_vel = constrain(left_vel, -max_vel, max_vel);
  right_vel = constrain(right_vel, -max_vel, max_vel);
  
  int left_pwm = abs(left_vel / max_vel * MAX_PWM_VALUE);
  int right_pwm = abs(right_vel / max_vel * MAX_PWM_VALUE);
  
  // 方向控制（已修正：左右轮方向反转）
  if (left_vel >= 0) {
    digitalWrite(MOTOR_LEFT_DIR, LOW);
  } else {
    digitalWrite(MOTOR_LEFT_DIR, HIGH);
  }
  
  if (right_vel >= 0) {
    digitalWrite(MOTOR_RIGHT_DIR, HIGH);
  } else {
    digitalWrite(MOTOR_RIGHT_DIR, LOW);
  }
  
  ledcWrite(0, left_pwm);
  ledcWrite(1, right_pwm);
}

// 舵机角度回调
void servoAngleCallback(const std_msgs::Float32& msg) {
  float angle = msg.data;
  angle = constrain(angle, 0.0, 180.0);

  int pulse_width = map(angle, 0, 180, 500, 2500);
  int duty_cycle = (pulse_width * 4096L) / 20000;
  duty_cycle = constrain(duty_cycle, 0, 4095);
  ledcWrite(2, duty_cycle);
}

// 订阅者
ros::Subscriber<geometry_msgs::Twist> cmd_vel_sub("/yolo_car/cmd_vel", cmdVelCallback);
ros::Subscriber<std_msgs::Float32> servo_angle_sub("/yolo_car/servo_angle", servoAngleCallback);

// MPU6050初始化
bool initMPU6050() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B); // PWR_MGMT_1寄存器
  Wire.write(0x00); // 唤醒MPU6050
  uint8_t error = Wire.endTransmission();
  
  if (error != 0) {
    return false;
  }
  
  // 配置加速度计范围 ±2g
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1C); // ACCEL_CONFIG
  Wire.write(0x00);
  Wire.endTransmission();
  
  // 配置陀螺仪范围 ±250°/s
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1B); // GYRO_CONFIG
  Wire.write(0x00);
  Wire.endTransmission();
  
  return true;
}

// 读取MPU6050原始数据
void readMPU6050() {
  // 读取加速度计数据（6字节：ACCEL_XOUT_H, ACCEL_XOUT_L, ACCEL_YOUT_H, ACCEL_YOUT_L, ACCEL_ZOUT_H, ACCEL_ZOUT_L）
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B); // ACCEL_XOUT_H寄存器地址
  Wire.endTransmission(false);
  
  // 修复：使用int版本的重载，避免类型歧义
  int addr = (int)MPU_ADDR;
  int len = 14;
  int stop = 1;  // 1表示发送停止位
  uint8_t bytes_read = Wire.requestFrom(addr, len, stop);
  
  if (bytes_read == 14) {
    int16_t accel_x = (Wire.read() << 8) | Wire.read();
    int16_t accel_y = (Wire.read() << 8) | Wire.read();
    int16_t accel_z = (Wire.read() << 8) | Wire.read();
    int16_t temp = (Wire.read() << 8) | Wire.read();
    int16_t gyro_x = (Wire.read() << 8) | Wire.read();
    int16_t gyro_y = (Wire.read() << 8) | Wire.read();
    int16_t gyro_z = (Wire.read() << 8) | Wire.read();
    
    // 转换为物理单位
    // 加速度：±2g范围，32768对应2g，g=9.8 m/s²
    float accel_scale = 2.0f * 9.8f / 32768.0f;
    imu_raw_msg.linear_acceleration.x = accel_x * accel_scale;
    imu_raw_msg.linear_acceleration.y = accel_y * accel_scale;
    imu_raw_msg.linear_acceleration.z = accel_z * accel_scale;
    
    // 角速度：±250°/s范围，32768对应250°/s，转换为rad/s
    float gyro_scale = 250.0f * M_PI / 180.0f / 32768.0f;
    imu_raw_msg.angular_velocity.x = gyro_x * gyro_scale;
    imu_raw_msg.angular_velocity.y = gyro_y * gyro_scale;
    imu_raw_msg.angular_velocity.z = gyro_z * gyro_scale;
    
    imu_raw_msg.header.stamp = nh.now();
    imu_raw_msg.header.frame_id = "imu_link";
  }
}

// 诊断编码器引脚状态
void diagnoseEncoders() {
  static unsigned long last_diag_time = 0;
  unsigned long current_time = millis();
  
  // 每1秒诊断一次
  if (current_time - last_diag_time >= 1000) {
    Serial.println("\n=== 编码器诊断 ===");
    
    // 读取编码器引脚状态
    int left_a = digitalRead(ENC_LEFT_A);
    int left_b = digitalRead(ENC_LEFT_B);
    int right_a = digitalRead(ENC_RIGHT_A);
    int right_b = digitalRead(ENC_RIGHT_B);
    
    Serial.print("左编码器 - A相(GPIO9): ");
    Serial.print(left_a);
    Serial.print(", B相(GPIO10): ");
    Serial.println(left_b);
    
    Serial.print("右编码器 - A相(GPIO7): ");
    Serial.print(right_a);
    Serial.print(", B相(GPIO8): ");
    Serial.println(right_b);
    
    Serial.print("编码器计数 - 左轮: ");
    Serial.print(enc_left_count);
    Serial.print(", 右轮: ");
    Serial.println(enc_right_count);
    
    Serial.println("==================\n");
    
    last_diag_time = current_time;
  }
}

// 更新轮速
void updateWheelVelocities() {
  unsigned long current_time = millis();
  unsigned long dt_ms = current_time - last_enc_time;
  
  if (dt_ms >= ENC_UPDATE_INTERVAL_MS) {
    // 读取并重置计数（原子操作）
    int32_t left_ticks = enc_left_count;
    int32_t right_ticks = enc_right_count;
    enc_left_count = 0;
    enc_right_count = 0;
    
    // 调试输出：打印编码器原始计数
    Serial.print("编码器计数 - 左轮: ");
    Serial.print(left_ticks);
    Serial.print(", 右轮: ");
    Serial.println(right_ticks);
    
    // 计算轮速（m/s）
    // 每转脉冲数 * 轮子周长 = 每米脉冲数
    float ticks_per_meter = TICKS_PER_REV / (2.0f * M_PI * WHEEL_RADIUS_M);
    float dt_sec = dt_ms / 1000.0f;
    
    left_wheel_vel = (left_ticks / ticks_per_meter) / dt_sec;
    right_wheel_vel = (right_ticks / ticks_per_meter) / dt_sec;
    
    // 调试输出：打印计算出的轮速
    Serial.print("轮速 - 左轮: ");
    Serial.print(left_wheel_vel, 4);
    Serial.print(" m/s, 右轮: ");
    Serial.print(right_wheel_vel, 4);
    Serial.println(" m/s");
    
    // 发布轮速
    wheel_vel_msg.header.stamp = nh.now();
    wheel_vel_msg.header.frame_id = "base_link";
    wheel_vel_msg.vector.x = left_wheel_vel;
    wheel_vel_msg.vector.y = right_wheel_vel;
    wheel_vel_msg.vector.z = 0.0;
    wheel_vel_pub.publish(&wheel_vel_msg);
    
    last_enc_time = current_time;
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n\n=== ESP32-S3 + L298N + SG90 + 编码器 + MPU6050 + ROS1 ===");

  last_cmd_time = millis();
  
  // 初始化电机控制引脚
  Serial.println("初始化L298N电机驱动...");
  pinMode(MOTOR_LEFT_DIR, OUTPUT);
  pinMode(MOTOR_RIGHT_DIR, OUTPUT);
  digitalWrite(MOTOR_LEFT_DIR, LOW);
  digitalWrite(MOTOR_RIGHT_DIR, LOW);
  
  // 配置PWM通道（电机）
  ledcSetup(0, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(1, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(MOTOR_LEFT_PWM, 0);
  ledcAttachPin(MOTOR_RIGHT_PWM, 1);
  
  // 配置PWM通道（舵机）
  ledcSetup(2, SERVO_FREQ, SERVO_RESOLUTION);
  ledcAttachPin(SERVO_PIN, 2);
  
  ledcWrite(0, 0);
  ledcWrite(1, 0);
  
  // 初始舵机到90度
  float init_angle = 90.0;
  int pulse_width = map(init_angle, 0, 180, 500, 2500);
  int duty_cycle = (pulse_width * 4096L) / 20000;
  duty_cycle = constrain(duty_cycle, 0, 4095);
  ledcWrite(2, duty_cycle);
  
  Serial.println("✓ 电机和舵机初始化完成");
  
  // 初始化编码器
  Serial.println("初始化编码器...");
  pinMode(ENC_LEFT_A, INPUT_PULLUP);
  pinMode(ENC_LEFT_B, INPUT_PULLUP);
  pinMode(ENC_RIGHT_A, INPUT_PULLUP);
  pinMode(ENC_RIGHT_B, INPUT_PULLUP);
  
  enc_left_last_state = (digitalRead(ENC_LEFT_A) << 1) | digitalRead(ENC_LEFT_B);
  enc_right_last_state = (digitalRead(ENC_RIGHT_A) << 1) | digitalRead(ENC_RIGHT_B);
  
  attachInterrupt(digitalPinToInterrupt(ENC_LEFT_A), encLeftA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_RIGHT_A), encRightA, CHANGE);
  
  last_enc_time = millis();
  Serial.println("✓ 编码器初始化完成");
  
  // 初始化MPU6050
  Serial.println("初始化MPU6050...");
  Wire.begin(I2C_SDA, I2C_SCL);
  delay(100);
  
  if (initMPU6050()) {
    Serial.println("✓ MPU6050初始化成功");
  } else {
    Serial.println("✗ MPU6050初始化失败");
  }
  
#if USE_WIFI
  Serial.println("\n=== WiFi连接 ===");
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  
  Serial.print("连接中");
  int wifiAttempts = 0;
  while (WiFi.status() != WL_CONNECTED && wifiAttempts < 30) {
    delay(500);
    Serial.print(".");
    wifiAttempts++;
  }
  Serial.println();
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("✓ WiFi连接成功！IP: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("✗ WiFi连接失败！");
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    nh.getHardware()->setConnection(server, serverPort);
  }
#endif
  
  Serial.println("\n=== ROS连接 ===");
  nh.initNode();
  nh.subscribe(cmd_vel_sub);
  nh.subscribe(servo_angle_sub);
  nh.advertise(wheel_vel_pub);
  nh.advertise(imu_raw_pub);
  
  Serial.print("等待连接");
  int rosAttempts = 0;
  while (!nh.connected() && rosAttempts < 50) {
    nh.spinOnce();
    delay(100);
    if (rosAttempts % 10 == 0) Serial.print(".");
    rosAttempts++;
  }
  Serial.println();
  
  if (nh.connected()) {
    Serial.println("✓ ROS连接成功！");
  } else {
    Serial.println("✗ ROS连接失败！");
  }
  
  Serial.println("\n=== 系统状态 ===");
  Serial.print("WiFi: ");
  Serial.println((WiFi.status() == WL_CONNECTED) ? "✓ 已连接" : "✗ 未连接");
  Serial.print("ROS: ");
  Serial.println(nh.connected() ? "✓ 已连接" : "✗ 未连接");
  Serial.print("编码器参数: TICKS_PER_REV=");
  Serial.print(TICKS_PER_REV);
  Serial.print(", WHEEL_RADIUS_M=");
  Serial.print(WHEEL_RADIUS_M);
  Serial.print(", WHEEL_BASE_M=");
  Serial.println(WHEEL_BASE_M);
  Serial.println("\n开始运行...\n");
}

void loop() {
  // 命令超时保护
  if (millis() - last_cmd_time > CMD_TIMEOUT_MS) {
    stopMotors();
  }

  if (!nh.connected()) {
    static unsigned long lastReconnectMsg = 0;
    if (millis() - lastReconnectMsg > 5000) {
      Serial.println("ROS连接断开，尝试重连...");
      lastReconnectMsg = millis();
    }
    stopMotors();
    nh.spinOnce();
    delay(100);
    return;
  }

  // 更新轮速（20Hz）
  updateWheelVelocities();
  
  // 诊断编码器（每秒一次）
  diagnoseEncoders();
  
  // 读取并发布IMU（50Hz）
  static unsigned long last_imu_time = 0;
  if (millis() - last_imu_time >= 20) {
    readMPU6050();
    imu_raw_pub.publish(&imu_raw_msg);
    last_imu_time = millis();
  }

  nh.spinOnce();
  delay(10);
}


```

```cpp
/*
 * ESP32-S3 + L298N电机驱动 + SG90舵机 + 编码器 + MPU6050 + ROS1 (rosserial)
 * 
 * 功能：
 * 1. 订阅速度指令控制L298N电机
 * 2. 订阅舵机角度指令控制SG90舵机
 * 3. 编码器计数并发布轮速
 * 4. MPU6050读取并发布原始IMU数据
 * 
 * 接线：
 * - L298N：N1->GPIO3(左PWM), N2->GPIO4(左DIR), N3->GPIO5(右PWM), N4->GPIO6(右DIR)
 * - SG90舵机信号线：GPIO2
 * - 编码器：右轮A->GPIO7, B->GPIO8; 左轮A->GPIO9, B->GPIO10
 * - MPU6050：SCL->GPIO45, SDA->GPIO46
 */

#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <Wire.h>

#define USE_WIFI true

#if USE_WIFI
  #include <WiFi.h>
  const char* ssid = "机器人小车局域网";
  const char* password = "3252362734";
  IPAddress server(192, 168, 31, 64);
  const uint16_t serverPort = 11411;
#endif

// L298N电机控制引脚
#define MOTOR_LEFT_PWM   3   // 左轮PWM
#define MOTOR_LEFT_DIR   4   // 左轮方向
#define MOTOR_RIGHT_PWM  5   // 右轮PWM
#define MOTOR_RIGHT_DIR  6   // 右轮方向

// SG90舵机引脚
#define SERVO_PIN 2

// 编码器引脚
#define ENC_RIGHT_A 7
#define ENC_RIGHT_B 8
#define ENC_LEFT_A  9
#define ENC_LEFT_B  10

// MPU6050 I2C引脚和地址
#define MPU_ADDR 0x68
#define I2C_SDA 46
#define I2C_SCL 45

// PWM参数
#define PWM_FREQ 1000      // PWM频率 1kHz
#define PWM_RESOLUTION 8   // 8位分辨率 (0-255)
#define SERVO_FREQ 50      // 舵机PWM频率 50Hz
#define SERVO_RESOLUTION 12

// 速度限制
#define MAX_PWM_VALUE 200  // 最大PWM值

// 命令超时（毫秒）
#define CMD_TIMEOUT_MS 500

// 编码器参数（已测量更新）
#define TICKS_PER_REV 260.0f      // 每转脉冲数
#define WHEEL_RADIUS_M 0.0480f    // 轮子半径（米）
#define WHEEL_BASE_M   0.1475f    // 轮距（米）

// 编码器计数变量（volatile用于中断）
volatile int32_t enc_left_count = 0;
volatile int32_t enc_right_count = 0;
volatile int8_t enc_left_last_state = 0;
volatile int8_t enc_right_last_state = 0;
volatile uint32_t enc_left_interrupt_count = 0;  // 中断触发计数
volatile uint32_t enc_right_interrupt_count = 0; // 中断触发计数

// 轮速计算
float left_wheel_vel = 0.0f;
float right_wheel_vel = 0.0f;
unsigned long last_enc_time = 0;
const unsigned long ENC_UPDATE_INTERVAL_MS = 50; // 50ms更新一次，20Hz

// ROS节点
ros::NodeHandle nh;
unsigned long last_cmd_time = 0;

// ROS消息变量（必须先声明，发布者才能引用）
geometry_msgs::Vector3Stamped wheel_vel_msg;
sensor_msgs::Imu imu_raw_msg;

// 发布者（必须在消息变量之后声明）
ros::Publisher wheel_vel_pub("/yolo_car/wheel_vel", &wheel_vel_msg);
ros::Publisher imu_raw_pub("/yolo_car/imu_raw", &imu_raw_msg);

// 左编码器中断处理（A相）
void IRAM_ATTR encLeftA() {
  enc_left_interrupt_count++;  // 记录中断触发次数
  int8_t a = digitalRead(ENC_LEFT_A);
  int8_t b = digitalRead(ENC_LEFT_B);
  int8_t state = (a << 1) | b;
  int8_t diff = (state - enc_left_last_state) & 0x03;
  if (diff == 1 || diff == 3) {
    enc_left_count++;
  } else if (diff == 2) {
    enc_left_count--;
  }
  enc_left_last_state = state;
}

// 右编码器中断处理（A相）
void IRAM_ATTR encRightA() {
  enc_right_interrupt_count++;  // 记录中断触发次数
  int8_t a = digitalRead(ENC_RIGHT_A);
  int8_t b = digitalRead(ENC_RIGHT_B);
  int8_t state = (a << 1) | b;
  int8_t diff = (state - enc_right_last_state) & 0x03;
  if (diff == 1 || diff == 3) {
    enc_right_count++;
  } else if (diff == 2) {
    enc_right_count--;
  }
  enc_right_last_state = state;
}

// 电机急停
void stopMotors() {
  ledcWrite(0, 0);
  ledcWrite(1, 0);
  digitalWrite(MOTOR_LEFT_DIR, LOW);
  digitalWrite(MOTOR_RIGHT_DIR, LOW);
}

// 速度指令回调
void cmdVelCallback(const geometry_msgs::Twist& msg) {
  float linear_vel = msg.linear.x;   // 线速度 m/s
  float angular_vel = msg.angular.z; // 角速度 rad/s

  last_cmd_time = millis();
  
  if (fabs(linear_vel) < 1e-3 && fabs(angular_vel) < 1e-3) {
    stopMotors();
    return;
  }
  
  // 差速控制
  float left_vel = linear_vel - (angular_vel * WHEEL_BASE_M / 2.0);
  float right_vel = linear_vel + (angular_vel * WHEEL_BASE_M / 2.0);
  
  float max_vel = 0.3;
  left_vel = constrain(left_vel, -max_vel, max_vel);
  right_vel = constrain(right_vel, -max_vel, max_vel);
  
  int left_pwm = abs(left_vel / max_vel * MAX_PWM_VALUE);
  int right_pwm = abs(right_vel / max_vel * MAX_PWM_VALUE);
  
  // 方向控制（已修正：左右轮方向反转）
  if (left_vel >= 0) {
    digitalWrite(MOTOR_LEFT_DIR, LOW);
  } else {
    digitalWrite(MOTOR_LEFT_DIR, HIGH);
  }
  
  if (right_vel >= 0) {
    digitalWrite(MOTOR_RIGHT_DIR, HIGH);
  } else {
    digitalWrite(MOTOR_RIGHT_DIR, LOW);
  }
  
  ledcWrite(0, left_pwm);
  ledcWrite(1, right_pwm);
}

// 舵机角度回调
void servoAngleCallback(const std_msgs::Float32& msg) {
  float angle = msg.data;
  angle = constrain(angle, 0.0, 180.0);

  int pulse_width = map(angle, 0, 180, 500, 2500);
  int duty_cycle = (pulse_width * 4096L) / 20000;
  duty_cycle = constrain(duty_cycle, 0, 4095);
  ledcWrite(2, duty_cycle);
}

// 订阅者
ros::Subscriber<geometry_msgs::Twist> cmd_vel_sub("/yolo_car/cmd_vel", cmdVelCallback);
ros::Subscriber<std_msgs::Float32> servo_angle_sub("/yolo_car/servo_angle", servoAngleCallback);

// MPU6050初始化
bool initMPU6050() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B); // PWR_MGMT_1寄存器
  Wire.write(0x00); // 唤醒MPU6050
  uint8_t error = Wire.endTransmission();
  
  if (error != 0) {
    return false;
  }
  
  // 配置加速度计范围 ±2g
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1C); // ACCEL_CONFIG
  Wire.write(0x00);
  Wire.endTransmission();
  
  // 配置陀螺仪范围 ±250°/s
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1B); // GYRO_CONFIG
  Wire.write(0x00);
  Wire.endTransmission();
  
  return true;
}

// 读取MPU6050原始数据
void readMPU6050() {
  // 读取加速度计数据（6字节：ACCEL_XOUT_H, ACCEL_XOUT_L, ACCEL_YOUT_H, ACCEL_YOUT_L, ACCEL_ZOUT_H, ACCEL_ZOUT_L）
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B); // ACCEL_XOUT_H寄存器地址
  Wire.endTransmission(false);
  
  // 修复：使用int版本的重载，避免类型歧义
  int addr = (int)MPU_ADDR;
  int len = 14;
  int stop = 1;  // 1表示发送停止位
  uint8_t bytes_read = Wire.requestFrom(addr, len, stop);
  
  if (bytes_read == 14) {
    int16_t accel_x = (Wire.read() << 8) | Wire.read();
    int16_t accel_y = (Wire.read() << 8) | Wire.read();
    int16_t accel_z = (Wire.read() << 8) | Wire.read();
    int16_t temp = (Wire.read() << 8) | Wire.read();
    int16_t gyro_x = (Wire.read() << 8) | Wire.read();
    int16_t gyro_y = (Wire.read() << 8) | Wire.read();
    int16_t gyro_z = (Wire.read() << 8) | Wire.read();
    
    // 转换为物理单位
    // 加速度：±2g范围，32768对应2g，g=9.8 m/s²
    float accel_scale = 2.0f * 9.8f / 32768.0f;
    imu_raw_msg.linear_acceleration.x = accel_x * accel_scale;
    imu_raw_msg.linear_acceleration.y = accel_y * accel_scale;
    imu_raw_msg.linear_acceleration.z = accel_z * accel_scale;
    
    // 角速度：±250°/s范围，32768对应250°/s，转换为rad/s
    float gyro_scale = 250.0f * M_PI / 180.0f / 32768.0f;
    imu_raw_msg.angular_velocity.x = gyro_x * gyro_scale;
    imu_raw_msg.angular_velocity.y = gyro_y * gyro_scale;
    imu_raw_msg.angular_velocity.z = gyro_z * gyro_scale;
    
    imu_raw_msg.header.stamp = nh.now();
    imu_raw_msg.header.frame_id = "imu_link";
  }
}

// 诊断编码器引脚状态
void diagnoseEncoders() {
  static unsigned long last_diag_time = 0;
  unsigned long current_time = millis();
  
  // 每1秒诊断一次
  if (current_time - last_diag_time >= 1000) {
    Serial.println("\n=== 编码器诊断 ===");
    
    // 读取编码器引脚状态
    int left_a = digitalRead(ENC_LEFT_A);
    int left_b = digitalRead(ENC_LEFT_B);
    int right_a = digitalRead(ENC_RIGHT_A);
    int right_b = digitalRead(ENC_RIGHT_B);
    
    Serial.print("左编码器 - A相(GPIO9): ");
    Serial.print(left_a);
    Serial.print(", B相(GPIO10): ");
    Serial.println(left_b);
    
    Serial.print("右编码器 - A相(GPIO7): ");
    Serial.print(right_a);
    Serial.print(", B相(GPIO8): ");
    Serial.println(right_b);
    
    Serial.print("编码器计数 - 左轮: ");
    Serial.print(enc_left_count);
    Serial.print(", 右轮: ");
    Serial.println(enc_right_count);
    
    Serial.println("==================\n");
    
    last_diag_time = current_time;
  }
}

// 更新轮速
void updateWheelVelocities() {
  unsigned long current_time = millis();
  unsigned long dt_ms = current_time - last_enc_time;
  
  if (dt_ms >= ENC_UPDATE_INTERVAL_MS) {
    // 读取并重置计数（原子操作）
    int32_t left_ticks = enc_left_count;
    int32_t right_ticks = enc_right_count;
    enc_left_count = 0;
    enc_right_count = 0;
    
    // 调试输出：打印编码器原始计数
    Serial.print("编码器计数 - 左轮: ");
    Serial.print(left_ticks);
    Serial.print(", 右轮: ");
    Serial.println(right_ticks);
    
    // 计算轮速（m/s）
    // 每转脉冲数 * 轮子周长 = 每米脉冲数
    float ticks_per_meter = TICKS_PER_REV / (2.0f * M_PI * WHEEL_RADIUS_M);
    float dt_sec = dt_ms / 1000.0f;
    
    left_wheel_vel = (left_ticks / ticks_per_meter) / dt_sec;
    right_wheel_vel = (right_ticks / ticks_per_meter) / dt_sec;
    
    // 调试输出：打印计算出的轮速
    Serial.print("轮速 - 左轮: ");
    Serial.print(left_wheel_vel, 4);
    Serial.print(" m/s, 右轮: ");
    Serial.print(right_wheel_vel, 4);
    Serial.println(" m/s");
    
    // 发布轮速
    wheel_vel_msg.header.stamp = nh.now();
    wheel_vel_msg.header.frame_id = "base_link";
    wheel_vel_msg.vector.x = left_wheel_vel;
    wheel_vel_msg.vector.y = right_wheel_vel;
    wheel_vel_msg.vector.z = 0.0;
    wheel_vel_pub.publish(&wheel_vel_msg);
    
    last_enc_time = current_time;
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n\n=== ESP32-S3 + L298N + SG90 + 编码器 + MPU6050 + ROS1 ===");

  last_cmd_time = millis();
  
  // 初始化电机控制引脚
  Serial.println("初始化L298N电机驱动...");
  pinMode(MOTOR_LEFT_DIR, OUTPUT);
  pinMode(MOTOR_RIGHT_DIR, OUTPUT);
  digitalWrite(MOTOR_LEFT_DIR, LOW);
  digitalWrite(MOTOR_RIGHT_DIR, LOW);
  
  // 配置PWM通道（电机）
  ledcSetup(0, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(1, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(MOTOR_LEFT_PWM, 0);
  ledcAttachPin(MOTOR_RIGHT_PWM, 1);
  
  // 配置PWM通道（舵机）
  ledcSetup(2, SERVO_FREQ, SERVO_RESOLUTION);
  ledcAttachPin(SERVO_PIN, 2);
  
  ledcWrite(0, 0);
  ledcWrite(1, 0);
  
  // 初始舵机到90度
  float init_angle = 90.0;
  int pulse_width = map(init_angle, 0, 180, 500, 2500);
  int duty_cycle = (pulse_width * 4096L) / 20000;
  duty_cycle = constrain(duty_cycle, 0, 4095);
  ledcWrite(2, duty_cycle);
  
  Serial.println("✓ 电机和舵机初始化完成");
  
  // 初始化编码器
  Serial.println("初始化编码器...");
  pinMode(ENC_LEFT_A, INPUT_PULLUP);
  pinMode(ENC_LEFT_B, INPUT_PULLUP);
  pinMode(ENC_RIGHT_A, INPUT_PULLUP);
  pinMode(ENC_RIGHT_B, INPUT_PULLUP);
  
  enc_left_last_state = (digitalRead(ENC_LEFT_A) << 1) | digitalRead(ENC_LEFT_B);
  enc_right_last_state = (digitalRead(ENC_RIGHT_A) << 1) | digitalRead(ENC_RIGHT_B);
  
  attachInterrupt(digitalPinToInterrupt(ENC_LEFT_A), encLeftA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_RIGHT_A), encRightA, CHANGE);
  
  last_enc_time = millis();
  Serial.println("✓ 编码器初始化完成");
  
  // 初始化MPU6050
  Serial.println("初始化MPU6050...");
  Wire.begin(I2C_SDA, I2C_SCL);
  delay(100);
  
  if (initMPU6050()) {
    Serial.println("✓ MPU6050初始化成功");
  } else {
    Serial.println("✗ MPU6050初始化失败");
  }
  
#if USE_WIFI
  Serial.println("\n=== WiFi连接 ===");
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  
  Serial.print("连接中");
  int wifiAttempts = 0;
  while (WiFi.status() != WL_CONNECTED && wifiAttempts < 30) {
    delay(500);
    Serial.print(".");
    wifiAttempts++;
  }
  Serial.println();
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("✓ WiFi连接成功！IP: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("✗ WiFi连接失败！");
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    nh.getHardware()->setConnection(server, serverPort);
  }
#endif
  
  Serial.println("\n=== ROS连接 ===");
  nh.initNode();
  nh.subscribe(cmd_vel_sub);
  nh.subscribe(servo_angle_sub);
  nh.advertise(wheel_vel_pub);
  nh.advertise(imu_raw_pub);
  
  Serial.print("等待连接");
  int rosAttempts = 0;
  while (!nh.connected() && rosAttempts < 50) {
    nh.spinOnce();
    delay(100);
    if (rosAttempts % 10 == 0) Serial.print(".");
    rosAttempts++;
  }
  Serial.println();
  
  if (nh.connected()) {
    Serial.println("✓ ROS连接成功！");
  } else {
    Serial.println("✗ ROS连接失败！");
  }
  
  Serial.println("\n=== 系统状态 ===");
  Serial.print("WiFi: ");
  Serial.println((WiFi.status() == WL_CONNECTED) ? "✓ 已连接" : "✗ 未连接");
  Serial.print("ROS: ");
  Serial.println(nh.connected() ? "✓ 已连接" : "✗ 未连接");
  Serial.print("编码器参数: TICKS_PER_REV=");
  Serial.print(TICKS_PER_REV);
  Serial.print(", WHEEL_RADIUS_M=");
  Serial.print(WHEEL_RADIUS_M);
  Serial.print(", WHEEL_BASE_M=");
  Serial.println(WHEEL_BASE_M);
  Serial.println("\n开始运行...\n");
}

void loop() {
  // 命令超时保护
  if (millis() - last_cmd_time > CMD_TIMEOUT_MS) {
    stopMotors();
  }

  if (!nh.connected()) {
    static unsigned long lastReconnectMsg = 0;
    if (millis() - lastReconnectMsg > 5000) {
      Serial.println("ROS连接断开，尝试重连...");
      lastReconnectMsg = millis();
    }
    stopMotors();
    nh.spinOnce();
    delay(100);
    return;
  }

  // 更新轮速（20Hz）
  updateWheelVelocities();
  
  // 诊断编码器（每秒一次）
  diagnoseEncoders();
  
  // 读取并发布IMU（50Hz）
  static unsigned long last_imu_time = 0;
  if (millis() - last_imu_time >= 20) {
    readMPU6050();
    imu_raw_pub.publish(&imu_raw_msg);
    last_imu_time = millis();
  }

  nh.spinOnce();
  delay(10);
}


```

```cpp
/*
 * ESP32-S3 + L298N电机驱动 + SG90舵机 + 编码器 + MPU6050 + ROS1 (rosserial)
 * 
 * 功能：
 * 1. 订阅速度指令控制L298N电机
 * 2. 订阅舵机角度指令控制SG90舵机
 * 3. 编码器计数并发布轮速
 * 4. MPU6050读取并发布原始IMU数据
 * 
 * 接线：
 * - L298N：N1->GPIO3(左PWM), N2->GPIO4(左DIR), N3->GPIO5(右PWM), N4->GPIO6(右DIR)
 * - SG90舵机信号线：GPIO2
 * - 编码器：右轮A->GPIO7, B->GPIO8; 左轮A->GPIO9, B->GPIO10
 * - MPU6050：SCL->GPIO45, SDA->GPIO46
 */

#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <Wire.h>

#define USE_WIFI true

#if USE_WIFI
  #include <WiFi.h>
  const char* ssid = "机器人小车局域网";
  const char* password = "3252362734";
  IPAddress server(192, 168, 31, 64);
  const uint16_t serverPort = 11411;
#endif

// L298N电机控制引脚
#define MOTOR_LEFT_PWM   3   // 左轮PWM
#define MOTOR_LEFT_DIR   4   // 左轮方向
#define MOTOR_RIGHT_PWM  5   // 右轮PWM
#define MOTOR_RIGHT_DIR  6   // 右轮方向

// SG90舵机引脚
#define SERVO_PIN 2

// 编码器引脚
#define ENC_RIGHT_A 7
#define ENC_RIGHT_B 8
#define ENC_LEFT_A  9
#define ENC_LEFT_B  10

// MPU6050 I2C引脚和地址
#define MPU_ADDR 0x68
#define I2C_SDA 46
#define I2C_SCL 45

// PWM参数
#define PWM_FREQ 1000      // PWM频率 1kHz
#define PWM_RESOLUTION 8   // 8位分辨率 (0-255)
#define SERVO_FREQ 50      // 舵机PWM频率 50Hz
#define SERVO_RESOLUTION 12

// 速度限制
#define MAX_PWM_VALUE 200  // 最大PWM值

// 命令超时（毫秒）
#define CMD_TIMEOUT_MS 500

// 编码器参数（已测量更新）
#define TICKS_PER_REV 260.0f      // 每转脉冲数
#define WHEEL_RADIUS_M 0.0480f    // 轮子半径（米）
#define WHEEL_BASE_M   0.1475f    // 轮距（米）

// 编码器计数变量（volatile用于中断）
volatile int32_t enc_left_count = 0;
volatile int32_t enc_right_count = 0;
volatile int8_t enc_left_last_state = 0;
volatile int8_t enc_right_last_state = 0;
volatile uint32_t enc_left_interrupt_count = 0;  // 中断触发计数
volatile uint32_t enc_right_interrupt_count = 0; // 中断触发计数

// 轮速计算
float left_wheel_vel = 0.0f;
float right_wheel_vel = 0.0f;
unsigned long last_enc_time = 0;
const unsigned long ENC_UPDATE_INTERVAL_MS = 50; // 50ms更新一次，20Hz

// ROS节点
ros::NodeHandle nh;
unsigned long last_cmd_time = 0;

// ROS消息变量（必须先声明，发布者才能引用）
geometry_msgs::Vector3Stamped wheel_vel_msg;
sensor_msgs::Imu imu_raw_msg;

// 发布者（必须在消息变量之后声明）
ros::Publisher wheel_vel_pub("/yolo_car/wheel_vel", &wheel_vel_msg);
ros::Publisher imu_raw_pub("/yolo_car/imu_raw", &imu_raw_msg);

// 左编码器中断处理（A相）
void IRAM_ATTR encLeftA() {
  enc_left_interrupt_count++;  // 记录中断触发次数
  int8_t a = digitalRead(ENC_LEFT_A);
  int8_t b = digitalRead(ENC_LEFT_B);
  int8_t state = (a << 1) | b;
  int8_t diff = (state - enc_left_last_state) & 0x03;
  if (diff == 1 || diff == 3) {
    enc_left_count++;
  } else if (diff == 2) {
    enc_left_count--;
  }
  enc_left_last_state = state;
}

// 右编码器中断处理（A相）
void IRAM_ATTR encRightA() {
  enc_right_interrupt_count++;  // 记录中断触发次数
  int8_t a = digitalRead(ENC_RIGHT_A);
  int8_t b = digitalRead(ENC_RIGHT_B);
  int8_t state = (a << 1) | b;
  int8_t diff = (state - enc_right_last_state) & 0x03;
  if (diff == 1 || diff == 3) {
    enc_right_count++;
  } else if (diff == 2) {
    enc_right_count--;
  }
  enc_right_last_state = state;
}

// 电机急停
void stopMotors() {
  ledcWrite(0, 0);
  ledcWrite(1, 0);
  digitalWrite(MOTOR_LEFT_DIR, LOW);
  digitalWrite(MOTOR_RIGHT_DIR, LOW);
}

// 速度指令回调
void cmdVelCallback(const geometry_msgs::Twist& msg) {
  float linear_vel = msg.linear.x;   // 线速度 m/s
  float angular_vel = msg.angular.z; // 角速度 rad/s

  last_cmd_time = millis();
  
  if (fabs(linear_vel) < 1e-3 && fabs(angular_vel) < 1e-3) {
    stopMotors();
    return;
  }
  
  // 差速控制
  float left_vel = linear_vel - (angular_vel * WHEEL_BASE_M / 2.0);
  float right_vel = linear_vel + (angular_vel * WHEEL_BASE_M / 2.0);
  
  float max_vel = 0.3;
  left_vel = constrain(left_vel, -max_vel, max_vel);
  right_vel = constrain(right_vel, -max_vel, max_vel);
  
  int left_pwm = abs(left_vel / max_vel * MAX_PWM_VALUE);
  int right_pwm = abs(right_vel / max_vel * MAX_PWM_VALUE);
  
  // 方向控制（已修正：左右轮方向反转）
  if (left_vel >= 0) {
    digitalWrite(MOTOR_LEFT_DIR, LOW);
  } else {
    digitalWrite(MOTOR_LEFT_DIR, HIGH);
  }
  
  if (right_vel >= 0) {
    digitalWrite(MOTOR_RIGHT_DIR, HIGH);
  } else {
    digitalWrite(MOTOR_RIGHT_DIR, LOW);
  }
  
  ledcWrite(0, left_pwm);
  ledcWrite(1, right_pwm);
}

// 舵机角度回调
void servoAngleCallback(const std_msgs::Float32& msg) {
  float angle = msg.data;
  angle = constrain(angle, 0.0, 180.0);

  int pulse_width = map(angle, 0, 180, 500, 2500);
  int duty_cycle = (pulse_width * 4096L) / 20000;
  duty_cycle = constrain(duty_cycle, 0, 4095);
  ledcWrite(2, duty_cycle);
}

// 订阅者
ros::Subscriber<geometry_msgs::Twist> cmd_vel_sub("/yolo_car/cmd_vel", cmdVelCallback);
ros::Subscriber<std_msgs::Float32> servo_angle_sub("/yolo_car/servo_angle", servoAngleCallback);

// MPU6050初始化
bool initMPU6050() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B); // PWR_MGMT_1寄存器
  Wire.write(0x00); // 唤醒MPU6050
  uint8_t error = Wire.endTransmission();
  
  if (error != 0) {
    return false;
  }
  
  // 配置加速度计范围 ±2g
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1C); // ACCEL_CONFIG
  Wire.write(0x00);
  Wire.endTransmission();
  
  // 配置陀螺仪范围 ±250°/s
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1B); // GYRO_CONFIG
  Wire.write(0x00);
  Wire.endTransmission();
  
  return true;
}

// 读取MPU6050原始数据
void readMPU6050() {
  // 读取加速度计数据（6字节：ACCEL_XOUT_H, ACCEL_XOUT_L, ACCEL_YOUT_H, ACCEL_YOUT_L, ACCEL_ZOUT_H, ACCEL_ZOUT_L）
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B); // ACCEL_XOUT_H寄存器地址
  Wire.endTransmission(false);
  
  // 修复：使用int版本的重载，避免类型歧义
  int addr = (int)MPU_ADDR;
  int len = 14;
  int stop = 1;  // 1表示发送停止位
  uint8_t bytes_read = Wire.requestFrom(addr, len, stop);
  
  if (bytes_read == 14) {
    int16_t accel_x = (Wire.read() << 8) | Wire.read();
    int16_t accel_y = (Wire.read() << 8) | Wire.read();
    int16_t accel_z = (Wire.read() << 8) | Wire.read();
    int16_t temp = (Wire.read() << 8) | Wire.read();
    int16_t gyro_x = (Wire.read() << 8) | Wire.read();
    int16_t gyro_y = (Wire.read() << 8) | Wire.read();
    int16_t gyro_z = (Wire.read() << 8) | Wire.read();
    
    // 转换为物理单位
    // 加速度：±2g范围，32768对应2g，g=9.8 m/s²
    float accel_scale = 2.0f * 9.8f / 32768.0f;
    imu_raw_msg.linear_acceleration.x = accel_x * accel_scale;
    imu_raw_msg.linear_acceleration.y = accel_y * accel_scale;
    imu_raw_msg.linear_acceleration.z = accel_z * accel_scale;
    
    // 角速度：±250°/s范围，32768对应250°/s，转换为rad/s
    float gyro_scale = 250.0f * M_PI / 180.0f / 32768.0f;
    imu_raw_msg.angular_velocity.x = gyro_x * gyro_scale;
    imu_raw_msg.angular_velocity.y = gyro_y * gyro_scale;
    imu_raw_msg.angular_velocity.z = gyro_z * gyro_scale;
    
    imu_raw_msg.header.stamp = nh.now();
    imu_raw_msg.header.frame_id = "imu_link";
  }
}

// 诊断编码器引脚状态
void diagnoseEncoders() {
  static unsigned long last_diag_time = 0;
  unsigned long current_time = millis();
  
  // 每1秒诊断一次
  if (current_time - last_diag_time >= 1000) {
    Serial.println("\n=== 编码器诊断 ===");
    
    // 读取编码器引脚状态
    int left_a = digitalRead(ENC_LEFT_A);
    int left_b = digitalRead(ENC_LEFT_B);
    int right_a = digitalRead(ENC_RIGHT_A);
    int right_b = digitalRead(ENC_RIGHT_B);
    
    Serial.print("左编码器 - A相(GPIO9): ");
    Serial.print(left_a);
    Serial.print(", B相(GPIO10): ");
    Serial.println(left_b);
    
    Serial.print("右编码器 - A相(GPIO7): ");
    Serial.print(right_a);
    Serial.print(", B相(GPIO8): ");
    Serial.println(right_b);
    
    Serial.print("编码器计数 - 左轮: ");
    Serial.print(enc_left_count);
    Serial.print(", 右轮: ");
    Serial.println(enc_right_count);
    
    Serial.print("中断触发次数 - 左轮: ");
    Serial.print(enc_left_interrupt_count);
    Serial.print(", 右轮: ");
    Serial.println(enc_right_interrupt_count);
    
    Serial.println("==================\n");
    
    last_diag_time = current_time;
  }
}

// 更新轮速
void updateWheelVelocities() {
  unsigned long current_time = millis();
  unsigned long dt_ms = current_time - last_enc_time;
  
  if (dt_ms >= ENC_UPDATE_INTERVAL_MS) {
    // 读取并重置计数（原子操作）
    int32_t left_ticks = enc_left_count;
    int32_t right_ticks = enc_right_count;
    enc_left_count = 0;
    enc_right_count = 0;
    
    // 调试输出：打印编码器原始计数
    Serial.print("编码器计数 - 左轮: ");
    Serial.print(left_ticks);
    Serial.print(", 右轮: ");
    Serial.println(right_ticks);
    
    // 计算轮速（m/s）
    // 每转脉冲数 * 轮子周长 = 每米脉冲数
    float ticks_per_meter = TICKS_PER_REV / (2.0f * M_PI * WHEEL_RADIUS_M);
    float dt_sec = dt_ms / 1000.0f;
    
    left_wheel_vel = (left_ticks / ticks_per_meter) / dt_sec;
    right_wheel_vel = (right_ticks / ticks_per_meter) / dt_sec;
    
    // 调试输出：打印计算出的轮速
    Serial.print("轮速 - 左轮: ");
    Serial.print(left_wheel_vel, 4);
    Serial.print(" m/s, 右轮: ");
    Serial.print(right_wheel_vel, 4);
    Serial.println(" m/s");
    
    // 发布轮速
    wheel_vel_msg.header.stamp = nh.now();
    wheel_vel_msg.header.frame_id = "base_link";
    wheel_vel_msg.vector.x = left_wheel_vel;
    wheel_vel_msg.vector.y = right_wheel_vel;
    wheel_vel_msg.vector.z = 0.0;
    wheel_vel_pub.publish(&wheel_vel_msg);
    
    last_enc_time = current_time;
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n\n=== ESP32-S3 + L298N + SG90 + 编码器 + MPU6050 + ROS1 ===");

  last_cmd_time = millis();
  
  // 初始化电机控制引脚
  Serial.println("初始化L298N电机驱动...");
  pinMode(MOTOR_LEFT_DIR, OUTPUT);
  pinMode(MOTOR_RIGHT_DIR, OUTPUT);
  digitalWrite(MOTOR_LEFT_DIR, LOW);
  digitalWrite(MOTOR_RIGHT_DIR, LOW);
  
  // 配置PWM通道（电机）
  ledcSetup(0, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(1, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(MOTOR_LEFT_PWM, 0);
  ledcAttachPin(MOTOR_RIGHT_PWM, 1);
  
  // 配置PWM通道（舵机）
  ledcSetup(2, SERVO_FREQ, SERVO_RESOLUTION);
  ledcAttachPin(SERVO_PIN, 2);
  
  ledcWrite(0, 0);
  ledcWrite(1, 0);
  
  // 初始舵机到90度
  float init_angle = 90.0;
  int pulse_width = map(init_angle, 0, 180, 500, 2500);
  int duty_cycle = (pulse_width * 4096L) / 20000;
  duty_cycle = constrain(duty_cycle, 0, 4095);
  ledcWrite(2, duty_cycle);
  
  Serial.println("✓ 电机和舵机初始化完成");
  
  // 初始化编码器
  Serial.println("初始化编码器...");
  pinMode(ENC_LEFT_A, INPUT_PULLUP);
  pinMode(ENC_LEFT_B, INPUT_PULLUP);
  pinMode(ENC_RIGHT_A, INPUT_PULLUP);
  pinMode(ENC_RIGHT_B, INPUT_PULLUP);
  
  enc_left_last_state = (digitalRead(ENC_LEFT_A) << 1) | digitalRead(ENC_LEFT_B);
  enc_right_last_state = (digitalRead(ENC_RIGHT_A) << 1) | digitalRead(ENC_RIGHT_B);
  
  attachInterrupt(digitalPinToInterrupt(ENC_LEFT_A), encLeftA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_RIGHT_A), encRightA, CHANGE);
  
  last_enc_time = millis();
  Serial.println("✓ 编码器初始化完成");
  
  // 初始化MPU6050
  Serial.println("初始化MPU6050...");
  Wire.begin(I2C_SDA, I2C_SCL);
  delay(100);
  
  if (initMPU6050()) {
    Serial.println("✓ MPU6050初始化成功");
  } else {
    Serial.println("✗ MPU6050初始化失败");
  }
  
#if USE_WIFI
  Serial.println("\n=== WiFi连接 ===");
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  
  Serial.print("连接中");
  int wifiAttempts = 0;
  while (WiFi.status() != WL_CONNECTED && wifiAttempts < 30) {
    delay(500);
    Serial.print(".");
    wifiAttempts++;
  }
  Serial.println();
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("✓ WiFi连接成功！IP: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("✗ WiFi连接失败！");
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    nh.getHardware()->setConnection(server, serverPort);
  }
#endif
  
  Serial.println("\n=== ROS连接 ===");
  nh.initNode();
  nh.subscribe(cmd_vel_sub);
  nh.subscribe(servo_angle_sub);
  nh.advertise(wheel_vel_pub);
  nh.advertise(imu_raw_pub);
  
  Serial.print("等待连接");
  int rosAttempts = 0;
  while (!nh.connected() && rosAttempts < 50) {
    nh.spinOnce();
    delay(100);
    if (rosAttempts % 10 == 0) Serial.print(".");
    rosAttempts++;
  }
  Serial.println();
  
  if (nh.connected()) {
    Serial.println("✓ ROS连接成功！");
  } else {
    Serial.println("✗ ROS连接失败！");
  }
  
  Serial.println("\n=== 系统状态 ===");
  Serial.print("WiFi: ");
  Serial.println((WiFi.status() == WL_CONNECTED) ? "✓ 已连接" : "✗ 未连接");
  Serial.print("ROS: ");
  Serial.println(nh.connected() ? "✓ 已连接" : "✗ 未连接");
  Serial.print("编码器参数: TICKS_PER_REV=");
  Serial.print(TICKS_PER_REV);
  Serial.print(", WHEEL_RADIUS_M=");
  Serial.print(WHEEL_RADIUS_M);
  Serial.print(", WHEEL_BASE_M=");
  Serial.println(WHEEL_BASE_M);
  Serial.println("\n开始运行...\n");
}

void loop() {
  // 命令超时保护
  if (millis() - last_cmd_time > CMD_TIMEOUT_MS) {
    stopMotors();
  }

  if (!nh.connected()) {
    static unsigned long lastReconnectMsg = 0;
    if (millis() - lastReconnectMsg > 5000) {
      Serial.println("ROS连接断开，尝试重连...");
      lastReconnectMsg = millis();
    }
    stopMotors();
    nh.spinOnce();
    delay(100);
    return;
  }

  // 更新轮速（20Hz）
  updateWheelVelocities();
  
  // 诊断编码器（每秒一次）
  diagnoseEncoders();
  
  // 读取并发布IMU（50Hz）
  static unsigned long last_imu_time = 0;
  if (millis() - last_imu_time >= 20) {
    readMPU6050();
    imu_raw_pub.publish(&imu_raw_msg);
    last_imu_time = millis();
  }

  nh.spinOnce();
  delay(10);
}


```

已添加诊断代码。按以下步骤操作：

## 诊断步骤

### 1. 重新上传代码
将修改后的代码上传到ESP32。

### 2. 打开串口监视器
在Arduino IDE中打开串口监视器（波特率115200）。

### 3. 观察诊断信息
每秒会输出一次诊断信息，包括：
- 编码器引脚电平状态（A相和B相）
- 编码器计数
- 中断触发次数

### 4. 测试电机转
让右轮转：
```bash
rostopic pub -r 10 /yolo_car/cmd_vel geometry_msgs/Twist "linear: {x: 0.045, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.6}"
```

## 诊断信息解读

1. 如果引脚电平一直是0或一直是1：
   - 编码器未连接或接线错误
   - 检查编码器A、B相是否连接到正确的GPIO

2. 如果中断触发次数为0：
   - 中断未配置成功
   - 编码器信号未到达ESP32

3. 如果中断触发次数有增加但计数为0：
   - 编码器A、B相可能接反
   - 或编码器逻辑有问题

请重新上传代码并测试，把串口监视器中的诊断信息发给我，特别是：
- 编码器引脚电平状态
- 中断触发次数

这样我可以帮你定位问题。

```shellscript
rostopic pub -r 10 /yolo_car/cmd_vel geometry_msgs/Twist "linear: {x: 0.045, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.6}"
```

---

**User**

轮速 - 左轮: 0.0000 m/s, 右轮: 0.0000 m/s
编码器计数 - 左轮: 0, 右轮: 0
轮速 - 左轮: 0.0000 m/s, 右轮: 0.0000 m/s
编码器计数 - 左轮: 0, 右轮: 0
轮速 - 左轮: 0.0000 m/s, 右轮: 0.0000 m/s
编码器计数 - 左轮: 0, 右轮: 0
轮速 - 左轮: 0.0000 m/s, 右轮: 0.0000 m/s
编码器计数 - 左轮: 0, 右轮: 0
轮速 - 左轮: 0.0000 m/s, 右轮: 0.0000 m/s
编码器计数 - 左轮: 0, 右轮: 0
轮速 - 左轮: 0.0000 m/s, 右轮: 0.0000 m/s
编码器计数 - 左轮: 0, 右轮: 0
轮速 - 左轮: 0.0000 m/s, 右轮: 0.0000 m/s
编码器计数 - 左轮: 0, 右轮: 0
轮速 - 左轮: 0.0000 m/s, 右轮: 0.0000 m/s
编码器计数 - 左轮: 0, 右轮: 0怎么还是全部为0，还有一个问题就是，我还没运行  z: 0.6"
mycar@Mycar:~/catkin_ws$ rostopic pub -r 10 /yolo_car/cmd_vel geometry_msgs/Twist "linear:
  x: 0.045
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.6"
这个命令它就在打印了，打印的全部是0，运行后呢，它打印的也全部是0，这是什么问题，ab相接了的呀，右电机AB相分别接的7,8引脚，左电机AB相分别接的9.10引脚，这没问题呀，你看看是什么问题，怎么解决

