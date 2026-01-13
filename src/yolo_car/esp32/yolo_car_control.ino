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

