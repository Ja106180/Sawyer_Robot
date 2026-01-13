/*
 * ESP32-S3 + L298N电机驱动 + SG90舵机 + 编码器 + MPU6050 + ROS1 (rosserial)
 *
 * 功能：
 * 1. 订阅速度指令控制TB6612电机 (/my_car_yolo/cmd_vel)
 * 2. 订阅角度指令控制360°舵机 (/my_car_yolo/servo_angle)
 * 3. 编码器计数并发布轮速 (/my_car_yolo/wheel_encoders)
 * 4. MPU6050读取并发布原始IMU数据 (/my_car_yolo/imu_raw)
 * 5. ESP32只接收ROS命令并执行，不做自主控制
 *
 * 接线（根据用户要求）：
 * - MPU6050: SCL->GPIO1, SDA->GPIO2
 * - SG90舵机信号线: GPIO3
 * - L298N电机驱动: N1->GPIO4(左PWM), N2->GPIO5(左DIR), N3->GPIO6(右PWM), N4->GPIO7(右DIR)
 * - 编码器: 右轮A->GPIO8, B->GPIO9; 左轮A->GPIO10, B->GPIO11
 */

#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Float32.h>
#include <Wire.h>

// WiFi配置 (用户指定)
const char* ssid = "机器人小车局域网";
const char* password = "3252362734";
IPAddress server(192, 168, 31, 64);  // ROS master IP
const uint16_t serverPort = 11411;

// MPU6050引脚 (用户指定)
#define MPU_ADDR 0x68
#define I2C_SDA 2  // 用户指定GPIO2
#define I2C_SCL 1  // 用户指定GPIO1

// SG90舵机引脚 (用户指定)
#define SERVO_PIN 3  // 用户指定GPIO3

// TB6612电机驱动引脚
// 注意：根据实际测试，物理左电机接在原来的 B 通道（GPIO10/11/12），物理右电机接在原来的 A 通道（GPIO9/8/7）
// 这里重新命名，让 MOTOR_LEFT_* 永远表示“物理左轮”，MOTOR_RIGHT_* 永远表示“物理右轮”
#define MOTOR_LEFT_IN1   10  // 物理左轮方向控制1 (原 BIN1)
#define MOTOR_LEFT_IN2   11  // 物理左轮方向控制2 (原 BIN2)
#define MOTOR_LEFT_PWM   12  // 物理左轮PWM        (原 PWMB)
#define MOTOR_RIGHT_IN1   9  // 物理右轮方向控制1 (原 AIN1)
#define MOTOR_RIGHT_IN2   8  // 物理右轮方向控制2 (原 AIN2)
#define MOTOR_RIGHT_PWM   7  // 物理右轮PWM        (原 PWMA)

// 编码器引脚 (用户指定接线)
#define ENC_RIGHT_A 45  // E2A - 右轮A相
#define ENC_RIGHT_B 46  // E2B - 右轮B相
#define ENC_LEFT_A  41  // E1A - 左轮A相
#define ENC_LEFT_B  42  // E1B - 左轮B相
#define ENC_ADC    15  // ADC引脚

// PWM参数
#define PWM_FREQ 1000      // PWM频率 1kHz
#define PWM_RESOLUTION 8   // 8位分辨率 (0-255)
#define SERVO_FREQ 50      // 舵机PWM频率 50Hz
#define SERVO_RESOLUTION 12

// 速度限制
#define MAX_PWM_VALUE 200  // 最大PWM值

// 命令超时（毫秒）
#define CMD_TIMEOUT_MS 500

// 编码器参数 (JGB-520电机：11线编码器，减速比30:1)
#define TICKS_PER_REV 1320.0f     // 四倍频：11×4×30 = 1320脉冲/轮
#define WHEEL_RADIUS_M 0.0670f    // 轮子半径：67mm = 0.067m
#define WHEEL_BASE_M   0.1940f    // 轮距：194mm = 0.194m

// 目标点控制参数 (全局变量，方便修改)
// ESP32只接收并执行ROS命令，不使用内部PID控制

// 编码器计数变量（volatile用于中断）
volatile int32_t enc_left_count = 0;
volatile int32_t enc_right_count = 0;
volatile int8_t enc_left_last_state = 0;
volatile int8_t enc_right_last_state = 0;

// 轮速计算
float left_wheel_vel = 0.0f;
float right_wheel_vel = 0.0f;
unsigned long last_enc_time = 0;
const unsigned long ENC_UPDATE_INTERVAL_MS = 50; // 50ms更新一次

// ROS节点
ros::NodeHandle nh;
unsigned long last_cmd_time = 0;

// ESP32只执行ROS命令，不维护内部状态

// ROS消息变量
geometry_msgs::Twist cmd_vel_msg;
sensor_msgs::Imu imu_raw_msg;
std_msgs::Int32MultiArray wheel_encoders_msg;
std_msgs::Float32 servo_angle_msg;

// 舵机点动控制状态变量
bool servo_searching_ = false;  // 是否正在搜索（由ROS命令控制）

// 点动参数（全局变量，方便你后续调整）
const uint8_t SERVO_PULSE_DUTY = 40;        // 点动时的占空比（0~255，越小越弱）
const uint16_t SERVO_ON_TIME_MS = 30;       // 每次点动"开"的时间（毫秒）
const uint16_t SERVO_OFF_TIME_MS = 500;     // 每次点动之间"关"的时间（毫秒）

// 点动控制的时间戳
unsigned long servo_last_pulse_time_ = 0;
bool servo_pulse_state_ = false;  // false=关, true=开

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

// 电机急停 (TB6612)
void stopMotors() {
  // 停止PWM
  ledcWrite(0, 0);  // 左轮PWM
  ledcWrite(1, 0);  // 右轮PWM

  // 设置为低电平（浮动停止）
  digitalWrite(MOTOR_LEFT_IN1, LOW);
  digitalWrite(MOTOR_LEFT_IN2, LOW);
  digitalWrite(MOTOR_RIGHT_IN1, LOW);
  digitalWrite(MOTOR_RIGHT_IN2, LOW);
}

// 将期望线速度映射为PWM，占空比带“死区补偿”和非线性
int speedToPwm(float vel, float max_vel) {
  // 非常小的速度当作0，防止无意义的小抖动
  if (fabs(vel) < 1e-3f) {
    return 0;
  }

  // 速度归一化到 0~1
  float mag = fabs(vel) / max_vel;
  if (mag > 1.0f) mag = 1.0f;

  // 为了保证“只要ROS想让它走，就一定能推得动车身”，
  // 这里设置一个最小PWM（死区补偿），实际值需要根据电机和地面摩擦微调。
  const int PWM_MIN = 80;            // 最小能推动车身的PWM（需要现场微调）
  const int PWM_MAX = MAX_PWM_VALUE; // 上限仍然用 MAX_PWM_VALUE

  int pwm = PWM_MIN + (int)((PWM_MAX - PWM_MIN) * mag);
  if (pwm > PWM_MAX) pwm = PWM_MAX;
  if (pwm < 0) pwm = 0;
  return pwm;
}

// 速度指令回调
void cmdVelCallback(const geometry_msgs::Twist& msg) {
  // 直接控制电机
  float linear_vel = msg.linear.x;
  float angular_vel = msg.angular.z;

  last_cmd_time = millis();

  if (fabs(linear_vel) < 1e-3 && fabs(angular_vel) < 1e-3) {
    stopMotors();
    return;
  }

  // 差速控制
  float left_vel = linear_vel - (angular_vel * WHEEL_BASE_M / 2.0);
  float right_vel = linear_vel + (angular_vel * WHEEL_BASE_M / 2.0);

  // 最大期望轮线速度（m/s），与ROS端控制逻辑一致
  float max_vel = 0.3f;
  left_vel = constrain(left_vel, -max_vel, max_vel);
  right_vel = constrain(right_vel, -max_vel, max_vel);

  // 将线速度映射为PWM，占空比带死区补偿
  int left_pwm = speedToPwm(left_vel, max_vel);
  int right_pwm = speedToPwm(right_vel, max_vel);

  // TB6612方向控制
  // 左轮方向控制（物理左轮：IN1=HIGH, IN2=LOW 为“前进”）
  if (left_vel >= 0) {
    // 正转
    digitalWrite(MOTOR_LEFT_IN1, HIGH);
    digitalWrite(MOTOR_LEFT_IN2, LOW);
  } else {
    // 反转
    digitalWrite(MOTOR_LEFT_IN1, LOW);
    digitalWrite(MOTOR_LEFT_IN2, HIGH);
  }

  // 右轮方向控制（物理右轮：接线与左轮相反，IN1=LOW, IN2=HIGH 为“前进”）
  if (right_vel >= 0) {
    // 正转（前进）
    digitalWrite(MOTOR_RIGHT_IN1, LOW);
    digitalWrite(MOTOR_RIGHT_IN2, HIGH);
  } else {
    // 反转（后退）
    digitalWrite(MOTOR_RIGHT_IN1, HIGH);
    digitalWrite(MOTOR_RIGHT_IN2, LOW);
  }

  ledcWrite(0, left_pwm);
  ledcWrite(1, right_pwm);
}

// 舵机控制回调（点动方式）
// ROS端发送：0.0 = 停止，1.0 = 开始点动搜索
void servoAngleCallback(const std_msgs::Float32& msg) {
  float val = msg.data;

  if (fabs(val) < 0.001f) {
    // 停止：关闭搜索标志，立即停止PWM输出
    servo_searching_ = false;
    ledcWrite(2, 0);
    Serial.println(">>> 舵机指令: 停止");
  } else {
    // 开始搜索：设置搜索标志，点动逻辑在loop()中执行
    servo_searching_ = true;
    Serial.println(">>> 舵机指令: 开始点动搜索");
  }
}

// 订阅者
ros::Subscriber<geometry_msgs::Twist> cmd_vel_sub("/my_car_yolo/cmd_vel", cmdVelCallback);
ros::Subscriber<std_msgs::Float32> servo_angle_sub("/my_car_yolo/servo_angle", servoAngleCallback);

// 发布者
ros::Publisher imu_raw_pub("/my_car_yolo/imu_raw", &imu_raw_msg);
ros::Publisher wheel_encoders_pub("/my_car_yolo/wheel_encoders", &wheel_encoders_msg);

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
  Wire.write(0x1C);
  Wire.write(0x00);
  Wire.endTransmission();

  // 配置陀螺仪范围 ±250°/s
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1B);
  Wire.write(0x00);
  Wire.endTransmission();

  return true;
}

// 读取MPU6050原始数据
void readMPU6050() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);

  int addr = (int)MPU_ADDR;
  int len = 14;
  int stop = 1;
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
    float accel_scale = 2.0f * 9.8f / 32768.0f;
    imu_raw_msg.linear_acceleration.x = accel_x * accel_scale;
    imu_raw_msg.linear_acceleration.y = accel_y * accel_scale;
    imu_raw_msg.linear_acceleration.z = accel_z * accel_scale;

    float gyro_scale = 250.0f * M_PI / 180.0f / 32768.0f;
    imu_raw_msg.angular_velocity.x = gyro_x * gyro_scale;
    imu_raw_msg.angular_velocity.y = gyro_y * gyro_scale;
    imu_raw_msg.angular_velocity.z = gyro_z * gyro_scale;

    imu_raw_msg.header.stamp = nh.now();
    imu_raw_msg.header.frame_id = "imu_link";
  }
}

// 更新轮速和编码器数据
void updateWheelVelocities() {
  unsigned long current_time = millis();
  unsigned long dt_ms = current_time - last_enc_time;

  if (dt_ms >= ENC_UPDATE_INTERVAL_MS) {
    int32_t left_ticks = enc_left_count;
    int32_t right_ticks = enc_right_count;
    enc_left_count = 0;
    enc_right_count = 0;

    float ticks_per_meter = TICKS_PER_REV / (2.0f * M_PI * WHEEL_RADIUS_M);
    float dt_sec = dt_ms / 1000.0f;

    left_wheel_vel = (left_ticks / ticks_per_meter) / dt_sec;
    right_wheel_vel = (right_ticks / ticks_per_meter) / dt_sec;

    // 更新消息 (使用Int32MultiArray: [left_pulses, right_pulses, left_velocity*1000, right_velocity*1000])
    wheel_encoders_msg.data_length = 4;
    wheel_encoders_msg.data = (int32_t*)malloc(sizeof(int32_t) * 4);
    wheel_encoders_msg.data[0] = left_ticks;           // 左轮脉冲数
    wheel_encoders_msg.data[1] = right_ticks;          // 右轮脉冲数
    wheel_encoders_msg.data[2] = (int32_t)(left_wheel_vel * 1000.0f);  // 左轮速度(m/s)*1000
    wheel_encoders_msg.data[3] = (int32_t)(right_wheel_vel * 1000.0f); // 右轮速度(m/s)*1000

    wheel_encoders_pub.publish(&wheel_encoders_msg);
    free(wheel_encoders_msg.data);  // 释放内存

    last_enc_time = current_time;
  }
}

// ESP32只接收ROS命令并执行，不做自主控制
// 所有的控制逻辑都在ROS端实现

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("\n\n=== ESP32-S3 + L298N + SG90 + 编码器 + MPU6050 + ROS1 ===");

  last_cmd_time = millis();

  // 初始化TB6612电机驱动引脚
  Serial.println("初始化TB6612电机驱动...");
  pinMode(MOTOR_LEFT_IN1, OUTPUT);
  pinMode(MOTOR_LEFT_IN2, OUTPUT);
  pinMode(MOTOR_RIGHT_IN1, OUTPUT);
  pinMode(MOTOR_RIGHT_IN2, OUTPUT);

  // 初始为停止状态
  digitalWrite(MOTOR_LEFT_IN1, LOW);
  digitalWrite(MOTOR_LEFT_IN2, LOW);
  digitalWrite(MOTOR_RIGHT_IN1, LOW);
  digitalWrite(MOTOR_RIGHT_IN2, LOW);

  // 配置PWM通道（TB6612电机）
  ledcSetup(0, PWM_FREQ, PWM_RESOLUTION);  // 左轮PWM (PWMA)
  ledcSetup(1, PWM_FREQ, PWM_RESOLUTION);  // 右轮PWM (PWMB)
  ledcAttachPin(MOTOR_LEFT_PWM, 0);   // GPIO7 - PWMA
  ledcAttachPin(MOTOR_RIGHT_PWM, 1);  // GPIO12 - PWMB

  // 配置PWM通道（舵机 - 使用500Hz, 8bit，因为它是普通电机）
  const int SERVO_MOTOR_FREQ = 500;      // 500Hz
  const int SERVO_MOTOR_RES = 8;         // 8 bit -> 0~255
  ledcSetup(2, SERVO_MOTOR_FREQ, SERVO_MOTOR_RES);
  ledcAttachPin(SERVO_PIN, 2);

  ledcWrite(0, 0);
  ledcWrite(1, 0);
  // 舵机初始化时不发送任何控制信号，保持上电前的物理位置
  ledcWrite(2, 0);

  // 注意：不强制设置舵机初始角度，保持手动设置的位置
  // 舵机只在收到ROS端的servo_angle话题时才运动

  Serial.println("✓ 电机和舵机初始化完成（保持当前位置）");

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

  // WiFi连接
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

  // ROS连接
  Serial.println("\n=== ROS连接 ===");
  nh.initNode();
  nh.subscribe(cmd_vel_sub);
  nh.subscribe(servo_angle_sub);
  nh.advertise(imu_raw_pub);
  nh.advertise(wheel_encoders_pub);

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
  Serial.print("电机参数: TICKS_PER_REV=");
  Serial.print(TICKS_PER_REV);
  Serial.print(" (JGB-520: 11线×4倍频×30减速比)");
  Serial.print(", 轮子半径=");
  Serial.print(WHEEL_RADIUS_M);
  Serial.print("m, 轮距=");
  Serial.print(WHEEL_BASE_M);
  Serial.println("m");

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
    // 舵机在ROS断开时不再输出PWM，保持当前位置
    ledcWrite(2, 0);
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

  // 舵机点动控制逻辑（如果正在搜索）
  if (servo_searching_) {
    unsigned long current_time = millis();
    
    if (!servo_pulse_state_) {
      // 当前是"关"状态，检查是否到了该"开"的时间
      if (current_time - servo_last_pulse_time_ >= SERVO_OFF_TIME_MS) {
        // 开始点动：开一小会儿
        ledcWrite(2, SERVO_PULSE_DUTY);
        servo_pulse_state_ = true;
        servo_last_pulse_time_ = current_time;
      }
    } else {
      // 当前是"开"状态，检查是否到了该"关"的时间
      if (current_time - servo_last_pulse_time_ >= SERVO_ON_TIME_MS) {
        // 关闭：停很久
        ledcWrite(2, 0);
        servo_pulse_state_ = false;
        servo_last_pulse_time_ = current_time;
      }
    }
  } else {
    // 不在搜索状态，确保PWM输出为0
    if (servo_pulse_state_) {
      ledcWrite(2, 0);
      servo_pulse_state_ = false;
    }
  }

  // ESP32只接收ROS命令并执行，不做自主控制

  nh.spinOnce();
  delay(10);
}
