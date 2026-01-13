/*
 * ESP32-S3 + TB6612电机驱动 + 编码器 + ICM42688-P + ROS1 (rosserial)
 * 
 * 功能：
 * 1. 订阅速度指令控制TB6612电机 (/ApriTag_car/cmd_vel)
 * 2. 编码器计数并发布轮速 (/ApriTag_car/wheel_encoders)
 * 3. ICM42688-P发布原始IMU数据 (/ApriTag_car/imu_gyro, /ApriTag_car/imu_accel)
 * 4. ESP32只接收ROS命令并执行，不做自主控制
 * 
 * 接线（根据用户要求）：
 * - ICM42688-P: SCL->GPIO1, SDA->GPIO2
 * - TB6612电机驱动: 左轮IN1->GPIO10, IN2->GPIO11, PWM->GPIO12
 *                   右轮IN1->GPIO9, IN2->GPIO8, PWM->GPIO7
 * - 编码器: 右轮A->GPIO45, B->GPIO46; 左轮A->GPIO41, B->GPIO42
 */

#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Int32MultiArray.h>
#include <Wire.h>
#include <WiFi.h>

// WiFi配置
const char* ssid = "机器人小车局域网";
const char* password = "3252362734";
IPAddress server(192, 168, 31, 64);  // ROS master IP
const uint16_t serverPort = 11411;

// ICM42688-P引脚和地址
#define ICM42688_ADDR_0 0x68
#define ICM42688_ADDR_1 0x69
#define ICM42688_ADDR ICM42688_ADDR_1
#define I2C_SDA 2
#define I2C_SCL 1

// ========== IMU参数 ==========
const int IMU_LOOP_PERIOD_MS = 10;          // IMU更新周期（10ms = 100Hz）
// ==============================

uint8_t icm42688_addr = 0;

// IMU原始数据变量

// TB6612电机驱动引脚
#define MOTOR_LEFT_IN1   10
#define MOTOR_LEFT_IN2   11
#define MOTOR_LEFT_PWM   12
#define MOTOR_RIGHT_IN1   9
#define MOTOR_RIGHT_IN2   8
#define MOTOR_RIGHT_PWM   7

// 编码器引脚
#define ENC_RIGHT_A 45
#define ENC_RIGHT_B 46
#define ENC_LEFT_A  41
#define ENC_LEFT_B  42

// PWM参数
#define PWM_FREQ 1000
#define PWM_RESOLUTION 8
#define MAX_PWM_VALUE 200
#define CMD_TIMEOUT_MS 500

// 编码器参数
#define TICKS_PER_REV 1320.0f
#define WHEEL_RADIUS_M 0.0335f
#define WHEEL_BASE_M   0.1940f

// 编码器计数变量（volatile用于中断）
volatile int32_t enc_left_count = 0;
volatile int32_t enc_right_count = 0;
volatile int8_t enc_left_last_state = 0;
volatile int8_t enc_right_last_state = 0;

// 轮速计算
float left_wheel_vel = 0.0f;
float right_wheel_vel = 0.0f;
unsigned long last_enc_time = 0;
const unsigned long ENC_UPDATE_INTERVAL_MS = 50;

// ROS节点
ros::NodeHandle nh;
unsigned long last_cmd_time = 0;

// ROS消息变量
geometry_msgs::Twist cmd_vel_msg;
geometry_msgs::Vector3 imu_gyro_msg;     // 陀螺仪数据 (rad/s)
geometry_msgs::Vector3 imu_accel_msg;    // 加速度计数据 (m/s²)
std_msgs::Int32MultiArray wheel_encoders_msg;

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
  digitalWrite(MOTOR_LEFT_IN1, LOW);
  digitalWrite(MOTOR_LEFT_IN2, LOW);
  digitalWrite(MOTOR_RIGHT_IN1, LOW);
  digitalWrite(MOTOR_RIGHT_IN2, LOW);
}

// 将期望线速度映射为PWM
int speedToPwm(float vel, float max_vel) {
  if (fabs(vel) < 1e-6f) {  // 更小的死区，只在速度接近0时停止
    return 0;
  }
  float mag = fabs(vel) / max_vel;
  if (mag > 1.0f) mag = 1.0f;
  int pwm = (int)(MAX_PWM_VALUE * mag);
  if (pwm > MAX_PWM_VALUE) pwm = MAX_PWM_VALUE;
  if (pwm < 0) pwm = 0;
  return pwm;
}

// 速度指令回调
void cmdVelCallback(const geometry_msgs::Twist& msg) {
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

  float max_vel = 0.3f;
  left_vel = constrain(left_vel, -max_vel, max_vel);
  right_vel = constrain(right_vel, -max_vel, max_vel);

  int left_pwm = speedToPwm(left_vel, max_vel);
  int right_pwm = speedToPwm(right_vel, max_vel);

  // TB6612方向控制
  if (left_vel >= 0) {
    digitalWrite(MOTOR_LEFT_IN1, LOW);
    digitalWrite(MOTOR_LEFT_IN2, HIGH);
  } else {
    digitalWrite(MOTOR_LEFT_IN1, HIGH);
    digitalWrite(MOTOR_LEFT_IN2, LOW);
  }

  // 右轮方向反接，逻辑取反
  if (right_vel >= 0) {
    digitalWrite(MOTOR_RIGHT_IN1, HIGH);
    digitalWrite(MOTOR_RIGHT_IN2, LOW);
  } else {
    digitalWrite(MOTOR_RIGHT_IN1, LOW);
    digitalWrite(MOTOR_RIGHT_IN2, HIGH);
  }

  ledcWrite(0, left_pwm);
  ledcWrite(1, right_pwm);
}

// 订阅者
ros::Subscriber<geometry_msgs::Twist> cmd_vel_sub("/ApriTag_car/cmd_vel", cmdVelCallback);

// 发布者
ros::Publisher imu_gyro_pub("/ApriTag_car/imu_gyro", &imu_gyro_msg);
ros::Publisher imu_accel_pub("/ApriTag_car/imu_accel", &imu_accel_msg);
ros::Publisher wheel_encoders_pub("/ApriTag_car/wheel_encoders", &wheel_encoders_msg);

// ICM42688寄存器操作函数
bool writeICMRegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(icm42688_addr);
  Wire.write(reg);
  Wire.write(value);
  return (Wire.endTransmission() == 0);
}

bool readICMRegisters(uint8_t reg, uint8_t* data, uint8_t length) {
  Wire.beginTransmission(icm42688_addr);
  Wire.write(reg);
  if (Wire.endTransmission() != 0) return false;
  
  uint8_t received = Wire.requestFrom((uint8_t)icm42688_addr, (uint8_t)length);
  if (received < length) return false;
  
  for (uint8_t i = 0; i < length; i++) {
    data[i] = Wire.read();
  }
  return true;
}

// ICM42688初始化
bool initICM42688() {
  bool found = false;
  uint8_t addr_list[2] = {ICM42688_ADDR, (ICM42688_ADDR == ICM42688_ADDR_1) ? ICM42688_ADDR_0 : ICM42688_ADDR_1};
  
  for (int i = 0; i < 2; i++) {
    Wire.beginTransmission(addr_list[i]);
    if (Wire.endTransmission() == 0) {
      icm42688_addr = addr_list[i];
      found = true;
      break;
    }
  }
  
  if (!found) {
    return false;
  }
  
  writeICMRegister(0x4E, 0x0F);
  delay(50);
  writeICMRegister(0x50, 0x04);
  delay(10);
  writeICMRegister(0x4F, 0x04);
  delay(10);
  
  uint8_t test_data[14];
  if (readICMRegisters(0x1D, test_data, 14)) {
    return true;
  }
  return false;
}

// 读取并处理IMU数据（互补滤波）
void processIMU() {
  // 如果IMU未初始化，尝试重新初始化
  if (icm42688_addr == 0) {
    static unsigned long last_reinit_time = 0;
    if (millis() - last_reinit_time > 5000) {
      initICM42688();
      last_reinit_time = millis();
    }
    return;
  }

  uint8_t data[14];
  if (!readICMRegisters(0x1D, data, 14)) {
    return;
  }

  // 读取原始数据
  int16_t accel_raw[3], gyro_raw[3];
  accel_raw[0] = (int16_t)((data[2] << 8) | data[3]);
  accel_raw[1] = (int16_t)((data[4] << 8) | data[5]);
  accel_raw[2] = (int16_t)((data[6] << 8) | data[7]);

  gyro_raw[0] = (int16_t)((data[8] << 8) | data[9]);
  gyro_raw[1] = (int16_t)((data[10] << 8) | data[11]);
  gyro_raw[2] = (int16_t)((data[12] << 8) | data[13]);

  // 转换为物理单位
  const float ACCEL_SCALE = (16.0f / 32768.0f) * 9.80665f;  // m/s²
  float accel_x = accel_raw[0] * ACCEL_SCALE;
  float accel_y = accel_raw[1] * ACCEL_SCALE;
  float accel_z = accel_raw[2] * ACCEL_SCALE;

  const float GYRO_SCALE = (2000.0f / 32768.0f) * (PI / 180.0f);  // rad/s
  float gyro_x = gyro_raw[0] * GYRO_SCALE;
  float gyro_y = gyro_raw[1] * GYRO_SCALE;
  float gyro_z = gyro_raw[2] * GYRO_SCALE;

  // 设置陀螺仪消息 (rad/s)
  imu_gyro_msg.x = gyro_x;
  imu_gyro_msg.y = gyro_y;
  imu_gyro_msg.z = gyro_z;

  // 设置加速度计消息 (m/s²)
  imu_accel_msg.x = accel_x;
  imu_accel_msg.y = accel_y;
  imu_accel_msg.z = accel_z;
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

    float ticks_per_meter = TICKS_PER_REV / (2.0f * PI * WHEEL_RADIUS_M);
    float dt_sec = dt_ms / 1000.0f;

    left_wheel_vel = (left_ticks / ticks_per_meter) / dt_sec;
    right_wheel_vel = (right_ticks / ticks_per_meter) / dt_sec;

    // 更新消息
    wheel_encoders_msg.data_length = 4;
    wheel_encoders_msg.data = (int32_t*)malloc(sizeof(int32_t) * 4);
    wheel_encoders_msg.data[0] = left_ticks;
    wheel_encoders_msg.data[1] = right_ticks;
    wheel_encoders_msg.data[2] = (int32_t)(left_wheel_vel * 1000.0f);
    wheel_encoders_msg.data[3] = (int32_t)(right_wheel_vel * 1000.0f);

    wheel_encoders_pub.publish(&wheel_encoders_msg);
    free(wheel_encoders_msg.data);

    last_enc_time = current_time;
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  last_cmd_time = millis();

  // 初始化TB6612电机驱动引脚
  pinMode(MOTOR_LEFT_IN1, OUTPUT);
  pinMode(MOTOR_LEFT_IN2, OUTPUT);
  pinMode(MOTOR_RIGHT_IN1, OUTPUT);
  pinMode(MOTOR_RIGHT_IN2, OUTPUT);

  digitalWrite(MOTOR_LEFT_IN1, LOW);
  digitalWrite(MOTOR_LEFT_IN2, LOW);
  digitalWrite(MOTOR_RIGHT_IN1, LOW);
  digitalWrite(MOTOR_RIGHT_IN2, LOW);

  // 配置PWM通道
  ledcSetup(0, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(1, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(MOTOR_LEFT_PWM, 0);
  ledcAttachPin(MOTOR_RIGHT_PWM, 1);

  ledcWrite(0, 0);
  ledcWrite(1, 0);

  // 初始化编码器
  pinMode(ENC_LEFT_A, INPUT_PULLUP);
  pinMode(ENC_LEFT_B, INPUT_PULLUP);
  pinMode(ENC_RIGHT_A, INPUT_PULLUP);
  pinMode(ENC_RIGHT_B, INPUT_PULLUP);

  enc_left_last_state = (digitalRead(ENC_LEFT_A) << 1) | digitalRead(ENC_LEFT_B);
  enc_right_last_state = (digitalRead(ENC_RIGHT_A) << 1) | digitalRead(ENC_RIGHT_B);

  attachInterrupt(digitalPinToInterrupt(ENC_LEFT_A), encLeftA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_RIGHT_A), encRightA, CHANGE);

  last_enc_time = millis();

  // 初始化ICM42688-P
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(100000);
  delay(500);

  bool imu_init_success = false;
  for (int retry = 0; retry < 3; retry++) {
    if (initICM42688()) {
      if (icm42688_addr != 0) {
        imu_init_success = true;
        break;
      }
    }
    if (retry < 2) delay(500);
  }

  // WiFi连接
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  int wifiAttempts = 0;
  while (WiFi.status() != WL_CONNECTED && wifiAttempts < 30) {
    delay(500);
    wifiAttempts++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    nh.getHardware()->setConnection(server, serverPort);
  }

  // ROS连接
  nh.initNode();
  nh.subscribe(cmd_vel_sub);
  nh.advertise(imu_gyro_pub);
  nh.advertise(imu_accel_pub);
  nh.advertise(wheel_encoders_pub);

  int rosAttempts = 0;
  while (!nh.connected() && rosAttempts < 50) {
    nh.spinOnce();
    delay(100);
    rosAttempts++;
  }
}

void loop() {
  // 命令超时保护
  if (millis() - last_cmd_time > CMD_TIMEOUT_MS) {
    stopMotors();
  }

  if (!nh.connected()) {
    stopMotors();
    nh.spinOnce();
    delay(100);
    return;
  }

  // 更新轮速（20Hz）
  updateWheelVelocities();

  // 处理并发布IMU原始数据（100Hz = 10ms间隔）
  static unsigned long last_imu_time = 0;
  if (millis() - last_imu_time >= IMU_LOOP_PERIOD_MS) {
    processIMU();
    imu_gyro_pub.publish(&imu_gyro_msg);
    imu_accel_pub.publish(&imu_accel_msg);
    last_imu_time = millis();
  }

  nh.spinOnce();
  delay(1);
}

