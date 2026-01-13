/*
 * ESP32-S3 两轮自平衡小车 - LQR控制 + Madgwick滤波
 * 
 * 功能：
 * 1. MadgwickAHRS姿态融合（工业级）
 * 2. LQR最优控制（4维状态反馈）
 * 3. 编码器位置和速度估计
 * 4. TB6612电机驱动
 * 5. 串口调试输出
 * 
 * 硬件连接：
 * - ICM42688-P: SCL->GPIO1, SDA->GPIO2
 * - TB6612: 左轮IN1->GPIO10, IN2->GPIO11, PWM->GPIO12
 *           右轮IN1->GPIO9, IN2->GPIO8, PWM->GPIO7
 * - 编码器: 左轮A->GPIO41, B->GPIO42
 *           右轮A->GPIO45, B->GPIO46
 */

#include <Wire.h>
#include <MadgwickAHRS.h>

// ========== 物理参数 ==========
#define WHEEL_RADIUS_M   0.0335f   // 轮子半径：33.5mm
#define WHEEL_BASE_M     0.1940f   // 轮距：194mm
#define CENTER_HEIGHT_M  0.075f    // 重心高度：75mm
#define MASS_KG          1.5f      // 质量：1.5kg
#define TICKS_PER_REV    1320.0f   // 编码器每转脉冲数

// ========== ICM42688-P配置 ==========
#define ICM42688_ADDR_0  0x68
#define ICM42688_ADDR_1  0x69
#define ICM42688_ADDR    ICM42688_ADDR_1
#define I2C_SDA          2
#define I2C_SCL          1

// ========== TB6612电机驱动引脚 ==========
#define MOTOR_LEFT_IN1   10
#define MOTOR_LEFT_IN2   11
#define MOTOR_LEFT_PWM   12
#define MOTOR_RIGHT_IN1  9
#define MOTOR_RIGHT_IN2  8
#define MOTOR_RIGHT_PWM  7

// ========== 编码器引脚 ==========
#define ENC_LEFT_A       41
#define ENC_LEFT_B       42
#define ENC_RIGHT_A      45
#define ENC_RIGHT_B      46

// ========== PWM参数 ==========
#define PWM_FREQ         1000
#define PWM_RESOLUTION   8
#define MAX_PWM_VALUE    200
#define DEADZONE_PWM     35        // 电机死区PWM值（实测）

// ========== 控制参数 ==========
#define CONTROL_FREQ_HZ  100       // 控制频率：100Hz
#define LOOP_PERIOD_MS   10        // 循环周期：10ms
#define CALIB_SAMPLES    200       // 陀螺仪校准样本数
#define MAX_VELOCITY     0.25f     // 最大速度：0.25 m/s

// ========== LQR增益矩阵（工程经验值）==========
// 状态向量：x = [θ, θ_dot, x, x_dot]^T
// 控制律：u = -K * x
float K[4] = {120.0, 25.0, 1.5, 6.0};
// K[0]: 角度权重（最重要）
// K[1]: 角速度权重（阻尼）
// K[2]: 位置权重（较小）
// K[3]: 速度权重（较小）

// ========== 全局变量 ==========
uint8_t icm42688_addr = 0;

// 编码器计数（volatile用于中断）
volatile int32_t enc_left_count = 0;
volatile int32_t enc_right_count = 0;
volatile int8_t enc_left_last_state = 0;
volatile int8_t enc_right_last_state = 0;

// 状态变量
float pitch = 0.0f;           // pitch角度（rad）
float pitch_rate = 0.0f;      // pitch角速度（rad/s）
float position = 0.0f;        // 位置（m）
float velocity = 0.0f;        // 速度（m/s）

// 陀螺仪零偏
float gyro_bias_x = 0.0f;
float gyro_bias_y = 0.0f;
float gyro_bias_z = 0.0f;

// 校准状态
bool calibrated = false;
int calib_count = 0;
float gyro_sum_x = 0.0f;
float gyro_sum_y = 0.0f;
float gyro_sum_z = 0.0f;

// Madgwick滤波器实例
Madgwick filter;

// ========== ICM42688寄存器操作 ==========
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

// ========== ICM42688初始化 ==========
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
    
    if (!found) return false;
    
    // 软复位
    writeICMRegister(0x4E, 0x0F);
    delay(50);
    
    // 配置加速度计：±16g, 100Hz
    writeICMRegister(0x50, 0x04);
    delay(10);
    
    // 配置陀螺仪：±2000dps, 100Hz
    writeICMRegister(0x4F, 0x04);
    delay(10);
    
    // 测试读取
    uint8_t test_data[14];
    return readICMRegisters(0x1D, test_data, 14);
}

// ========== 编码器中断处理 ==========
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

// ========== 读取IMU数据 ==========
bool readIMU(float &ax, float &ay, float &az, float &gx, float &gy, float &gz) {
    uint8_t data[14];
    if (!readICMRegisters(0x1D, data, 14)) {
        return false;
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
    const float GYRO_SCALE = (2000.0f / 32768.0f) * (PI / 180.0f);  // rad/s
    
    ax = accel_raw[0] * ACCEL_SCALE;
    ay = accel_raw[1] * ACCEL_SCALE;
    az = accel_raw[2] * ACCEL_SCALE;
    gx = gyro_raw[0] * GYRO_SCALE;
    gy = gyro_raw[1] * GYRO_SCALE;
    gz = gyro_raw[2] * GYRO_SCALE;
    
    return true;
}

// ========== 更新编码器数据（用PWM方向判断正负）==========
void updateEncoder(float dt, int motor_pwm) {
    // 读取并清零计数（原子操作）
    noInterrupts();
    int32_t left_ticks = enc_left_count;
    int32_t right_ticks = enc_right_count;
    enc_left_count = 0;
    enc_right_count = 0;
    interrupts();
    
    // 计算增量距离（使用绝对值）
    float ticks_per_meter = TICKS_PER_REV / (2.0f * PI * WHEEL_RADIUS_M);
    float left_delta = abs(left_ticks) / ticks_per_meter;
    float right_delta = abs(right_ticks) / ticks_per_meter;
    float delta_pos = (left_delta + right_delta) / 2.0f;
    
    // 根据PWM符号判断移动方向
    if (motor_pwm > 0) {
        position += delta_pos;      // 向前
        velocity = delta_pos / dt;
    } else if (motor_pwm < 0) {
        position -= delta_pos;      // 向后
        velocity = -delta_pos / dt;
    } else {
        velocity = 0;               // 静止
    }
}

// ========== LQR控制 ==========
float lqrControl(float pitch, float pitch_rate, float position, float velocity) {
    // 控制律: u = -K * x
    return -(K[0] * pitch + K[1] * pitch_rate + K[2] * position + K[3] * velocity);
}

// ========== 速度到PWM映射 ==========
int speedToPwm(float vel) {
    if (fabs(vel) < 1e-6f) {
        return 0;  // 静止
    }
    
    // 归一化到[-1, 1]
    float ratio = vel / MAX_VELOCITY;
    ratio = constrain(ratio, -1.0f, 1.0f);
    
    // 线性映射：[-1,1] → [-(200-35), -35] U [35, 200]
    int pwm = 0;
    if (ratio > 0) {
        pwm = DEADZONE_PWM + (MAX_PWM_VALUE - DEADZONE_PWM) * ratio;
    } else {
        pwm = -(DEADZONE_PWM + (MAX_PWM_VALUE - DEADZONE_PWM) * fabs(ratio));
    }
    
    return constrain(pwm, -MAX_PWM_VALUE, MAX_PWM_VALUE);
}

// ========== 电机控制（参考代码逻辑）==========
void stopMotors() {
    ledcWrite(0, 0);
    ledcWrite(1, 0);
    digitalWrite(MOTOR_LEFT_IN1, LOW);
    digitalWrite(MOTOR_LEFT_IN2, LOW);
    digitalWrite(MOTOR_RIGHT_IN1, LOW);
    digitalWrite(MOTOR_RIGHT_IN2, LOW);
}

void driveMotors(int left_pwm, int right_pwm) {
    // 限幅
    left_pwm = constrain(left_pwm, -MAX_PWM_VALUE, MAX_PWM_VALUE);
    right_pwm = constrain(right_pwm, -MAX_PWM_VALUE, MAX_PWM_VALUE);
    
    // 左轮方向控制
    if (left_pwm == 0) {
        digitalWrite(MOTOR_LEFT_IN1, LOW);
        digitalWrite(MOTOR_LEFT_IN2, LOW);
        ledcWrite(0, 0);
    } else if (left_pwm > 0) {
        digitalWrite(MOTOR_LEFT_IN1, LOW);   // PWM>0: 向前
        digitalWrite(MOTOR_LEFT_IN2, HIGH);
        ledcWrite(0, left_pwm);
    } else {
        digitalWrite(MOTOR_LEFT_IN1, HIGH);  // PWM<0: 向后
        digitalWrite(MOTOR_LEFT_IN2, LOW);
        ledcWrite(0, -left_pwm);             // 使用绝对值
    }
    
    // 右轮方向控制（方向反接，逻辑取反）
    if (right_pwm == 0) {
        digitalWrite(MOTOR_RIGHT_IN1, LOW);
        digitalWrite(MOTOR_RIGHT_IN2, LOW);
        ledcWrite(1, 0);
    } else if (right_pwm > 0) {
        digitalWrite(MOTOR_RIGHT_IN1, HIGH);  // PWM>0: 向前（反接）
        digitalWrite(MOTOR_RIGHT_IN2, LOW);
        ledcWrite(1, right_pwm);
    } else {
        digitalWrite(MOTOR_RIGHT_IN1, LOW);   // PWM<0: 向后（反接）
        digitalWrite(MOTOR_RIGHT_IN2, HIGH);
        ledcWrite(1, -right_pwm);             // 使用绝对值
    }
}

// ========== Setup ==========
void setup() {
    Serial.begin(115200);
    delay(1000);
    
    Serial.println("\n========================================");
    Serial.println("ESP32-S3 两轮自平衡小车");
    Serial.println("Madgwick滤波 + LQR控制");
    Serial.println("========================================");
    
    // 初始化I2C
    Wire.begin(I2C_SDA, I2C_SCL);
    Wire.setClock(400000);  // 400kHz
    delay(100);
    
    // 初始化ICM42688
    Serial.print("初始化ICM42688...");
    if (!initICM42688()) {
        Serial.println(" 失败!");
        Serial.println("请检查IMU连接");
        while (1) {
            delay(1000);
        }
    }
    Serial.println(" 成功!");
    Serial.print("IMU地址: 0x");
    Serial.println(icm42688_addr, HEX);
    
    // 初始化电机驱动
    pinMode(MOTOR_LEFT_IN1, OUTPUT);
    pinMode(MOTOR_LEFT_IN2, OUTPUT);
    pinMode(MOTOR_RIGHT_IN1, OUTPUT);
    pinMode(MOTOR_RIGHT_IN2, OUTPUT);
    
    ledcSetup(0, PWM_FREQ, PWM_RESOLUTION);
    ledcSetup(1, PWM_FREQ, PWM_RESOLUTION);
    ledcAttachPin(MOTOR_LEFT_PWM, 0);
    ledcAttachPin(MOTOR_RIGHT_PWM, 1);
    
    stopMotors();
    Serial.println("电机驱动初始化完成");
    
    // 初始化编码器
    pinMode(ENC_LEFT_A, INPUT_PULLUP);
    pinMode(ENC_LEFT_B, INPUT_PULLUP);
    pinMode(ENC_RIGHT_A, INPUT_PULLUP);
    pinMode(ENC_RIGHT_B, INPUT_PULLUP);
    
    enc_left_last_state = (digitalRead(ENC_LEFT_A) << 1) | digitalRead(ENC_LEFT_B);
    enc_right_last_state = (digitalRead(ENC_RIGHT_A) << 1) | digitalRead(ENC_RIGHT_B);
    
    attachInterrupt(digitalPinToInterrupt(ENC_LEFT_A), encLeftA, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENC_RIGHT_A), encRightA, CHANGE);
    
    Serial.println("编码器初始化完成");
    
    // 初始化Madgwick滤波器
    filter.begin(CONTROL_FREQ_HZ);
    Serial.println("Madgwick滤波器初始化完成");
    
    // 陀螺仪零偏校准
    Serial.println("\n开始陀螺仪零偏校准...");
    Serial.print("请保持小车静止 (");
    Serial.print(CALIB_SAMPLES);
    Serial.println(" 个样本)");
    
    for (int i = 0; i < CALIB_SAMPLES; i++) {
        float ax, ay, az, gx, gy, gz;
        if (readIMU(ax, ay, az, gx, gy, gz)) {
            gyro_sum_x += gx;
            gyro_sum_y += gy;
            gyro_sum_z += gz;
            calib_count++;
        }
        
        if ((i + 1) % 50 == 0) {
            Serial.print("  进度: ");
            Serial.print(i + 1);
            Serial.print("/");
            Serial.println(CALIB_SAMPLES);
        }
        
        delay(10);
    }
    
    gyro_bias_x = gyro_sum_x / calib_count;
    gyro_bias_y = gyro_sum_y / calib_count;
    gyro_bias_z = gyro_sum_z / calib_count;
    
    Serial.println("\n校准完成!");
    Serial.print("陀螺仪零偏 X: ");
    Serial.print(gyro_bias_x * 180.0f / PI, 4);
    Serial.println(" deg/s");
    Serial.print("陀螺仪零偏 Y: ");
    Serial.print(gyro_bias_y * 180.0f / PI, 4);
    Serial.println(" deg/s");
    Serial.print("陀螺仪零偏 Z: ");
    Serial.print(gyro_bias_z * 180.0f / PI, 4);
    Serial.println(" deg/s");
    
    // 等待Madgwick滤波器稳定
    Serial.println("\n等待Madgwick滤波器稳定...");
    for (int i = 0; i < 200; i++) {  // 2秒
        float ax, ay, az, gx, gy, gz;
        if (readIMU(ax, ay, az, gx, gy, gz)) {
            gx -= gyro_bias_x;
            gy -= gyro_bias_y;
            gz -= gyro_bias_z;
            filter.updateIMU(gx, gy, gz, ax, ay, az);
        }
        delay(10);
    }
    Serial.println("滤波器稳定完成");
    
    // 编码器位置初始化为0
    position = 0.0f;
    velocity = 0.0f;
    
    calibrated = true;
    
    Serial.println("\n========================================");
    Serial.println("系统就绪! 请手扶小车保持垂直");
    Serial.println("松手后小车将自动平衡");
    Serial.println("========================================");
    Serial.println("\n输出格式: Pitch(deg) | Rate(deg/s) | Vel(m/s) | PWM");
    Serial.println();
    
    delay(2000);
}

// ========== Loop ==========
void loop() {
    static unsigned long last_time = millis();
    static unsigned long last_print_time = millis();
    
    unsigned long current_time = millis();
    float dt = (current_time - last_time) / 1000.0f;
    
    // 防止dt异常
    if (dt <= 0 || dt > 0.1f) {
        dt = LOOP_PERIOD_MS / 1000.0f;
    }
    last_time = current_time;
    
    // 1. 读取IMU数据
    float ax, ay, az, gx, gy, gz;
    if (!readIMU(ax, ay, az, gx, gy, gz)) {
        Serial.println("IMU读取失败!");
        stopMotors();
        delay(10);
        return;
    }
    
    // 2. 补偿陀螺仪零偏
    gx -= gyro_bias_x;
    gy -= gyro_bias_y;
    gz -= gyro_bias_z;
    
    // 3. Madgwick滤波更新姿态
    filter.updateIMU(gx, gy, gz, ax, ay, az);
    
    // 4. 提取pitch角和pitch角速度
    pitch = filter.getPitch();  // 获取pitch角度（rad）
    pitch_rate = gx;            // pitch角速度就是gyro_x（rad/s）
    
    // 注意：如果pitch角度方向不对，可能需要取反或使用getRoll()
    // 根据你的坐标系定义，前倾为负，如果不对，可以改为：pitch = -filter.getPitch();
    
    // 5. LQR控制
    float control_velocity = lqrControl(pitch, pitch_rate, position, velocity);
    
    // 6. 速度到PWM转换
    int motor_pwm = speedToPwm(control_velocity);
    
    // 7. 更新编码器数据（需要motor_pwm来判断方向）
    updateEncoder(dt, motor_pwm);
    
    // 8. 驱动电机（左右轮相同PWM，平衡模式）
    driveMotors(motor_pwm, motor_pwm);
    
    // 9. 串口输出（20Hz，每50ms输出一次）
    if (current_time - last_print_time >= 50) {
        Serial.print("P:");
        Serial.print(pitch * 180.0f / PI, 2);
        Serial.print(" | R:");
        Serial.print(pitch_rate * 180.0f / PI, 1);
        Serial.print(" | V:");
        Serial.print(velocity, 3);
        Serial.print(" | PWM:");
        Serial.println(motor_pwm);
        
        last_print_time = current_time;
    }
    
    // 10. 精确延时（100Hz控制频率）
    unsigned long elapsed = millis() - current_time;
    if (elapsed < LOOP_PERIOD_MS) {
        delay(LOOP_PERIOD_MS - elapsed);
    }
}

