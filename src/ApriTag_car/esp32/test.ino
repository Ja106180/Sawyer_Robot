/*
 * ESP32 IMU角度输出 - 互补滤波版本（平衡小车专用）
 * 
 * 功能：
 * 1. 上电零偏校准（200样本，约2秒）
 * 2. 互补滤波融合加速度计和陀螺仪（alpha=0.98）
 * 3. 运行时零偏自适应（静止时轻微更新）
 * 4. 100Hz输出频率（每10ms一次）
 * 
 * 坐标系定义：
 * - X轴正方向 = 左侧
 * - Y轴正方向 = 后方
 * - Z轴正方向 = 向下
 * 
 * 角度定义：
 * - Pitch（俯仰角）：绕X轴旋转，前后倾斜（平衡控制主要用这个）
 * - Yaw（航向角）：绕Z轴旋转，水平旋转
 */

#include <Wire.h>

#define ICM42688_ADDR_0 0x68
#define ICM42688_ADDR_1 0x69
#define ICM42688_ADDR ICM42688_ADDR_1
#define I2C_SDA 2
#define I2C_SCL 1

// ========== 可调参数 ==========
const int CALIB_SAMPLES = 200;              // 上电校准样本数（约2秒）
const float COMPLEMENTARY_ALPHA = 0.98f;    // 互补滤波系数（98%陀螺仪，2%加速度计）
const float GYRO_STATIC_THRESH = 0.01f;     // 陀螺仪静止阈值（rad/s）
const float BIAS_ADAPT_ALPHA = 0.001f;     // 零偏自适应系数（缓慢更新）
const int LOOP_PERIOD_MS = 10;              // 主循环周期（10ms = 100Hz）
// ===============================

uint8_t icm42688_addr = 0;

// 角度变量（弧度）
float pitch_ = 0.0f, yaw_ = 0.0f;

// 陀螺仪零偏
float gyro_bias_x_ = 0.0f, gyro_bias_y_ = 0.0f, gyro_bias_z_ = 0.0f;

// 校准状态
bool calibrated_ = false;
int calib_count_ = 0;
float gyro_sum_x_ = 0.0f, gyro_sum_y_ = 0.0f, gyro_sum_z_ = 0.0f;

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

void setup() {
    Serial.begin(115200);
    delay(1000);
    
    Wire.begin(I2C_SDA, I2C_SCL);
    Wire.setClock(100000);
    delay(500);
    
    if (!initICM42688()) {
        Serial.println("ERROR: IMU initialization failed!");
        while(1) delay(1000);
    }
    
    Serial.println("IMU initialized.");
    Serial.print("Calibrating gyro bias (");
    Serial.print(CALIB_SAMPLES);
    Serial.println(" samples)...");
    Serial.println("Please keep the car still...");
}

void loop() {
    if (icm42688_addr == 0) {
        static unsigned long last_reinit_time = 0;
        if (millis() - last_reinit_time > 5000) {
            initICM42688();
            last_reinit_time = millis();
        }
        delay(100);
        return;
    }
    
    uint8_t data[14];
    if (!readICMRegisters(0x1D, data, 14)) {
        delay(10);
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
    const float ACCEL_SCALE = (16.0f / 32768.0f) * 9.80665f;
    float accel_x = accel_raw[0] * ACCEL_SCALE;
    float accel_y = accel_raw[1] * ACCEL_SCALE;
    float accel_z = accel_raw[2] * ACCEL_SCALE;
    
    const float GYRO_SCALE = (2000.0f / 32768.0f) * (PI / 180.0f);
    float gyro_x = gyro_raw[0] * GYRO_SCALE;
    float gyro_y = gyro_raw[1] * GYRO_SCALE;
    float gyro_z = gyro_raw[2] * GYRO_SCALE;
    
    unsigned long current_time = millis();
    static unsigned long last_time = 0;
    float dt = (current_time - last_time) / 1000.0f;
    if (dt <= 0 || dt > 0.1) dt = LOOP_PERIOD_MS / 1000.0f;
    last_time = current_time;
    
    // ========== 上电校准阶段 ==========
    if (!calibrated_) {
        gyro_sum_x_ += gyro_x;
        gyro_sum_y_ += gyro_y;
        gyro_sum_z_ += gyro_z;
        calib_count_++;
        
        if (calib_count_ % 50 == 0) {
            Serial.print("Calibrating... ");
            Serial.print(calib_count_);
            Serial.print("/");
            Serial.println(CALIB_SAMPLES);
        }
        
        if (calib_count_ >= CALIB_SAMPLES) {
            // 计算陀螺仪零偏（平均值）
            gyro_bias_x_ = gyro_sum_x_ / calib_count_;
            gyro_bias_y_ = gyro_sum_y_ / calib_count_;
            gyro_bias_z_ = gyro_sum_z_ / calib_count_;
            
            // 用加速度计初始化初始角度
            float accel_magnitude = sqrt(accel_x*accel_x + accel_y*accel_y + accel_z*accel_z);
            if (accel_magnitude > 7.0f && accel_magnitude < 12.0f && accel_z > 0) {
                pitch_ = atan2(accel_y, accel_z);
            } else {
                pitch_ = 0.0f;
            }
            yaw_ = 0.0f;
            
            calibrated_ = true;
            Serial.println();
            Serial.println("=== Calibration Complete ===");
            Serial.print("Gyro bias X: ");
            Serial.print(gyro_bias_x_ * 180.0f / PI, 4);
            Serial.println(" deg/s");
            Serial.print("Gyro bias Y: ");
            Serial.print(gyro_bias_y_ * 180.0f / PI, 4);
            Serial.println(" deg/s");
            Serial.print("Gyro bias Z: ");
            Serial.print(gyro_bias_z_ * 180.0f / PI, 4);
            Serial.println(" deg/s");
            Serial.print("Initial Pitch: ");
            Serial.print(pitch_ * 180.0f / PI, 2);
            Serial.println(" deg");
            Serial.println();
            Serial.println("Starting angle output (100Hz)...");
            Serial.println("Format: Pitch Yaw (degrees)");
            Serial.println();
        } else {
            delay(LOOP_PERIOD_MS);
            return;
        }
    }
    
    // ========== 正常运行阶段 ==========
    // 补偿陀螺仪零偏
    float gyro_x_corrected = gyro_x - gyro_bias_x_;
    float gyro_y_corrected = gyro_y - gyro_bias_y_;
    float gyro_z_corrected = gyro_z - gyro_bias_z_;
    
    // 运行时零偏自适应（仅在静止时更新）
    if (fabs(gyro_x_corrected) < GYRO_STATIC_THRESH) {
        gyro_bias_x_ = (1.0f - BIAS_ADAPT_ALPHA) * gyro_bias_x_ + BIAS_ADAPT_ALPHA * gyro_x;
    }
    if (fabs(gyro_y_corrected) < GYRO_STATIC_THRESH) {
        gyro_bias_y_ = (1.0f - BIAS_ADAPT_ALPHA) * gyro_bias_y_ + BIAS_ADAPT_ALPHA * gyro_y;
    }
    if (fabs(gyro_z_corrected) < GYRO_STATIC_THRESH) {
        gyro_bias_z_ = (1.0f - BIAS_ADAPT_ALPHA) * gyro_bias_z_ + BIAS_ADAPT_ALPHA * gyro_z;
    }
    
    // 互补滤波更新Pitch
    // 计算加速度计幅值，用于检查数据有效性
    float accel_magnitude = sqrt(accel_x*accel_x + accel_y*accel_y + accel_z*accel_z);
    bool accel_valid = (accel_magnitude > 7.0f && accel_magnitude < 12.0f) && (accel_z > 0);
    
    if (accel_valid) {
        // 从加速度计计算角度
        float accel_pitch = atan2(accel_y, accel_z);
        
        // 互补滤波：98%陀螺仪 + 2%加速度计
        // Pitch: 绕X轴旋转，使用gyro_x
        pitch_ = COMPLEMENTARY_ALPHA * (pitch_ + gyro_x_corrected * dt) + (1.0f - COMPLEMENTARY_ALPHA) * accel_pitch;
    } else {
        // 如果加速度计数据无效，只使用陀螺仪积分
        pitch_ = pitch_ + gyro_x_corrected * dt;
    }
    
    // Yaw角只用陀螺仪积分（没有磁力计）
    yaw_ += gyro_z_corrected * dt;
    
    // 限制yaw在[-π, π]范围
    if (yaw_ > PI) yaw_ -= 2.0f * PI;
    if (yaw_ < -PI) yaw_ += 2.0f * PI;
    
    // 输出角度（度）
    float pitch_deg = pitch_ * 180.0f / PI;
    float yaw_deg = yaw_ * 180.0f / PI;
    
        Serial.print(pitch_deg, 2);
    Serial.print(" ");
    Serial.println(yaw_deg, 2);
        
    // 精确控制循环周期为10ms（100Hz）
    unsigned long elapsed = millis() - current_time;
    if (elapsed < LOOP_PERIOD_MS) {
        delay(LOOP_PERIOD_MS - elapsed);
    }
}
