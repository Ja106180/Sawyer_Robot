/**
 * 第3课：一维卡尔曼滤波示例
 * 
 * 场景：估计房间温度
 * - 真实温度：25°C（我们不知道，这是要估计的）
 * - 温度计测量：有噪声
 * - 模型：温度变化很慢
 */

#include <iostream>
#include <random>
#include <iomanip>
#include <cmath>

class SimpleKalmanFilter {
private:
    // 状态
    double state_;           // 估计的温度
    double uncertainty_;     // 不确定性（方差）
    
    // 噪声参数
    double process_noise_;   // 过程噪声（模型不完美）
    double measurement_noise_; // 测量噪声（传感器误差）
    
public:
    SimpleKalmanFilter(double initial_state, double initial_uncertainty,
                      double process_noise, double measurement_noise)
        : state_(initial_state),
          uncertainty_(initial_uncertainty),
          process_noise_(process_noise),
          measurement_noise_(measurement_noise) {
        std::cout << "=== 卡尔曼滤波器初始化 ===" << std::endl;
        std::cout << "初始估计: " << state_ << "°C" << std::endl;
        std::cout << "初始不确定性: " << uncertainty_ << std::endl;
        std::cout << "过程噪声: " << process_noise_ << std::endl;
        std::cout << "测量噪声: " << measurement_noise_ << std::endl;
        std::cout << std::endl;
    }
    
    /**
     * 预测步骤（Prediction）
     * 根据模型预测下一个状态
     */
    void predict() {
        // 模型：温度基本不变
        // state_ 保持不变
        
        // 不确定性增加（因为我们不完美）
        uncertainty_ = uncertainty_ + process_noise_;
    }
    
    /**
     * 更新步骤（Update）
     * 用测量值校正预测
     */
    void update(double measurement) {
        // 计算卡尔曼增益
        double kalman_gain = uncertainty_ / (uncertainty_ + measurement_noise_);
        
        // 融合预测和测量
        state_ = state_ + kalman_gain * (measurement - state_);
        
        // 更新不确定性（变小了）
        uncertainty_ = (1.0 - kalman_gain) * uncertainty_;
    }
    
    /**
     * 一步完整的预测-更新
     */
    void step(double measurement) {
        predict();
        update(measurement);
    }
    
    // Getter函数
    double getState() const { return state_; }
    double getUncertainty() const { return uncertainty_; }
};

/**
 * 生成带噪声的测量值（模拟真实传感器）
 */
double generateNoisyMeasurement(double true_value, double noise_std) {
    static std::random_device rd;
    static std::mt19937 gen(rd());
    std::normal_distribution<double> dist(0.0, noise_std);
    return true_value + dist(gen);
}

int main() {
    std::cout << "========================================" << std::endl;
    std::cout << "   一维卡尔曼滤波示例：温度估计" << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << std::endl;
    
    // 真实温度（我们不知道，这是要估计的）
    const double TRUE_TEMPERATURE = 25.0;
    
    // 初始化卡尔曼滤波器
    double initial_guess = 20.0;      // 初始猜测
    double initial_uncertainty = 10.0; // 初始不确定性（很大）
    double process_noise = 0.1;       // 过程噪声（温度变化很小）
    double measurement_noise = 2.0;   // 测量噪声（温度计误差）
    
    SimpleKalmanFilter kf(initial_guess, initial_uncertainty,
                         process_noise, measurement_noise);
    
    std::cout << "真实温度: " << TRUE_TEMPERATURE << "°C" << std::endl;
    std::cout << std::endl;
    std::cout << "开始滤波..." << std::endl;
    std::cout << std::endl;
    
    // 打印表头
    std::cout << std::fixed << std::setprecision(2);
    std::cout << "步骤 | 测量值 | 估计值 | 不确定性 | 误差" << std::endl;
    std::cout << "-----|--------|--------|----------|------" << std::endl;
    
    // 运行20步
    for (int i = 0; i < 20; i++) {
        // 生成带噪声的测量值
        double measurement = generateNoisyMeasurement(TRUE_TEMPERATURE, 
                                                      sqrt(measurement_noise));
        
        // 执行一步预测-更新
        kf.step(measurement);
        
        // 获取更新后的估计值
        double estimate_after = kf.getState();
        double uncertainty = kf.getUncertainty();
        double error = std::abs(estimate_after - TRUE_TEMPERATURE);
        
        // 打印结果
        std::cout << std::setw(4) << (i + 1) << " | "
                  << std::setw(6) << measurement << " | "
                  << std::setw(6) << estimate_after << " | "
                  << std::setw(8) << uncertainty << " | "
                  << std::setw(4) << error << std::endl;
    }
    
    std::cout << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << "最终估计: " << kf.getState() << "°C" << std::endl;
    std::cout << "真实值: " << TRUE_TEMPERATURE << "°C" << std::endl;
    std::cout << "最终误差: " << std::abs(kf.getState() - TRUE_TEMPERATURE) 
              << "°C" << std::endl;
    std::cout << "最终不确定性: " << kf.getUncertainty() << std::endl;
    std::cout << "========================================" << std::endl;
    
    return 0;
}

