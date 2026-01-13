#include "motor.h"

/* 简单比例映射：线速度控制前进/后退，角速度控制差速 */
static void pwm_set(TIM_TypeDef* TIMx, uint8_t channel, uint16_t value)
{
    if (value > 999) value = 999; /* 1k 为上限 */
    switch (channel) {
    case 1: TIMx->CCR1 = value; break;
    case 2: TIMx->CCR2 = value; break;
    case 3: TIMx->CCR3 = value; break;
    case 4: TIMx->CCR4 = value; break;
    default: break;
    }
}

void motor_init(void)
{
    /* 时钟 */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

    /* DIR 引脚 */
    GPIO_InitTypeDef gpio;
    gpio.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
    gpio.GPIO_Mode = GPIO_Mode_Out_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &gpio);
    GPIO_ResetBits(GPIOB, GPIO_Pin_0 | GPIO_Pin_1);

    /* PWM 引脚 PA6/PA7 复用推挽 */
    gpio.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
    gpio.GPIO_Mode = GPIO_Mode_AF_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &gpio);

    /* TIM3: 1 kHz PWM (72MHz / 72 / 1000) */
    TIM_TimeBaseInitTypeDef tim;
    tim.TIM_Period = 999;
    tim.TIM_Prescaler = 71;
    tim.TIM_ClockDivision = TIM_CKD_DIV1;
    tim.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM3, &tim);

    TIM_OCInitTypeDef oc;
    oc.TIM_OCMode = TIM_OCMode_PWM1;
    oc.TIM_OutputState = TIM_OutputState_Enable;
    oc.TIM_Pulse = 0;
    oc.TIM_OCPolarity = TIM_OCPolarity_High;

    TIM_OC1Init(TIM3, &oc);
    TIM_OC2Init(TIM3, &oc);
    TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);
    TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);

    TIM_ARRPreloadConfig(TIM3, ENABLE);
    TIM_Cmd(TIM3, ENABLE);
}

void motor_stop(void)
{
    pwm_set(TIM3, 1, 0);
    pwm_set(TIM3, 2, 0);
    GPIO_ResetBits(GPIOB, GPIO_Pin_0 | GPIO_Pin_1);
}

void motor_set_cmd(float linear_x, float angular_z)
{
    /* 简单比例与限幅，可按需要调整 */
    const float max_lin = 0.5f;   /* m/s */
    const float max_ang = 1.0f;   /* rad/s */
    if (linear_x > max_lin) linear_x = max_lin;
    if (linear_x < -max_lin) linear_x = -max_lin;
    if (angular_z > max_ang) angular_z = max_ang;
    if (angular_z < -max_ang) angular_z = -max_ang;

    /* 差速合成 */
    float left = linear_x - angular_z * 0.3f;  /* 0.3 可视为半轮距标度 */
    float right = linear_x + angular_z * 0.3f;

    /* 归一化到 [-1,1] */
    float abs_l = (left >= 0) ? left : -left;
    float abs_r = (right >= 0) ? right : -right;
    float scale = 1.0f;
    float max_v = (abs_l > abs_r) ? abs_l : abs_r;
    if (max_v > 1.0f) scale = 1.0f / max_v;
    left *= scale;
    right *= scale;

    /* 方向与占空比 */
    uint16_t duty_l = (uint16_t)(abs_l * 999);
    uint16_t duty_r = (uint16_t)(abs_r * 999);

    /* 左轮方向：若需反向，可在此处取反 GPIO_SetBits/ResetBits */
    if (left >= 0) {
        GPIO_ResetBits(GPIOB, GPIO_Pin_0);
    } else {
        GPIO_SetBits(GPIOB, GPIO_Pin_0);
    }
    if (right >= 0) {
        GPIO_ResetBits(GPIOB, GPIO_Pin_1);
    } else {
        GPIO_SetBits(GPIOB, GPIO_Pin_1);
    }

    pwm_set(TIM3, 1, duty_l);
    pwm_set(TIM3, 2, duty_r);
}

