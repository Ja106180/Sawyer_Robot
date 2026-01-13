#include "servo.h"

/* 50Hz, 20ms 周期，1ms-2ms 脉宽 */
void servo_init(void)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

    GPIO_InitTypeDef gpio;
    gpio.GPIO_Pin = GPIO_Pin_1;
    gpio.GPIO_Mode = GPIO_Mode_AF_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &gpio);

    /* TIM2: 50Hz -> 20ms, 使用 72MHz/(72*20000)=50Hz */
    TIM_TimeBaseInitTypeDef tim;
    tim.TIM_Period = 19999;
    tim.TIM_Prescaler = 71;
    tim.TIM_ClockDivision = TIM_CKD_DIV1;
    tim.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM2, &tim);

    TIM_OCInitTypeDef oc;
    oc.TIM_OCMode = TIM_OCMode_PWM1;
    oc.TIM_OutputState = TIM_OutputState_Enable;
    oc.TIM_Pulse = 1500; /* 1.5ms 中位 */
    oc.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OC2Init(TIM2, &oc);
    TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Enable);

    TIM_ARRPreloadConfig(TIM2, ENABLE);
    TIM_Cmd(TIM2, ENABLE);
}

void servo_set_angle(float deg)
{
    /* 0~180 映射到 1.0~2.0 ms */
    if (deg < 0.0f) deg = 0.0f;
    if (deg > 180.0f) deg = 180.0f;
    float pulse_ms = 1.0f + (deg / 180.0f); /* 1.0~2.0ms */
    uint16_t ccr = (uint16_t)(pulse_ms / 20.0f * 20000.0f); /* 周期 20ms，ARR=19999 */
    TIM2->CCR2 = ccr;
}

