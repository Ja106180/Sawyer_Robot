#pragma once
#include "stm32f10x.h"

/* 引脚映射
 * 左轮 PWM: TIM3_CH1 PA6
 * 右轮 PWM: TIM3_CH2 PA7
 * 左轮 DIR: PB0
 * 右轮 DIR: PB1
 */

void motor_init(void);
void motor_stop(void);
void motor_set_cmd(float linear_x, float angular_z);

