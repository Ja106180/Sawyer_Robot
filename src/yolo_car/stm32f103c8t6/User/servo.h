#pragma once
#include "stm32f10x.h"

/* 舵机：TIM2_CH2 PA1，50Hz */
void servo_init(void);
void servo_set_angle(float deg);

