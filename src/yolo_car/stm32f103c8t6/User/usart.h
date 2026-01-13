#pragma once
#include "stm32f10x.h"
#include <stdint.h>

void usart1_init(uint32_t baud);
int usart1_read(void);          /* 返回 -1 表示无数据 */
void usart1_write(uint8_t c);
uint32_t millis(void);
void delay_ms(uint32_t ms);

/* 供 rosserial 使用的硬件封装 */
class STM32Hardware
{
public:
    void init() { /* 已在 main 中初始化，无需重复 */ }
    int read() { return usart1_read(); }
    void write(uint8_t* data, int length)
    {
        for (int i = 0; i < length; ++i) usart1_write(data[i]);
    }
    unsigned long time() { return millis(); }
};

