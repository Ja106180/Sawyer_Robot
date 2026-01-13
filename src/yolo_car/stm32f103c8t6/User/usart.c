#include "usart.h"

static volatile uint32_t systick_ms = 0;

void SysTick_Handler(void)
{
    systick_ms++;
}

uint32_t millis(void)
{
    return systick_ms;
}

void delay_ms(uint32_t ms)
{
    uint32_t start = millis();
    while ((millis() - start) < ms) {;}
}

void usart1_init(uint32_t baud)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA, ENABLE);

    GPIO_InitTypeDef gpio;
    /* TX PA9 */
    gpio.GPIO_Pin = GPIO_Pin_9;
    gpio.GPIO_Mode = GPIO_Mode_AF_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &gpio);

    /* RX PA10 */
    gpio.GPIO_Pin = GPIO_Pin_10;
    gpio.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &gpio);

    USART_InitTypeDef us;
    us.USART_BaudRate = baud;
    us.USART_WordLength = USART_WordLength_8b;
    us.USART_StopBits = USART_StopBits_1;
    us.USART_Parity = USART_Parity_No;
    us.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    us.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART1, &us);
    USART_Cmd(USART1, ENABLE);
}

int usart1_read(void)
{
    if (USART_GetFlagStatus(USART1, USART_FLAG_RXNE) != RESET) {
        return USART_ReceiveData(USART1) & 0xFF;
    }
    return -1;
}

void usart1_write(uint8_t c)
{
    while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET) {;}
    USART_SendData(USART1, c);
}

