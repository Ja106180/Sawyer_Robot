#include "stm32f10x.h"                  // Device header

uint8_t data;
volatile uint8_t rx_flag = 0;

void Chuankou_Init(void){
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_Init(GPIOA,&GPIO_InitStructure);
	
	
	USART_InitTypeDef USART_InitStructure;
	USART_StructInit(&USART_InitStructure);
	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_Init(USART1,&USART_InitStructure);
	
	USART_ITConfig(USART1,USART_IT_RXNE,ENABLE);
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_Init(&NVIC_InitStructure);
	
	USART_Cmd(USART1,ENABLE);
}

//这是一个发送一个字节的函数
void serial_Tx(uint8_t Tx_data){
	int i=10000;
	USART_SendData(USART1,Tx_data);
	while(USART_GetFlagStatus(USART1,USART_FLAG_TXE)==RESET){
		if(--i == 0){
			break;
		}
	}
}

void serial_Rx(void){
	if(USART_GetITStatus(USART1,USART_IT_RXNE)==SET){
		data = USART_ReceiveData(USART1);
		rx_flag = 1;
		USART_ClearITPendingBit(USART1,USART_IT_RXNE);
	}
}

void serial_Tx_Array(uint8_t* array,uint16_t length){
	int i;
	for(i=0;i<length;i++){
		serial_Tx(array[i]);
	}
}

void serial_Tx_String(char*string){
	int i;
	for(i=0;string[i]!='\0';i++){
		serial_Tx(string[i]);
	}
}

int Num_pow(int x,int y){
	int result=1;
	while(y--){
		result*=x;	
	}
	return result;
}

void serial_Tx_Num(uint32_t Num,uint8_t length){
	int i;
	for(i=0;i<length;i++){
		serial_Tx(Num/Num_pow(10,length-i-1)%10+'0');
	}
}
