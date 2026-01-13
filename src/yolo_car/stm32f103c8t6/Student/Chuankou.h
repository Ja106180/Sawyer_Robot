#ifndef CHUANKOU_H
#define CHUANKOU_H

extern uint8_t data;
extern volatile uint8_t rx_flag;
void Chuankou_Init(void);
void serial_Tx(int Tx_data);
void serial_Rx(void);

void serial_Tx_String(char*string);
void serial_Tx_Array(uint8_t* array,uint16_t length);
void serial_Tx_Num(uint32_t Num,uint8_t length);

#endif