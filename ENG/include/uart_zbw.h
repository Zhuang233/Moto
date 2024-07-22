#ifndef __UART_ZBW_H__
#define __UART_ZBW_H__
#include "main.h"

#define FRAME_HEAD 0x55
#define FRAME_TAIL 0xaa

#define USART1_RX_BUFFER_SIZE 128

typedef struct __attribute__((packed)){
	uint8_t head;
	int32_t qs_pos;
	int32_t hy_pos;
	uint16_t theta1;
	uint16_t theta2;
	uint16_t theta3;
	uint8_t resetable;
	uint8_t zero;
	uint8_t tail;
}FiveJointCtrlDataTD;

typedef union{
	FiveJointCtrlDataTD data;
	uint8_t bytes[sizeof(FiveJointCtrlDataTD)];
}DataUnion;

extern DataUnion sync_data_from_a;

void usart_dma_init(void);
void decode_ctrl_data(void);
#endif
