#ifndef __UART_ZBW_H__
#define __UART_ZBW_H__
#include "main.h"

#define FRAME_HEAD 0x55
#define FRAME_TAIL 0xaa

#define USART1_RX_BUFFER_SIZE 128
#define SYNC_TO_A_SIZE (sizeof(BackDataUnion))

typedef struct __attribute__((packed)){
	uint8_t head;
	int32_t qs_pos;
	int32_t hy_pos;
	uint16_t theta1;
	uint16_t theta2;
	uint16_t theta3;
	uint8_t resetable;
	uint8_t power_less_flag;
	uint8_t tail;
}FiveJointCtrlDataTD;

typedef struct __attribute__((packed)){
	uint8_t head;
	int32_t qs_pos_read;
	int32_t hy_pos_read;
	uint16_t theta1_read;
	uint16_t theta2_read;
	uint16_t theta3_read;
	uint8_t reset_state;
	uint8_t tail;
}FiveJointBackDataTD;

typedef union{
	FiveJointCtrlDataTD data;
	uint8_t bytes[sizeof(FiveJointCtrlDataTD)];
}DataUnion;

typedef union{
	FiveJointBackDataTD data;
	uint8_t bytes[sizeof(FiveJointBackDataTD)];
}BackDataUnion;

extern DataUnion sync_data_from_a;
extern BackDataUnion sync_data_to_a;

void usart_dma_init(void);
void decode_ctrl_data(void);
void sync_data_to_a_init(void);
void data_sync_uart(void);
#endif
