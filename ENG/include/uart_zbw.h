#ifndef __UART_ZBW_H__
#define __UART_ZBW_H__


#include "main.h"

#define FRAME_HEAD 0x55
#define FRAME_TAIL 0xaa
#define FRAME_SIZE (sizeof(sync_data_from_a)+2) // 接收帧长度
#define SYNC_TO_A_SIZE 26// 发送数据长度
//#define SYNC_FROM_A_SIZE 8// 接收数据长度

#define FRAME_SIZE2 (sizeof(MyCtrlTD)+2) // 接收帧长度
	
typedef struct __attribute__((packed)){
	uint8_t key1;
	uint8_t key2;
	float vx;
	float vy;
	float vz;
	float roll;
	float pitch;
	float yaw;
	uint8_t zero[2];
}MyCtrlTD;

typedef union{
	MyCtrlTD data;
	uint8_t bytes[sizeof(MyCtrlTD)];
}MyCtrlUnion;

typedef struct {
	int32_t qs_pos;
	int32_t hy_pos;
	uint16_t theta1;
	uint16_t theta2;
	uint16_t theta3;
}FiveJointCtrlDataTD;

typedef union{
	FiveJointCtrlDataTD data;
	uint8_t bytes[sizeof(FiveJointCtrlDataTD)];
}DataUnion;

extern DataUnion sync_data_from_a;
extern uint8_t sync_data_to_a[SYNC_TO_A_SIZE];

void usart_init(void);
void data_sync_uart(void);
#endif
