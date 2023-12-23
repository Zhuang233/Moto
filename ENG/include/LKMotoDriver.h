#ifndef __LKMOTODRIVER_H__
#define __LKMOTODRIVER_H__
#include "main.h"
#include "can.h"

#define LK_MOTO_CAN_HANDEL hcan1

#define DEVICE_STD_ID						(0x140)
#define DEVICE_STD_BOARDCAST_ID	(0x280)


typedef enum
{
	LK_Motor1_ID = 0x141,
	LK_Motor2_ID = 0x142,
	LK_Motor3_ID = 0x143,
	LK_Motor4_ID = 0x144,
	LK_Motor5_ID = 0x145,
	LK_Motor6_ID = 0x146,
	LK_Motor7_ID = 0x147,
	LK_Motor8_ID = 0x148
}LKMotorIdTD;

void LKSetSpeed(LKMotorIdTD moto_id, uint32_t speed);

#endif

