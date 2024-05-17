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




typedef struct{
	// TODO: 完善电机状态
	int8_t temperature;	//单位：度
	int16_t current;			//-2048~2048 -> -33A~33A
	int16_t speed;				//单位：度/秒
	int16_t speed_desired; // 目标速度
	uint16_t encoder;			//编码器位置0~65535
	uint16_t angle_desired; 		// 目标角度
	uint16_t maxSpeed;			//速度绝对值限制
	float theta;					//绝对角度0-360

}LKMotoStateTD;

extern LKMotoStateTD LKMotoState[8];

void LKSetSpeed(LKMotorIdTD moto_id, uint32_t speed);
void LKSaveMotoMsg(CAN_HandleTypeDef *hcan, uint32_t RxFifo);

//以下为广播模式
void LKSetMotoCurrent(CAN_HandleTypeDef* hcan, int16_t C1, int16_t C2, int16_t C3, int16_t C4);
void LKSetMotoSpeed(CAN_HandleTypeDef* hcan, int16_t C1, int16_t C2, int16_t C3, int16_t C4);
void LKSetMotoPos(CAN_HandleTypeDef* hcan, int16_t C1, int16_t C2, int16_t C3, int16_t C4); // 电机有问题，没考虑减速比，只能转65.535度
void BroadcastSpdRev(LKMotoStateTD* state);

#endif

