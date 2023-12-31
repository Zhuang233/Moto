#ifndef __DJIMOTODRIVER_H__
#define __DJIMOTODRIVER_H__
#include "main.h"
#include "can.h"
#include "stdbool.h"

// 电机组别，Ahead电机id 1-4 Back5-8
typedef enum 
{
	Ahead,
	Back
}MotoGroupe;

// 电机ID
typedef enum
{
	CAN_Motor_ALL_ID = 0x200,
	CAN_Motor1_ID = 0x201,
	CAN_Motor2_ID = 0x202,
	CAN_Motor3_ID = 0x203,
	CAN_Motor4_ID = 0x204,
	CAN_Motor5_ID = 0x205,
	CAN_Motor6_ID = 0x206,
	CAN_Motor7_ID = 0x207,
	CAN_Motor8_ID = 0x208,
}CAN_Message_ID;


// 电机状态
typedef struct 
{
	int16_t		speed_actual;
	int16_t 	speed_last;
	int16_t 	speed_desired;
	int16_t 	real_current;
	int16_t 	given_current;
	int32_t  	angle_desired;
	uint16_t 	angle_actual;
	uint16_t	angle_vague;
	uint16_t 	angle_last;
	int32_t		angle;
	int16_t 	turns;
	int8_t		temperature;
	int16_t		original_position;
  bool      first_run;
	bool 		whe_use_pid;
//	TIME 		time;
}MotoStateTD;

extern MotoStateTD MotoState[16];

void SetMotoCurrent(CAN_HandleTypeDef* hcan, MotoGroupe group, int16_t C1, int16_t C2, int16_t C3, int16_t C4);
void CAN1_Set_AheadCur(int16_t C1, int16_t C2, int16_t C3, int16_t C4);
#endif
