#include "JointReset.h"
#include <stdbool.h>
#include "time.h"
#include "DJIMotoDriver.h"
#include "pid.h"
#include "cmsis_os.h"


PidTD pid_moto_pos[2];
PidTD pid_moto_spd[2];

// 前伸-----------------------------------------

#define QS_POS_P 1
#define QS_POS_I 0
#define QS_POS_D 0
#define QS_SPD_P 7
#define QS_SPD_I 0
#define QS_SPD_D 0

uint8_t qs_inited = 0;
TimeTD t_duzhuan;
float detect_time = 0;
int last_detect_angle = 0;
int detect_angle = 0;
uint8_t start_stage = 1;
float qs_reset_speed = -3000;
PidTD* pid_qs_spd = &pid_moto_spd[1];
PidTD* pid_qs_pos = &pid_moto_pos[1];
// 前伸-----------------------------------------

void duzhuan_TimeInit(TimeTD *time);
void qs_init();


void qs_init(){
	pidInit(pid_qs_pos, 2000, 10000, QS_POS_P, QS_POS_I, QS_POS_D);
	pidInit(pid_qs_spd, 2000, 10000, QS_SPD_P, QS_SPD_I, QS_SPD_D);
	MotoStateInit(&MotoState[1]);
}

// 前伸位置初始化 堵转检测
void reset_qs(){
	qs_init();
	duzhuan_TimeInit(&t_duzhuan);
	while(qs_inited == 0){
		// 判断堵转
		GetDt(&t_duzhuan,MILLISECOND);
		detect_time +=t_duzhuan.dt;
		if (detect_time > 500){
			detect_time = 0;
			last_detect_angle =  detect_angle;
			detect_angle = MotoState[1].angle;
			start_stage = 0;
		}
		
		// 不转了
		if((MotoState[1].angle - last_detect_angle < -8000) || start_stage){
				// 速度环
				pid_calculate_inc(pid_qs_spd, qs_reset_speed, MotoState[1].speed_actual);
				SetMotoCurrent(&hcan1, Ahead, 0, pid_qs_spd->outPID, 0, 0);
		}
		else{
			SetMotoCurrent(&hcan1, Ahead, 0, 0, 0, 0);
			osDelay(20);
			pidInit(pid_qs_pos, 2000, 3000, QS_POS_P, QS_POS_I, QS_POS_D);
			pidInit(pid_qs_spd, 2000, 10000, QS_SPD_P, QS_SPD_I, QS_SPD_D);
			MotoStateInit(&MotoState[1]);
			qs_inited = 1;
		}
		osDelay(1);
	}
}


void duzhuan_TimeInit(TimeTD *time){
	TimeInit(time);
}