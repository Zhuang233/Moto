#include "JointReset.h"
#include "time.h"
#include "DJIMotoDriver.h"
#include "pid.h"
#include "cmsis_os.h"
#include "wd.h"


PidTD pid_moto_pos[2];
PidTD pid_moto_spd[2];

int16_t dji_moto_current_to_send[2] = {0};

// 前伸-----------------------------------------

#define QS_POS_OUT_LIMIT 5000

#define QS_POS_P 0.5
#define QS_POS_I 0
#define QS_POS_D 0
#define QS_SPD_P 10
#define QS_SPD_I 1
#define QS_SPD_D 0


bool qs_inited = false;
TimeTD t_duzhuan;
float detect_time = 0;
int last_detect_angle = 0;
int detect_angle = 0;
uint8_t start_stage = 1;
float qs_reset_speed = -3000;
PidTD* pid_qs_spd = &pid_moto_spd[1];
PidTD* pid_qs_pos = &pid_moto_pos[1];
// 前伸-----------------------------------------

// 横移-----------------------------------------

#define HY_POS_P 0.5
#define HY_POS_I 0
#define HY_POS_D 0
#define HY_SPD_P 10
#define HY_SPD_I 0.01
#define HY_SPD_D 0

bool hy_inited = false;
PidTD* pid_hy_spd = &pid_moto_spd[0];
PidTD* pid_hy_pos = &pid_moto_pos[0];

// 横移-----------------------------------------


void duzhuan_TimeInit(TimeTD *time);
void qs_init();


void qs_init(){
	pidInit(pid_qs_pos, 2000, 10000, QS_POS_P, QS_POS_I, QS_POS_D);
	pidInit(pid_qs_spd, 3000, 10000, QS_SPD_P, QS_SPD_I, QS_SPD_D);
	MotoStateInit(&MotoState[1]);
}

void hy_init(){
	pidInit(pid_hy_pos, 2000, 10000, HY_POS_P, HY_POS_I, HY_POS_D);
	pidInit(pid_hy_spd, 3000, 10000, 20, HY_SPD_I, HY_SPD_D);
	MotoStateInit(&MotoState[0]);
}

// 前伸位置初始化 堵转检测
void reset_qs(){
	qs_init();
	duzhuan_TimeInit(&t_duzhuan);
	GetDt(&t_duzhuan,MILLISECOND);
	while(qs_inited == false){
		// 判断堵转
		GetDt(&t_duzhuan,MILLISECOND);
		detect_time +=t_duzhuan.dt;
		if (detect_time > 300){
			detect_time = 0;
			last_detect_angle =  detect_angle;
			detect_angle = MotoState[1].angle;
			start_stage = 0;
		}
		
		// 不转了
		if((MotoState[1].angle - last_detect_angle < -8000) || start_stage){
				// 速度环
				pid_calculate_inc(pid_qs_spd, qs_reset_speed, MotoState[1].speed_actual);
				dji_moto_current_to_send[1] = pid_qs_spd->outPID;
				SetMotoCurrent(&hcan1, Ahead, dji_moto_current_to_send[0], dji_moto_current_to_send[1], 0, 0);
		}
		else{
			dji_moto_current_to_send[1] = 0;
			SetMotoCurrent(&hcan1, Ahead, dji_moto_current_to_send[0], dji_moto_current_to_send[1], 0, 0);
			osDelay(20);
			pidInit(pid_qs_pos, 2000, QS_POS_OUT_LIMIT, QS_POS_P, QS_POS_I, QS_POS_D);
			pidInit(pid_qs_spd, 2000, 10000, QS_SPD_P, QS_SPD_I, QS_SPD_D);
			MotoStateInit(&MotoState[1]);
			qs_inited = true;
		}
		osDelay(1);
	}
}


void duzhuan_TimeInit(TimeTD *time){
	TimeInit(time);
}



// 横移位置初始化测试
void reset_hy(){
	float speed = 800;
	hy_init();
	while(hy_inited == false){
		if(hy_reset == false){
			// 速度环
			pid_calculate_inc(pid_hy_spd, speed, MotoState[0].speed_actual);
			dji_moto_current_to_send[0] = pid_hy_spd->outPID;
			SetMotoCurrent(&hcan1, Ahead, dji_moto_current_to_send[0], dji_moto_current_to_send[1], 0, 0);
		}
		else{
			dji_moto_current_to_send[0] = 0;
			SetMotoCurrent(&hcan1, Ahead, dji_moto_current_to_send[0], dji_moto_current_to_send[1], 0, 0);
			osDelay(20);
			pidInit(pid_hy_pos, 2000, 3000, HY_POS_P, HY_POS_I, HY_POS_D);
			pidInit(pid_hy_spd, 2000, 10000, HY_SPD_P, HY_SPD_I, HY_SPD_D);
			MotoStateInit(&MotoState[0]);
			hy_inited = true;
		}
		osDelay(1);	
	}
	MotoState[0].angle_desired = -200000;
}