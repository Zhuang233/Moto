#include "test.h"
#include "cmsis_os.h"
#include "pid.h"
#include "DJIMotoDriver.h"
#include "RcDriver.h"

#define TEST_PID_POS_P 1
#define TEST_PID_POS_I 0
#define TEST_PID_POS_D 0
#define TEST_PID_SPD_P 1
#define TEST_PID_SPD_I 0
#define TEST_PID_SPD_D 0


PidTD pid_test_spd;
PidTD pid_test_pos;
MotoStateTD* p_moto_state_test = &MotoState[0];
int current_test;


void test_pid_pos_init(){
	pidInit(&pid_test_pos, 2000, 10000, TEST_PID_POS_P, TEST_PID_POS_I, TEST_PID_POS_D);
	pidInit(&pid_test_spd, 2000, 10000, TEST_PID_SPD_P, TEST_PID_SPD_I, TEST_PID_SPD_D);
	MotoStateInit(p_moto_state_test);
	current_test = 0;
}

// 0号电机测试位置环
void test_pid_pos(){
		pid_calculate(&pid_test_pos, (float)p_moto_state_test->angle_desired , (float)p_moto_state_test->angle);
		p_moto_state_test->speed_desired = (int)pid_test_pos.outPID;
		pid_calculate(&pid_test_spd, p_moto_state_test->speed_desired , p_moto_state_test->speed_actual);
		current_test = (int)pid_test_spd.outPID;
		SetMotoCurrent(&hcan1, Ahead, current_test, 0, 0 ,0);
}

void test_rc_moto(){
	MotoState[0].angle_desired += RC_CtrlData.rc.ch2;
	if(MotoState[0].angle_desired > 1950000) MotoState[0].angle_desired = 1950000;
	if(MotoState[0].angle_desired < 0) MotoState[0].angle_desired = 0;
	osDelay(1);
}