#ifndef __ROBOARM_H__
#define __ROBOARM_H__

////pitch
//#define ARM_ANGLE_MAX_1 45077
//#define ARM_ANGLE_MIN_1 12650
//#define ARM_ANGLE_STD_1 28492

////roll
//#define ARM_ANGLE_MAX_2 59069
//#define ARM_ANGLE_MIN_2 0
//#define ARM_ANGLE_STD_2 25823

////yall
//#define ARM_ANGLE_MAX_3 40000
//#define ARM_ANGLE_MIN_3 10000
//#define ARM_ANGLE_STD_3 24370

//pitch
#define ARM_ANGLE_MAX_1 32767
#define ARM_ANGLE_MIN_1 1
#define ARM_ANGLE_STD_1 1

//roll
#define ARM_ANGLE_MAX_2 62600
#define ARM_ANGLE_MIN_2 1
#define ARM_ANGLE_STD_2 33500

//yall
#define ARM_ANGLE_MAX_3 32300
#define ARM_ANGLE_MIN_3 1
#define ARM_ANGLE_STD_3 16000

#define ARM_ANGLE_MAX_QS 389000
#define ARM_ANGLE_MAX_HY 720000
#define ARM_ANGLE_CENTER_HY -194951

void Update_RoboArm_Pos(void);
void RoboArm_Pid_Init(void);
void RoboArm_RC_Ctrl(void);
void RoboArm_Pos_Init(void);
void RoboArm_UART_Ctrl(void);

#endif
