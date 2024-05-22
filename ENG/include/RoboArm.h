#ifndef __ROBOARM_H__
#define __ROBOARM_H__

//pitch
#define ARM_ANGLE_MAX_1 45077
#define ARM_ANGLE_MIN_1 12650
#define ARM_ANGLE_STD_1 28492

//roll
#define ARM_ANGLE_MAX_2 42583
#define ARM_ANGLE_MIN_2 9858
#define ARM_ANGLE_STD_2 25823

//yall
#define ARM_ANGLE_MAX_3 40000
#define ARM_ANGLE_MIN_3 10000
#define ARM_ANGLE_STD_3 24370

void Update_RoboArm_Pos(void);
void RoboArm_Pid_Init(void);
void RoboArm_RC_Ctrl(void);
void RoboArm_Pos_Init(void);
void RoboArm_UART_Ctrl(void);

#endif