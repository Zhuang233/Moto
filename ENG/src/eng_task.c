#include "eng_task.h"
#include "main.h"
#include "usart.h"
#include "cmsis_os.h"
#include "can.h"
#include "DJIMotoDriver.h"
#include "LKMotoDriver.h"
#include "pid.h"
#include "trace.h"
#include "test.h"
#include "chassis.h"
#include "RoboArm.h"
#include "uart_zbw.h"
#include "JointReset.h"
#include "RoboArm.h"

// 电机任务函数
void MotoTask(void const * argument)
{	
	RoboArm_Pos_Init();
	RoboArm_Pid_Init();
  osDelay(3000); //前伸复位得等3s 2006上电没那么快工作
	reset_qs();
  for(;;)
  {
    RoboArm_UART_Ctrl();
		Update_RoboArm_Pos();
  }
}

void wait_lift_allow(){
	while(!sync_data_from_a.data.resetable){
		osDelay(1);
	}
}

void LedTask(void const * argument)
{
	wait_lift_allow();
	reset_hy();
  for(;;)
  {
     HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_RESET);
		 osDelay(500);
		 HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_SET);
		 osDelay(500);
  }
}

void DataSyncAnCTask(void const * argument){

  for(;;)
  {
    osDelay(1); 
  }
}


