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
#include "stdbool.h"

bool qs_had_auto_reset = false;

// 电机任务函数
void MotoTask(void const * argument)
{	
	RoboArm_Pos_Init();
	RoboArm_Pid_Init();
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
	osDelay(1000);
	roll_yaw_reseted = true;
	osDelay(2000); //前伸复位得等3s 2006上电没那么快工作
	reset_qs();
	last_roll_init();
	qs_had_auto_reset = true;
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
	sync_data_to_a_init();
	osDelay(500);
	sync_data_to_a.data.head = FRAME_HEAD;
	sync_data_to_a.data.tail = FRAME_TAIL;
  for(;;)
  {
		data_sync_uart();
    osDelay(10); 
  }
}

void qsTask(void const * argument){
  for(;;)
  {
		// 如果上电自动复位过一次，且发现前伸复位标志被修改为未初始化，说明a板要求前伸重新复位
		if(qs_had_auto_reset && !qs_inited){
			reset_qs();
		}
		osDelay(1);
  }
}
