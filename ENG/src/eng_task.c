#include "eng_task.h"


//#include "FreeRTOS.h"
//#include "task.h"
//#include "main.h"
#include "cmsis_os.h"

#include "can.h"
#include "DJIMotoDriver.h"
#include "LKMotoDriver.h"
#include "pid.h"
#include "trace.h"

int zbwtest = 0;


// 单元测试/回归测试用
void TestTask(void const * argument)
{
	PidTD pidtest;
	pidInit(&pidtest, 2000, 5000, 2.5, 0.00001, 0);
  for(;;)
  {	
		/*---------------------------------------------------


		// 测试电机电流直接控制
		SetMotoCurrent(&hcan1, Ahead, 300, 900, 2700, 8100);
		
		// 速度环测试
		pid_calculate(&pidtest, 500, MotoState[0].speed_actual);
		trace_pid(&pidtest);
		SetMotoCurrent(&hcan1, Ahead, pidtest.outPID, 0, 0, 0);
		osDelay(1);
		
		-----------------------------------------------------*/

		// 瓴控电机速度闭环测试,单位：度/秒（未减速前）
		LKSetSpeed(LK_Motor1_ID, 90000);
		
  }
}


// 电机任务函数
void Mototask(void const * argument)
{
  for(;;)
  {
    osDelay(1);
  }
}


void LedTask(void const * argument)
{
  for(;;)
  {
     HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_RESET);
		 osDelay(500);
		 HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_SET);
		 osDelay(500);
  }
}


