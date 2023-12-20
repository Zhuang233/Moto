#include "eng_task.h"


//#include "FreeRTOS.h"
//#include "task.h"
//#include "main.h"
#include "cmsis_os.h"

#include "can.h"
#include "DJIMotoDriver.h"

int zbwtest = 0;

// 电机任务函数
void Mototask(void const * argument)
{
  for(;;)
  {
		//SetMotoCurrent(&hcan1, Ahead, 3000, 3000, 3000, 3000);
		CAN1_Set_AheadCur(300, 200, 300, 300);
    osDelay(10);
		zbwtest++;
		
  }
}


void led_task(void const * argument)
{
  /* USER CODE BEGIN led_task */
  /* Infinite loop */
  for(;;)
  {
     HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_RESET);
		 osDelay(500);
		 HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_SET);
		 osDelay(500);
		 HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_RESET);
		 osDelay(500);
  }
  /* USER CODE END led_task */
}
