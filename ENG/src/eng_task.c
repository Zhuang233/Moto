#include "eng_task.h"


//#include "FreeRTOS.h"
//#include "task.h"
//#include "main.h"
#include "cmsis_os.h"

#include "can.h"
#include "DJIMotoDriver.h"

// 电机任务函数
void Mototask(void const * argument)
{
  for(;;)
  {
		SetMotoCurrent(&hcan1, Ahead, 3000, 3000, 3000, 3000);
    osDelay(1);
  }
}