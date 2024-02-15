#include "wd.h"
#include "main.h"

bool hy_reset = false;

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	// 横移微动
	if(GPIO_Pin == GPIO_PIN_9){
		// 检测到上升沿
		if(HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_9) == GPIO_PIN_SET){
			hy_reset = true;
		}
		// 检测到下降沿
		else if(HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_9) == GPIO_PIN_RESET){
			hy_reset = false;
		}
	}
}
