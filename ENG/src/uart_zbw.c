#include "uart_zbw.h"
#include "usart.h"
#include <stdbool.h>
#include "string.h"
#include "LKMotoDriver.h"
#include "DJIMotoDriver.h"
#include "JointReset.h"
#include "stdbool.h"

// 同步数据
/*
瓴控前三轴位置uint16_t
横移位置int32_t 
前伸位置int32_t 
*/


// AC板通信协议
/*
帧头 0x55
帧尾 0xaa
*/

DataUnion sync_data_from_a;
BackDataUnion sync_data_to_a;
uint8_t		USART1_Rx_Buffer[USART1_RX_BUFFER_SIZE] = {0};
bool first_reset_qs_flag = true;

void decode_ctrl_data(){
	if((USART1_Rx_Buffer[0] == 0x55) && (USART1_Rx_Buffer[18] == 0xAA)){
		memcpy(&sync_data_from_a, &USART1_Rx_Buffer, sizeof(sync_data_from_a));
		
		// 处理前伸手动重置,复位信号跳变为1，qs重新复位一次
		if(sync_data_from_a.data.reset_qs_flag == 1 && first_reset_qs_flag){
			first_reset_qs_flag = false;
			qs_inited = false;
		}
		if(sync_data_from_a.data.reset_qs_flag == 0){
			first_reset_qs_flag = true;
		}
	}
}

void sync_data_to_a_init(){
	sync_data_to_a.data.hy_pos_read = 0;
	sync_data_to_a.data.qs_pos_read = 0;
	sync_data_to_a.data.theta1_read = 0;
	sync_data_to_a.data.theta2_read = 0;
	sync_data_to_a.data.theta3_read = 0;
	sync_data_to_a.data.reset_state = 0;
}

void data_sync_uart(){
	sync_data_to_a.data.theta1_read = LKMotoState[0].encoder;
	sync_data_to_a.data.theta2_read = LKMotoState[1].encoder;
	sync_data_to_a.data.theta3_read = LKMotoState[2].encoder;
	sync_data_to_a.data.hy_pos_read = MotoState[0].angle;
	sync_data_to_a.data.qs_pos_read = MotoState[1].angle;
	sync_data_to_a.data.reset_state = sync_data_to_a.data.reset_state | (qs_inited << 0);
	HAL_UART_Transmit_DMA(&huart1, sync_data_to_a.bytes, SYNC_TO_A_SIZE);
}

int uart_receive_dma_no_it(UART_HandleTypeDef* huart, uint8_t* pData, uint32_t Size)
{
  uint32_t tmp1 = 0;

  tmp1 = huart->RxState;
	
	if (tmp1 == HAL_UART_STATE_READY)
	{
		if ((pData == NULL) || (Size == 0))
		{
			return HAL_ERROR;
		}

		huart->pRxBuffPtr = pData;
		huart->RxXferSize = Size;
		huart->ErrorCode  = HAL_UART_ERROR_NONE;

		/* Enable the DMA Stream */
		HAL_DMA_Start(huart->hdmarx, (uint32_t)&huart->Instance->DR, (uint32_t)pData, Size);

		/* 
		 * Enable the DMA transfer for the receiver request by setting the DMAR bit
		 * in the UART CR3 register 
		 */
		SET_BIT(huart->Instance->CR3, USART_CR3_DMAR);

		return HAL_OK;
	}
	else
	{
		return HAL_BUSY;
	}
}




void usart_dma_init(void)
{
	__HAL_UART_CLEAR_IDLEFLAG(&huart1);
	__HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
  
	uart_receive_dma_no_it(&huart1, USART1_Rx_Buffer, USART1_RX_BUFFER_SIZE);
}
// dma

