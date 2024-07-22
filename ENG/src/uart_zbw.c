#include "uart_zbw.h"
#include "usart.h"
#include <stdbool.h>
#include "string.h"
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
uint8_t		USART1_Rx_Buffer[USART1_RX_BUFFER_SIZE] = {0};

void decode_ctrl_data(){
	if((USART1_Rx_Buffer[0] == 0x55) && (USART1_Rx_Buffer[17] == 0xAA)){
		memcpy(&sync_data_from_a, &USART1_Rx_Buffer, sizeof(sync_data_from_a));
	}
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

