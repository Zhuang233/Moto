#include "can_zbw.h"
#include "main.h"
#include "can.h"




// 初始化can过滤器
void can_filter_init(void)
{

  CAN_FilterTypeDef CAN_FilterInitStructure;
	
  CAN_FilterInitStructure.FilterActivation = ENABLE;
  CAN_FilterInitStructure.FilterMode = CAN_FILTERMODE_IDMASK;
  CAN_FilterInitStructure.FilterScale = CAN_FILTERSCALE_32BIT;
  CAN_FilterInitStructure.FilterIdHigh = 0x0000;
  CAN_FilterInitStructure.FilterIdLow = 0x0000;
  CAN_FilterInitStructure.FilterMaskIdHigh = 0x0000;
  CAN_FilterInitStructure.FilterMaskIdLow = 0x0000;
  CAN_FilterInitStructure.FilterBank = 0;
  CAN_FilterInitStructure.FilterFIFOAssignment = CAN_RX_FIFO0; // 使用FIFO0接收邮箱
	
  HAL_CAN_ConfigFilter(&hcan1, &CAN_FilterInitStructure);
  HAL_CAN_Start(&hcan1);
  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);


  CAN_FilterInitStructure.SlaveStartFilterBank = 14;
  CAN_FilterInitStructure.FilterBank = 14;
  CAN_FilterInitStructure.FilterFIFOAssignment = CAN_RX_FIFO1; // 使用FIFO1接收邮箱
  HAL_CAN_ConfigFilter(&hcan2, &CAN_FilterInitStructure);
  HAL_CAN_Start(&hcan2);
  HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO1_MSG_PENDING);

}