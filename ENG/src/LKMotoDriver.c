#include "LKMotoDriver.h"



void LKSetSpeed(LKMotorIdTD moto_id, uint32_t speed){
		CAN_TxHeaderTypeDef canTxHeader;
		uint32_t send_mail_box;
		uint8_t TX_Data[8];
	
		canTxHeader.StdId = moto_id;
		canTxHeader.ExtId = 0x00;
		canTxHeader.RTR = CAN_RTR_DATA;
		canTxHeader.IDE = CAN_ID_STD;
		canTxHeader.DLC = 8;
		
		TX_Data[0] = 0xA2;
		TX_Data[1] = 0x00;
		TX_Data[2] = 0x00;
		TX_Data[3] = 0x00;
		TX_Data[4] = *(uint8_t*)(&speed);
		TX_Data[5] = *((uint8_t*)(&speed)+1);
		TX_Data[6] = *((uint8_t*)(&speed)+2);
		TX_Data[7] = *((uint8_t*)(&speed)+3);
		
		/* send can command */
		HAL_CAN_AddTxMessage(&LK_MOTO_CAN_HANDEL, &canTxHeader, TX_Data, &send_mail_box);
}


