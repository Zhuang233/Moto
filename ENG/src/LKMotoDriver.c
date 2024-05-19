#include "LKMotoDriver.h"

typedef enum
{
	Read_Multi_Angle = 0x92,
	Broadcast_Spd_Rev = 0xA2,
	Broadcast_Current_Rev = 0xA1,
}LKControlByteTD;

union Multi_Angle{
	uint8_t data[7];
  int64_t value;
};

uint8_t rx_data_lk[8]; //FIFO接收缓存区
LKMotoStateTD LKMotoState[8];

void LKUpdateMotoState(LKMotoStateTD* state);
void ReadMultiAngle(LKMotoStateTD* state);

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



void LKSetMotoCurrent_single(LKMotorIdTD moto_id, int16_t current){
	CAN_TxHeaderTypeDef canTxHeader;
	uint32_t send_mail_box;
	uint8_t TX_Data[8];

	canTxHeader.StdId = moto_id;
	canTxHeader.ExtId = 0x00;
	canTxHeader.RTR = CAN_RTR_DATA;
	canTxHeader.IDE = CAN_ID_STD;
	canTxHeader.DLC = 8;
	
	TX_Data[0] = 0xA1;
	TX_Data[1] = 0x00;
	TX_Data[2] = 0x00;
	TX_Data[3] = 0x00;
	TX_Data[4] = *(uint8_t*)(&current);
	TX_Data[5] = *((uint8_t*)(&current)+1);
	TX_Data[6] = 0x00;
	TX_Data[7] = 0x00;
	
	/* send can command */
	HAL_CAN_AddTxMessage(&LK_MOTO_CAN_HANDEL, &canTxHeader, TX_Data, &send_mail_box);
}


void LKSaveMotoMsg(CAN_HandleTypeDef *hcan, uint32_t RxFifo){
	CAN_RxHeaderTypeDef Rx_Msg;

  HAL_CAN_GetRxMessage(hcan, RxFifo, &Rx_Msg, rx_data_lk);					// 接收can1 fifo0邮箱的数据帧。不接收会导致fifo邮箱一直爆满，无限进入接收中断，卡死所有任务。

  switch (Rx_Msg.StdId)			//can1 motor message decode
  {
		case LK_Motor1_ID:
		case LK_Motor2_ID:
		case LK_Motor3_ID:
		case LK_Motor4_ID:
		case LK_Motor5_ID:
		case LK_Motor6_ID:
		case LK_Motor7_ID:
		case LK_Motor8_ID:
			{
				static uint8_t i = 0;
				//get motor id
				i = Rx_Msg.StdId - LK_Motor1_ID; // 组内编号
				LKUpdateMotoState(&LKMotoState[i]);
				break;
			}
    default:
				break;		
  }

}

void LKUpdateMotoState(LKMotoStateTD* state){
	
	switch(rx_data_lk[0])
	{
		case Read_Multi_Angle:
		{
			ReadMultiAngle(state);
		}
		break;
		case Broadcast_Spd_Rev:
		case Broadcast_Current_Rev:
		{
			BroadcastSpdRev(state);
		}
		break;
	default:   
			break;
	}
	// 读取多圈角度
	


}

void ReadMultiAngle(LKMotoStateTD* state)
{



}


void BroadcastSpdRev(LKMotoStateTD* state){
	state->temperature = rx_data_lk[1];
	state->current = rx_data_lk[2] + (rx_data_lk[3] << 8);
	state->speed = rx_data_lk[4] + (rx_data_lk[5] << 8);
	state->encoder = rx_data_lk[6] + (rx_data_lk[7] << 8);
	state->theta = (float)state->encoder * 360 / 65535;
}

//------------------------------------------------------------------
// 广播模式

// 设置电机电流
// 参数：can句柄，电机组别，电机1 2 3 4电流
void LKSetMotoCurrent(CAN_HandleTypeDef* hcan, int16_t C1, int16_t C2, int16_t C3, int16_t C4)
{
  uint8_t TX_Data[8];
  uint32_t send_mail_box;
	CAN_TxHeaderTypeDef Tx_Msg;
	
	// 电机标识符
	Tx_Msg.StdId=0x280;

	
	Tx_Msg.IDE=CAN_ID_STD;		  // 不使用扩展标识符
	Tx_Msg.RTR=CAN_RTR_DATA;		// 消息类型为数据帧，
	Tx_Msg.DLC=8; 							//一帧8字节
  
  TX_Data[0] = C1;
	TX_Data[1] = C1 >> 8;
	TX_Data[2] = C2;
	TX_Data[3] = C2 >> 8;
	TX_Data[4] = C3;
	TX_Data[5] = C3 >> 8;
	TX_Data[6] = C4;
	TX_Data[7] = C4 >> 8;
  
  HAL_CAN_AddTxMessage(hcan, &Tx_Msg, TX_Data, &send_mail_box);    //将数据储存进邮箱FIFOx
}




// 设置电机速度
// 参数：can句柄，电机组别，电机1 2 3 4速度
void LKSetMotoSpeed(CAN_HandleTypeDef* hcan, int16_t C1, int16_t C2, int16_t C3, int16_t C4)
{
  uint8_t TX_Data[8];
  uint32_t send_mail_box;
	CAN_TxHeaderTypeDef Tx_Msg;
	
	// 电机标识符
	Tx_Msg.StdId=0x281;

	
	Tx_Msg.IDE=CAN_ID_STD;		  // 不使用扩展标识符
	Tx_Msg.RTR=CAN_RTR_DATA;		// 消息类型为数据帧，
	Tx_Msg.DLC=8; 							//一帧8字节
  
  TX_Data[0] = C1;
	TX_Data[1] = C1 >> 8;
	TX_Data[2] = C2;
	TX_Data[3] = C2 >> 8;
	TX_Data[4] = C3;
	TX_Data[5] = C3 >> 8;
	TX_Data[6] = C4;
	TX_Data[7] = C4 >> 8;
  
  HAL_CAN_AddTxMessage(hcan, &Tx_Msg, TX_Data, &send_mail_box);    //将数据储存进邮箱FIFOx
}

// 设置电机位置
// 参数：can句柄，电机组别，电机1 2 3 4位置
void LKSetMotoPos(CAN_HandleTypeDef* hcan, int16_t C1, int16_t C2, int16_t C3, int16_t C4)
{
  uint8_t TX_Data[8];
  uint32_t send_mail_box;
	CAN_TxHeaderTypeDef Tx_Msg;
	
	// 电机标识符
	Tx_Msg.StdId=0x282;

	
	Tx_Msg.IDE=CAN_ID_STD;		  // 不使用扩展标识符
	Tx_Msg.RTR=CAN_RTR_DATA;		// 消息类型为数据帧，
	Tx_Msg.DLC=8; 							//一帧8字节
  
  TX_Data[0] = C1;
	TX_Data[1] = C1 >> 8;
	TX_Data[2] = C2;
	TX_Data[3] = C2 >> 8;
	TX_Data[4] = C3;
	TX_Data[5] = C3 >> 8;
	TX_Data[6] = C4;
	TX_Data[7] = C4 >> 8;
  
  HAL_CAN_AddTxMessage(hcan, &Tx_Msg, TX_Data, &send_mail_box);    //将数据储存进邮箱FIFOx
}


