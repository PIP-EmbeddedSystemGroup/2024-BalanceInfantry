/**
  ******************************************************************************
  * @file    uart_task.c
  * @author  ������ҵ��ѧ-������
  * @version V1.0
  * @date    2022/2/20
  * @brief   UART����ͷ�ļ�
  ******************************************************************************
  * @attention
  *
  *
  ******************************************************************************
  */
	
/* Includes ------------------------------------------------------------------*/
#include "Header.h"
u8 Uart_fast_transimit_Data[8] = {0};
u8 Data[8]={0};
CAN_TxHeaderTypeDef	TxMessage;

u8 tx_switch = 1;

void Com_Task(void const * argument)
{
	portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
	
	while(1)
	{
		vTaskDelayUntil(&xLastWakeTime,1);//1000HZ
		TxMessage.DLC = 8;//数据
		
		if(tx_switch ==1)
		{
			TxMessage.StdId=0x206;
			Data[0] = Shooting.Friction;
			Data[1] = Command.Cap_Flag;
			tx_switch=tx_switch<<1;
		}
		else if(tx_switch ==2)
		{
			TxMessage.StdId=0x207;
			Data[0] = Command.StorageFlag;
			Data[1] = Command.UIFlag;
			tx_switch=tx_switch>>1;
		}
		Data[2] = Command.Height_Set;
		Data[3] = Command.Chassis_Power_Switch;
		Data[4] = (u8)(Command.Vx >> 8);
		Data[5] = (u8)Command.Vx;
		Data[6] = Command.SideWay_Set;
		Data[7] = 0;
	
		HAL_CAN_AddTxMessage(&hcan2,&TxMessage,Data,(uint32_t*)CAN_TX_MAILBOX0);//发送
	}
}



