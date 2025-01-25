#include "can.h"
#include "stm32f4xx_hal_can.h"
#include "Can_Bus.h"
#include "universal.h"
#include "Configuration.h"

#include "Communicate_Task.h"

#define RAD_2_DEGREE 57.2957795f    // 180/pi
#define DEGREE_2_RAD 0.01745329252f // pi/180
#define LK9025_SPEED_LPF 0.85f
static CAN_TxHeaderTypeDef		TxMessage;
CAN_RxHeaderTypeDef 		RxMessage[2];
float PowerData[4] = {0};
uint16_t can_id[6] = {0};
uint8_t Data_Buf[8]={0};
int8_t LK9025_Tx_Count = 1;

void LK9025_Data_Deal(LK9025_Motor_t *Receive,uint8_t Data[])
{
	if(Data[0] == 0xA1)
		{
		Receive->Temperature      =   Data[1];//瀹氬瓙绾垮湀娓╁害
		Receive->Tcurrent.Real    =  (Data[3]<<8)|Data[2];//瀹炴椂鐢电杞煩鐢垫祦
//		Receive->Speed.Real       =	 (1-LK9025_SPEED_LPF) * Receive->Speed.Real +  LK9025_SPEED_LPF * DEGREE_2_RAD*(float)((int16_t)(Data[5]<<8 |Data[4]));//缂栫爜鍣ㄩ€熷害鏁版嵁
		Receive->Speed.Real       =	 DEGREE_2_RAD*(float)((int16_t)(Data[5]<<8 |Data[4]));
		Receive->Position.Real    =  (Data[7]<<8)|Data[6];//缂栫爜鍣ㄤ綅缃暟鎹�
		}
}


void LK9025_Tcurrent_Output(uint32_t ID,int16_t Tcurrent)
{
	uint8_t Data[8]={0};
	TxMessage.StdId=ID;//CAN总线ID
  TxMessage.DLC=8;  //数据帧长度
	Data[0]=(uint8_t)0XA1;
	Data[1]=(uint8_t)0X00;
	Data[2]=(uint8_t)0X00;
	Data[3]=(uint8_t)0X00;
	Data[4]=(uint8_t)Tcurrent;  //目标电流值
	Data[5]=(uint8_t)(Tcurrent >> 8);
	Data[6]=(uint8_t)0x00;
	Data[7]=(uint8_t)0x00;
	if(HAL_CAN_AddTxMessage(&hcan1,&TxMessage,Data,(uint32_t*)CAN_TX_MAILBOX0) != HAL_OK)//CAN1发送
    {
        if(HAL_CAN_AddTxMessage(&hcan1,&TxMessage,Data,(uint32_t*)CAN_TX_MAILBOX1) != HAL_OK)
            HAL_CAN_AddTxMessage(&hcan1,&TxMessage,Data,(uint32_t*)CAN_TX_MAILBOX2) ;
    }
}

void LK9025_Output_Normal(float output_left,float output_right)
{
//	if(LK9025_Tx_Count == 1)
//	{
		output_left = output_left * 403.44f - 36.746f;	
		//output_left = output_left *195.3f;
		output_left = Func_Limit(output_left,1800,-1800);
		LK9025_Tcurrent_Output(LEFT_WHEEL_ID,output_left);
		//LK9025_Tx_Count=LK9025_Tx_Count << 1;
//	}else
//	{
		output_right = output_right * 403.44f - 36.746f;
		//output_right = output_right *195.3f;
		output_right = Func_Limit(output_right,1800,-1800);
		LK9025_Tcurrent_Output(RIGHT_WHEEL_ID,output_right);
		//LK9025_Tx_Count=LK9025_Tx_Count >> 1;
//	}
	

	
}
 
void LK9025_Output_Zero(void)
{ 
	if(LK9025_Tx_Count == 1)
	{
		LK9025_Tcurrent_Output(LEFT_WHEEL_ID,0);
		LK9025_Tx_Count=LK9025_Tx_Count << 1;
	}else
	{
		LK9025_Tcurrent_Output(RIGHT_WHEEL_ID,0);
		LK9025_Tx_Count=LK9025_Tx_Count >> 1;
	}
}

void HMI_Communicate(Robot_Status_t* status)//用户交互通信
{
    uint8_t Data[8]={0};
	TxMessage.StdId=0xAA;//CAN鎬荤嚎璁惧ID
  TxMessage.DLC=8;  //鏍囧噯甯�8瀛楄妭鏁版嵁甯�
	Data[0]=status->INS.INS_Init_Status;
	Data[1]=status->Body.Body_Upright_Status;
	Data[2]=status->Body.Body_Soar_Status;
	Data[3]=status->Leg.Leg_Init_Status;
	Data[4]=status->Leg.Leg_Height_Status;
	Data[5]=status->Gimbal.Gimbal_Reposition_Status;
	Data[6]=status->INS.INS_Fail_Status;
	Data[7]=(uint8_t)0x00;
	if(HAL_CAN_AddTxMessage(&hcan1,&TxMessage,Data,(uint32_t*)CAN_TX_MAILBOX0) != HAL_OK)//CAN1鍙戦€�
    {
        if(HAL_CAN_AddTxMessage(&hcan1,&TxMessage,Data,(uint32_t*)CAN_TX_MAILBOX1) != HAL_OK)
            HAL_CAN_AddTxMessage(&hcan1,&TxMessage,Data,(uint32_t*)CAN_TX_MAILBOX2) ;
    }
}

void Gimbal_Communicate(Robot_Status_t* status)
{
    uint8_t Data[8]={0};
	TxMessage.StdId=0xBB;//CAN鎬荤嚎璁惧ID
  TxMessage.DLC=5;  //鏍囧噯甯�8瀛楄妭鏁版嵁甯�
	Data[0]=status->INS.INS_Init_Status;
	Data[1]=status->Body.Body_Upright_Status;
	Data[2]=status->Body.Body_Soar_Status;
	Data[3]=status->Leg.Leg_Init_Status;
	Data[4]=status->Leg.Leg_Height_Status;
	if(HAL_CAN_AddTxMessage(&hcan2,&TxMessage,Data,(uint32_t*)CAN_TX_MAILBOX0) != HAL_OK)//CAN1鍙戦€�
    {
        if(HAL_CAN_AddTxMessage(&hcan2,&TxMessage,Data,(uint32_t*)CAN_TX_MAILBOX1) != HAL_OK)
            HAL_CAN_AddTxMessage(&hcan2,&TxMessage,Data,(uint32_t*)CAN_TX_MAILBOX2) ;
    }
}
void Gimbal_Data_Receive(void)
{
	uint8_t Data_Buf[8]={0};
	HAL_CAN_GetRxMessage(&hcan2,CAN_RX_FIFO0,&RxMessage[0],Data_Buf);//锟斤拷取锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷
	
	switch(RxMessage[0].StdId)//通
	{
		case 0x206:
		{
			Command.Shooting_Friction = Data_Buf[0];
			Command.Cap_Flag = Data_Buf[1];
			Command.Height_Set = Data_Buf[2];
			Command.Chassis_Power_Switch = Data_Buf[3];
			Command.Vx = (Data_Buf[4] << 8) | Data_Buf[5];
			can_id[3]++;
			break;
		}
		case 0x207:
		{
			Command.Storage_Flag = Data_Buf[0];
			Command.UIFlag = Data_Buf[1];
			Command.Height_Set = Data_Buf[2];
			Command.Chassis_Power_Switch = Data_Buf[3];
			Command.Vx = (Data_Buf[4] << 8) | Data_Buf[5];
			can_id[4]++;
			break;
		}
		case 0X208:
		{
			//接收YAW轴6020电机数据反馈包
			GM6020_YAW.Position.Real_fp32 = ((Data_Buf[0]<<8)|Data_Buf[1]) * 0.043f;
			GM6020_YAW.Speed.Real = (Data_Buf[2]<<8)|Data_Buf[3];
			GM6020_YAW.TCurrent = (Data_Buf[4]<<8)|Data_Buf[5];
			GM6020_YAW.Temperature = Data_Buf[6];
		}
		default:
			break;
	}
}

void CAN1_Receive(void)
{
  HAL_CAN_GetRxMessage(&hcan1,CAN_RX_FIFO0,&RxMessage[1],Data_Buf);//
	switch(RxMessage[1].StdId)//
	{
		case 0x141:
		{
			can_id[0]++;
			LK9025_Data_Deal(&LK9025_Motor[LEFT],Data_Buf);
			//Odometer.x += L9015_Motor[LEFT].Speed.Real * 0.001f;
			break;
		}
		case 0x142:
		{
			can_id[1]++;
			LK9025_Data_Deal(&LK9025_Motor[RIGHT],Data_Buf);
			//Odometer.x -= L9015_Motor[RIGHT].Speed.Real * 0.001f;
			break;
		}
		default:
			break;
	}
	can_id[2] = can_id[0] - can_id[1];
}

/**
  * @brief  瓒呯骇鐢靛鍣ㄨ緭鍑哄姛鐜囨帶鍒�
  * @param  
  * @retval None
  */
void Cap_Output(uint16_t Power)
{
		uint8_t Data[8]={0};
		TxMessage.StdId=0x603;        
		TxMessage.DLC=8;
		Data[0]=(uint8_t)(Power >> 8);
		Data[1]=(uint8_t)(Power);
		HAL_CAN_AddTxMessage(&hcan1,&TxMessage,Data,(uint32_t*)CAN_TX_MAILBOX0);//CAN1鍙戦€�
}

/**
  * @brief  CAN鎬荤嚎鍚姩
  * @param  None
  * @retval None
  */
void CAN_Start(void)
{
    CAN1_Filter_Init();
	HAL_CAN_Start(&hcan1);
	CAN2_Filter_Init();
	HAL_CAN_Start(&hcan2);
	
     HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
	HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
	//HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO1_MSG_PENDING);
}



/**
  * @brief  CAN1鐨勬护娉㈠櫒鍒濆鍖�
  * @param  CAN_HandleTypeDef
  * @retval None
  */
void CAN1_Filter_Init(void)
{
	CAN_FilterTypeDef  CAN1_FilterConfig;

	CAN1_FilterConfig.FilterIdHigh = 0x0000;
	CAN1_FilterConfig.FilterIdLow = 0x0000;
	CAN1_FilterConfig.FilterMaskIdHigh = 0x0000;
	CAN1_FilterConfig.FilterMaskIdLow = 0x0000;
	CAN1_FilterConfig.FilterFIFOAssignment = CAN_FilterFIFO0;
	CAN1_FilterConfig.FilterBank = 0;
	CAN1_FilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
	CAN1_FilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
	CAN1_FilterConfig.FilterActivation = ENABLE;
	//CAN1_FilterConfig.SlaveStartFilterBank = 14;
	if(HAL_CAN_ConfigFilter(&hcan1, &CAN1_FilterConfig) != HAL_OK)
	{
			while(1);
	}
	HAL_CAN_ActivateNotification(&hcan1,CAN_IT_RX_FIFO0_MSG_PENDING);
}

/**
  * @brief  CAN2鐨勬护娉㈠櫒鍒濆鍖�
  * @param  CAN_HandleTypeDef
  * @retval None
  */
void CAN2_Filter_Init(void)
{
	CAN_FilterTypeDef  CAN_FilterConfig;

	CAN_FilterConfig.FilterIdHigh = 0x0000;
	CAN_FilterConfig.FilterIdLow = 0x0000;
	CAN_FilterConfig.FilterMaskIdHigh = 0x0000;
	CAN_FilterConfig.FilterMaskIdLow = 0x0000;
	CAN_FilterConfig.FilterFIFOAssignment = CAN_FilterFIFO0;
	CAN_FilterConfig.FilterBank = 15;//鐢变簬CAN2浠庡睘浜嶤AN1锛屾护娉㈠櫒缁勪綅浜嶤AN1涔嬪悗锛岄渶瑕佽缃负14-27涔嬮棿
	CAN_FilterConfig.SlaveStartFilterBank = 14;
	CAN_FilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
	CAN_FilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
	CAN_FilterConfig.FilterActivation = ENABLE;
    if(HAL_CAN_ConfigFilter(&hcan2, &CAN_FilterConfig) != HAL_OK)
	{
		while(1);
	}
	HAL_CAN_ActivateNotification(&hcan2,CAN_IT_RX_FIFO0_MSG_PENDING);
	
	
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	if(hcan->Instance==CAN1)
	{
		CAN1_Receive();
	}
	if(hcan->Instance==CAN2)
	{
		Gimbal_Data_Receive();
	}


}