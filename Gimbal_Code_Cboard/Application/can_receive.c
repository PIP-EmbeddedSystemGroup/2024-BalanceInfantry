/**
 ******************************************************************************
 * @file    motor.c
 * @author  Tinker.Jia
 * @version V1.0
 * @date    2018/11/7
 * @brief   M3508����챵���Ĳ�������
 ******************************************************************************
 * @attention
 *
 *
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "Header.h"

static CAN_TxHeaderTypeDef TxMessage;
static CAN_RxHeaderTypeDef RxMessage;
float PowerData[4] = {0};
Robot_Status_t Robot_Status={0};
u8 convertflag[2] = {0};
/***********************************************************************************/
/***********************************************************************************/
/******************************�й�M3508�Ĵ�������**********************************/
/***********************************************************************************/
/***********************************************************************************/
/**
 * @brief  M3508��������
 * @param  ����Ϊ��������ĵ���ֵ
 * @retval None
 */
void Ammo_Output(s16 gm1_iq, s16 gm2_iq, s16 gm3_iq)
{
	u8 Data[8] = {0};
	TxMessage.StdId = 0x200;		  // ��̨����Ͳ�������ı�׼֡ID
	TxMessage.DLC = 8;				  // ���ݳ���
	Data[0] = (uint8_t)(gm1_iq >> 8); // Ħ����
	Data[1] = (uint8_t)gm1_iq;
	Data[2] = (uint8_t)(gm2_iq >> 8); // Ħ����
	Data[3] = (uint8_t)gm2_iq;
	Data[4] = (uint8_t)(gm3_iq >> 8);
	;
	Data[5] = (uint8_t)gm3_iq;
	Data[6] = 0;
	Data[7] = 0;
	HAL_CAN_AddTxMessage(&hcan1, &TxMessage, Data, (uint32_t *)CAN_TX_MAILBOX0); // ����
}

/**
 * @brief  CAN1总线数据接收函数
 * @param  None
 * @retval None
 */
void CAN1_Receive(void)
{
	u8 Data_Buf[8] = {0};

	HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &RxMessage, Data_Buf); // ��ȡ����������
	switch (RxMessage.StdId)										  // ͨ��֡��ID�ж������ĸ��Ǹ����
	{
	case 0x201:
	{
		Frequency[1]++;							// �豸֡�ʼ��
		Motor_Data_Deal(&FRI3508[0], Data_Buf); // ����������ݴ���
		break;
	}
	case 0x202:
	{
		Frequency[2]++;
		Motor_Data_Deal(&FRI3508[1], Data_Buf);
		break;
	}
	case 0x203:
	{
		Frequency[3]++;
		Motor_Data_Deal(&GM2006, Data_Buf);
		GM2006_Position_Deal(&GM2006); // �����������Ա��������漰����Ȧ��ת��������Ҫ���������Ա����������Ա�������һ��ת��
		break;
	}

	case 0x206:
	{
		Frequency[6]++;
		Motor_Data_Deal(&GM6020[PITCH], Data_Buf);
		break;
	}
	default:
		break;
	}
}

/**
 * @brief  CAN2总线数据接收函数
 * @param  None
 * @retval None
 */
void CAN2_Receive(void)
{
	u8 Data_Buf[8] = {0};

	HAL_CAN_GetRxMessage(&hcan2, CAN_RX_FIFO0, &RxMessage, Data_Buf); // ��ȡ����������
	switch (RxMessage.StdId)										  // ͨ��֡��ID�ж������ĸ��Ǹ����
	{
	case 0x208:
	{
		Frequency[4]++;
		Motor_Data_Deal(&GM6020[YAW], Data_Buf);
		break;
	}
    
    case 0xBB:
    {
        Frequency[5]++;
        Chassis_Data_Deal(&Robot_Status,Data_Buf);
    }
	default:
		break;
	}
}

/**
 * @brief  �Խ��յ������ݽ������黹ԭ
 * @param  ���ݴ洢λ������������
 * @retval None
 */
void Motor_Data_Deal(Motor_t *Receive, u8 Data[])
{
	Receive->Position.Real = (Data[0] << 8) | Data[1]; // ʵʱλ��ֵ��M3508Ϊ��Ա�������GM6020Ϊ���Ա�����
	Receive->Speed.Real = (Data[2] << 8) | Data[3];	   // ʵʱ�ٶ�ֵ
	Receive->TCurrent = (Data[4] << 8) | Data[5];	   // ʵʱ���ת�أ�����������ǵ��ʵʱ�����Ť��
	Receive->Temperature = Data[6];					   // �����ʵʱ�¶�ֵ
}

void Chassis_Data_Deal(Robot_Status_t* status, u8 Data[])
{
    status->INS.INS_Init_Status = Data[0];
    status->Body.Body_Upright_Status= Data[1];
    status->Body.Body_Soar_Status = Data[2];
    status->Leg.Leg_Init_Status = Data[3];
    status->Leg.Leg_Height_Status = Data[4];
}
/**
 * @brief  2006����������
 * @param  None
 * @retval None
 */
void GM2006_Position_Deal(Motor_t *Receive)
{
	if (Receive->Position.Flag == 0) // ��¼����λ��
	{
		Receive->Position.Flag = 1;
		Receive->Position.InitConvert = Receive->Position.Real / 64; // �Ա�����������С����С����Ϊ64
		Receive->Position.LastReal = Receive->Position.Real;
	}

	if (Receive->Position.Real - Receive->Position.LastReal > 4096) // ǰ���������ݲ�ֵ����4096������Ϊ��ת��һȦ
		Receive->Position.Count--;
	else if (Receive->Position.LastReal - Receive->Position.Real > 4096)
		Receive->Position.Count++;
	if (Receive->Position.Count < 0) // ���ٱ�Ϊ1��36����ת��36Ȧ֮�󣬾���Ϊ����Ա�����ת��һȦ
		Receive->Position.Count = 35;
	else if (Receive->Position.Count > 35)
		Receive->Position.Count = 0;
	Receive->Position.Convert = (Receive->Position.Count * 128) % 4608 + (Receive->Position.Real / 64) - Receive->Position.InitConvert; // ��Ա�����ת��Ϊ���Ա�����
	Receive->Position.Convert = ((Receive->Position.Convert + 4608) % 4608) * 1.77777778f;												// ����������ֵת����0-8191
	Receive->Position.LastReal = Receive->Position.Real;																				// ���ݵ���
}

/**
 * @brief  ��̨�����������
 * @param  ����Ϊ�ĸ�����ĵ���ֵ
 * @retval None
 */
void Gimbal_Output(s16 gm4_iq, s16 gm2_iq)
{
	u8 Data[8] = {0};
	TxMessage.StdId = 0x1FF; // ��̨����Ͳ�������ı�׼֡ID
	TxMessage.DLC = 8;		 // ���ݳ���
	Data[0] = 0;
	Data[1] = 0;
	Data[2] = (uint8_t)(gm2_iq >> 8); // Pitch;
	Data[3] = (uint8_t)gm2_iq;
	Data[4] = 0;
	Data[5] = 0;
	Data[6] = 0;
	Data[7] = 0;
	HAL_CAN_AddTxMessage(&hcan1, &TxMessage, Data, (uint32_t *)CAN_TX_MAILBOX0); // ����
	Data[0] = 0;
	Data[1] = 0;
	Data[2] = 0; // Pitch;
	Data[3] = 0;
	Data[4] = 0;
	Data[5] = 0;
	Data[6] = (uint8_t)(gm4_iq >> 8);
	Data[7] = (uint8_t)gm4_iq;
	HAL_CAN_AddTxMessage(&hcan2, &TxMessage, Data, (uint32_t *)CAN_TX_MAILBOX0); // ����
}

/**
 * @brief  ����������������
 * @param  ����Ϊ���������趨����
 * @retval None
 */
void Cap_Output(uint16_t temPower)
{
	u8 Data[8] = {0};
	TxMessage.StdId = 0x210;			// �������ݵı�׼֡ID
	TxMessage.DLC = 8;					// ���ݳ���
	Data[0] = (uint8_t)(temPower >> 8); // Yaw
	Data[1] = (uint8_t)temPower;
	HAL_CAN_AddTxMessage(&hcan1, &TxMessage, Data, (uint32_t *)CAN_TX_MAILBOX0); // ����
}
