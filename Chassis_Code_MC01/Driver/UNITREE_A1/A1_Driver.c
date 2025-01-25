/*
 * @Author: 静_火 lintianlang0918@qq.com
 * @Date: 2024-03-19 11:23:19
 * @LastEditors: 静_火 lintianlang0918@qq.com
 * @LastEditTime: 2024-03-20 08:47:14
 * @FilePath: \MC-01_chassis\Driver\UNITREE_A1\A1_Driver.c
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
/*
 * @Author: 静_火 lintianlang0918@qq.com
 * @Date: 2024-03-19 11:23:19
 * @LastEditors: 静_火 lintianlang0918@qq.com
 * @LastEditTime: 2024-03-20 08:42:16
 * @FilePath: \MC-01_chassis\Driver\UNITREE_A1\A1_Driver.c
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
/*
 * @Author: 静_火 lintianlang0918@qq.com
 * @Date: 2024-03-19 11:23:19
 * @LastEditors: 静_火 lintianlang0918@qq.com
 * @LastEditTime: 2024-03-19 13:47:38
 * @FilePath: \MC-01_chassis\Driver\UNITREE_A1\A1_Driver.c
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */

#include "usart.h"

#include "A1_Driver.h"
#include "RS485.h"
#include "CRC.h"
#include "Configuration.h"

#include "Balance_Task.h"
#include "A1_Task.h"	

float A1_Pos_offset[4] = {0};//宇树A1位置初始化偏移量，用于在上电复位时进行角度标定
/**
 * @brief  左侧并联腿A1电机数据接收函数
 * @param  None
 * @retval None
 */
void A1_Data_Rx_Left(void)
{
	if ((A1_Receive_Data_Left[0] == 0xFE) && (A1_Receive_Data_Left[1] == 0xEE))
	{
		Leg[LEFT].uart_feedback_count++;
		switch (A1_Receive_Data_Left[2])
		{
		case 0:
			A1_Motor[0].Position.Origin = A1_Receive_Data_Left[30] + (uint32_t)(A1_Receive_Data_Left[31] << 8) + (uint32_t)(A1_Receive_Data_Left[32] << 16) + (uint32_t)(A1_Receive_Data_Left[33] << 24);
			A1_Motor[0].T.Origin = A1_Receive_Data_Left[12] + (uint16_t)(A1_Receive_Data_Left[13] << 8);
			A1_Motor[0].T.Real = A1_Motor[0].T.Origin / 256.0f;
			A1_Motor[0].Position.Realf = A1_Motor[0].Position.Origin * 0.002333f - 21.059237f + 180.0f;
			A1_Motor[0].Speed.Origin = (A1_Receive_Data_Right[14] | (int16_t)A1_Receive_Data_Right[15]<<8);
			A1_Motor[0].Speed.real = A1_Motor[2].Speed.Origin *0.11f/128.0f;
			Leg[LEFT].theta1 = A1_Motor[0].Position.Realf * 0.017453f;
			Leg[LEFT].theta1_dot  = A1_Motor[0].Speed.real;
			break;
		case 1:
			A1_Motor[1].Position.Origin = A1_Receive_Data_Left[30] + (uint32_t)(A1_Receive_Data_Left[31] << 8) + (uint32_t)(A1_Receive_Data_Left[32] << 16) + (uint32_t)(A1_Receive_Data_Left[33] << 24);
			A1_Motor[1].T.Origin = A1_Receive_Data_Left[12] + (uint16_t)(A1_Receive_Data_Left[13] << 8);
			A1_Motor[1].T.Real = A1_Motor[1].T.Origin / 256.0f;
			A1_Motor[1].Position.Realf = A1_Motor[1].Position.Origin * 0.002333f - 31.997858f;
			A1_Motor[1].Speed.Origin = (A1_Receive_Data_Right[14] | (int16_t)A1_Receive_Data_Right[15]<<8);
			A1_Motor[1].Speed.real = A1_Motor[2].Speed.Origin*0.11f /128.0f;
			Leg[LEFT].theta4 = A1_Motor[1].Position.Realf * 0.017453f;
			Leg[LEFT].theta4_dot  = A1_Motor[1].Speed.real;
			break;
		}
	}else
	{
		HAL_UART_DMAStop(&huart1);
		for(int count = 0;count<78;count++)
			A1_Receive_Data_Left[count] = 0;
		HAL_UART_Receive_DMA(&huart1,A1_Receive_Data_Left,78);
	}
}

void A1_Data_Rx_Right(void)
{
	if ((A1_Receive_Data_Right[0] == 0xFE) && (A1_Receive_Data_Right[1] == 0xEE))
	{
		Leg[RIGHT].uart_feedback_count++;
		switch (A1_Receive_Data_Right[2])
		{
		case 0:
			A1_Motor[2].Position.Origin = A1_Receive_Data_Right[30] | (uint32_t)(A1_Receive_Data_Right[31] << 8) |(uint32_t)(A1_Receive_Data_Right[32] << 16) | (uint32_t)(A1_Receive_Data_Right[33] << 24);
			A1_Motor[2].T.Origin = A1_Receive_Data_Right[12] + (uint16_t)(A1_Receive_Data_Right[13] << 8);
			A1_Motor[2].T.Real = A1_Motor[2].T.Origin / 256.0f;
			A1_Motor[2].Position.Realf = A1_Motor[2].Position.Origin * 0.002333f - 1.996280f + 180.0f;
			A1_Motor[2].Speed.Origin = (A1_Receive_Data_Right[14] | (int16_t)A1_Receive_Data_Right[15]<<8);
			A1_Motor[2].Speed.real = A1_Motor[2].Speed.Origin *0.11f/128.0f;
			Leg[RIGHT].theta1 = A1_Motor[2].Position.Realf * 0.017453f;
			Leg[RIGHT].theta1_dot  = A1_Motor[2].Speed.real;
			break;
		case 1:
			A1_Motor[3].Position.Origin = A1_Receive_Data_Right[30] + (uint32_t)(A1_Receive_Data_Right[31] << 8) + (uint32_t)(A1_Receive_Data_Right[32] << 16) + (uint32_t)(A1_Receive_Data_Right[33] << 24);
			A1_Motor[3].T.Origin = A1_Receive_Data_Right[12] + (uint16_t)(A1_Receive_Data_Right[13] << 8);
			A1_Motor[3].T.Real = A1_Motor[3].T.Origin / 256.0f;
			A1_Motor[3].Position.Realf = A1_Motor[3].Position.Origin * 0.002333f - 36.477219f;
			A1_Motor[3].Speed.Origin = (A1_Receive_Data_Right[14] | (int16_t)A1_Receive_Data_Right[15]<<8);
			A1_Motor[3].Speed.real = A1_Motor[2].Speed.Origin *0.11f/128.0f;
			Leg[RIGHT].theta4 = A1_Motor[3].Position.Realf * 0.017453f;
			Leg[RIGHT].theta4_dot  = A1_Motor[3].Speed.real;
			break;
		}
	}
}

void A1_Modify_Data(A1_CMD_Tx_t* motor_s){
    motor_s->hex_len = 34;
    motor_s->motor_send_data.head.start[0] = 0xFE;
    motor_s->motor_send_data.head.start[1] = 0xEE;
    motor_s->motor_send_data.head.motorID = motor_s->id;
    motor_s->motor_send_data.head.reserved = 0x0;
    //motor_s->motor_send_data.Mdata.mode = motor_s->mode;
	  motor_s->motor_send_data.Mdata.mode = 0x0A;
    motor_s->motor_send_data.Mdata.ModifyBit = 0xFF;
    motor_s->motor_send_data.Mdata.ReadBit = 0x0;
    motor_s->motor_send_data.Mdata.reserved = 0x0;
    motor_s->motor_send_data.Mdata.Modify.L = 0;
    motor_s->motor_send_data.Mdata.T = motor_s->T*256;
    motor_s->motor_send_data.Mdata.W = motor_s->W*128;
    motor_s->motor_send_data.Mdata.Pos = (int)(motor_s->Pos * 2607.594715f);
    motor_s->motor_send_data.Mdata.K_P = motor_s->K_P*2048;
    
    motor_s->motor_send_data.Mdata.K_W = motor_s->K_W*1024;
    
    motor_s->motor_send_data.Mdata.LowHzMotorCmdIndex = 0;
    motor_s->motor_send_data.Mdata.LowHzMotorCmdByte = 0;
    motor_s->motor_send_data.Mdata.Res[0] = motor_s->Res;
    motor_s->motor_send_data.CRCdata.u32 = crc32_core((uint32_t*)(&(motor_s->motor_send_data)), 7);
}

void A1_Transmit_Data_Deal_Left(A1_CMD_Tx_t *motor_s){

	  A1_Transmit_Data_Left[0]  = motor_s->motor_send_data.head.start[0];
		A1_Transmit_Data_Left[1]  = motor_s->motor_send_data.head.start[1];
		A1_Transmit_Data_Left[2]  = motor_s->motor_send_data.head.motorID;
		A1_Transmit_Data_Left[3]  = motor_s->motor_send_data.head.reserved;
		A1_Transmit_Data_Left[4]  = motor_s->motor_send_data.Mdata.mode;
		A1_Transmit_Data_Left[5]  = motor_s->motor_send_data.Mdata.ModifyBit;
		A1_Transmit_Data_Left[6]  = motor_s->motor_send_data.Mdata.ReadBit;
		A1_Transmit_Data_Left[7]  = motor_s->motor_send_data.Mdata.reserved = 0x0;
		A1_Transmit_Data_Left[8]  = motor_s->motor_send_data.Mdata.Modify.uint8_t[0];
		A1_Transmit_Data_Left[9]  = motor_s->motor_send_data.Mdata.Modify.uint8_t[1];
		A1_Transmit_Data_Left[10] = motor_s->motor_send_data.Mdata.Modify.uint8_t[2];
		A1_Transmit_Data_Left[11] = motor_s->motor_send_data.Mdata.Modify.uint8_t[3];
		A1_Transmit_Data_Left[12] = (uint8_t)motor_s->motor_send_data.Mdata.T;
		A1_Transmit_Data_Left[13] = (uint8_t)(motor_s->motor_send_data.Mdata.T>>8);
		A1_Transmit_Data_Left[14] = (uint8_t)motor_s->motor_send_data.Mdata.W;
		A1_Transmit_Data_Left[15] = (uint8_t)(motor_s->motor_send_data.Mdata.W>>8);
		A1_Transmit_Data_Left[16] = 0;
		A1_Transmit_Data_Left[17] = 0;
		A1_Transmit_Data_Left[18] = 0;
		A1_Transmit_Data_Left[19] = 0;
		A1_Transmit_Data_Left[20] = (uint8_t)motor_s->motor_send_data.Mdata.K_P;
		A1_Transmit_Data_Left[21] = (uint8_t)(motor_s->motor_send_data.Mdata.K_P>>8);
		A1_Transmit_Data_Left[22] = (uint8_t)motor_s->motor_send_data.Mdata.K_W;
		A1_Transmit_Data_Left[23] = (uint8_t)(motor_s->motor_send_data.Mdata.K_W>>8);
		A1_Transmit_Data_Left[24] = motor_s->motor_send_data.Mdata.LowHzMotorCmdIndex;
		A1_Transmit_Data_Left[25] = motor_s->motor_send_data.Mdata.LowHzMotorCmdByte;
		A1_Transmit_Data_Left[26] = motor_s->motor_send_data.Mdata.Res[0].uint8_t[0];
		A1_Transmit_Data_Left[27] = motor_s->motor_send_data.Mdata.Res[0].uint8_t[1];
		A1_Transmit_Data_Left[28] = motor_s->motor_send_data.Mdata.Res[0].uint8_t[2];
		A1_Transmit_Data_Left[29] = motor_s->motor_send_data.Mdata.Res[0].uint8_t[3];
		A1_Transmit_Data_Left[30] = motor_s->motor_send_data.CRCdata.uint8_t[0];
		A1_Transmit_Data_Left[31] = motor_s->motor_send_data.CRCdata.uint8_t[1];
		A1_Transmit_Data_Left[32] = motor_s->motor_send_data.CRCdata.uint8_t[2];
		A1_Transmit_Data_Left[33] = motor_s->motor_send_data.CRCdata.uint8_t[3];

}

void A1_Transmit_Data_Deal_Right(A1_CMD_Tx_t *motor_s){

		A1_Transmit_Data_Right[0]  = motor_s->motor_send_data.head.start[0];
		A1_Transmit_Data_Right[1]  = motor_s->motor_send_data.head.start[1];
		A1_Transmit_Data_Right[2]  = motor_s->motor_send_data.head.motorID;
		A1_Transmit_Data_Right[3]  = motor_s->motor_send_data.head.reserved;
		A1_Transmit_Data_Right[4]  = motor_s->motor_send_data.Mdata.mode;
		A1_Transmit_Data_Right[5]  = motor_s->motor_send_data.Mdata.ModifyBit;
		A1_Transmit_Data_Right[6]  = motor_s->motor_send_data.Mdata.ReadBit;
		A1_Transmit_Data_Right[7]  = motor_s->motor_send_data.Mdata.reserved = 0x0;
		A1_Transmit_Data_Right[8]  = motor_s->motor_send_data.Mdata.Modify.uint8_t[0];
		A1_Transmit_Data_Right[9]  = motor_s->motor_send_data.Mdata.Modify.uint8_t[1];
		A1_Transmit_Data_Right[10] = motor_s->motor_send_data.Mdata.Modify.uint8_t[2];
		A1_Transmit_Data_Right[11] = motor_s->motor_send_data.Mdata.Modify.uint8_t[3];
		A1_Transmit_Data_Right[12] = (uint8_t)motor_s->motor_send_data.Mdata.T;
		A1_Transmit_Data_Right[13] = (uint8_t)(motor_s->motor_send_data.Mdata.T>>8);
		A1_Transmit_Data_Right[14] = (uint8_t)motor_s->motor_send_data.Mdata.W;
		A1_Transmit_Data_Right[15] = (uint8_t)(motor_s->motor_send_data.Mdata.W>>8);
		A1_Transmit_Data_Right[16] = 0;
		A1_Transmit_Data_Right[17] = 0;
		A1_Transmit_Data_Right[18] = 0;
		A1_Transmit_Data_Right[19] = 0;
		A1_Transmit_Data_Right[20] = (uint8_t)motor_s->motor_send_data.Mdata.K_P;
		A1_Transmit_Data_Right[21] = (uint8_t)(motor_s->motor_send_data.Mdata.K_P>>8);
		A1_Transmit_Data_Right[22] = (uint8_t)motor_s->motor_send_data.Mdata.K_W;
		A1_Transmit_Data_Right[23] = (uint8_t)(motor_s->motor_send_data.Mdata.K_W>>8);
		A1_Transmit_Data_Right[24] = motor_s->motor_send_data.Mdata.LowHzMotorCmdIndex;
		A1_Transmit_Data_Right[25] = motor_s->motor_send_data.Mdata.LowHzMotorCmdByte;
		A1_Transmit_Data_Right[26] = motor_s->motor_send_data.Mdata.Res[0].uint8_t[0];
		A1_Transmit_Data_Right[27] = motor_s->motor_send_data.Mdata.Res[0].uint8_t[1];
		A1_Transmit_Data_Right[28] = motor_s->motor_send_data.Mdata.Res[0].uint8_t[2];
		A1_Transmit_Data_Right[29] = motor_s->motor_send_data.Mdata.Res[0].uint8_t[3];
		A1_Transmit_Data_Right[30] = motor_s->motor_send_data.CRCdata.uint8_t[0];
		A1_Transmit_Data_Right[31] = motor_s->motor_send_data.CRCdata.uint8_t[1];
		A1_Transmit_Data_Right[32] = motor_s->motor_send_data.CRCdata.uint8_t[2];
		A1_Transmit_Data_Right[33] = motor_s->motor_send_data.CRCdata.uint8_t[3];

}