/*
 * @Author: frozen-fire 2812643217@qq.com
 * @Date: 2025-02-18 13:33:09
 * @LastEditors: frozen-fire 2812643217@qq.com
 * @LastEditTime: 2025-02-18 14:38:31
 * @FilePath: \2024-BalanceInfantry\Chassis_Code_MC01\Task\A1_Task.c
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "stdint.h"
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "usart.h"
#include "gpio.h"

#include "Can_Bus.h"
#include "Configuration.h"
#include "A1_Driver.h"
#include "universal.h"
#include <PID.h>
#include "CRC.h"
#include "Dwt_Timer.h"
#include "arm_math.h"

#include "RS485.h"

#include "A1_Task.h"
#include "Communicate_Task.h"
//#include "Balance_Task.h"

A1_Motor_t A1_Motor[4] = {0};
A1_CMD_Tx_t A1_Control[4] = {0};

//VMC逆解算矩阵
arm_matrix_instance_f32 A_sorce_Left;
arm_matrix_instance_f32 A_sorce_Right;
arm_matrix_instance_f32 A_inverse_Left;
arm_matrix_instance_f32 A_inverse_Right;


float Support_F[2] = {62.0f,62.0f};//{62.0f,62.0f}支撑力补偿，抵消机体所受重力
//float Roll_Extra_comp_p = 80.0f;

static uint32_t A1_dwt_cnt = 0; 
float A1Task_dt;

//宇树A1电机控制命令发送进程
void A1_Tx_Task(void const * argument)
{
	
	portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
	
	for(int i; i<4;i++)
	{
		A1_Control[i].hex_len = 34;
		A1_Control[i].motor_send_data.head.start[0] = 0xFE;
		A1_Control[i].motor_send_data.head.start[1] = 0xEE;
		A1_Control[i].motor_send_data.head.motorID = A1_Control[i].id;
		A1_Control[i].motor_send_data.head.reserved = 0x0;
		//motor_s->motor_send_data.Mdata.mode = motor_s->mode;
		A1_Control[i].motor_send_data.Mdata.mode = 0x0A;
		A1_Control[i].motor_send_data.Mdata.ModifyBit = 0xFF;
		A1_Control[i].motor_send_data.Mdata.ReadBit = 0x0;
		A1_Control[i].motor_send_data.Mdata.reserved = 0x0;
		A1_Control[i].motor_send_data.Mdata.Modify.L = 0;
	}
	
	while(1)
	{
		vTaskDelayUntil(&xLastWakeTime,1);
		A1Task_dt = DWT_GetDeltaT(&A1_dwt_cnt);
		//HAL_GPIO_TogglePin(GPIOC,RS485_VCC_Pin);
		//达妙板载RS485没有自动流控，需要在发送前和接收前控制485芯片的流向pin脚
		//在发送前拉高DIR引脚，将数据流向设置为芯片到外设，将MCU内部数据发送至电机
		//之后使用UART发送完成中断回调函数将DIR引脚拉低，开始接收电机反馈数据
		//由于板载485总线在4.8Mbps的通讯速率下不能完全将波特率锁定至4.8Mbps，会出现发生频率很低的通讯失败
		//解决方法是将串口超采样倍率降低至8倍，降低后并不会出现通讯错误的情况；同时检验电机反馈帧的帧头，若帧头错误则重启接收
		A1_Modify_Data(&A1_Control[0]);
		A1_Transmit_Data_Deal_Left(&A1_Control[0]);
		HAL_GPIO_WritePin(GPIOC,RS485_DIR1_Pin,GPIO_PIN_SET);
		HAL_UART_Transmit_DMA(&huart1,A1_Transmit_Data_Left,34);
		HAL_UART_Receive_DMA(&huart1,A1_Receive_Data_Left,78);
		//Leg[0].transmit_count++;
		//DWT_Delay(0.0001f);
		A1_Modify_Data(&A1_Control[2]);
		A1_Transmit_Data_Deal_Right(&A1_Control[2]);
		HAL_UART_Transmit_DMA(&huart6,A1_Transmit_Data_Right,34);
		HAL_UART_Receive_DMA(&huart6,A1_Receive_Data_Right,78);
		//Leg[1].transmit_count++;
		
		DWT_Delay(0.0003f);

		A1_Modify_Data(&A1_Control[1]);
		A1_Transmit_Data_Deal_Left(&A1_Control[1]);
		HAL_GPIO_WritePin(GPIOC,RS485_DIR1_Pin,GPIO_PIN_SET);
		HAL_UART_Transmit_DMA(&huart1,A1_Transmit_Data_Left,34);
		HAL_UART_Receive_DMA(&huart1,A1_Receive_Data_Left,78);
		//Leg[0].transmit_count++;
		//DWT_Delay(0.0001f);
		A1_Modify_Data(&A1_Control[3]);
		A1_Transmit_Data_Deal_Right(&A1_Control[3]);
		HAL_UART_Transmit_DMA(&huart6,A1_Transmit_Data_Right,34);
		HAL_UART_Receive_DMA(&huart6,A1_Receive_Data_Right,78);
		//Leg[1].transmit_count++;
		
//		Leg[0].transmit_error_count = Leg[0].transmit_count - Leg[0].uart_feedback_count;
//		Leg[1].transmit_error_count = Leg[1].transmit_count - Leg[1].uart_feedback_count;
		
	}

}

/**
 * @brief 左腿VMC
 * 
 */
void Jointmotor_Control_Cacl_Left(void)
{
	Leg[0].A11 = l1 * arm_sin_f32(Leg[0].theta0 - Leg[0].theta3) * arm_sin_f32(Leg[0].theta1 - Leg[0].theta2) / arm_sin_f32(Leg[0].theta3 - Leg[0].theta2);
	Leg[0].A12 = l1 * arm_cos_f32(Leg[0].theta0 - Leg[0].theta3) * arm_sin_f32(Leg[0].theta1 - Leg[0].theta2) / (Leg[0].L0 * arm_sin_f32(Leg[0].theta3 - Leg[0].theta2));
	Leg[0].T0 = ( Leg[1].A11 * Leg[0].F * 0.00011f+ Leg[0].A12 * Leg[0].Tp * 0.11f);
	Leg[0].A21 = l4 * arm_sin_f32(Leg[0].theta0 - Leg[0].theta2) * arm_sin_f32(Leg[0].theta3 - Leg[0].theta4) / arm_sin_f32(Leg[0].theta3 - Leg[0].theta2);
	Leg[0].A22 = l4 * arm_cos_f32(Leg[0].theta0 - Leg[0].theta2) * arm_sin_f32(Leg[0].theta3 - Leg[0].theta4) / (Leg[0].L0 * arm_sin_f32(Leg[0].theta3 - Leg[0].theta2));
	Leg[0].T1 = ( Leg[0].A21 * Leg[0].F  * 0.00011f+ Leg[0].A22 * Leg[0].Tp * 0.11f);
	
	Leg[0].A_matrix[0] = Leg[0].A11;
	Leg[0].A_matrix[1] = Leg[0].A12;
	Leg[0].A_matrix[2] = Leg[0].A21;
	Leg[0].A_matrix[3] = Leg[0].A22;
	
	if(arm_mat_inverse_f32(&A_sorce_Left,&A_inverse_Left) != ARM_MATH_SUCCESS)  
	    Leg[0].transmit_error_count++;
	

	Leg[0].F_Feedback  = Leg[0].A_inverse_matrix[0] * A1_Motor[0].T.Real + Leg[0].A_inverse_matrix[1] * A1_Motor[1].T.Real;
	Leg[0].Tp_Feedback = Leg[0].A_inverse_matrix[2] * A1_Motor[0].T.Real + Leg[0].A_inverse_matrix[3] * A1_Motor[1].T.Real;
	Leg[0].P = Leg[0].F_Feedback * arm_cos_f32(Leg[0].theta * 0.000767f) + (Leg[0].Tp_Feedback * arm_sin_f32(Leg[0].theta * 0.000767f)) / Leg[0].L0;
}

void Jointmotor_Control_Cacl_Right(void)
{
	Leg[1].A11 = l1 * arm_sin_f32(Leg[1].theta0 - Leg[1].theta3) * arm_sin_f32(Leg[1].theta1 - Leg[1].theta2) / arm_sin_f32(Leg[1].theta3 - Leg[1].theta2);
	Leg[1].A12 = l1 * arm_cos_f32(Leg[1].theta0 - Leg[1].theta3) * arm_sin_f32(Leg[1].theta1 - Leg[1].theta2) / (Leg[1].L0 * arm_sin_f32(Leg[1].theta3 - Leg[1].theta2));
	Leg[1].T0 = ( Leg[1].A11 * Leg[1].F * 0.00011f+ Leg[1].A12 * Leg[1].Tp * 0.11f);
	Leg[1].A21 = l4 * arm_sin_f32(Leg[1].theta0 - Leg[1].theta2) * arm_sin_f32(Leg[1].theta3 - Leg[1].theta4) / arm_sin_f32(Leg[1].theta3 - Leg[1].theta2);
	Leg[1].A22 = l4 * arm_cos_f32(Leg[1].theta0 - Leg[1].theta2) * arm_sin_f32(Leg[1].theta3 - Leg[1].theta4) / (Leg[1].L0 * arm_sin_f32(Leg[1].theta3 - Leg[1].theta2));
	Leg[1].T1 = ( Leg[1].A21 * Leg[1].F  * 0.00011f+ Leg[1].A22 * Leg[1].Tp * 0.11f);
	
	Leg[1].A_matrix[0] = Leg[1].A11;
	Leg[1].A_matrix[1] = Leg[1].A12;
	Leg[1].A_matrix[2] = Leg[1].A21;
	Leg[1].A_matrix[3] = Leg[1].A22;
	
	if(arm_mat_inverse_f32(&A_sorce_Right,&A_inverse_Right) != ARM_MATH_SUCCESS)  
		Leg[1].transmit_error_count++;

	Leg[1].F_Feedback  = Leg[1].A_inverse_matrix[0] * A1_Motor[2].T.Real + Leg[1].A_inverse_matrix[1] * A1_Motor[3].T.Real;
	Leg[1].Tp_Feedback = Leg[1].A_inverse_matrix[2] * A1_Motor[2].T.Real + Leg[1].A_inverse_matrix[3] * A1_Motor[3].T.Real;
	Leg[1].P = Leg[1].F_Feedback * arm_cos_f32(Leg[1].theta * 0.000767f) + (Leg[1].Tp_Feedback * arm_sin_f32(Leg[1].theta * 0.000767f)) / Leg[1].L0;
}

void VMC_IK_Matrix_Init(void){
	arm_mat_init_f32(&A_sorce_Left,2,2,Leg[0].A_matrix);
	arm_mat_init_f32(&A_inverse_Left,2,2,Leg[0].A_inverse_matrix);
	arm_mat_init_f32(&A_sorce_Right,2,2,Leg[1].A_matrix);
	arm_mat_init_f32(&A_inverse_Right,2,2,Leg[1].A_inverse_matrix);
}

void A1_LimitMotorOutput(float Torque_Limit) {
    A1_Control[0].T = Func_Limit(Leg[0].T0, Torque_Limit, -Torque_Limit);
    A1_Control[1].T = Func_Limit(Leg[0].T1, Torque_Limit, -Torque_Limit);
    A1_Control[2].T = Func_Limit(Leg[1].T0, Torque_Limit, -Torque_Limit);
    A1_Control[3].T = Func_Limit(Leg[1].T1, Torque_Limit, -Torque_Limit);
}