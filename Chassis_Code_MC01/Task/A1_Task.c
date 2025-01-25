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


float Support_F[2] = {62.0f,62.0f};//支撑力补偿，抵消机体所受重力
//float Roll_Extra_comp_p = 80.0f;

static uint32_t A1_dwt_cnt = 0; 
float A1Task_dt;

//宇树A1电机控制命令发送进程
void A1_Tx_Task(void const * argument)
{
    

	A1_Control[0].id = 0x00;
	A1_Control[1].id = 0x01;
	A1_Control[2].id = 0x00;
	A1_Control[3].id = 0x01;
	
	arm_mat_init_f32(&A_sorce_Left,2,2,Leg[0].A_matrix);
	arm_mat_init_f32(&A_inverse_Left,2,2,Leg[0].A_inverse_matrix);
	arm_mat_init_f32(&A_sorce_Right,2,2,Leg[1].A_matrix);
	arm_mat_init_f32(&A_inverse_Right,2,2,Leg[1].A_inverse_matrix);
	
	portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
	
	while(1)
	{
		vTaskDelayUntil(&xLastWakeTime,1);
		A1Task_dt = DWT_GetDeltaT(&A1_dwt_cnt);
		//HAL_GPIO_TogglePin(GPIOC,RS485_VCC_Pin);
			if(Command.Chassis_Power_Switch == OFF)
			{
				Leg_Output(&Leg[0],0,0);
				Leg_Output(&Leg[1],0,0);
			}
			else
			{
                if(Robot_Status.Body.Body_Upright_Status == YES)
                {
                    Leg_Output(&Leg[LEFT] , L0_pid[LEFT].Output +Support_F[LEFT] * arm_cos_f32(Leg[0].theta) * (1.0f + arm_sin_f32(Ins_Data.Roll)) - Leg_ROLL_Compensate_pid[0].Output , -Leg[LEFT].U[1] - Leg_theta_Harmonize_pid[0].Output);//&Leg[0],L0_pid[0].Output + Support_F * arm_cos_f32(Leg[0].theta),Theta0_pid[0].Output
                    Leg_Output(&Leg[RIGHT] , L0_pid[RIGHT].Output + Support_F[RIGHT] * arm_cos_f32(Leg[1].theta) * (1.0f - arm_sin_f32(Ins_Data.Roll)) + Leg_ROLL_Compensate_pid[0].Output, Leg[RIGHT].U[1] - Leg_theta_Harmonize_pid[0].Output);//&Leg[1],L0_pid[0].Output + Support_F * arm_cos_f32(Leg[1].theta),Theta0_pid[1].Output
                }
                else
                {
                    Leg_Output(&Leg[0],-10,0);
                    Leg_Output(&Leg[1],-10,0);
                }
			}
			
			//分别计算左右腿虚拟力控，放在这里是为了减轻主任务Balance_Task的资源消耗，应该有助于保持进程执行周期的稳定
			//同时也是因为这是并联腿控制特有的算法，放在这里感觉比较合理
			Jointmotor_Control_Cacl_Left();
			Jointmotor_Control_Cacl_Right();
			
			//电机输出限幅，目前限制的转子最大转矩是正负2NM，对应的输出轴最大转矩是正负18NM左右，在平地上应该是绝对够用的
			//之后要实现跳跃和飞坡落地应该需要放宽限制
			A1_Control[0].T =  Func_Limit(Leg[0].T0,2.0,-2.0);
			A1_Control[1].T =  Func_Limit(Leg[0].T1,2.0,-2.0);
			A1_Control[2].T =  Func_Limit(Leg[1].T0,2.0,-2.0);
			A1_Control[3].T =  Func_Limit(Leg[1].T1,2.0,-2.0);
			
//		if(Leg[0].transmit_count % 2 == 0)//通过计数变量做分频发送
//		{
//			A1_Modify_Data(&A1_Control[0]);
//			A1_Transmit_Data_Deal_Left(&A1_Control[0]);
//			HAL_GPIO_WritePin(GPIOC,RS485_DIR1_Pin,GPIO_PIN_SET);
//			HAL_UART_Transmit_DMA(&huart1,A1_Transmit_Data_Left,34);
//			HAL_UART_Receive_DMA(&huart1,A1_Receive_Data_Left,78);
//			Leg[0].transmit_count++;
//		}
//		else
//		{
//			A1_Modify_Data(&A1_Control[1]);
//			A1_Transmit_Data_Deal_Left(&A1_Control[1]);
//			HAL_GPIO_WritePin(GPIOC,RS485_DIR1_Pin,GPIO_PIN_SET);
//			HAL_UART_Transmit_DMA(&huart1,A1_Transmit_Data_Left,34);
//			HAL_UART_Receive_DMA(&huart1,A1_Receive_Data_Left,78);
//			Leg[0].transmit_count++;
//		}

//		if(Leg[1].transmit_count % 2 == 0)//锟斤拷锟斤拷锟斤拷锟捷凤拷锟斤拷
//		{
//			A1_Modify_Data(&A1_Control[2]);
//			A1_Transmit_Data_Deal_Right(&A1_Control[2]);
//			HAL_UART_Transmit_DMA(&huart6,A1_Transmit_Data_Right,34);
//			HAL_UART_Receive_DMA(&huart6,A1_Receive_Data_Right,78);
//			Leg[1].transmit_count++;
//		}
//		else
//		{
//			A1_Modify_Data(&A1_Control[3]);
//			A1_Transmit_Data_Deal_Right(&A1_Control[3]);
//			HAL_UART_Transmit_DMA(&huart6,A1_Transmit_Data_Right,34);
//			HAL_UART_Receive_DMA(&huart6,A1_Receive_Data_Right,78);
//			Leg[1].transmit_count++;
//		}
		
		
		
		
		
		//达妙板载RS485没有自动流控，需要在发送前和接收前控制485芯片的流向pin脚
		//在发送前拉高DIR引脚，将数据流向设置为芯片到外设，将MCU内部数据发送至电机
		//之后使用UART发送完成中断回调函数将DIR引脚拉低，开始接收电机反馈数据
		//由于板载485总线在4.8Mbps的通讯速率下不能完全将波特率锁定至4.8Mbps，会出现发生频率很低的通讯失败
		//解决方法是将串口超采样倍率降低至8倍，降低后基本不会出现通讯错误的情况；同时检验电机反馈帧的帧头，若帧头错误则重启接收
		A1_Modify_Data(&A1_Control[0]);
		A1_Transmit_Data_Deal_Left(&A1_Control[0]);
		HAL_GPIO_WritePin(GPIOC,RS485_DIR1_Pin,GPIO_PIN_SET);
		HAL_UART_Transmit_DMA(&huart1,A1_Transmit_Data_Left,34);
		HAL_UART_Receive_DMA(&huart1,A1_Receive_Data_Left,78);
		//Leg[0].transmit_count++;
		DWT_Delay(0.0001f);
		A1_Modify_Data(&A1_Control[2]);
		A1_Transmit_Data_Deal_Right(&A1_Control[2]);
		HAL_UART_Transmit_DMA(&huart6,A1_Transmit_Data_Right,34);
		HAL_UART_Receive_DMA(&huart6,A1_Receive_Data_Right,78);
		//Leg[1].transmit_count++;
		
		DWT_Delay(0.0002f);

		A1_Modify_Data(&A1_Control[1]);
		A1_Transmit_Data_Deal_Left(&A1_Control[1]);
		HAL_GPIO_WritePin(GPIOC,RS485_DIR1_Pin,GPIO_PIN_SET);
		HAL_UART_Transmit_DMA(&huart1,A1_Transmit_Data_Left,34);
		HAL_UART_Receive_DMA(&huart1,A1_Receive_Data_Left,78);
		//Leg[0].transmit_count++;
		DWT_Delay(0.0001f);
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
	    //Leg[0].transmit_error_count++;
		;

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
	    //Leg[1].transmit_error_count++;
		;

	Leg[1].F_Feedback  = Leg[1].A_inverse_matrix[0] * A1_Motor[2].T.Real + Leg[1].A_inverse_matrix[1] * A1_Motor[3].T.Real;
	Leg[1].Tp_Feedback = Leg[1].A_inverse_matrix[2] * A1_Motor[2].T.Real + Leg[1].A_inverse_matrix[3] * A1_Motor[3].T.Real;
	Leg[1].P = Leg[1].F_Feedback * arm_cos_f32(Leg[1].theta * 0.000767f) + (Leg[1].Tp_Feedback * arm_sin_f32(Leg[1].theta * 0.000767f)) / Leg[1].L0;
}