/*
 * @Author: 静_火 lintianlang0918@qq.com
 * @Date: 2024-03-07 19:47:28
 * @LastEditors: 静_火 lintianlang0918@qq.com
 * @LastEditTime: 2024-04-04 02:51:59
 * @FilePath: \MC-01_chassis\Task\Monitor_Task.c
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "tim.h"

#include "Can_Bus.h"
#include "Configuration.h"
#include "PID.h"

#include "Communicate_Task.h"

Robot_Status_t Robot_Status = {0};
uint16_t Status_Counter [2] = {0};

static int32_t operate_count = 0;

static uint8_t Height_Set_Last = 0;
void Monitor_Task(void const * argument)
{
	portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
	while(1)
	{
		vTaskDelayUntil(&xLastWakeTime,5);
		operate_count++;
		if((operate_count%1)==0)//200Hz
		{
			Robot_Fall_Detect();
			//Robot_Param_Switch ();
			if(Command.Chassis_Power_Switch == OFF)
			{
				State_Variables.X = 0;
				State_Variables.X_dot = 0;
				State_Variables.X_dot_cov = 0;
				State_Variables.X_dot_meansure = 0;
				State_Variables.X_dot_predict = 0;
				State_Variables.target_X = 0;
				Leg[LEFT].U[1] = 0;
				Leg[RIGHT].U[1] = 0;
				Leg[LEFT].Tp = 0;
				Leg[RIGHT].Tp = 0;
				PID_Clear(&Balance_Yaw_Position_pid);
				PID_Clear(&Balance_Yaw_Speed_pid);
				PID_Clear(&Leg_theta_Harmonize_pid[0]);
			}
			// if(Command.Height_Set != Height_Set_Last)
			// 	Command.Height_Switch =1;//车体高度控制命令变化时将高度切换控制位置1，进行高度切换
			// Height_Set_Last = Command.Height_Set;
		}
		if((operate_count%10)==0)//20Hz
		{
			//监控各个任务进程的执行延时，若
			Robot_Task_Monitor();
		}
		if((operate_count%100)==0)//2Hz
		{
			;
		}
		
	}
}
/**
 * @brief 机器人摔倒检测函数
 * @param NONE
 * @retval NONE
*/
void Robot_Fall_Detect(void)
{
	if(Robot_Status.Body.Body_Upright_Status == YES)
	{
		
		if((Ins_Data.Pitch < -0.28f) ||(Ins_Data.Pitch > 0.28f))
		{
			if(Status_Counter [1]!=1)//确认定时器未被启动
			{
				HAL_TIM_Base_Start_IT (&htim5);//开启状态切换计时器
				Status_Counter [1] = 1;
			}
		}
		else
		{
			HAL_TIM_Base_Stop_IT(&htim5);//关闭计时器
			Status_Counter [0]= 0;
			Status_Counter [1] = 0;
		}
		
		if(Status_Counter [0] > 200)
			{
				Robot_Status.Body.Body_Upright_Status = NO;//将车体直立状态位设为NO
				State_Variables.X = 0;
				State_Variables.X_dot = 0;
				State_Variables.X_dot_cov = 0;
				State_Variables.X_dot_meansure = 0;
				State_Variables.X_dot_predict = 0;
				State_Variables.target_X = 0;
				Leg[LEFT].U[1] = 0;
				Leg[RIGHT].U[1] = 0;
				Leg[LEFT].Tp = 0;
				Leg[RIGHT].Tp = 0;
				PID_Clear(&Balance_Yaw_Position_pid);
				PID_Clear(&Balance_Yaw_Speed_pid);
				PID_Clear(&Leg_theta_Harmonize_pid[0]);
				Robot_Status.Param_Switch = 1;
				Status_Counter [0] = 0;//清空计时器
				Status_Counter [1] = 0;
			}
	}
	
	if(Robot_Status.Body.Body_Upright_Status == NO)
	{
		
		if((Ins_Data.Pitch > -0.05f) && (Ins_Data.Pitch < 0.05f))
		{
			if(Status_Counter [1]!=1)//确认定时器未被启动
			{
				HAL_TIM_Base_Start_IT (&htim5);//开启状态切换计时器
				Status_Counter [1] = 1;
			}
		}
		else
		{
			HAL_TIM_Base_Stop_IT(&htim5);//关闭计时器
			Status_Counter [0] = 0;//清空状态切换计时器
			Status_Counter [1] = 0;
		}
		
		if(Status_Counter [0]> 300)
		{
			Robot_Status.Body.Body_Upright_Status = YES;//将车体直立状态位设为YES
			HAL_TIM_Base_Stop_IT(&htim5);//关闭计时器
			Robot_Status.Param_Switch = 1;
			Status_Counter [0]= 0;//清空状态切换计时器
			Status_Counter [1]= 0;
			
		}
		
	}
}

void Robot_Param_Switch (void)
{
	if(Robot_Status .Param_Switch == 1)
	{
		if(Robot_Status.Body.Body_Upright_Status == NO)
		{
			//PID_Init(&Balance_fai_Position_pid,130.0f,0,0,6,0,1000,10000,500,POSITIONAL);
			Robot_Status .Param_Switch =0;
		}
		else if(Robot_Status.Body.Body_Upright_Status == YES)
		{
			//PID_Init(&Balance_fai_Position_pid,25.37f,0,0,6,0,1000,10000,500,POSITIONAL);
			Robot_Status .Param_Switch =0;
		}
	}
}

/**
 * @brief 机器人进程监控函数
 * @param NONE
 * @retval NONE
*/
void Robot_Task_Monitor(void)
{
	


}