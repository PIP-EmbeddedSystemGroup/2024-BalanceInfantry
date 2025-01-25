/**
  ******************************************************************************
  * @file    shoot_task.c
  * @author  北京工业大学-张曦梁
  * @version V1.0
  * @date    2021/11/21
  * @brief   发射进程
  ******************************************************************************
  * @attention
  *
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "Header.h"
Motor_t FRI3508[2] = {0};
Motor_t GM2006 = {0};
Shooting_t Shooting = {0};
uint16_t motor_pos[80] = {0};   
uint16_t reversal_time_count = 0;

void Shoot_Task(void const * argument)
{
	portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();

  while(1)
  {
		osDelayUntil(&xLastWakeTime,1);//1000HZ
		
    Shooting_Control();
		
		Ammo_PID_Control();
		
		Feed_PID_Control();
		
	  (RC_Ctl.rc.s2 == 3) ? Ammo_Output(0,0,0) : Ammo_Output(Ammo_Speed_pid[FRI_LEFT].Output,Ammo_Speed_pid[FRI_RIGHT].Output,Gimbal_Speed_pid[TRIGGER].Output);
	}
		
}

/**
  * @brief  舵机启动
  * @param  舵机启动参数
  * @retval None
  */
void Steering_Engine_ON(void)
{
  TIM8->CCR2 = 11;
	Command.StorageFlag    = 0;
}

void Steering_Engine_OFF(void)
{
	Command.StorageFlag  = 1;
	TIM8->CCR2 = 33;
}

void Friction_ON(s16 speed)
{
	FRI3508[0].Speed.Set = - speed;
	FRI3508[1].Speed.Set = speed;
	Shooting.Speed = 15000;
}

void Friction_OFF(void)
{
	FRI3508[0].Speed.Set = 0;
	FRI3508[1].Speed.Set = 0;
	Shooting.Speed = 0;
}

void Ammo_PID_Control(void)
{
	for(u8 pidCount = 0;pidCount < 2;pidCount++)
	{
		Func_Ramp(FRI3508[pidCount].Speed.Set,&FRI3508[pidCount].Speed.Step,30);
		PID_Calc(&Ammo_Speed_pid[pidCount],FRI3508[pidCount].Speed.Real,FRI3508[pidCount].Speed.Step);//转向电机带斜坡启动
	}	 
}

void Shooting_Control(void)
{
//记录历史数据
	motor_pos[79] = GM2006.Position.Convert;
	for(uint8_t i = 0;i < 79;i++)
	{
		motor_pos[i] = motor_pos[i+1];
	} 
    
  if(Shooting.Friction == 1)
	{
		Friction_ON(7050); 
//	Shoot_Miss_Detect(5800);
	}
	else
		Friction_OFF(); 
	
//判断热量是否允许射击
//	if(InfantryJudge.Shoot17mm1CoolingLimit - InfantryJudge.Real17mm1Heating >= 30 && Shooting.Friction == 1)
	if(Shooting.Friction == 1)
    {
		Shooting.Allow = OK;
    }
	else
	{
		Shooting.Allow = NO;
		Shooting.PosSetCount = 0;
	}
	
	if((Shooting.PosSetCount != 0) && (Shooting.Allow == OK))//发射子弹
	{
		if(reversal_time_count == 0)
		{
			Shooting.PosSetCount--;
			GM2006.Position.Set -= Shooting.PosRatio;   //正反转
		}
		if(motor_pos[79] == motor_pos[0] && (Shooting.PosSetCount < Shooting.Period * 0.47))//出现了不动的情况认为卡子弹了
		{
      GM2006.Position.Step = GM2006.Position.Convert;
      GM2006.Position.Set  = GM2006.Position.Convert;
      reversal_time_count  = Shooting.Period * 0.47f;
		}		
	}
	
	if(reversal_time_count != 0)//卡子弹的话就反转
	{
		reversal_time_count--;
		GM2006.Position.Set += Shooting.PosRatio; //正反转
	}	
}

void Feed_PID_Control(void)
{
		GM2006.Position.Set = Func_ValueRannge(GM2006.Position.Set,8192,0);//限制输入范围
		//Func_CircleRamp(GM2006.Position.Set,&GM2006.Position.Step,TRIGGER_POSITION_RAMP_RATIO);//斜坡启动函数
		PID_Calc(&Gimbal_Position_pid[TRIGGER],GM2006.Position.Convert,GM2006.Position.Set);//位置环PID
		PID_Calc(&Gimbal_Speed_pid[TRIGGER],GM2006.Speed.Real,Gimbal_Position_pid[TRIGGER].Output);//速度环PID	
}
