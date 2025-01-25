/**
  ******************************************************************************
  * @file    pid.c
  * @author  Tinker.Jia
  * @version V1.1
  * @date    2018/11/12
  * @brief   PID计算函数
  ******************************************************************************
  * @attention
  *	V1.1 删除了PID中的过零处理，并对PID函数进行了优化
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "Header.h"

PidTypeDef Ammo_Speed_pid[2] = {0};
PidTypeDef Gimbal_Speed_pid[3]={0};
PidTypeDef Gimbal_Position_pid[3]={0};
PidTypeDef Gimbal_TCurrent_pid[3]={0};

/**
  * @brief  PID参数的初始化
  * @param  参数太多了，就是一些基本设置
  * @retval None
  */
void My_PID_Init(PidTypeDef * pid,float kp,float ki,float kd,float max_out,float dead_band,float i_band,float max_input,float i_max_out,u8 mode)
{
	pid->Kp=kp;
	pid->Ki=ki;
	pid->Kd=kd;
	pid->Max_out=max_out;
	pid->I_max_out=i_max_out;
	pid->Dead_band=dead_band;
	pid->Intergral_band=i_band;
	pid->Max_input=max_input;
	//PID输出值
	pid->Input=0;
	pid->Output=0;
	pid->Intergral=0;
	pid->Mode=mode;
	//误差初始化
	pid->E[0]=0;
	pid->E[1]=0;
	pid->E[2]=0;//2最新 1上一次 0上上次
	pid->D_last=0;
}


/**
  * @brief  PID计算
  * @param  输入、反馈
  * @retval None
  */
void PID_Calc(PidTypeDef * pid, float rel_val, float set_val)
{
	float p = 0,//设定计算的PID值
				i = 0,
				d = 0;
	pid->E[2]=Func_Limit(set_val,pid->Max_input,-(pid->Max_input))-rel_val;//当前误差	
	
	if(pid==&Gimbal_Position_pid[YAW]||pid==&Gimbal_Position_pid[2])//过零处理，无论怎么转，都走劣弧，需要最大输入为码盘最大编码
	{
		if(pid->E[2]>4096)
			pid->E[2]=pid->E[2]-8192;
		else if((-pid->E[2])>4096)
			pid->E[2]=8192+pid->E[2];
	}

	if(Func_Abs(pid->E[2]) >=  pid->Dead_band)//当偏差值大于等于死区值，进入这个if
	{
		if(pid->Mode==Positional)
		{	
			if(Func_Abs(pid->E[2]) <= pid->Intergral_band)             //当偏差小于积分范围的时候，进入积分  注意！只有位置式PID有积分范围
				pid->Intergral =pid->Intergral+ (pid->Ki) * (pid->E[2]);//积分值
			else
				{pid->Intergral=pid->Intergral;}                
			pid->Intergral=Func_Limit(pid->Intergral,pid->I_max_out,-(pid->I_max_out));//积分限幅			
  		p = pid->Kp * (pid->E[2]);
			i = pid->Intergral;
			d = pid->Kd * (pid->E[2]-pid->E[1]);
			pid->D_last=d;
			pid->Output=p+i+d;
		}
		else if(pid->Mode==Incremental)
		{
			p=pid->Kp*((pid->E[2]-pid->E[1]));
			i=pid->Ki*pid->E[2];
			d=pid->Kd*(pid->E[2]-pid->E[1]*2+pid->E[0]);
			pid->Output+=p+i+d;
		}
//是否超出最大输出
		pid->Output=Func_Limit(pid->Output,pid->Max_out,-(pid->Max_out));

	}
	else
		pid->Output=0;	
	/*迭代误差*/
	pid->E[0] = pid->E[1];
	pid->E[1] = pid->E[2];
}

/**
  * @brief  底盘跟随PD计算
  * @param  输入、反馈
  * @retval None
  */
void Chassis_Follow_PD_Calc(PidTypeDef * pid, float real_val, float set_val)
{
	float Abs_Error = 0,
				p = 0,//设定计算的PID值
				d = 0;

	pid->E[2] = Func_Limit(set_val,pid->Max_input,-(pid->Max_input))-real_val;//当前误差	
	if(pid->E[2]>4096)
		pid->E[2]=pid->E[2]-8192;
	else if((-pid->E[2])>4096)
		pid->E[2]=8192+pid->E[2];
	Abs_Error = Func_Abs(pid->E[2]);
	if(Abs_Error >  3300)
	{
		pid->Kp = 7;
		pid->Kd = 0;
	}
	else if(Abs_Error >  1300)
	{
		pid->Kp = 5;
		pid->Kd = 150;
	}
	else if(Abs_Error >  600)
	{
		pid->Kp = 5;
		pid->Kd = 200;
	}
	else 
	{
		pid->Kp = 5;			
		pid->Kd = 220;
	}			
						
	p = pid->Kp * (pid->E[2]);
	d = pid->Kd * (pid->E[2]-pid->E[1]);
	pid->Output = p+d;
	pid->Output = Func_Limit(pid->Output,pid->Max_out,-(pid->Max_out));
	
	pid->E[0] = pid->E[1];
	pid->E[1] = pid->E[2];
}


/**
  * @brief  PID缓存清空
  * @param  要清空的地址
  * @retval None
  */
void PID_Clear(PidTypeDef * pid)
{
	pid->E[0] = 0;
	pid->E[1] = 0;
	pid->E[2] = 0;
	pid->Intergral = 0;
	pid->Output = 0;
}

