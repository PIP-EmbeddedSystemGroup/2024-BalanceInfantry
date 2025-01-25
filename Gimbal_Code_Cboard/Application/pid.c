/**
  ******************************************************************************
  * @file    pid.c
  * @author  Tinker.Jia
  * @version V1.1
  * @date    2018/11/12
  * @brief   PID���㺯��
  ******************************************************************************
  * @attention
  *	V1.1 ɾ����PID�еĹ��㴦������PID�����������Ż�
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
  * @brief  PID�����ĳ�ʼ��
  * @param  ����̫���ˣ�����һЩ��������
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
	//PID���ֵ
	pid->Input=0;
	pid->Output=0;
	pid->Intergral=0;
	pid->Mode=mode;
	//����ʼ��
	pid->E[0]=0;
	pid->E[1]=0;
	pid->E[2]=0;//2���� 1��һ�� 0���ϴ�
	pid->D_last=0;
}


/**
  * @brief  PID����
  * @param  ���롢����
  * @retval None
  */
void PID_Calc(PidTypeDef * pid, float rel_val, float set_val)
{
	float p = 0,//�趨�����PIDֵ
				i = 0,
				d = 0;
	pid->E[2]=Func_Limit(set_val,pid->Max_input,-(pid->Max_input))-rel_val;//��ǰ���	
	
	if(pid==&Gimbal_Position_pid[YAW]||pid==&Gimbal_Position_pid[2])//���㴦��������ôת�������ӻ�����Ҫ�������Ϊ����������
	{
		if(pid->E[2]>4096)
			pid->E[2]=pid->E[2]-8192;
		else if((-pid->E[2])>4096)
			pid->E[2]=8192+pid->E[2];
	}

	if(Func_Abs(pid->E[2]) >=  pid->Dead_band)//��ƫ��ֵ���ڵ�������ֵ���������if
	{
		if(pid->Mode==Positional)
		{	
			if(Func_Abs(pid->E[2]) <= pid->Intergral_band)             //��ƫ��С�ڻ��ַ�Χ��ʱ�򣬽������  ע�⣡ֻ��λ��ʽPID�л��ַ�Χ
				pid->Intergral =pid->Intergral+ (pid->Ki) * (pid->E[2]);//����ֵ
			else
				{pid->Intergral=pid->Intergral;}                
			pid->Intergral=Func_Limit(pid->Intergral,pid->I_max_out,-(pid->I_max_out));//�����޷�			
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
//�Ƿ񳬳�������
		pid->Output=Func_Limit(pid->Output,pid->Max_out,-(pid->Max_out));

	}
	else
		pid->Output=0;	
	/*�������*/
	pid->E[0] = pid->E[1];
	pid->E[1] = pid->E[2];
}

/**
  * @brief  ���̸���PD����
  * @param  ���롢����
  * @retval None
  */
void Chassis_Follow_PD_Calc(PidTypeDef * pid, float real_val, float set_val)
{
	float Abs_Error = 0,
				p = 0,//�趨�����PIDֵ
				d = 0;

	pid->E[2] = Func_Limit(set_val,pid->Max_input,-(pid->Max_input))-real_val;//��ǰ���	
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
  * @brief  PID�������
  * @param  Ҫ��յĵ�ַ
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

