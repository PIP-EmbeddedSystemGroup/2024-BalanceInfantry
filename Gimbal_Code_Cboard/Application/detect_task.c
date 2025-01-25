/**
  ******************************************************************************
  * @file    detect_task.c
  * @author  北京工业大学-贾桐
  * @version V1.0
  * @date    2019/3/1
  * @brief   监视进程
  ******************************************************************************
  * @attention
  *
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "Header.h"


u16 Frequency[10] = {1000,1000,1000,1000,1000,1000,1000,1000,1000,100};
u16 UartFrequency[4] = {1000,1000,1000,1000};
u16 RefereeCount = 40;
void Detect_Task(void const * argument)
{
	portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
  while(1)
  {
		osDelayUntil(&xLastWakeTime,500);//2HZ
		Facility_Frequency();
		Uart_Frequency();
  } 
}




/**
  * @brief  设备帧率检测，掉线则复位控制器
  * @param  None
  * @retval 状态值
  */
void Facility_Frequency(void)
{
//if(Frequency[1] == 0)
//	User.wrongflag[1] = 1;
//if(Frequency[2] == 0)
//	User.wrongflag[2] = 1;
//if(Frequency[3] == 0)
//	User.wrongflag[3] = 1;
//if(Frequency[4] == 0)
//	User.wrongflag[4] = 1;
//if(Frequency[5] == 0)
//	User.wrongflag[5] = 1;
//if(Frequency[6] == 0)
//	User.wrongflag[6] = 1;

//Frequency[1] = 0;
//Frequency[2] = 0;
//Frequency[3] = 0;
//Frequency[4] = 0;
//Frequency[5] = 0;
//Frequency[6] = 0;
}

void Uart_Frequency(void)
{
//	if((UartFrequency[0] == 0)||(UartFrequency[1] == 0)||(UartFrequency[2] == 0)||(UartFrequency[3] == 0))
//	{
//		PowerLimit.Cut = ON;
//	  HAL_NVIC_SystemReset();
//	}
//	
//	UartFrequency[0] = 0;
//	UartFrequency[1] = 0;
//	UartFrequency[2] = 0;
//	UartFrequency[3] = 0;

}
/**
  * @brief  遥控器检测
  * @param  None
  * @retval 状态值
  */
u8 Remote_Detect(void)
{
	u8 Restart = 0;
	Restart += Remote_Value_Detect(RC_Ctl.rc.ch0);
	Restart += Remote_Value_Detect(RC_Ctl.rc.ch1);
	Restart += Remote_Value_Detect(RC_Ctl.rc.ch2);
	Restart += Remote_Value_Detect(RC_Ctl.rc.ch3);
	if(Restart != 0)
		return ERROR;
	else
		return NORMAL;
}

/**
  * @brief  遥控器数值检测
  * @param  通道值
  * @retval 状态值
  */
u8 Remote_Value_Detect(u16 Channel)
{
	if((Channel > 1684) || (Channel < 364))
		return ERROR;
	else
		return NORMAL;
}

void Buzzer_on(void)
{
	TIM12->CCR1 = 500;
}

void Buzzer_off(void)
{
	TIM12->CCR1 = 0;
}
