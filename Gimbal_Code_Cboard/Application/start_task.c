/**
  ******************************************************************************
  * @file    start_task.c
  * @author  北京工业大学-贾桐
  * @version V1.0
  * @date    2019/3/1
  * @brief   初始化任务函数
  ******************************************************************************
  * @attention
  *
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "Header.h"
osThreadId Detect_TaskHandle;
osThreadId Shoot_TaskHandle;
osThreadId Remote_TaskHandle;
osThreadId INSTaskHandle;
osThreadId Gimbal_TaskHandle;
osThreadId Com_TaskHandle;
extern osThreadId Start_TaskHandle;



void Start_Task(void)
{
	portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
	taskENTER_CRITICAL();//进入临界区
	//创建监视进程
	osThreadDef(Detect_Task, Detect_Task, osPriorityBelowNormal, 0, 128);
  Detect_TaskHandle = osThreadCreate(osThread(Detect_Task), NULL);
	
	taskEXIT_CRITICAL();            //退出临界区

	osDelayUntil(&xLastWakeTime,100);
	
	INS_Init();
	
	taskENTER_CRITICAL();//进入临界区
	
	//创建BMI088进程
	osThreadDef(INSTask, INS_Task, osPriorityHigh, 0, 1024);
  INSTaskHandle = osThreadCreate(osThread(INSTask), NULL);
	
	//创建遥控进程
	osThreadDef(Remote_Task, Remote_Task, osPriorityHigh, 0, 256);
  Remote_TaskHandle = osThreadCreate(osThread(Remote_Task), NULL);
	
	//创建云台进程
	osThreadDef(Gimbal_Task, Gimbal_Task, osPriorityNormal, 0, 512);
  Gimbal_TaskHandle = osThreadCreate(osThread(Gimbal_Task), NULL);
	
	//创建数据传输进程
	osThreadDef(Com_Task, Com_Task, osPriorityNormal, 0, 256);
  Com_TaskHandle = osThreadCreate(osThread(Com_Task), NULL);

  //创建射击进程
	osThreadDef(Shoot_Task, Shoot_Task, osPriorityNormal, 0, 256);
  Shoot_TaskHandle = osThreadCreate(osThread(Shoot_Task), NULL);
	
	User_Init();
	vTaskDelete(Start_TaskHandle);	//删除初始化任务
	taskEXIT_CRITICAL();            //退出临界区
}

void User_Init(void)
{
	RC_Ctl.rc.ch0 = 1024;
	RC_Ctl.rc.ch1 = 1024;
	RC_Ctl.rc.ch2 = 1024;
	RC_Ctl.rc.ch3 = 1024;	
	RC_Ctl.rc.s1  = 3;
	RC_Ctl.rc.s2  = 3;

	convertflag[0] = 0;
  convertflag[1] = 0;
	
	Shooting.RemainHeating          = 360;
	Shooting.Period                 = 64;//此数值最大80，射频 = 1000 / 周期
	Shooting.PosRatio               = 16;
	
  Uart_Data.Yaw_Output = 0;
	Uart_Data.Pitch_Position = PITCH_POSITION_INIT;

	Command.SpeedZoom = 1;

	//PID初始化分别为KP KI KD 最大输出 死区 积分范围 最大输入 最大积分值（输出值=最大积分值*KI）
	//注意！！！！！！
	//积分范围只有在位置式PID计算的时候起作用 
	My_PID_Init(&Ammo_Speed_pid[0],9,0.05f,0,10000,0,8000,CHASSIS_MAX_SPEED,300,POSITIONAL);
	My_PID_Init(&Ammo_Speed_pid[1],9,0.05f,0,10000,0,8000,CHASSIS_MAX_SPEED,300,POSITIONAL);
	
	My_PID_Init(&Gimbal_Position_pid[PITCH],0.18f,0,0,10000,0,4000,8192,10,POSITIONAL);
	My_PID_Init(&Gimbal_Speed_pid[PITCH],280,1.0f,0,30000,0,200,250,700,POSITIONAL);
	
	My_PID_Init(&Gimbal_Position_pid[YAW],0.018f,0,0,10000,0,4000,8192,10,POSITIONAL);
	My_PID_Init(&Gimbal_Speed_pid[YAW],20000,20.0f,0,30000,0,200,250,2500,POSITIONAL);
	
	//拨弹
  My_PID_Init(&Gimbal_Speed_pid[TRIGGER],12,0.01f,0,10000,0,10000,10000,800,POSITIONAL);
  My_PID_Init(&Gimbal_Position_pid[TRIGGER],6,0,0,10000,0,0,8192,2000,POSITIONAL);

}

