/**
  ******************************************************************************
  * @file    start_task.c
  * @author  ������ҵ��ѧ-��ͩ
  * @version V1.0
  * @date    2019/3/1
  * @brief   ��ʼ��������
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
	taskENTER_CRITICAL();//�����ٽ���
	//�������ӽ���
	osThreadDef(Detect_Task, Detect_Task, osPriorityBelowNormal, 0, 128);
  Detect_TaskHandle = osThreadCreate(osThread(Detect_Task), NULL);
	
	taskEXIT_CRITICAL();            //�˳��ٽ���

	osDelayUntil(&xLastWakeTime,100);
	
	INS_Init();
	
	taskENTER_CRITICAL();//�����ٽ���
	
	//����BMI088����
	osThreadDef(INSTask, INS_Task, osPriorityHigh, 0, 1024);
  INSTaskHandle = osThreadCreate(osThread(INSTask), NULL);
	
	//����ң�ؽ���
	osThreadDef(Remote_Task, Remote_Task, osPriorityHigh, 0, 256);
  Remote_TaskHandle = osThreadCreate(osThread(Remote_Task), NULL);
	
	//������̨����
	osThreadDef(Gimbal_Task, Gimbal_Task, osPriorityNormal, 0, 512);
  Gimbal_TaskHandle = osThreadCreate(osThread(Gimbal_Task), NULL);
	
	//�������ݴ������
	osThreadDef(Com_Task, Com_Task, osPriorityNormal, 0, 256);
  Com_TaskHandle = osThreadCreate(osThread(Com_Task), NULL);

  //�����������
	osThreadDef(Shoot_Task, Shoot_Task, osPriorityNormal, 0, 256);
  Shoot_TaskHandle = osThreadCreate(osThread(Shoot_Task), NULL);
	
	User_Init();
	vTaskDelete(Start_TaskHandle);	//ɾ����ʼ������
	taskEXIT_CRITICAL();            //�˳��ٽ���
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
	Shooting.Period                 = 64;//����ֵ���80����Ƶ = 1000 / ����
	Shooting.PosRatio               = 16;
	
  Uart_Data.Yaw_Output = 0;
	Uart_Data.Pitch_Position = PITCH_POSITION_INIT;

	Command.SpeedZoom = 1;

	//PID��ʼ���ֱ�ΪKP KI KD ������ ���� ���ַ�Χ ������� ������ֵ�����ֵ=������ֵ*KI��
	//ע�⣡����������
	//���ַ�Χֻ����λ��ʽPID�����ʱ�������� 
	My_PID_Init(&Ammo_Speed_pid[0],9,0.05f,0,10000,0,8000,CHASSIS_MAX_SPEED,300,POSITIONAL);
	My_PID_Init(&Ammo_Speed_pid[1],9,0.05f,0,10000,0,8000,CHASSIS_MAX_SPEED,300,POSITIONAL);
	
	My_PID_Init(&Gimbal_Position_pid[PITCH],0.18f,0,0,10000,0,4000,8192,10,POSITIONAL);
	My_PID_Init(&Gimbal_Speed_pid[PITCH],280,1.0f,0,30000,0,200,250,700,POSITIONAL);
	
	My_PID_Init(&Gimbal_Position_pid[YAW],0.018f,0,0,10000,0,4000,8192,10,POSITIONAL);
	My_PID_Init(&Gimbal_Speed_pid[YAW],20000,20.0f,0,30000,0,200,250,2500,POSITIONAL);
	
	//����
  My_PID_Init(&Gimbal_Speed_pid[TRIGGER],12,0.01f,0,10000,0,10000,10000,800,POSITIONAL);
  My_PID_Init(&Gimbal_Position_pid[TRIGGER],6,0,0,10000,0,0,8192,2000,POSITIONAL);

}

