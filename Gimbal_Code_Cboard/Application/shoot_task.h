/**
  ******************************************************************************
  * @file    shoot_task.c
  * @author  ������ҵ��ѧ-������
  * @version V1.0
  * @date    2021/11/21
  * @brief   �������ͷ�ļ�
  ******************************************************************************
  * @attention
  *
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/


#include "universal.h"
#define NORMAL_PI 3.14159265f
typedef struct
{
	u8 Last_s1;//��һ�β���ֵ�ô���
	u8 Friction;//Ħ���ֿ���״̬��־��1Ϊ������0Ϊ�ر�
	u8 PosSetCount;//�������𵥷�������
	s8 PosRatio;//б��б��
	u8 Status;//����״̬
	u8 Allow;//�Ƿ�������
	u16 Period;//��������
	s16 RemainHeating;//ʣ������
	u16 Speed;
	u16 PressTime;
}Shooting_t;


void Shoot_Task(void const * argument);
void Friction_ON(s16 speed);
void Friction_OFF(void);
void Steering_Engine_ON(void);
void Steering_Engine_OFF(void);
void Shooting_Control(void);
void Feed_PID_Control(void);
void Ammo_PID_Control(void);

extern Shooting_t Shooting;
extern Motor_t FRI3508[2];
extern Motor_t GM2006;

