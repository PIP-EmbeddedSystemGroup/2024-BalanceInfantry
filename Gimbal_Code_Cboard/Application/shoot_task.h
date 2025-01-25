/**
  ******************************************************************************
  * @file    shoot_task.c
  * @author  北京工业大学-张曦梁
  * @version V1.0
  * @date    2021/11/21
  * @brief   发射进程头文件
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
	u8 Last_s1;//上一次拨杆值得储存
	u8 Friction;//摩擦轮开启状态标志，1为开启，0为关闭
	u8 PosSetCount;//用来区别单发与连发
	s8 PosRatio;//斜坡斜率
	u8 Status;//发射状态
	u8 Allow;//是否允许发射
	u16 Period;//发射周期
	s16 RemainHeating;//剩余热量
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

