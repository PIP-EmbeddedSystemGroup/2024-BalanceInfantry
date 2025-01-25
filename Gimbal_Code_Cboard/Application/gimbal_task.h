/**
 ******************************************************************************
 * @file    gimbal_task.c
 * @author  ������ҵ��ѧ-��ͩ
 * @version V1.0
 * @date    2019/3/1
 * @brief   ��̨����ͷ�ļ�
 ******************************************************************************
 * @attention
 *
 *
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/

#include "universal.h"
#define NORMAL_PI 3.14159265f
typedef struct // ������ݽṹ��
{
	struct
	{
		s16 Set;  // �ٶ��趨ֵ
		s16 Real; // �ٶ���ʵֵ
		s16 Step; // �ٶ�б���趨ֵ
	} Speed;
	struct
	{
		s16 Set;		 // λ���趨ֵ
		s16 Real;		 // λ����ʵֵ
		s16 Step;		 // λ��б���趨ֵ
		s16 LastReal;	 // �ϴε���ʵֵ
		s16 Convert;	 // ���Ա��������ת��ֵ
		s16 InitConvert; // �ϵ����λ��ת��ֵ
		s8 Count;		 // ��Ա�����Ȧ������
		s8 Flag;		 // ��־λ
	} Position;
	s16 TCurrent;	 // ���ת�أ�������ת��ֵ�����嵥λ��δ֪
	s16 Temperature; // ʵ���¶�ֵ
	s16 Rotate;		 // ת������
	s16 Output;
} Motor_t;

typedef struct
{
	u16 Max;
	u16 Min;
	u16 Dynamic_Max;
	u16 Dynamic_Min;
} AutoLimit_t;

void Gimbal_Task(void const *argument);
void Gimbal_Dynamic_Limit(s16 *input, u16 encoder, u16 imu, AutoLimit_t *autolimit);
void PITCH_PID_Control(void);
void Feed_Control(void);
void Feed_PID_Control(void);
void Gimbal_Disable(void);
void YAW_PID_WithIMU(void);

extern Motor_t GM6020[2];
extern s16 YAW_OUTPUT;
