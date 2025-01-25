/**
 ******************************************************************************
 * @file    remote_task.c
 * @author  ������ҵ��ѧ-��ͩ
 * @version V1.0
 * @date    2019/3/1
 * @brief   ң�ؽ���ͷ�ļ�
 ******************************************************************************
 * @attention
 *
 *
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/

#include "universal.h"
#define MANUAL_ATTACK 0
#define AUTO_ATTACK 1

#define ATTACK_NORMAL_ARMOR 0
#define ARMOR_ROTATE 1
#define ARMOR_NONE_PREDICT 2

typedef struct
{
  float SpeedZoom;              // ctrl shift���ٶȵĿ�����
  uint8_t Chassis_Power_Switch; // 底盘开关控制位
  u8 RotateFlag;                // ���ݱ�־λ
  u8 StorageFlag;               // ���ֱ�־λ
  u8 AimAssitFlag;              // ������׼��־λ
  u8 UIFlag;                    // �û�UIˢ�±�־λ
  u8 GimbalFollowOFF;           // ��̨���濪��
  u8 Height_Set;                // ���̸߶ȱ�־λ
  u8 Cap_Flag;                  // �������ݿ��ر�־λ
  s8 TriggerAnti;               // ���ֵ����ת
	u8 SideWay_Set;				//左右侧身控制位，左右PEEK
	
  float Mouse_x;
  float Mouse_y;
  s16 Vx;
} Command_t; // ģʽ����

void Remote_Task(void const *argument);
void Remote_Control(void);
void Computer_control(void);
void Mouse_Control(void);
u8 Key_Press(u16 key[], u16 SetTime);
void Key_Deal(void);

extern Command_t Command;
