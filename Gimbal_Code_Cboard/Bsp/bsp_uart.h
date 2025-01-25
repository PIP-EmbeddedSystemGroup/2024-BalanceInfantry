/**
  ******************************************************************************
  * @file    bsp_uart.h
  * @author  北京工业大学-贾桐
  * @version V1.0
  * @date    2018/11/8
  * @brief   头文件
  ******************************************************************************
  * @attention
 
 
  ******************************************************************************
  */

#include "usart.h"

typedef struct
{
	struct
	{
		uint16_t ch0;//遥控器的五个通道
        uint8_t ch0_switch;
        uint8_t ch0_switch_last;
		uint16_t ch1;
		uint16_t ch2;
		uint16_t ch3;
		uint16_t ch4;
		uint8_t s1;//左右侧拨杆
		uint8_t s2;
	}rc;
	struct
	{
		int16_t x;//鼠标x向移动速度
		int16_t y;//鼠标y向移动速度
		int16_t z;//这个据说是鼠标z向移动速度，但是具体是哪个我也不知道，没测出来
		uint8_t press_l;//鼠标左键
		uint8_t press_r;//鼠标右键
	}mouse;
	struct
	{
		uint8_t v_l;//键盘高八位数据
		uint8_t v_h;//低八位
	}key;
}RC_Ctl_t;

typedef struct//本结构体用来保存对键盘长按，短按的操作
{
	u16 Z1[2];
	u16 X1[2];
	u16 Q[2];
	u16 W[2];
	u16 E[2];
	u16 R[2];
	u16 A[2];
	u16 S[2];
	u16 D[2];
	u16 F[2];
	u16 G[2];
	u16 C[2];
	u16 V[2];
	u16 B[2];
	u16 Shift[2];
	u16 Ctrl[2];
}Key_t;

typedef struct
{
	u8 hitmode;
	s16 Pitch_Position;
	s16 Yaw_Output;
	struct
	{
		s16 Pitch;
		s16 Yaw;
	}Angle;
	struct
	{
		s16 Pitch;
		s16 Yaw;
	}Speed;
}Uart_Data_t;

void Slave_DataReceive(void);
void Remote_DateReceive(void);
void UART_Start(void);
void UART_RxIdleCallback(UART_HandleTypeDef *huart);

extern Key_t Key;
extern RC_Ctl_t RC_Ctl;
extern Uart_Data_t Uart_Data;
