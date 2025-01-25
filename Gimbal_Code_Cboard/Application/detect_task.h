/**
  ******************************************************************************
  * @file    detect_task.h
  * @author  北京工业大学-贾桐
  * @version V1.0
  * @date    2019/3/1
  * @brief   监视任务头文件
  ******************************************************************************
  * @attention
  *
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/




void Detect_Task(void const * argument);
void Buzzer_on(void);
void Buzzer_off(void);
void Facility_Frequency(void);
void Uart_Frequency(void);
u8 Remote_Detect(void);
u8 Remote_Value_Detect(u16 Channel);

extern u16 Frequency[10];
extern u16 RefereeCount;
extern u16 UartFrequency[4];

