/**
  ******************************************************************************
  * @file    universal.h
  * @author  北京工业大学-贾桐
  * @version V1.0
  * @date    2019/3/1
  * @brief   通用函数头文件
  ******************************************************************************
  * @attention
  *
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stdint.h"

typedef int32_t  s32;
typedef int16_t s16;
typedef int8_t  s8;

typedef uint32_t  u32;
typedef uint16_t u16;
typedef uint8_t  u8;

s8 Func_Signal(s16 value);
void Func_Ramp(s16 input,s16 *rev,s16 step);
void Func_FRamp(s16 input,float *rev,float step);
void Func_CircleRamp(s16 input,s16 *rev,s16 step);
float Func_Abs(float value);
float Func_Limit(float value,float max,float min);
float Func_Ramp_Limit(float value,float max,float min,u8 mode);
s16 Func_ValueRannge(s16 value,s16 max,s16 min);
