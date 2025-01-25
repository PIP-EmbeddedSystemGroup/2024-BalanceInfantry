/**
  ******************************************************************************
  * @file    universal.h
  * @author  ������ҵ��ѧ-��ͩ
  * @version V1.0
  * @date    2019/3/1
  * @brief   ͨ�ú���ͷ�ļ�
  ******************************************************************************
  * @attention
  *
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stdint.h"

int8_t Func_Signal(int16_t value);
void Func_Ramp(int16_t input,int16_t *rev,int16_t step);
void Func_FRamp(float input,float *rev,float step);
void Func_CircleRamp(int16_t input,int16_t *rev,int16_t step);
float Func_Abs(float value);
float Func_Limit(float value,float max,float min);
float Func_Ramp_Limit(float value,float max,float min,uint8_t mode);
int16_t Func_ValueRannge(int16_t value,int16_t max,int16_t min);
int8_t Func_MaxRange(int8_t value,int8_t max);
int float_rounding(float raw);
