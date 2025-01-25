/**
  ******************************************************************************
  * @file    universal.c
  * @author  北京工业大学-贾桐
  * @version V1.0
  * @date    2018/11/8
  * @brief   通用函数
  ******************************************************************************
  * @attention
  * 这里打算写个通用函数，方便以后用做其他的程序上，直接拷贝就OK啦！
  *	2019年3月12日进行更新，圆圈步进函数进行了BUG修复，在临近值产生震荡问题已修复，但是代码过长，正考虑如何修改。
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "Header.h"

/**
  * @brief  斜坡函数，用于步进式PID
  * @param  input rev step
  * @retval 直接通过指针修改输入量数值
  */
void Func_Ramp(s16 input,s16 *rev,s16 step)
{
	switch(Func_Signal(input - *rev))
	{
		case 1:
			if(input - *rev > step)
				*rev += step;
			else
				*rev = input;
			break;
		case -1:
			if(*rev - input > step)
				*rev -= step;
			else
				*rev = input;		
			break;
		default:
			break;
	}
}

/**
  * @brief  浮点数斜坡
  * @param  input rev step
  * @retval 直接通过指针修改输入量数值
  */
void Func_FRamp(s16 input,float *rev,float step)
{
	switch(Func_Signal(input - *rev))
	{
		case 1:
			if(input - *rev > step)
				*rev += step;
			else
				*rev = input;
			break;
		case -1:
			if(*rev - input > step)
				*rev -= step;
			else
				*rev = input;		
			break;
		default:
			break;
	}
}

/**
  * @brief  有零点的斜坡函数，用于步进式PID
  * @param  input rev step
  * @retval 直接通过指针修改输入量数值
  */

void Func_CircleRamp(s16 input,s16 *rev,s16 step)
{
	if(Func_Abs(input-*rev) > 4096)
	{
		switch(Func_Signal(input - *rev))
		{
			case 1:
				if(8192 + *rev -input > step)
					*rev -= step;
				else
					*rev = input;
				break;
			case -1:
				if(8192 + input - *rev > step)
					*rev += step;
				else
					*rev = input;		
				break;
			default:
				break;
		}
	}
	else
	{
		switch(Func_Signal(input - *rev))
		{
			case 1:
				if(input - *rev > step)
					*rev += step;
				else
					*rev = input;
				break;
			case -1:
				if(*rev - input > step)
					*rev -= step;
				else
					*rev = input;		
				break;
			default:
				break;
		}
	}
	*rev = Func_ValueRannge(*rev,8192,0);
}
/**
  * @brief  求取数值的正负
  * @param  value
  * @retval 正负
  */
s8 Func_Signal(s16 value)
{
	if(value < 0)
		return -1;
	else
		return 1;
}

/**
  * @brief  求取数值的绝对值
  * @param  value
  * @retval 绝对值
  */
float Func_Abs(float value)
{
	if(value >= 0)
		return value;
	else 
		return -value;	
}

/**
  * @brief  对编码器数值进行限幅
  * @param  value 给定值
  * @param  mode  1是外圈模式（数值在最大值和最小值之外） 0是内圈模式（数值在最小值和最大值之间）
  * @retval 限幅值
  */
float Func_Ramp_Limit(float value,float max,float min,u8 mode)
{
//	value = Func_ValueRannge(value,8192,0);
	if(mode)
	{
		if(value < min && value > max)
		{
			if(value >  ((max + min) / 2))
				return min;
			else
				return max;
		}
	}
	else
	{
		if(((max + min) / 2) > 4096)
		{
			if((value < min) && (value > ((max + min) / 2 - 4096)))
				return min;
			else if(value < ((max + min) / 2 - 4096) || value > max)
				return max;
		}
		else
		{
			if((value > max) && (value < ((max + min) / 2 + 4096)))
				return max;
			else if(value > ((max + min) / 2 + 4096) || value < min)
				return min;			
		}
	}
	return value;	
}

/**
  * @brief  对数值进行限幅
  * @param  value
  * @retval 限幅值
  */
float Func_Limit(float value,float max,float min)
{
	if(value>max)
		return value=max;
	else if(value<min)
		return value=min;
	else
		return value;
} 

/**
  * @brief  对数值范围进行处理
  * @param  value
  * @retval 
  */
s16 Func_ValueRannge(s16 value,s16 max,s16 min)
{
	if(value>max)
		value = value % max;
	else if(value<min)
		value = value - min + max;
	return value;
}

//四阶巴特沃斯滤波器
const float b[5]={0.0009334986129548445,0.003733994451819,0.005600991677729,0.003733994451819,0.0009334986129548445};
const float a[5]={1,-2.976844333696732,3.422309529377639,-1.786106600218039,0.355577382344410};

float xBuf[5]={0};
float yBuf[5]={0};

float IIRLowPass(float x)
{
	int i;
	//运算之前Buf向前移动一个位置，以保存之前Buf的数据；
	for(i=4; i>0;i--)
	{
		yBuf[i] = yBuf[i-1];
		xBuf[i] = xBuf[i-1];
	}
	xBuf[0] = x;
	yBuf[0] = 0;
	for(i=1;i<5;i++)
	{
		yBuf[0] = yBuf[0] + b[i]*xBuf[i];
		yBuf[0] = yBuf[0] - a[i]*yBuf[i];
	}
	yBuf[0] = yBuf[0] + b[0]*xBuf[0];
	return yBuf[0];
}
