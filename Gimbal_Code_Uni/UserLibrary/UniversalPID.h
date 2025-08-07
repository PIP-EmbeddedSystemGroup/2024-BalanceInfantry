#ifndef _PID_H_
#define _PID_H_

/**
 * @attention   采用UTF-8字符集编码
 */

#include "stdint.h"

/// @brief 使初始化参数书写格式更加清晰
typedef struct
{
    float kP;           //比例项系数
    float kI;           //积分项系数
    float kD;           //微分项系数
    float DeltaTime;    //控制器更新时间 单位:s
    int DifferentialFreqDiv;    //微分更新时间分频系数

    float CircleResolution; //位置环存在角度量过零点问题时 角度一圈所对应的数值大小

    float MaxError;     //误差限幅
    float MaxOutput;    //输出限幅
    float I_Max;        //积分限幅

    float DeadBand;     //控制死区
    float IntegralBand; //积分区间
} PID_InitStruct_t;

/// @brief PID结构体
typedef struct
{
    float kP;
    float kI_mdt;       //mdt: Multiplied by Delta Time -> 一般会省略
    float kD_ddt;       //ddt: Divided by Delta Time    -> 一般会省略
    float DeltaTime;
    int DifferentialFreqDiv;    //微分更新时间分频系数
    int DifferentialCounter;

    float Error[2];     //0:本次误差 1:上次误差

    float P;            //比例项
    float I;            //积分项
    float D;            //微分项

    float CircleResolution; //位置环存在角度量过零点问题时 角度一圈所对应的数值大小

    float MaxError;
    float MaxOut;
    float I_Max;

    float DeadBand;
    float IntegralBand;

    float Output;
} PID_t;

void PID_Init(PID_t* pid, PID_InitStruct_t* pidParam);
void PID_Calc(PID_t* pid, float real, float set);
void PID_AngleCalc(PID_t* pid, float real, float set);
void PID_Clear(PID_t* pid);

#endif
