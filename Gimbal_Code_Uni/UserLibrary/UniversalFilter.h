#ifndef _UNIVERSAL_FILTER_H_
#define _UNIVERSAL_FILTER_H_

/**
 * @attention   采用UTF-8字符集编码
 */

#include "stdint.h"

typedef struct
{
    float Factor;
    float DeltaTime;

    float Output;
} LPF_1stOrderIIR_t;   //一阶IIR低通滤波器

typedef struct
{
    float X_Last; //上一时刻的最优结果  X(k-|k-1)
    float X_Mid;  //当前时刻的预测结果  X(k|k-1)
    float X_Now;  //当前时刻的最优结果  X(k|k)
    float P_Mid;  //当前时刻预测结果的协方差  P(k|k-1)
    float P_Now;  //当前时刻最优结果的协方差  P(k|k)
    float P_Last; //上一时刻最优结果的协方差  P(k-1|k-1)
    float Kg;     //kalman增益
    float A;      //系统参数
    float U;
    float Q;
    float R;
} KalmanFilter1st_t;

typedef struct
{
    float* Data;
    unsigned Len;
    unsigned Index;
    float Mean;
    int EnterWithShift;
} MeanQueue_t;

void LPF_1stOrderIIR_Init(LPF_1stOrderIIR_t* filter, float deltaTime, float cutOffFreq);
void LPF_1stOrderIIR_Update(LPF_1stOrderIIR_t* filter, float value);
void KalmanFilter1st_Init(KalmanFilter1st_t* filter, float t_q, float t_r);
float KalmanFilter1st_Update(KalmanFilter1st_t* filter, float data);
void MeanQueue_Init(MeanQueue_t* queue, unsigned len, int enterWithShift);
void MeanQueue_Clear(MeanQueue_t* queue, float data);
void MeanQueue_Enter(MeanQueue_t* queue, float data);

#endif
