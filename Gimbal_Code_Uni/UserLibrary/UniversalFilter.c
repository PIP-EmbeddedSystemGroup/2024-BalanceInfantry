/**
 * @attention   采用UTF-8字符集编码
 * @brief       通用滤波器库
 * @authors     北工大PIP战队 樊郡捷
 * @version     v1.1
 * @date        2025-02-27
 * @details     实现了通用的滤波器
 */

#include "header.h"

 /// @brief 一阶IIR低通滤波器初始化
 /// @param filter       滤波器类
 /// @param deltaTime    采样周期
 /// @param cutOffFreq   截止频率
void LPF_1stOrderIIR_Init(LPF_1stOrderIIR_t* filter, float deltaTime, float cutOffFreq)
{
    filter->DeltaTime = deltaTime;
    filter->Factor = filter->DeltaTime * 2 * PI * cutOffFreq;

    filter->Output = 0;
}

void LPF_1stOrderIIR_Update(LPF_1stOrderIIR_t* filter, float value)
{
    if (filter->Factor > 1) //不满足采样定理
        filter->Output = value;

    filter->Output = filter->Factor * value + (1 - filter->Factor) * filter->Output;
}

//一阶卡尔曼滤波器
//R固定，Q越大，代表越信任侧量值，Q无穷代表只用测量值
void KalmanFilter1st_Init(KalmanFilter1st_t* filter, float t_q, float t_r)
{
    filter->X_Last = 0;
    filter->P_Last = 0;
    filter->Q = t_q;
    filter->R = t_r;
    filter->X_Mid = filter->X_Last;
}

float KalmanFilter1st_Update(KalmanFilter1st_t* filter, float data)
{
    filter->X_Mid = filter->X_Last;                                         //百度对应公式(1)   x(k|k-1) = A*X(k-1|k-1)+B*U(k)+W(K)
    filter->P_Mid = filter->P_Last + filter->Q;                             //百度对应公式(2)   p(k|k-1) = A*p(k-1|k-1)*A'+Q
    filter->Kg = filter->P_Mid / (filter->P_Mid + filter->R);               //百度对应公式(4)   kg(k) = p(k|k-1)*H'/(H*p(k|k-1)*H'+R)
    filter->X_Now = filter->X_Mid + filter->Kg * (data - filter->X_Mid);    //百度对应公式(3)   x(k|k) = X(k|k-1)+kg(k)*(Z(k)-H*X(k|k-1))
    filter->P_Now = (1 - filter->Kg) * filter->P_Mid;                       //百度对应公式(5)   p(k|k) = (I-kg(k)*H)*P(k|k-1)
    filter->P_Last = filter->P_Now;                                         //状态更新
    filter->X_Last = filter->X_Now;
    return filter->X_Now;                                                   //输出预测结果x(k|k)
}

/// @brief 实现了入队自动迭代队列整体的均值 减小计算直流分量时的计算量
/// @param queue 队列对象
/// @param Len   队列长度
/// @param enterWithShift 是否需要在入队时插入到第一个元素 而非做为循环队列 >0头插 <0尾差
void MeanQueue_Init(MeanQueue_t* queue, unsigned len, int enterWithShift)
{
    queue->Len = len;
    queue->Index = 0;
    queue->Data = (float*)pvPortMalloc(sizeof(float) * queue->Len);
    for (int i = 0; i < queue->Len; i++)
        queue->Data[i] = 0.f;
    queue->Mean = 0;
    queue->EnterWithShift = enterWithShift;
}

void MeanQueue_Clear(MeanQueue_t* queue, float data)
{
    queue->Index = 0;
    for (int i = 0; i < queue->Len; i++)
        queue->Data[i] = data;
    queue->Mean = data;
}

void MeanQueue_Enter(MeanQueue_t* queue, float data)
{
    if (queue->EnterWithShift > 0)
    {
        queue->Mean = queue->Mean + (data - queue->Data[queue->Len - 1]) / queue->Len;
        for (int i = queue->Len - 1; i >= 1; i--)
            queue->Data[i] = queue->Data[i - 1];
        queue->Data[0] = data;
    }
    else if (queue->EnterWithShift < 0)
    {
        queue->Mean = queue->Mean + (data - queue->Data[0]) / queue->Len;
        for (int i = 0; i < queue->Len - 1; i++)
            queue->Data[i] = queue->Data[i + 1];
        queue->Data[queue->Len - 1] = data;
    }
    else
    {
        queue->Mean = queue->Mean + (data - queue->Data[queue->Index]) / queue->Len;
        queue->Data[queue->Index++] = data;
        if (queue->Index >= queue->Len)
            queue->Index = 0;
    }
}
