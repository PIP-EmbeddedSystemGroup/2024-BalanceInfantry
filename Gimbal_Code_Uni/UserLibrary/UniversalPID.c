/**
 * @attention   采用UTF-8字符集编码
 * @brief       通用PID库
 * @authors     北工大PIP战队 樊郡捷
 * @version     v1.0
 * @date        2024-10-14
 * @details     实现了通用的PID控制器
 *              角度量的位置环实现了解卷绕后转劣弧
 */

#include "header.h"

 /// @brief PID控制器结构体初始化
 /// @param pid      要初始化的PID控制器
 /// @param param    PID控制器参数
void PID_Init(PID_t* pid, PID_InitStruct_t* pidParam)
{
    pid->kP = pidParam->kP;
    pid->kI_mdt = pidParam->kI * pidParam->DeltaTime;
    pid->kD_ddt = pidParam->kD / pidParam->DeltaTime;
    pid->DeltaTime = pidParam->DeltaTime;

    pid->CircleResolution = pidParam->CircleResolution;

    pid->MaxError = pidParam->MaxError;
    pid->MaxOut = pidParam->MaxOutput;
    pid->I_Max = pidParam->I_Max;

    pid->DeadBand = pidParam->DeadBand;
    pid->IntegralBand = pidParam->IntegralBand;

    pid->P = 0;
    pid->I = 0;
    pid->D = 0;
    pid->Error[0] = 0;
    pid->Error[1] = 0;
    pid->Output = 0;
}

/// @brief PID控制器更新计算
/// @param pid  PID控制器
/// @param real 被控量当前的真实值
/// @param set  目标值
void PID_Calc(PID_t* pid, float real, float set)
{
    pid->Error[0] = ConstrainF(set - real, pid->MaxError, -pid->MaxError);

    if (fabsf(pid->Error[0]) < pid->DeadBand)
    {
        pid->Error[1] = 0;
        pid->Output = 0;
        return;
    }

    if (pid->Error[0] > pid->DeadBand)
        pid->Error[0] -= pid->DeadBand;
    else if (pid->Error[0] < -pid->DeadBand)
        pid->Error[0] += pid->DeadBand;

    pid->P = pid->kP * pid->Error[0];
    if (fabsf(pid->Error[0]) <= pid->IntegralBand)
    {
        pid->I += pid->kI_mdt * pid->Error[0];
        pid->I = ConstrainF(pid->I, pid->I_Max, -pid->I_Max);
    }
    pid->D = pid->kD_ddt * (pid->Error[0] - pid->Error[1]);

    pid->Output = pid->P + pid->I + pid->D;
    pid->Output = ConstrainF(pid->Output, pid->MaxOut, -pid->MaxOut);

    pid->Error[1] = pid->Error[0];
}

/// @brief 角度量存在过零点问题时(存在卷绕时)使用优先转劣弧的PID控制器更新计算
/// @param pid  PID控制器
/// @param real 被控量当前的真实值
/// @param set  目标值
void PID_AngleCalc(PID_t* pid, float real, float set)
{
    //过零点时优先转劣弧
    pid->Error[0] = ConstrainF(AngleDiffF(set, real, pid->CircleResolution), pid->MaxError, -pid->MaxError);

    if (fabsf(pid->Error[0]) < pid->DeadBand)
    {
        pid->Error[1] = 0;
        pid->Output = 0;
        return;
    }

    if (pid->Error[0] > pid->DeadBand)
        pid->Error[0] -= pid->DeadBand;
    else if (pid->Error[0] < -pid->DeadBand)
        pid->Error[0] += pid->DeadBand;

    pid->P = pid->kP * pid->Error[0];
    if (fabsf(pid->Error[0]) <= pid->IntegralBand)
    {
        pid->I += pid->kI_mdt * pid->Error[0];
        pid->I = ConstrainF(pid->I, pid->I_Max, -pid->I_Max);
    }
    pid->D = pid->kD_ddt * (pid->Error[0] - pid->Error[1]);

    pid->Output = pid->P + pid->I + pid->D;
    pid->Output = ConstrainF(pid->Output, pid->MaxOut, -pid->MaxOut);

    pid->Error[1] = pid->Error[0];
}

/// @brief 清除PID控制器运算过程中的时间相关参量
/// @param pid PID控制器结构体
void PID_Clear(PID_t* pid)
{
    pid->Error[0] = 0;
    pid->Error[1] = 0;
    pid->I = 0;
    pid->Output = 0;
}
