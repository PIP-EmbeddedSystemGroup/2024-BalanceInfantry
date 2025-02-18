/**
 * @attention   采用UTF-8字符集编码
 * @brief       通用电机接口
 * @authors     北工大PIP战队 樊郡捷
 * @version     v1.0
 * @date        2024-10-14
 * @details     统一了不同电机 提供不同电机的通用维护函数
 */

#include "header.h"

 /// @brief 如果有计算输出轴总旋转角度的需求 则在每次电机更新时调用此函数来解卷绕
 /// @param motor 电机
void Motor_UpdateUnwrap(Motor_t* motor)
{
    int DeltaAngle = motor->Position.Real - motor->Position.PrevReal;

    //过零点时计算劣弧
    float HalfCircle = motor->EncoderResolution / 2;
    if (DeltaAngle > HalfCircle)
        DeltaAngle -= motor->EncoderResolution;
    else if (DeltaAngle < -HalfCircle)
        DeltaAngle += motor->EncoderResolution;

    motor->Position.Unwraped += DeltaAngle;
    motor->Position.UnwrapedAfterGear.Real = (float)motor->Position.Unwraped / (float)motor->EncoderResolution / motor->GearRatio * 360.f;
    motor->Position.PrevReal = motor->Position.Real;
}

/// @brief 通过编码器分辨率和编码器值计算电机转子角度
/// @param motor 电机
float Motor_GetPositionDegree(Motor_t* motor)
{
    return motor->Position.Real * 360.f / motor->EncoderResolution;
}

/// @brief 将电机速度环PID计算集成到一起
/// @param motor    电机
/// @param speedPID 维护该电机速度环的PID结构体
void Motor_SpeedPID_Calc(Motor_t* motor, PID_t* speedPID)
{
    PID_Calc(speedPID, motor->Speed.Real, motor->Speed.Set);
    motor->Current.Set = speedPID->Output;
}

/// @brief 将电机位置双环PID计算集成到一起
/// @param motor        电机
/// @param positionPID  维护该电机位置环的PID结构体
/// @param speedPID     维护该电机速度环的PID结构体
void Motor_PositionPID_Calc(Motor_t* motor, PID_t* positionPID, PID_t* speedPID)
{
    PID_AngleCalc(positionPID, motor->Position.Real, motor->Position.Set);
    PID_Calc(speedPID, motor->Speed.Real, motor->Speed.Set);
    motor->Current.Set = speedPID->Output;
}

/// @brief 将电机位置双环PID计算集成到一起 (控制输出轴总旋转角度)
/// @param motor        电机
/// @param positionPID  维护该电机位置环的PID结构体
/// @param speedPID     维护该电机速度环的PID结构体
void Motor_UnwrapedAfterGearPositionPID_Calc(Motor_t* motor, PID_t* positionPID, PID_t* speedPID)
{
    PID_Calc(positionPID, motor->Position.UnwrapedAfterGear.Real, motor->Position.UnwrapedAfterGear.Set);
    motor->Speed.Set = positionPID->Output;
    PID_Calc(speedPID, motor->Speed.Real, motor->Speed.Set);
    motor->Current.Set = speedPID->Output;
}
