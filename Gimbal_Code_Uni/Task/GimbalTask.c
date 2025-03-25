/**
 * @attention   采用UTF-8字符集编码
 * @brief       云台任务 改进俯仰限幅
 * @authors     北工大PIP战队 樊郡捷
 * @version     v1.1
 * @date        2025-02-18
 * @details     云台控制器的更新任务
 */

#include "header.h"

#define GIMBAL_TASK_UPDATE_TICK 2

Gimbal_t Gimbal;

void Gimbal_YawPID_Calc(void);
void Gimbal_PitchPID_CalcWithConstrain(void);
void Gimbal_Disable(void);

void GimbalTask(void const* argument)
{
    uint32_t PreviousWakeTime = osKernelSysTick();

    PID_InitStruct_t PID_InitParam = { 0 };
    PID_InitParam.DeltaTime = GIMBAL_TASK_UPDATE_TICK / 1000.f;

    // DM4310_Init(&Gimbal.Pitch.Motor, 0x000, 0x001, 0, 0);    //不使用DM4310自带的速度环位置环PID
    GM6020_Init(&Gimbal.Pitch.Motor, 2);

    PID_InitParam.kP = 0.32f;
    PID_InitParam.kI = 0;
    PID_InitParam.kD = 0;
    PID_InitParam.CircleResolution = 360;
    PID_InitParam.MaxError = 180;
    PID_InitParam.MaxOutput = 180;
    PID_InitParam.I_Max = 180;
    PID_InitParam.DeadBand = 0;
    PID_InitParam.IntegralBand = FLOAT_MAX_VAL;
    PID_Init(&Gimbal.Yaw.PositionPID, &PID_InitParam);

    PID_InitParam.kP = 14000;
    PID_InitParam.kI = 4000;
    PID_InitParam.kD = 0;
    PID_InitParam.MaxError = 36;
    PID_InitParam.MaxOutput = 30000;
    PID_InitParam.I_Max = 2000;
    PID_InitParam.DeadBand = 0;
    PID_InitParam.IntegralBand = FLOAT_MAX_VAL;
    PID_Init(&Gimbal.Yaw.SpeedPID, &PID_InitParam);

    PID_InitParam.kP = 1.5f;
    PID_InitParam.kI = 0;
    PID_InitParam.kD = 0;
    PID_InitParam.CircleResolution = 360;
    PID_InitParam.MaxError = 90;
    PID_InitParam.MaxOutput = 36;
    PID_InitParam.I_Max = 0;
    PID_InitParam.DeadBand = 0;
    PID_InitParam.IntegralBand = 0;
    PID_Init(&Gimbal.Pitch.PositionPID, &PID_InitParam);

    PID_InitParam.kP = 2500;
    PID_InitParam.kI = 2500;
    PID_InitParam.kD = 0;
    PID_InitParam.MaxError = 36;
    PID_InitParam.MaxOutput = 30000;
    PID_InitParam.I_Max = 8000;
    PID_InitParam.DeadBand = 0;
    PID_InitParam.IntegralBand = FLOAT_MAX_VAL;
    PID_Init(&Gimbal.Pitch.SpeedPID, &PID_InitParam);

    osDelay(1000);

    while (1)
    {
        osDelayUntil(&PreviousWakeTime, GIMBAL_TASK_UPDATE_TICK);

        if (!Controller.isEnable)
        {
            Gimbal_Disable();
            continue;
        }

        Gimbal_YawPID_Calc();
        Gimbal_PitchPID_CalcWithConstrain();

        ChassisBoard_SendYawOutput(Gimbal.Yaw.SpeedPID.Output);
        Gimbal.Pitch.Motor.Current.Set = Gimbal.Pitch.SpeedPID.Output;
        DjiMotor_CanID_0x1FF_Output();
    }
}

/// @brief 计算Yaw轴PID
/// @param  
void Gimbal_YawPID_Calc(void)
{
    PID_AngleCalc(&Gimbal.Yaw.PositionPID, Inertial.Yaw, Controller.View.Yaw);
    float SpeedSet = Gimbal.Yaw.PositionPID.Output;
    if (Controller.AimingOn && Vision.isDetected)
        SpeedSet += Vision.Yaw.CompensatePID.Output;
    PID_Calc(&Gimbal.Yaw.SpeedPID, Inertial.Gyro[AZ], SpeedSet);
}

/// @brief 计算Pitch轴PID 根据Pitch轴电机编码器 计算超出限幅的角度大小 并检测云台目标角度是否会超出限幅
/// @param  
void Gimbal_PitchPID_CalcWithConstrain(void)
{
    const int PitchTopEnc = PITCH_MAX_ENCODER;
    const int PitchBottomEnc = PITCH_MIN_ENCODER;   //云台上抬对应编码器数值增大

    //下列定义永远对应上抬增大的结果
    //与目标的差
    float ErrAngle = AngleDiffF(Controller.View.Pitch, Inertial.Pitch, Gimbal.Pitch.PositionPID.CircleResolution);
    //与限幅的差
    int TopDeltaEnc, BottomDeltaEnc;
    if (AngleDiffF(PitchTopEnc, PitchBottomEnc, Gimbal.Pitch.Motor.EncoderResolution) > 0)  //云台上抬对应编码器数值增大
    {
        TopDeltaEnc = AngleDiffF(PitchTopEnc, Gimbal.Pitch.Motor.Position.Real, Gimbal.Pitch.Motor.EncoderResolution);
        BottomDeltaEnc = AngleDiffF(PitchBottomEnc, Gimbal.Pitch.Motor.Position.Real, Gimbal.Pitch.Motor.EncoderResolution);
    }
    else    //云台上抬对应编码器数值减小
    {
        TopDeltaEnc = -AngleDiffF(PitchTopEnc, Gimbal.Pitch.Motor.Position.Real, Gimbal.Pitch.Motor.EncoderResolution);
        BottomDeltaEnc = -AngleDiffF(PitchBottomEnc, Gimbal.Pitch.Motor.Position.Real, Gimbal.Pitch.Motor.EncoderResolution);
    }
    float TopDeltaAngle = TopDeltaEnc * 360.f / Gimbal.Pitch.Motor.EncoderResolution;
    float BottomDeltaAngle = BottomDeltaEnc * 360.f / Gimbal.Pitch.Motor.EncoderResolution;
    if (ErrAngle > TopDeltaAngle)
        Controller.View.Pitch = Controller.View.Pitch - ErrAngle + TopDeltaAngle;
    else if (ErrAngle < BottomDeltaAngle)
        Controller.View.Pitch = Controller.View.Pitch - ErrAngle + BottomDeltaAngle;
    PID_AngleCalc(&Gimbal.Pitch.PositionPID, Inertial.Pitch, Controller.View.Pitch);
    PID_Calc(&Gimbal.Pitch.SpeedPID, Inertial.Gyro[AX], Gimbal.Pitch.PositionPID.Output);
}

void Gimbal_Disable(void)
{
    Gimbal.Pitch.Motor.Current.Set = 0;
    DjiMotor_CanID_0x1FF_Output();
    ChassisBoard_SendYawOutput(0);

    PID_Clear(&Gimbal.Pitch.PositionPID);
    PID_Clear(&Gimbal.Pitch.SpeedPID);
    PID_Clear(&Gimbal.Yaw.PositionPID);
    PID_Clear(&Gimbal.Yaw.SpeedPID);
}
