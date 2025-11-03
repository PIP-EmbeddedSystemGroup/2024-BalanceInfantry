/**
 * @attention   采用UTF-8字符集编码
 * @brief       视觉自瞄任务
 * @authors     北工大PIP战队 樊郡捷
 * @version     v1.0
 * @date        2024-10-18
 * @details     实现了牟昱东25赛季版视觉下位机 实现了自瞄
 */

#include "header.h"

#define VISION_DAEMON_TASK_UPDATE_TICK 100

osThreadId VisionDaemonHandle;

Vision_t Vision;

float Vision_YawPathPreview(void);
float Vision_PitchTrajectory(void);

float tempf = 1.f;

void VisionDaemonTask(void const* argument)
{
    uint32_t PreviousWakeTime = osKernelSysTick();

    float PrevYawAngle = 0;
    int PrevDetectState = RESET;
    while (1)
    {
        osDelayUntil(&PreviousWakeTime, VISION_DAEMON_TASK_UPDATE_TICK);

        if (!PrevDetectState && Vision.isDetected)
            PrevYawAngle = Vision.TargetYaw.Angle;
        PrevDetectState = Vision.isDetected;

        Vision.TargetYaw.Angle = Inertial.Yaw + -Vision.NUC->YawAngle * tempf;   //目标方位角 1.2修正量纲误差
        float DeltaYawAngle = Vision.TargetYaw.Angle - PrevYawAngle;
        Vision.TargetYaw.AngularSpeed = KalmanFilter1st_Update(
            &Vision.TargetYaw.AngularSpeedFilter,
            DEG2RAD(DeltaYawAngle / (VISION_DAEMON_TASK_UPDATE_TICK / 1000.f)));
        PrevYawAngle = Vision.TargetYaw.Angle;
        Vision.HeightErr = Vision.NUC->Distance * sinf(DEG2RAD(Vision.NUC->PitchAngle + (Inertial.Pitch)));

        NUC_SendConfig();
    }
}

extern osThreadId DebugHandle;
void VisionTask(void const* argument)
{
    uint32_t PrevWakeTick = HAL_GetTick();

    NUC_Init();
    Vision.NUC = &NUC;

    PID_InitStruct_t PID_InitParam = { 0 };
    PID_InitParam.DeltaTime = 0.006;

    PID_InitParam.kP = 10;
    const float AimingPID_kI = 0;
    const float AimingPID_kD = 0;
    PID_InitParam.MaxError = FLOAT_MAX_VAL;
    PID_InitParam.MaxOutput = 90;
    PID_InitParam.I_Max = 50;
    PID_InitParam.DeadBand = 0;
    PID_InitParam.IntegralBand = FLOAT_MAX_VAL;
    PID_Init(&Vision.AimingPID[AXIS_YAW], &PID_InitParam);
    PID_Init(&Vision.AimingPID[AXIS_PITCH], &PID_InitParam);

    Vision.isDetected = RESET;
    Vision.AmmoSpeed = 20;
    Vision.HeightErr = 0;
    arm_rfft_fast_init_f32(&Vision.TargetYaw.FFT, VISION_FFT_LEN * 2);  //FFT补零至两倍长度
    HanningWindow_Init(Vision.TargetYaw.HanningWindow, VISION_FFT_LEN);
    Vision.TargetYaw.Angle = 0;
    Vision.TargetYaw.AngleRaw = 0;
    Vision.TargetYaw.AngularSpeed = 0;
    KalmanFilter1st_Init(&Vision.TargetYaw.AngularSpeedFilter, 10, 100);
    Vision.TargetYaw.QueueIndex = 0;
    for (int i = 0; i < VISION_FFT_LEN; i++)
        Vision.TargetYaw.Queue[i] = 0;
    for (int i = 0; i < 32; i++)
        Vision.TargetYaw.FreqMag[i] = 0;
    Vision.TargetYaw.LastFftUpdateTick = 0;

    osThreadDef(VisionDaemon, VisionDaemonTask, osPriorityAboveNormal, 0, 128);
    VisionDaemonHandle = osThreadCreate(osThread(VisionDaemon), NULL);

    while (1)
    {
        osThreadSuspend(osThreadGetId());

        uint32_t NowTick = HAL_GetTick();
        float DeltaTime = 1.f / Vision.NUC->Freq;

        if (!Controller.isEnable || !Controller.AimingOn || Vision.NUC->Distance == 0)	//Distance几乎只有无目标时=0
        {
            Vision.isDetected = RESET;
            continue;
        }

        Vision.TargetYaw.AngleRaw = Inertial.Yaw + -Vision.NUC->YawAngleRaw * tempf; //目标方位角 1.2修正量纲误差
        if (Vision.isDetected == RESET)
        {
            Vision.isDetected = SET;
            for (int i = 0; i < VISION_FFT_LEN; i++)
                Vision.TargetYaw.Queue[i] = Vision.TargetYaw.AngleRaw;
        }
        for (int i = VISION_FFT_LEN - 1; i >= 1; i--)
            Vision.TargetYaw.Queue[i] = Vision.TargetYaw.Queue[i - 1];
        Vision.TargetYaw.Queue[0] = Vision.TargetYaw.AngleRaw;
        if (NowTick - Vision.TargetYaw.LastFftUpdateTick > 100)
        {
            Vision.TargetYaw.LastFftUpdateTick = NowTick;

            float FftBuf[VISION_FFT_LEN * 2], Spectrum[VISION_FFT_LEN * 2];
            arm_mult_f32(Vision.TargetYaw.Queue, Vision.TargetYaw.HanningWindow, FftBuf, VISION_FFT_LEN);   //原数据加汉宁窗
            for (int i = VISION_FFT_LEN; i < VISION_FFT_LEN * 2; i++)   //补零至两倍长度 提高频率分辨率
                FftBuf[i] = 0;
            arm_rfft_fast_f32(&Vision.TargetYaw.FFT, FftBuf, Spectrum, 0);
            arm_cmplx_mag_f32(Spectrum, Vision.TargetYaw.FreqMag, 32);  //只分析前32个频点

            for (int i = 0; i < VISION_FFT_LEN / 2; i++)
            {
                if ((float)i * ((float)Vision.NUC->Freq / VISION_FFT_LEN * 2) > 1.5f)
                    break;
                Vision.TargetYaw.FreqMag[i] = 0;
            }

            float MaxFreqMag;
            uint32_t MaxMagFreqIndex;
            arm_max_f32(Vision.TargetYaw.FreqMag, 32, &MaxFreqMag, &MaxMagFreqIndex);
            Vision.TargetYaw.Freq = MaxMagFreqIndex * (1.f / Vision.NUC->Freq * VISION_FFT_LEN * 2);

            // if (DebugHandle)
            //     osThreadResume(DebugHandle);
        }

        // Vision.AimingPID[AXIS_YAW].kI_mdt = AimingPID_kI * DeltaTime;
        // Vision.AimingPID[AXIS_YAW].kD_ddt = AimingPID_kD / DeltaTime;
        // PID_Calc(&Vision.AimingPID[AXIS_YAW], Vision.NUC->YawAngle, Vision_YawPathPreview() + Controller.View.YawVisionOffset);
        // Controller.View.Yaw += Vision.AimingPID[AXIS_YAW].Output * DeltaTime;
        Controller.View.Yaw = Inertial.Yaw + (-Vision.NUC->YawAngle + Controller.View.YawVisionOffset);// - Vision_YawPathPreview();

        // Vision.AimingPID[AXIS_PITCH].kI_mdt = AimingPID_kI * DeltaTime;
        // Vision.AimingPID[AXIS_PITCH].kD_ddt = AimingPID_kD / DeltaTime;
        // PID_Calc(&Vision.AimingPID[AXIS_PITCH], Vision.NUC->PitchAngle, 0);
        // if (Gimbal.Pitch.Motor.Position.Real <= PITCH_MAX_ENCODER && Gimbal.Pitch.Motor.Position.Real >= PITCH_MIN_ENCODER)
        //     Controller.View.Pitch += -Vision.AimingPID[AXIS_PITCH].Output * DeltaTime;
        Controller.View.Pitch = Vision_PitchTrajectory() + Controller.View.PitchVisionOffset;
    }
}

/// @brief 水平预瞄
/// @return 预瞄量的角度值
float ICR_R;    //全局旋转中心
float Vision_YawPathPreview(void)
{
    if (Controller.VisionMode != VISION_MODE_ARMOR)
        return 0;
    ICR_R = FusionINS.Robot.Speed.LR * VISION_CHASSIS_SPEED_SCALE / Vision.TargetYaw.AngularSpeed;  //在以瞄准目标位圆心旋转时，此值应于distance相等，通过此前提，调整速度缩放系数
    return RAD2DEG(asinf((Vision.TargetYaw.AngularSpeed * Vision.NUC->Distance - FusionINS.Robot.Speed.LR * VISION_CHASSIS_SPEED_SCALE) / Vision.AmmoSpeed));
}

/// @brief 垂直弹道修正
/// @return 角度值
float Vision_PitchTrajectory(void)
{
    // float 
    float MagicValue = 0;//(4.f - distance) * 0.05f / 4;
    float Tau = 3.2e-3f / (0.25f * (16.8e-3f * 16.8e-3f)) / Vision.AmmoSpeed;
    float T = Tau * (expf(Vision.NUC->Distance / (Vision.AmmoSpeed * Tau)) - 1.f);
    float SinTheta = 1.f / (Vision.AmmoSpeed * T) * ((Vision.HeightErr + MagicValue) + 0.5f * 9.8f * T * T);
    float Theta = asinf(SinTheta);
    return RAD2DEG(Theta);
}
