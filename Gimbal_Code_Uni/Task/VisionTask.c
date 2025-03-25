/**
 * @attention   采用UTF-8字符集编码
 * @brief       视觉自瞄任务
 * @authors     北工大PIP战队 樊郡捷
 * @version     v1.1
 * @date        2025-02-27
 * @details     实现了牟昱东25赛季版视觉下位机 实现了自瞄 实现了基于FFT频域分析的反小陀螺
 */

#include "header.h"

#define VISION_DAEMON_TASK_UPDATE_TICK 100
#define VISION_ANALYSE_TASK_UPDATE_TICK 20
#define VISION_ANALYSE_TASK_STACK_SIZE_WORD 2048

osThreadId VisionDaemonHandle;
osThreadId VisionAnalyseHandle;
uint32_t VisionAnalyseBuffer[VISION_ANALYSE_TASK_STACK_SIZE_WORD];
osStaticThreadDef_t VisionAnalyseControlBlock;

Vision_t Vision;

float Vision_YawPathPreview(void);
float Vision_PitchTrajectory(void);

float tempf = 1.05f;

void VisionDaemonTask(void const* argument)
{
    uint32_t PreviousWakeTime = osKernelSysTick();

    KalmanFilter1st_t AngularSpeedFilter;
    KalmanFilter1st_Init(&AngularSpeedFilter, 20, 100);

    LPF_1stOrderIIR_Init(&Vision.Pitch.HeightErrFilter, VISION_DAEMON_TASK_UPDATE_TICK / 1000.f, 1.f);

    float PrevYawAngle = 0;
    int PrevDetectState = RESET;
    while (1)
    {
        osDelayUntil(&PreviousWakeTime, VISION_DAEMON_TASK_UPDATE_TICK);
        uint32_t NowTick = HAL_GetTick();

        NUC_SendConfig();

        //目标角速度更新
        if (!PrevDetectState && Vision.isDetected)
            PrevYawAngle = Vision.Yaw.isRotate ? Vision.Yaw.TargetCenterAngle : Vision.Yaw.Angle.Unwraped;
        PrevDetectState = Vision.isDetected;
        if (NowTick - Vision.Yaw.LastTargetChangeTick > VISION_DAEMON_TASK_UPDATE_TICK) // 上次更新期间没有发生装甲板切换
        {
            float NowYawAngle = Vision.Yaw.isRotate ? Vision.Yaw.TargetCenterAngle : Vision.Yaw.Angle.Unwraped;
            float DeltaYawAngle = NowYawAngle - PrevYawAngle;
            Vision.Yaw.AngularSpeed = KalmanFilter1st_Update(
                &AngularSpeedFilter,
                DEG2RAD(DeltaYawAngle / (VISION_DAEMON_TASK_UPDATE_TICK / 1000.f)));
            PrevYawAngle = Vision.Yaw.isRotate ? Vision.Yaw.TargetCenterAngle : Vision.Yaw.Angle.Unwraped;
        }

        //目标高度差更新
        LPF_1stOrderIIR_Update(&Vision.Pitch.HeightErrFilter, Vision.NUC->Distance * sinf(DEG2RAD(Vision.NUC->PitchAngle + (Inertial.Pitch))));
        Vision.Pitch.HeightErr = Vision.Pitch.HeightErrFilter.Output;

        if (!Vision.isDetected)
            PID_Clear(&Vision.Yaw.CompensatePID);
    }
}

float tempm = 4.f;
float AngleAmplitude = 0;
#if DEBUG_MODE == DEBUG_MODE_FFT
extern osThreadId DebugHandle;
#endif
/// @brief 整体思路是: 将目标方位乘以目标距离得到的弧长近似为目标所在的纵向平面内目标方位的投影位置 以实现归一化的频谱分析
///                   在此基础上去除其直流分量以避免直流分量频谱混叠; 序列加窗补零以提高频率分辨率
///                   反小陀螺时为了计算实时的车体中心 利用观测到的数据生成一个完全反向的标准锯齿波 叠加到当前信号上 将得到的值认为是车体中心
/// @param argument
void VisionAnalyseTask(void const* argument)
{
    uint32_t PreviousWakeTime = osKernelSysTick();

    // float AngleAmplitude = 0;   //利用装甲板切换信号更新目标Yaw轴方位角锯齿波波形的幅度

    KalmanFilter1st_t PeakFreqFilter;
    KalmanFilter1st_Init(&PeakFreqFilter, 10, 100);

    KalmanFilter1st_t TargetCenterAngleFilter;
    KalmanFilter1st_Init(&TargetCenterAngleFilter, 8, 100);

    float PrevArcLen = Vision.Yaw.ArcQueue.Data[0];
    float FftBuf[VISION_FFT_POINT], Spectrum[VISION_FFT_POINT];
    float PrevAngle = 0;
    float PrevHeightErr = 0;
    float PrevAngleAmplitude = 0;
    while (1)
    {
        osDelayUntil(&PreviousWakeTime, VISION_ANALYSE_TASK_UPDATE_TICK);
        uint32_t NowTick = HAL_GetTick();

        if (!Controller.isEnable || Vision.NUC->Distance == 0 || Controller.VisionMode != VISION_MODE_ARMOR)
        {
            Vision.Yaw.isRotate = RESET;
            continue;
        }

        //目标切换更新 目标旋转方向更新 目标中心点更新
        float DeltaArcLen = Vision.Yaw.ArcQueue.Data[0] - PrevArcLen;
        if (fabsf(DeltaArcLen) > 0.1f && (NowTick - Vision.Yaw.LastTargetChangeTick) > 300) //移动超过0.25m则认为是一次切换
        {
            Vision.Yaw.LastTargetChangeTick = NowTick;
            Vision.Yaw.PeakFreqPhase = 0;   //锯齿波零相位处为阶跃点

            Vision.Yaw.RotateDir = DeltaArcLen > 0 ? 1 : -1;

            if ((NowTick - Vision.Yaw.LastTargetChangeTick) < 750)
            {
                float TempAngleAmplitude = -AngleDiffF(Vision.Yaw.Angle.Value, PrevAngle, 360) / 2 * 1.1f;
                if (TempAngleAmplitude * PrevAngleAmplitude > 0)    //异号则过滤 避免切换噪声
                    AngleAmplitude = TempAngleAmplitude;
                PrevAngleAmplitude = TempAngleAmplitude;
            }
            else
            {
                AngleAmplitude = 0;
            }

            Vision.Pitch.PrevHeightErr = PrevHeightErr;
        }
        PrevArcLen = Vision.Yaw.ArcQueue.Data[0];
        PrevHeightErr = Vision.Pitch.HeightErr;
        if ((NowTick - Vision.Yaw.LastTargetChangeTick) > 100)
            PrevAngle = Vision.Yaw.Angle.Value;

        for (int i = 0; i < VISION_FFT_LEN; i++)
            FftBuf[i] = (Vision.Yaw.ArcQueue.Data[i] - Vision.Yaw.ArcQueue.Mean) * Vision.Yaw.HanningWindow[i];
        for (int i = VISION_FFT_LEN; i < VISION_FFT_POINT; i++)   //补零 提高频率分辨率
            FftBuf[i] = 0;
        arm_rfft_fast_f32(&Vision.Yaw.FFT, FftBuf, Spectrum, 0);
        arm_cmplx_mag_f32(Spectrum, Vision.Yaw.FreqMag, 128);  //只分析前128个频点

        for (int i = 0; i < 128; i++)
        {
            if ((float)i * ((float)Vision.NUC->Freq / VISION_FFT_POINT) > 1.6f)
                break;
            Vision.Yaw.FreqMag[i] = 0;
        }

        float MaxFreqMag;
        uint32_t MaxMagFreqIndex;
        arm_max_f32(Vision.Yaw.FreqMag, 128, &MaxFreqMag, &MaxMagFreqIndex);
        Vision.Yaw.PeakFreq = KalmanFilter1st_Update(&PeakFreqFilter, MaxMagFreqIndex * ((float)Vision.NUC->Freq / VISION_FFT_POINT));
        Vision.Yaw.PeakFreqPhase += 2 * PI * Vision.Yaw.PeakFreq * (VISION_ANALYSE_TASK_UPDATE_TICK / 1000.f);
        Vision.Yaw.PeakFreqPhase = ConstrainF(Vision.Yaw.PeakFreqPhase, 2 * PI, 0);

        //相位信号就是所需频率的锯齿波 直接利用
        float TempTargetCenterAngle = KalmanFilter1st_Update(
            &TargetCenterAngleFilter,
            Vision.Yaw.Angle.Unwraped - AngleAmplitude * (Vision.Yaw.PeakFreqPhase - PI) / PI);
        TempTargetCenterAngle += Vision.Yaw.AngularSpeed * tempm;   //手动补一下滤波器造成的延迟
        Vision.Yaw.TargetCenterAngle = TempTargetCenterAngle;

        Vision.Yaw.isRotate = (MaxFreqMag > 1.5f && Vision.Yaw.PeakFreq > 1.5f && (NowTick - Vision.Yaw.LastTargetChangeTick) < 750) ? SET : RESET;

#if DEBUG_MODE == DEBUG_MODE_FFT
        if (DebugHandle)
            osThreadResume(DebugHandle);
#endif
    }
}

float Yaw_CompensatePID_kI = 0.75f;
float Yaw_CompensatePID_kD = 0.003f;
extern osThreadId DebugHandle;
void VisionTask(void const* argument)
{
    uint32_t PrevWakeTick = HAL_GetTick();

    NUC_Init();
    Vision.NUC = &NUC;

    PID_InitStruct_t PID_InitParam = { 0 };
    PID_InitParam.DeltaTime = 0.006;
    PID_InitParam.DifferentialFreqDiv = 5;

    PID_InitParam.kP = 0;
    // const float Yaw_CompensatePID_kI = 1.5f;
    // const float Yaw_CompensatePID_kD = 0;
    PID_InitParam.MaxError = FLOAT_MAX_VAL;
    PID_InitParam.MaxOutput = 90;
    PID_InitParam.I_Max = 90;
    PID_InitParam.DeadBand = 0;
    PID_InitParam.IntegralBand = 5;
    PID_Init(&Vision.Yaw.CompensatePID, &PID_InitParam);

    Vision.isDetected = RESET;
    Vision.Yaw.RotateDir = 1;
    Vision.Yaw.LastTargetChangeTick = 0;
    Vision.AmmoSpeed = 22;
    Vision.Pitch.HeightErr = 0;
    Vision.Pitch.PrevHeightErr = 0;
    arm_rfft_fast_init_f32(&Vision.Yaw.FFT, VISION_FFT_POINT);
    HanningWindow_Init(Vision.Yaw.HanningWindow, VISION_FFT_LEN);
    AngleUnwrap_Init(&Vision.Yaw.Angle, Inertial.Yaw, 360);
    Vision.Yaw.AngularSpeed = 0;
    MeanQueue_Init(&Vision.Yaw.ArcQueue, VISION_FFT_LEN, 1);  //用目标在极坐标系下的弧长对FFT输入量进行归一化 弧长计算注意解卷绕
    for (int i = 0; i < 128; i++)
        Vision.Yaw.FreqMag[i] = 0;

    osThreadDef(VisionDaemon, VisionDaemonTask, osPriorityAboveNormal, 0, 128);
    VisionDaemonHandle = osThreadCreate(osThread(VisionDaemon), NULL);

    osThreadStaticDef(VisionAnalyse, VisionAnalyseTask, osPriorityBelowNormal, 0, VISION_ANALYSE_TASK_STACK_SIZE_WORD, VisionAnalyseBuffer, &VisionAnalyseControlBlock);
    VisionAnalyseHandle = osThreadCreate(osThread(VisionAnalyse), NULL);

    while (1)
    {
        osThreadSuspend(osThreadGetId());

        //        if (Referee.ShootData.InitialSpeed > 16)
        //            Vision.AmmoSpeed = Referee.ShootData.InitialSpeed;
        Vision.AmmoSpeed = 25;


        uint32_t NowTick = HAL_GetTick();
        float DeltaTime = 1.f / Vision.NUC->Freq;

        if (Controller.isEnable && Vision.NUC->Distance != 0 && Vision.NUC->LossFrame <= 3)
            AngleUnwrap_Update(&Vision.Yaw.Angle, Inertial.Yaw + -Vision.NUC->YawAngle * tempf);  //更新目标方位角 1.2修正量纲误差

        if (!Controller.isEnable || Vision.NUC->Distance == 0)	//Distance几乎只有无目标时=0
        {
            Vision.isDetected = RESET;
            PID_Clear(&Vision.Yaw.CompensatePID);
            continue;
        }

        if (Vision.isDetected == RESET)
        {
            Vision.isDetected = SET;
            MeanQueue_Clear(&Vision.Yaw.ArcQueue, DEG2RAD(Vision.Yaw.Angle.Unwraped) * Vision.NUC->Distance);
        }

        MeanQueue_Enter(&Vision.Yaw.ArcQueue, DEG2RAD(Vision.Yaw.Angle.Unwraped) * Vision.NUC->Distance);

        if (!Controller.AimingOn)
            continue;

        Vision.Yaw.CompensatePID.kI_mdt = Yaw_CompensatePID_kI * DeltaTime;
        Vision.Yaw.CompensatePID.kD_ddt = Yaw_CompensatePID_kD / (DeltaTime * Vision.Yaw.CompensatePID.DifferentialFreqDiv);
        PID_Calc(&Vision.Yaw.CompensatePID, -Gimbal.Yaw.PositionPID.Error[0], 0);
        if (!Vision.Yaw.isRotate || !Controller.AimingInvRotateEnable)
        {
            if (Vision.NUC->LossFrame <= 3)
                Vision.Yaw.Target = Inertial.Yaw + (-Vision.NUC->YawAngle + Controller.View.YawVisionOffset) + Vision_YawPathPreview();
        }
        else
        {
            Vision.Yaw.Target = Vision.Yaw.TargetCenterAngle + Vision_YawPathPreview() + Controller.View.YawVisionOffset;
        }
        Controller.View.Yaw = Vision.Yaw.Target;

        // Vision.Pitch.Target = Inertial.Pitch + (Vision.NUC->PitchAngle + Controller.View.PitchVisionOffset);
        Vision.Pitch.Target = Vision_PitchTrajectory() + Controller.View.PitchVisionOffset;
        Controller.View.Pitch = Vision.Pitch.Target;
    }
}

/// @brief 计算弹丸飞行时间内目标转过的相位
/// @param
/// @return
float Vision_ShootingPhaseErr_Calc(void)
{
    return 2 * PI * Vision.Yaw.PeakFreq * Vision.NUC->Distance / Vision.AmmoSpeed;
}

float MagicVal = 0.1f;   //补偿玄学因素造成的延迟
int Vision_ShootingControl(void)
{
    static int PrevCtrlShootingOn = RESET;
    int tempPrevCtrlShootingOn = PrevCtrlShootingOn;
	
    switch (Controller.VisionMode)
    {
    case VISION_MODE_ARMOR:
        if (fabsf(DEG2RAD(AngleDiffF(Vision.Yaw.Target, Inertial.Yaw, 360)) * Vision.NUC->Distance) > 0.05f)    //瞄准位置不超出装甲板范围
            return RESET;
        if (Vision.Yaw.isRotate)
        {
            float ShootingPhase = PI - Vision_ShootingPhaseErr_Calc();
            // float MagicVal = 0.1f;   //补偿玄学因素造成的延迟
            ShootingPhase -= MagicVal * Vision.Yaw.PeakFreq * Vision.Yaw.PeakFreq;
            if (fabsf(AngleDiffF(ShootingPhase, Vision.Yaw.PeakFreqPhase, 2 * PI)) > (2 * PI * 0.25f / 2))   //目的是在相位180°处命中目标 假设装甲板占侧面面积的25%
                return RESET;
        }
        return Vision.Yaw.isRotate ? SET : RESET;
    case VISION_MODE_POWER_RUNE_LITTLE:
    case VISION_MODE_POWER_RUNE_BIG:
        PrevCtrlShootingOn = Controller.ShootingOn;
        return (!tempPrevCtrlShootingOn && Controller.ShootingOn) ? SET : RESET;
    default:
        return RESET;
    }
}

/// @brief 水平预瞄
/// @return 预瞄量的角度值
float ICR_R;    //全局旋转中心
float Vision_YawPathPreview(void)
{
    if (Controller.VisionMode != VISION_MODE_ARMOR || Vision.Yaw.isRotate)
        return 0;
    ICR_R = -FusionINS.Robot.Speed.LR * VISION_CHASSIS_SPEED_SCALE / Vision.Yaw.AngularSpeed;  //在以瞄准目标位圆心旋转时，此值应于distance相等，通过此前提，调整速度缩放系数
    return RAD2DEG(asinf((Vision.Yaw.AngularSpeed * Vision.NUC->Distance - -FusionINS.Robot.Speed.LR * VISION_CHASSIS_SPEED_SCALE) / Vision.AmmoSpeed));
}

float temph;
/// @brief 垂直弹道修正
/// @return 角度值
float Vision_PitchTrajectory(void)
{
    float HeightErr;
    //如果当前瞄准目标是下一个装甲板 或已经通过射击窗口 则用上次测得的值作为观测
    if (Vision.Yaw.isRotate && (Vision_ShootingPhaseErr_Calc() > PI || (Vision.Yaw.PeakFreqPhase > (PI + PI * 0.5f))))
        HeightErr = Vision.Pitch.PrevHeightErr;
    else
        HeightErr = Vision.Pitch.HeightErr;
    // if (Vision.Yaw.isRotate)
    //     HeightErr = (Vision.Pitch.HeightErr + Vision.Pitch.PrevHeightErr) / 2;
    // else
    //     HeightErr = Vision.Pitch.HeightErr;

    temph = HeightErr;

    float MagicValue = 0;//(4.f - distance) * 0.05f / 4;
    float Tau = 3.2e-3f / (0.25f * (16.8e-3f * 16.8e-3f)) / Vision.AmmoSpeed;
    float T = Tau * (expf(Vision.NUC->Distance / (Vision.AmmoSpeed * Tau)) - 1.f);
    float SinTheta = 1.f / (Vision.AmmoSpeed * T) * ((HeightErr + MagicValue) + 0.5f * 9.8f * T * T);
    float Theta = asinf(SinTheta);
    return RAD2DEG(Theta);
}
