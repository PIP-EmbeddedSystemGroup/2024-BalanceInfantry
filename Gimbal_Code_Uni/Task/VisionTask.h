#ifndef _VISION_TASK_H_
#define _VISION_TASK_H_

/**
 * @attention   采用UTF-8字符集编码
 */

#include "stdint.h"
#include "cmsis_os.h"

#include "UniversalPID.h"

#define VISION_FFT_LEN 128  //用来做频域分析的数据长度 实际计算时会继续补零
#define VISION_FFT_ZERO_PADDING 4   //4倍补0
#define VISION_FFT_POINT (VISION_FFT_LEN * VISION_FFT_ZERO_PADDING)

typedef enum
{
    VISION_MODE_ARMOR = 0x3B,
    VISION_MODE_POWER_RUNE_LITTLE = 0x6D,
    VISION_MODE_POWER_RUNE_BIG = 0x9E
} VisionMode_t;

typedef struct
{
    NUC_t* NUC;

    int isDetected;

    float AmmoSpeed;

    struct
    {
        int isRotate;
        int RotateDir;  //目标旋转方向 >0逆时针(从左到右) <0顺时针(从右到左)

        uint32_t LastTargetChangeTick;
        float TargetCenterAngle;

        PID_t CompensatePID;

        AngleUnwrap_t Angle;    //目标瞄准点方位角
        float AngularSpeed;     //目标角速度 rad/s

        arm_rfft_fast_instance_f32 FFT;
        float HanningWindow[VISION_FFT_LEN];
        MeanQueue_t ArcQueue;
        float FreqMag[128];

        float PeakFreq;
        float PeakFreqPhase;

        float Target;   //最终输出的方位角
    } Yaw;

    struct
    {
        float HeightErr;
        LPF_1stOrderIIR_t HeightErrFilter;
        float PrevHeightErr;

        float Target;   //最终输出的方位角
    } Pitch;

} Vision_t;

void VisionTask(void const* argument);
int Vision_ShootingControl(void);

extern Vision_t Vision;

#endif
