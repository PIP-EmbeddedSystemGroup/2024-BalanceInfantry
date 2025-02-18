#ifndef _VISION_TASK_H_
#define _VISION_TASK_H_

/**
 * @attention   采用UTF-8字符集编码
 */

#include "stdint.h"
#include "cmsis_os.h"

#include "UniversalPID.h"

#define VISION_FFT_LEN 128

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

    PID_t AimingPID[2];
    
    float AmmoSpeed;
    float HeightErr;

    struct
    {
        float Angle;        //目标方位角
        float AngularSpeed; //目标角速度 rad/s
        KalmanFilter1st_t AngularSpeedFilter;

        arm_rfft_fast_instance_f32 FFT;
        float Queue[VISION_FFT_LEN];
        int QueueIndex;
        float FreqMag[VISION_FFT_LEN / 2];

        float Freq;

        uint32_t LastFftUpdateTick;
    } TargetYaw;
} Vision_t;

void VisionTask(void const* argument);

extern Vision_t Vision;

#endif
