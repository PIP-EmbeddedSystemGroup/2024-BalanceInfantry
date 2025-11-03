#ifndef _SHOOTING_TASK_H_
#define _SHOOTING_TASK_H_

/**
 * @attention   采用UTF-8字符集编码
 */

#include "stdint.h"
#include "cmsis_os.h"

#include "DjiMotor.h"
#include "UniversalPID.h"

#define SHOOTING_FRI_L 0
#define SHOOTING_FRI_R 1

typedef struct
{
    struct
    {
        M3508_t Motor;
        LPF_1stOrderIIR_t SpeedFilter;
        PID_t SpeedPID;
    } Friction[2];

    struct
    {
        M2006_t Motor;
        PID_t PositionPID;
        PID_t SpeedPID;
        uint32_t LastFeedingTick;
        uint32_t LastStuckTick;
    } Feeding;
} Shooting_t;

void ShootingTask(void const* argument);

extern Shooting_t Shooting;

#endif
