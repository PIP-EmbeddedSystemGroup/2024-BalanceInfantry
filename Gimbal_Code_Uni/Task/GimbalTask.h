#ifndef _GIMBAL_TASK_H_
#define _GIMBAL_TASK_H_

/**
 * @attention   采用UTF-8字符集编码
 */

#include "stdint.h"
#include "cmsis_os.h"

#include "DM4310.h"
#include "UniversalPID.h"

#define RC_DBUS_S2_PAD_CTRL         1
#define RC_DBUS_S2_COMPUTER_CTRL    2
#define RC_DBUS_S2_DISABLE_CTRL     3

typedef struct
{
    struct
    {
        GM6020_t Motor;
        PID_t PositionPID;
        PID_t SpeedPID;
    } Pitch;
    
    struct
    {
        GM6020_t MotorWithoutInit;
        PID_t PositionPID;
        PID_t SpeedPID;
    } Yaw;
} Gimbal_t;

void GimbalTask(void const* argument);

extern Gimbal_t Gimbal;

#endif
