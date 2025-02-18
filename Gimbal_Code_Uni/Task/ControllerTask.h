#ifndef _CONTROLLER_TASK_H_
#define _CONTROLLER_TASK_H_

/**
 * @attention   采用UTF-8字符集编码
 */

#include "stdint.h"
#include "cmsis_os.h"

#include "MouseKeyboard.h"
#include "VisionTask.h"

#define CTRL_HEIGHT_SET_LOW     1   //底盘高度低
#define CTRL_HEIGHT_SET_MID     2   //底盘高度中
#define CTRL_HEIGHT_SET_HIGH    3   //底盘高度高

typedef struct
{
    int isEnable;

    int isSpinMode;

    struct
    {
        Key_t Up;
        Key_t Down;
    } PadS1;
    int FrictionOn;
    int ShootingOn;

    int AimingOn;
    VisionMode_t VisionMode;
    int AimingInvRotateEnable;

    struct
    {
        struct
        {
            float Set;
            float Real;
        } FB;   //Front Back
        struct
        {
            float Set;
            float Real;
        } LR;   //Left Right
    } Move; // m/s

    struct
    {
        float Yaw;
        float Pitch;
        float YawVisionOffset;
        float PitchVisionOffset;
    } View; // degree

    int HeightSet;
} Controller_t;

void ControllerTask(void const* argument);

extern Controller_t Controller;

#endif
