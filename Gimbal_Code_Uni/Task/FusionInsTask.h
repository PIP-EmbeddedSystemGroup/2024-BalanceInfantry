#ifndef _FUSION_INS_TASK_H_
#define _FUSION_INS_TASK_H_

/**
 * @attention   采用UTF-8字符集编码
 */

#include "stdint.h"
#include "cmsis_os.h"

typedef struct
{
    FusionKF_t KF;

    struct
    {
        struct
        {
            float FB;
            float LR;
        } Speed;
    } Robot;    //机器人坐标系

    struct
    {
        struct
        {
            float X;
            float Y;
        } Speed;
        struct
        {
            float X;
            float Y;
        } Position;
    } World;    //世界坐标系 取X轴为Yaw=0时朝向的轴
} FusionINS_t;

void FusionInsTask(void const* argument);

extern FusionINS_t FusionINS;

#endif
