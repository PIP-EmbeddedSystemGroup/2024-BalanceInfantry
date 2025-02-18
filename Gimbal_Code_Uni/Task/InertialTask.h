#ifndef _INERTIAL_TASK_H_
#define _INERTIAL_TASK_H_

/**
 * @attention   采用UTF-8字符集编码
 */

#include "stdint.h"
#include "cmsis_os.h"

#define AX 0    //垂直于Pitch平面
#define AY 1    //垂直于Roll平面
#define AZ 2    //垂直于Yaw平面
#define PITCH 0
#define YAW 1

typedef struct
{
    IMU_Data_t* IMU;
    PID_t ImuHeaterPID;

    float Q[4]; // 四元数估计值

    float Gyro[3];  // 角速度
    float Accel[3]; // 加速度

    LPF_1stOrderIIR_t AccelLPF[3];
    float MotionAccel_b[3]; // 机体坐标加速度
    float MotionAccel_n[3]; // 绝对系加速度

    LPF_1stOrderIIR_t GyroLPF[3];
    float dGyro[3];
    LPF_1stOrderIIR_t dGyroLPF[3];

    // 加速度在绝对系的向量表示
    float xn[3];
    float yn[3];
    float zn[3];

    // float atanxz;
    // float atanyz;

    // 位姿
    float Roll;
    float Pitch;
    float Yaw;
    float YawTotalAngle;
} Inertial_t;

void InertialTask(void const* argument);

extern Inertial_t Inertial;

#endif
