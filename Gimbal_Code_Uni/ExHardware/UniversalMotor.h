#ifndef _UNIVERSAL_MOTOR_H_
#define _UNIVERSAL_MOTOR_H_

/**
 * @attention   采用UTF-8字符集编码
 */

#include "stm32f4xx_hal.h"
#include "stdint.h"

#include "UniversalPID.h"

typedef enum
{
    MOTOR_TYPE_M3508,
    MOTOR_TYPE_M2006,
    MOTOR_TYPE_GM6020,
    MOTOR_TYPE_DM4310,
} MotorType_t;

typedef struct
{
    MotorType_t Type;		//电机类型
    int EncoderResolution;	//编码器旋转一圈所对应的数值大小
    float GearRatio;		//减速比

    struct
    {
        int Set;	//电流输出值
        int Real;	//电流真实值
    } Current;

    struct
    {
        int Set;	//速度设定值
        int Real;	//速度真实值
    } Speed;

    struct
    {
        int Set;		//位置设定值
        int Real;		//位置真实值

        //以下参数调用Motor_UpdateUnwrap函数更新解卷绕
        int PrevReal;	//上次位置真实值
        int Unwraped;	//解卷绕后的总旋转
        struct
        {
            int Set;	//设定值
            int Real;	//真实值
        } UnwrapedAfterGear;    //经过减速箱后的输出轴总旋转角度 单位:度
    } Position;

    int Temperature;	//实际温度值

    void* SpecialData;

    uint32_t LastOnlineTick;
} Motor_t;

void Motor_UpdateUnwrap(Motor_t* motor);
float Motor_GetPositionDegree(Motor_t* motor);
void Motor_SpeedPID_Calc(Motor_t* motor, PID_t* speedPID);
void Motor_PositionPID_Calc(Motor_t* motor, PID_t* positionPID, PID_t* speedPID);
void Motor_UnwrapedAfterGearPositionPID_Calc(Motor_t* motor, PID_t* positionPID, PID_t* speedPID);

#endif
