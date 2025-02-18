/**
 * @attention   采用UTF-8字符集编码
 * @brief       大疆电机接口
 * @authors     北工大PIP战队 樊郡捷
 * @version     v1.0
 * @date        2024-10-14
 * @details     M3508 M2006 GM6020等电机的通用维护函数
 *              采用注册机制维护了一个索引表 进而统一了不同电机
 */

#include "header.h"

//大疆电机的列表 对应三个CANID 用于统一发送 在各电机子类的初始化函数中被注册进入列表
//修改电调ID后只需修改电机初始化时的ID 理论上无需维护本文件
DjiMotor_t* DjiMotor_CanID_0x1FF_List[4] = { NULL };
DjiMotor_t* DjiMotor_CanID_0x200_List[4] = { NULL };
DjiMotor_t* DjiMotor_CanID_0x2FF_List[3] = { NULL };

void M3508_Init(M3508_t* motor, int ID)
{
    motor->Type = MOTOR_TYPE_M3508;
    motor->EncoderResolution = 8192;
    motor->GearRatio = 3591.f / 187.f;

    motor->Current.Set = 0;
    motor->Current.Real = 0;
    motor->Speed.Set = 0;
    motor->Speed.Real = 0;
    motor->Position.Set = 0;
    motor->Position.Real = 0;
    motor->Position.PrevReal = 0;
    motor->Position.Unwraped = 0;
    motor->Position.UnwrapedAfterGear.Set = 0;
    motor->Position.UnwrapedAfterGear.Real = 0;
    motor->Temperature = 0;
    
    motor->LastOnlineTick = 0;

    if (ID >= 1 && ID <= 4)
        DjiMotor_CanID_0x200_List[ID - 1] = motor;
    else if (ID >= 5 && ID <= 8)
        DjiMotor_CanID_0x1FF_List[ID - 5] = motor;
}

void M3508P14_Init(M3508_t* motor, int ID)
{
    motor->Type = MOTOR_TYPE_M3508;
    motor->EncoderResolution = 8192;
    motor->GearRatio = 14;

    motor->Current.Set = 0;
    motor->Current.Real = 0;
    motor->Speed.Set = 0;
    motor->Speed.Real = 0;
    motor->Position.Set = 0;
    motor->Position.Real = 0;
    motor->Position.PrevReal = 0;
    motor->Position.Unwraped = 0;
    motor->Position.UnwrapedAfterGear.Set = 0;
    motor->Position.UnwrapedAfterGear.Real = 0;
    motor->Temperature = 0;
    
    motor->LastOnlineTick = 0;

    if (ID >= 1 && ID <= 4)
        DjiMotor_CanID_0x200_List[ID - 1] = motor;
    else if (ID >= 5 && ID <= 8)
        DjiMotor_CanID_0x1FF_List[ID - 5] = motor;
}

void M2006_Init(M2006_t* motor, int ID)
{
    motor->Type = MOTOR_TYPE_M2006;
    motor->EncoderResolution = 8192;
    motor->GearRatio = 36.f;

    motor->Current.Set = 0;
    motor->Current.Real = 0;
    motor->Speed.Set = 0;
    motor->Speed.Real = 0;
    motor->Position.Set = 0;
    motor->Position.Real = 0;
    motor->Position.PrevReal = 0;
    motor->Position.Unwraped = 0;
    motor->Position.UnwrapedAfterGear.Set = 0;
    motor->Position.UnwrapedAfterGear.Real = 0;
    motor->Temperature = 0;

    motor->LastOnlineTick = 0;

    if (ID >= 1 && ID <= 4)
        DjiMotor_CanID_0x200_List[ID - 1] = motor;
    else if (ID >= 5 && ID <= 8)
        DjiMotor_CanID_0x1FF_List[ID - 5] = motor;
}

void GM6020_Init(GM6020_t* motor, int ID)
{
    motor->Type = MOTOR_TYPE_GM6020;
    motor->EncoderResolution = 8192;
    motor->GearRatio = 1;

    motor->Current.Set = 0;
    motor->Current.Real = 0;
    motor->Speed.Set = 0;
    motor->Speed.Real = 0;
    motor->Position.Set = 0;
    motor->Position.Real = 0;
    motor->Position.PrevReal = 0;
    motor->Position.Unwraped = 0;
    motor->Position.UnwrapedAfterGear.Set = 0;
    motor->Position.UnwrapedAfterGear.Real = 0;
    motor->Temperature = 0;
    
    motor->LastOnlineTick = 0;

    if (ID >= 1 && ID <= 4)
        DjiMotor_CanID_0x1FF_List[ID - 1] = motor;
    else if (ID >= 5 && ID <= 7)
        DjiMotor_CanID_0x2FF_List[ID - 5] = motor;
}

void DjiMotor_Update(DjiMotor_t* motor, uint8_t data[8])
{
    motor->Position.Real = (int16_t)((data[0] << 8) | data[1]);
    motor->Speed.Real = (int16_t)((data[2] << 8) | data[3]);
    motor->Current.Real = (int16_t)((data[4] << 8) | data[5]);
    if (motor->Type != MOTOR_TYPE_M2006)
        motor->Temperature = data[6];

    motor->LastOnlineTick = HAL_GetTick();
}

void DjiMotor_CanID_0x1FF_Output(void)
{
    CAN_TxHeaderTypeDef TxMessage = { 0 };
    uint8_t Data[8] = { 0 };

    TxMessage.StdId = 0x1FF;
    TxMessage.DLC = 8;

    for (int i = 0; i < 4; i++)
    {
        if (DjiMotor_CanID_0x1FF_List[i] != NULL)
        {
            Data[i * 2] = (uint8_t)(DjiMotor_CanID_0x1FF_List[i]->Current.Set >> 8);
            Data[i * 2 + 1] = (uint8_t)(DjiMotor_CanID_0x1FF_List[i]->Current.Set);
        }
    }
    uint32_t TxFifoUsed;
    HAL_CAN_AddTxMessage(&hcan1, &TxMessage, Data, &TxFifoUsed);
}

void DjiMotor_CanID_0x200_Output(void)
{
    CAN_TxHeaderTypeDef TxMessage = { 0 };
    uint8_t Data[8] = { 0 };

    TxMessage.StdId = 0x200;
    TxMessage.DLC = 8;

    for (int i = 0; i < 4; i++)
    {
        if (DjiMotor_CanID_0x200_List[i] != NULL)
        {
            Data[i * 2] = (uint8_t)(DjiMotor_CanID_0x200_List[i]->Current.Set >> 8);
            Data[i * 2 + 1] = (uint8_t)(DjiMotor_CanID_0x200_List[i]->Current.Set);
        }
    }

    uint32_t TxFifoUsed;
    HAL_CAN_AddTxMessage(&hcan1, &TxMessage, Data, &TxFifoUsed);
}

void DjiMotor_CanID_0x2FF_Output(void)
{
    CAN_TxHeaderTypeDef TxMessage = { 0 };
    uint8_t Data[8] = { 0 };

    TxMessage.StdId = 0x2FF;
    TxMessage.DLC = 8;

    for (int i = 0; i < 3; i++)
    {
        if (DjiMotor_CanID_0x2FF_List[i] != NULL)
        {
            Data[i * 2] = (uint8_t)(DjiMotor_CanID_0x2FF_List[i]->Current.Set >> 8);
            Data[i * 2 + 1] = (uint8_t)(DjiMotor_CanID_0x2FF_List[i]->Current.Set);
        }
    }

    uint32_t TxFifoUsed;
    HAL_CAN_AddTxMessage(&hcan1, &TxMessage, Data, &TxFifoUsed);
}
