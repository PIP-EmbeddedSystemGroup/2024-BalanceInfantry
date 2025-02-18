/**
 * @attention   采用UTF-8字符集编码
 * @brief       达妙4310电机接口
 * @authors     北工大PIP战队 樊郡捷
 * @version     v1.0
 * @date        2024-10-14
 * @details     DM4310电机的维护函数
 *              采用Motor的SpecialData指向顺序表的不同成员 进而存储了达妙电机特殊的数据 进而统一了不同电机
 */

#include "header.h"

typedef struct
{
    DM4310_t* Parent;

    int RxID;
    int TxID;
    uint16_t kP_int16;
    uint16_t kD_int16;

    int isEnable;
    int ErrorFlag;
} DM4310_Special_t;

#define DM4310_LIST_MAX_LENGTH 4
int DM4310_ListCounter = 0;
DM4310_Special_t DM4310_List[DM4310_LIST_MAX_LENGTH] = { 0 };

/// @brief DM4310初始化
/// @param motor    电机结构体
/// @param RxID     DM4310反馈报文的CANID 对应上位机中的MasterID
/// @param TxID     DM4310控制报文的CANID 对应上位机中的CANID
void DM4310_Init(DM4310_t* motor, int RxID, int TxID, float kP, float kD)
{
    motor->Type = MOTOR_TYPE_DM4310;
    motor->EncoderResolution = 65536;   //4310的位置反馈为0-65536对应-12.5rad - 12.5rad (-1.9894 - 1.9894圈) 这个值可以在上位机中调 但调为2π rad后也不对 很奇怪
    motor->GearRatio = 1 / 1.9894f / 2;     //虽然4310有减速箱 但是编码器读取的是输出轴的角度 通过设置这个值为多圈编码器圈数的导数 恰好可以在解卷绕时实现计算转子当前角度

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

    if (DM4310_ListCounter < DM4310_LIST_MAX_LENGTH)
    {
        DM4310_List[DM4310_ListCounter].Parent = motor;
        DM4310_List[DM4310_ListCounter].RxID = RxID;
        DM4310_List[DM4310_ListCounter].TxID = TxID;
        DM4310_List[DM4310_ListCounter].kP_int16 = kP * 4096 / 500;
        DM4310_List[DM4310_ListCounter].kD_int16 = kD * 4096 / 5;
        DM4310_List[DM4310_ListCounter].isEnable = RESET;
        DM4310_List[DM4310_ListCounter].ErrorFlag = 0;

        motor->SpecialData = &DM4310_List[DM4310_ListCounter];
        DM4310_ListCounter++;
    }
}

/// @brief 由于DM4310的ID需要手动设置 不具有一致性 因此调用此函数来查找有无对应ID的DM4310 再进一步进行更新
/// @param ID   报文的CANID
/// @param data 报文数据
void DM4310_TryUpdate(int ID, uint8_t data[8])
{
    for (int i = 0; i < DM4310_LIST_MAX_LENGTH; i++)
    {
        if (DM4310_List[i].Parent != NULL && DM4310_List[i].RxID == ID)
        {
            DM4310_Update(DM4310_List[i].Parent, data);
            return;
        }
    }
}

void DM4310_Update(DM4310_t* motor, uint8_t data[8])
{
    DM4310_Special_t* DM4310_Special = motor->SpecialData;

    DM4310_Special->ErrorFlag = data[0] >> 4;
    motor->Position.Real = (data[1] << 8) | (data[2]);
    motor->Speed.Real = ((data[3] << 4) | (data[4] >> 4)) / 4096.f * 60.f / 3.1416f * 60.f;
    motor->Current.Real = ((data[4] & 0xF) << 8) | (data[5]);
    motor->Temperature = data[6] > data[7] ? data[6] : data[7];

    motor->LastOnlineTick = HAL_GetTick();
}

/// @brief 控制MIT模式的DM4310 直接发送Position.Set
/// @param motor 电机结构体
void DM4310_MitOutput(DM4310_t* motor)
{
    DM4310_Special_t* DM4310_Special = motor->SpecialData;
    
    if (!DM4310_Special->isEnable)
        DM4310_Enable(motor);

    uint16_t SpeedSet = 0xFFF / 2 + motor->Speed.Set;
    uint16_t CurrentSet = 0xFFF / 2 + motor->Current.Set;

    CAN_TxHeaderTypeDef TxMessage = { 0 };
    uint8_t Data[8] = { 0 };

    TxMessage.StdId = DM4310_Special->TxID;
    TxMessage.DLC = 8;

    Data[0] = (uint8_t)(motor->Position.Set >> 8);
    Data[1] = (uint8_t)motor->Position.Set;
    Data[2] = (uint8_t)(SpeedSet >> 4);
    Data[3] = (uint8_t)(((SpeedSet << 4) & 0xF0) | ((DM4310_Special->kP_int16 >> 8) & 0x0F));
    Data[4] = (uint8_t)DM4310_Special->kP_int16;
    Data[5] = (uint8_t)(DM4310_Special->kD_int16 >> 4);
    Data[6] = (uint8_t)(((DM4310_Special->kD_int16 << 4) & 0xF0) | ((CurrentSet >> 8) & 0x0F));
    Data[7] = (uint8_t)CurrentSet;

    uint32_t TxFifoUsed;
    HAL_CAN_AddTxMessage(&hcan1, &TxMessage, Data, &TxFifoUsed);
}

void DM4310_Enable(DM4310_t* motor)
{
    DM4310_Special_t* DM4310_Special = motor->SpecialData;
    
	CAN_TxHeaderTypeDef TxMessage = { 0 };
	uint8_t Data[] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC };

	TxMessage.StdId = DM4310_Special->TxID;
	TxMessage.DLC = 8;

    uint32_t TxFifoUsed;
    HAL_CAN_AddTxMessage(&hcan1, &TxMessage, Data, &TxFifoUsed);

	DM4310_Special->isEnable = SET;
}

void DM4310_Disable(DM4310_t* motor)
{
    DM4310_Special_t* DM4310_Special = motor->SpecialData;
    
	CAN_TxHeaderTypeDef TxMessage = { 0 };
	uint8_t Data[] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD };

	TxMessage.StdId = DM4310_Special->TxID;
	TxMessage.DLC = 8;

    uint32_t TxFifoUsed;
    HAL_CAN_AddTxMessage(&hcan1, &TxMessage, Data, &TxFifoUsed);

	DM4310_Special->isEnable = RESET;
}

void DM4310_SetZero(DM4310_t* motor)
{
    DM4310_Special_t* DM4310_Special = motor->SpecialData;
    
	CAN_TxHeaderTypeDef TxMessage = { 0 };
	uint8_t Data[] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE };

	TxMessage.StdId = DM4310_Special->TxID;
	TxMessage.DLC = 8;

    uint32_t TxFifoUsed;
    HAL_CAN_AddTxMessage(&hcan1, &TxMessage, Data, &TxFifoUsed);
}
