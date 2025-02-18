/**
 * @attention   采用UTF-8字符集编码
 * @brief       底盘接口
 * @authors     北工大PIP战队 樊郡捷
 * @version     v1.0
 * @date        2024-10-14
 * @details     负责实现上下板通信逻辑
 */

#include "header.h"

Chassis_t Chassis;

void Chassis_Init(void)
{
    Chassis.LastOnlineTick = 0;
}



void ChassisBoard_ReceiveError(CAN_RxHeaderTypeDef* canHeader, uint8_t data[8])
{
    switch (canHeader->StdId)
    {
    case BOARD_COMM_CHASSIS_ERROR_CANID:
        Daemon.ErrorFlag &= ~0x0000FFFF;    //清空低16位
        Daemon.ErrorFlag |= ((data[0] << 8) | data[1]);
        Chassis.LastOnlineTick = HAL_GetTick();
        break;
    default:
        break;
    }
}

void ChassisBoard_ReceiveSpeed(CAN_RxHeaderTypeDef* canHeader, uint8_t data[8])
{
    switch (canHeader->StdId)
    {
    case BOARD_COMM_CHASSIS_ODOMETER_SPEED_CANID:
        memcpy(&Controller.Move.FB.Real, &data[0], 4);
        memcpy(&Controller.Move.LR.Real, &data[4], 4);
        FusionKF_UpdateMeasurement(&FusionINS.KF, Controller.Move.FB.Real, Controller.Move.LR.Real);
        break;
    default:
        break;
    }
}

/// @brief 每当云台接收到DBUS数据的时候 将其转发至底盘
void ChassisBoard_SendDBUS(void)
{
    // BoardCommCanMail_t* PadMail = (BoardCommCanMail_t*)osMailCAlloc(BoardCommTxMailQueue, 0);
    // if (PadMail != NULL)
    // {
    //     PadMail->ID = BOARD_COMM_DBUS_PAD_CANID;
    //     PadMail->DLC = 8;
    //     memcpy(&PadMail->Data[0], &DbusBuf[0], 6);      //ch0-ch3 s1 s2
    //     memcpy(&PadMail->Data[6], &DbusBuf[16], 2);     //ch4
    //     osMailPut(BoardCommTxMailQueue, PadMail);
    // }

    // BoardCommCanMail_t* MouseMail = (BoardCommCanMail_t*)osMailCAlloc(BoardCommTxMailQueue, 0);
    // if (MouseMail != NULL)
    // {
    //     MouseMail->ID = BOARD_COMM_DBUS_MOUSE_CANID;
    //     MouseMail->DLC = 8;
    //     memcpy(&MouseMail->Data[0], &DbusBuf[6], 8);    //x y z l r
    //     osMailPut(BoardCommTxMailQueue, MouseMail);
    // }

    // BoardCommCanMail_t* KeyMail = (BoardCommCanMail_t*)osMailCAlloc(BoardCommTxMailQueue, 0);
    // if (KeyMail != NULL)
    // {
    //     KeyMail->ID = BOARD_COMM_DBUS_KEY_CANID;
    //     KeyMail->DLC = 2;
    //     memcpy(&KeyMail->Data[0], &DbusBuf[14], 2);     //key raw
    //     osMailPut(BoardCommTxMailQueue, KeyMail);
    // }
}

/// @brief 每当云台接收到图传链路的控制数据的时候 将其转发至底盘
void ChassisBoard_SendVTM(void)
{
    // BoardCommCanMail_t* MouseMail = (BoardCommCanMail_t*)osMailCAlloc(BoardCommTxMailQueue, 0);
    // if (MouseMail != NULL)
    // {
    //     MouseMail->ID = BOARD_COMM_VTM_MOUSE_CANID;
    //     MouseMail->DLC = 8;
    //     memcpy(&MouseMail->Data[0], &DbusBuf[6], 8);    //x y z l r
    //     osMailPut(BoardCommTxMailQueue, MouseMail);
    // }

    // BoardCommCanMail_t* KeyMail = (BoardCommCanMail_t*)osMailCAlloc(BoardCommTxMailQueue, 0);
    // if (KeyMail != NULL)
    // {
    //     KeyMail->ID = BOARD_COMM_VTM_KEY_CANID;
    //     KeyMail->DLC = 2;
    //     memcpy(&KeyMail->Data[0], &DbusBuf[14], 2);     //key raw
    //     osMailPut(BoardCommTxMailQueue, KeyMail);
    // }
}

/// @brief 每当云台接收到视觉控制链路的控制数据的时候 将其转发至底盘
void ChassisBoard_SendRcVision(void)
{
    // BoardCommCanMail_t* MouseMail = (BoardCommCanMail_t*)osMailCAlloc(BoardCommTxMailQueue, 0);
    // if (MouseMail != NULL)
    // {
    //     MouseMail->ID = BOARD_COMM_RC_VISION_MOUSE_CANID;
    //     MouseMail->DLC = 8;
    //     memcpy(&MouseMail->Data[0], &NucRxBuf[22], 8);  //x y z l r
    //     osMailPut(BoardCommTxMailQueue, MouseMail);
    // }

    // BoardCommCanMail_t* KeyMail = (BoardCommCanMail_t*)osMailCAlloc(BoardCommTxMailQueue, 0);
    // if (KeyMail != NULL)
    // {
    //     KeyMail->ID = BOARD_COMM_RC_VISION_KEY_CANID;
    //     KeyMail->DLC = 2;
    //     memcpy(&KeyMail->Data[0], &NucRxBuf[30], 2);    //key raw
    //     osMailPut(BoardCommTxMailQueue, KeyMail);
    // }
}

void ChassisBoard_SendYawOutput(float YawCurrentSet)
{
    CAN_TxHeaderTypeDef	TxMessage = { 0 };
	TxMessage.StdId = 0x1FF;
	TxMessage.DLC = 8;

    uint8_t Data[8] = { 0 };
    int16_t YawCurrentSet_int16 = (int16_t)YawCurrentSet;
	Data[0] = 0;
	Data[1] = 0;
	Data[2] = 0; // Pitch;
	Data[3] = 0;
	Data[4] = 0;
	Data[5] = 0;
	Data[6] = (uint8_t)(YawCurrentSet_int16 >> 8);
	Data[7] = (uint8_t)YawCurrentSet_int16;
    uint32_t TxFifoUsed;
    HAL_CAN_AddTxMessage(&hcan2, &TxMessage, Data, &TxFifoUsed);

    // BoardCommCanMail_t* YawOutputMail = (BoardCommCanMail_t*)osMailCAlloc(BoardCommTxMailQueue, 0);
    // if (YawOutputMail != NULL)
    // {
    //     YawOutputMail->ID = BOARD_COMM_YAW_OUTPUT_CANID;
    //     YawOutputMail->DLC = 2;
    //     int16_t YawCurrentSet_int16 = (int16_t)YawCurrentSet;
    //     YawOutputMail->Data[0] = (uint8_t)(YawCurrentSet_int16 >> 8);
    //     YawOutputMail->Data[1] = (uint8_t)YawCurrentSet_int16;
    //     osMailPut(BoardCommTxMailQueue, YawOutputMail);
    // }
}

void ChassisBoard_SendError(void)
{
    // BoardCommCanMail_t* ErrorMail = (BoardCommCanMail_t*)osMailCAlloc(BoardCommTxMailQueue, 0);
    // if (ErrorMail != NULL)
    // {
    //     ErrorMail->ID = BOARD_COMM_GIMBAL_ERROR_CANID;
    //     ErrorMail->DLC = 2;
    //     ErrorMail->Data[0] = (uint8_t)(Daemon.ErrorFlag >> 8 >> 16);
    //     ErrorMail->Data[1] = (uint8_t)(Daemon.ErrorFlag >> 16);
    //     osMailPut(BoardCommTxMailQueue, ErrorMail);
    // }
}

void ChassisBoard_SendFusionInsSpeed(void)
{
    // BoardCommCanMail_t* SpeedMail = (BoardCommCanMail_t*)osMailCAlloc(BoardCommTxMailQueue, 0);
    // if (SpeedMail != NULL)
    // {
    //     SpeedMail->ID = BOARD_COMM_GIMBAL_FUSION_INS_SPEED_CANID;
    //     SpeedMail->DLC = 8;
    //     int16_t RobotFB = FusionINS.Robot.Speed.FB / 4.f * 32768;
    //     SpeedMail->Data[0] = (uint8_t)(RobotFB >> 8);
    //     SpeedMail->Data[1] = (uint8_t)(RobotFB);
    //     int16_t RobotLR = FusionINS.Robot.Speed.LR / 4.f * 32768;
    //     SpeedMail->Data[2] = (uint8_t)(RobotLR >> 8);
    //     SpeedMail->Data[3] = (uint8_t)(RobotLR);
    //     int16_t WorldX = FusionINS.World.Speed.X / 4.f * 32768;
    //     SpeedMail->Data[4] = (uint8_t)(WorldX >> 8);
    //     SpeedMail->Data[5] = (uint8_t)(WorldX);
    //     int16_t WorldY = FusionINS.World.Speed.Y / 4.f * 32768;
    //     SpeedMail->Data[6] = (uint8_t)(WorldY >> 8);
    //     SpeedMail->Data[7] = (uint8_t)(WorldY);
    //     osMailPut(BoardCommTxMailQueue, SpeedMail);
    // }
}
