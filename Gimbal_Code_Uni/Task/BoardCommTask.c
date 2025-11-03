/**
 * @attention   采用UTF-8字符集编码
 * @brief       上下板通信任务
 * @authors     北工大PIP战队 樊郡捷
 * @version     v1.0
 * @date        2024-10-14
 * @details     采用RTOS的邮箱机制 扩展了CAN外设原本只有3个的硬件发送缓冲区
 */

#include "header.h"

#define BOARD_COMM_TASK_UPDATE_TICK 1

/// @brief CAN的发送邮箱只有三个缓冲 在这个任务中通过消息队列的方式增加一个外部缓冲
///        COMM. -> communication
/// @param argument 
void BoardCommTask(void const* argument)
{
    uint32_t PreviousWakeTime = osKernelSysTick();

    CAN_TxHeaderTypeDef	TxMessage = { 0 };
    uint8_t tx_switch = 1;
    uint8_t Data[8] = { 0 };

    while (1)
    {
        osDelayUntil(&PreviousWakeTime, BOARD_COMM_TASK_UPDATE_TICK);

        TxMessage.DLC = 8;//数据

        if (tx_switch == 1)
        {
            TxMessage.StdId = 0x206;
            Data[0] = Controller.FrictionOn ? 1 : 0;// Shooting.Friction;
            Data[1] = 1;//Command.Cap_Flag;
            tx_switch = tx_switch << 1;
        }
        else if (tx_switch == 2)
        {
            TxMessage.StdId = 0x207;
            Data[0] = Controller.isJump ? 1 : 0;
            Data[1] = 0;//Command.UIFlag;
            tx_switch = tx_switch >> 1;
        }
        Data[2] = Controller.HeightSet;
        Data[3] = Controller.isEnable ? 1 : 0;
        int16_t MoveFB = (int16_t)Controller.Move.FB.Set;
        Data[4] = (uint8_t)(MoveFB >> 8);
        Data[5] = (uint8_t)MoveFB;
        Data[6] = Controller.Rotate_Flag;
        Data[7] = 0;

        uint32_t TxFifoUsed;
        HAL_CAN_AddTxMessage(&hcan2, &TxMessage, Data, &TxFifoUsed);
    }
}

/// @brief CAN的发送邮箱只有三个缓冲 在这个任务中通过消息队列的方式增加一个外部缓冲
///        COMM. -> communication
/// @param argument 
// void BoardCommTask(void const* argument)
// {
//     BoardCommTxMailQueue = osMailCreate(osMailQ(BoardCommTx), NULL);

//     while (1)
//     {
//         osEvent MailEvent;
//         BoardCommCanMail_t* TxMail;

//         MailEvent = osMailGet(BoardCommTxMailQueue, osWaitForever);    //如果队列为空 则调用此函数的任务将被挂起 直到队列中有消息可用
//         TxMail = (BoardCommCanMail_t*)MailEvent.value.p;

//         CAN_TxHeaderTypeDef TxMessage = { 0 };
//         TxMessage.StdId = TxMail->ID;
//         TxMessage.DLC = TxMail->DLC;
//         while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan2) == 0)
//             osDelay(1);
//         uint32_t TxFifoUsed;
//         HAL_CAN_AddTxMessage(&hcan2, &TxMessage, TxMail->Data, &TxFifoUsed);

//         osMailFree(BoardCommTxMailQueue, TxMail);
//     }
// }
