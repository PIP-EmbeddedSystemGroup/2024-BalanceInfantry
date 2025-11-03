/**
 * @attention   采用UTF-8字符集编码
 * @brief       调试任务
 * @authors     北工大PIP战队 樊郡捷
 * @version     v1.0
 * @date        2024-10-14
 * @details     发送数据供上位机绘制曲线
 */

#include "header.h"

#define DEBUG_TASK_UPDATE_TICK 10

struct
{
    uint32_t Head;
    float Value[6];
    uint32_t Tail;
} DebugData = { 0 };

void DebugTask(void const* argument)
{
    uint32_t PreviousWakeTime = osKernelSysTick();

    DebugData.Head = 0x55AA55AA;
    DebugData.Tail = 0xAA55AA55;

    while (1)
    {
        osDelayUntil(&PreviousWakeTime, DEBUG_TASK_UPDATE_TICK);

        DebugData.Value[0] = Vision.TargetYaw.Angle;
        DebugData.Value[1] = Vision.TargetYaw.AngularSpeed;
        DebugData.Value[2] = 0;
        DebugData.Value[3] = 0;
        DebugData.Value[4] = 0;
        DebugData.Value[5] = 0;

        HAL_UART_Transmit_DMA(&huart6, (uint8_t*)&DebugData, sizeof(DebugData));

        // CDC_Transmit_FS((uint8_t*)&DebugData, sizeof(DebugData));
    }
    // while (1)
    // {
    //     // osDelayUntil(&PreviousWakeTime, DEBUG_TASK_UPDATE_TICK);
    //     osThreadSuspend(osThreadGetId());

    //     for (int i = 0; i < 10; i++)
    //     {
    //         osDelay(10);

    //         DebugData.Value[0] = Vision.TargetYaw.FreqMag[i];
    //         DebugData.Value[1] = 0;
    //         DebugData.Value[2] = 0;
    //         DebugData.Value[3] = 0;
    //         DebugData.Value[4] = 0;
    //         DebugData.Value[5] = 0;

    //         HAL_UART_Transmit_DMA(&huart6, (uint8_t*)&DebugData, sizeof(DebugData));
    //     }

    //     // CDC_Transmit_FS((uint8_t*)&DebugData, sizeof(DebugData));
    // }
}
