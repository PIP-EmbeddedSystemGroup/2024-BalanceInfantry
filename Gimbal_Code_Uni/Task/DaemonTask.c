/**
 * @attention   采用UTF-8字符集编码
 * @brief       守护进程任务
 * @authors     北工大PIP战队 樊郡捷
 * @version     v1.1
 * @date        2024-10-25
 * @details     以较低频率监控机器人运行状态
 *              出现意料之外的情况时重启机器人
 *              实时更新上下板的ErrorFlag
 *              计算CPU占用率
 */

#include "header.h"

Daemon_t Daemon;

volatile int IdleCounter = 0;

void IdleTask(void const* argument)
{
    HAL_TIM_Base_Start(&htim7); //1MHz计数

    while (1)
    {
        //记录空闲的微秒数
        int CounterVal = __HAL_TIM_GetCounter(&htim7);
        while (CounterVal == __HAL_TIM_GetCounter(&htim7));
        IdleCounter++;
    }
}

void DaemonTask(void const* argument)
{
    uint32_t PreviousWakeTime = osKernelSysTick();

    Daemon.ErrorFlag = 0;
    Daemon.CpuUsage = 0;

    //USB初始化
    MX_USB_DEVICE_Init();

    //开激光
    Misc_Laser_Off();

    //创建一个空闲任务用于统计CPU占用率
    osThreadDef(Idle, IdleTask, osPriorityIdle, 0, 128);
    osThreadCreate(osThread(Idle), NULL);

    while (1)
    {
        osDelayUntil(&PreviousWakeTime, 100);

        Daemon.CpuUsage = 100.f - (IdleCounter / 1000000.f / 0.1f * 100.f);  //1000000是定时器频率 0.1是统计周期 100是百分比
        IdleCounter = 0;

        uint32_t NowTick = HAL_GetTick();

        // ChassisBoard_SendError();   //将底盘错误标志同步到云台
        // ChassisBoard_SendState();

        //遥控器掉线超过1秒 且未处于电脑控制模式或图传链路也掉线时 重启程序
        if (NowTick - RC_DBUS.LastOnlineTick > 1000)
            if (RC_DBUS.Pad.S2 != RC_DBUS_S2_COMPUTER_CTRL || NowTick - RC_VTM.LastOnlineTick > 200)
                HAL_NVIC_SystemReset();

        if (RC_DBUS.Pad.CH0 < 364 || RC_DBUS.Pad.CH0 > 1684 || RC_DBUS.Pad.CH1 < 364 || RC_DBUS.Pad.CH1 > 1684)
            HAL_NVIC_SystemReset();
    }
}
