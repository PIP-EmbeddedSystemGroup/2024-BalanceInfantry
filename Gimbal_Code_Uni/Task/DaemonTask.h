#ifndef _DAEMON_TASK_H_
#define _DAEMON_TASK_H_

/**
 * @attention   采用UTF-8字符集编码
 */

#include "stdint.h"
#include "cmsis_os.h"

typedef struct
{
    uint32_t ErrorFlag; //底盘和云台各16个错误标记位 底盘低8位 通过上下板通信传递 取值为ERR_xxx(RobotConfig.h)
    float CpuUsage;
} Daemon_t;

void DaemonTask(void const* argument);

extern Daemon_t Daemon;

#endif
