#ifndef _DEBUG_TASK_H_
#define _DEBUG_TASK_H_

/**
 * @attention   采用UTF-8字符集编码
 */

#include "stdint.h"
#include "cmsis_os.h"

#define DEBUG_MODE_NORMAL   0
#define DEBUG_MODE_FFT      1
#define DEBUG_MODE          DEBUG_MODE_NORMAL

void DebugTask(void const * argument);

#endif
