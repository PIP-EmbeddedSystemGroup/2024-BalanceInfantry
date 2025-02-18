#ifndef _BOARD_COMM_TASK_H_
#define _BOARD_COMM_TASK_H_

/**
 * @attention   采用UTF-8字符集编码
 */

#include "stdint.h"
#include "cmsis_os.h"

typedef struct
{
    int ID;     //报文ID
    int DLC;    //数据长度
    uint8_t Data[8];    //数据
} BoardCommCanMail_t;

void BoardComm_Init(void);
void BoardCommTask(void const* argument);

extern osMailQId BoardCommTxMailQueue;

#endif
