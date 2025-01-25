/*
 * @Author: 静_火 lintianlang0918@qq.com
 * @Date: 2024-03-07 19:47:56
 * @LastEditors: 静_火 lintianlang0918@qq.com
 * @LastEditTime: 2024-03-20 09:05:19
 * @FilePath: \MC-01_chassis\Task\Monitor_Task.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "stdint.h"


extern uint16_t Status_Counter[2];

void Monitor_Task(void const * argument);
void Robot_Fall_Detect(void);
void Robot_Param_Switch (void);
void Robot_Task_Monitor(void);