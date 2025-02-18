#ifndef _RC_DBUS_H_
#define _RC_DBUS_H_

/**
 * @attention   采用UTF-8字符集编码
 */

#include "stm32f4xx_hal.h"
#include "stdint.h"

typedef struct
{
    struct
    {
        uint16_t CH0;
        uint16_t CH1;
        uint16_t CH2;
        uint16_t CH3;
        uint16_t CH4;
        uint8_t S1;
        uint8_t S2;
    } Pad;  //手柄
    struct
    {
        int16_t X;
        int16_t Y;
        int16_t Z;  //鼠标滚轮 十分不灵敏
        uint8_t L;
        uint8_t R;
    } Mouse;
    union   //联合体: 使Code结构体和Raw变量共用同一内存空间
    {
        struct
        {
            int W : 1;
            int S : 1;
            int A : 1;
            int D : 1;
            int Shift : 1;
            int Ctrl : 1;
            int Q : 1;
            int E : 1;
            int R : 1;
            int F : 1;
            int G : 1;
            int Z : 1;
            int X : 1;
            int C : 1;
            int V : 1;
            int B : 1;
        } Code; //位域结构体: 其成员所占用的空间按位计算 (引用时由编译器自动生成位运算操作)

        uint16_t Raw;
    } Key;

    uint32_t LastOnlineTick;

    int UpdateCounter;
    uint32_t LastFreqUpdateTick;
    int Freq;
} RC_DBUS_t;

void RC_DBUS_Init(void);
void RC_DBUS_RxCallback(void);

extern uint8_t DbusBuf[18];
extern RC_DBUS_t RC_DBUS;

#endif
