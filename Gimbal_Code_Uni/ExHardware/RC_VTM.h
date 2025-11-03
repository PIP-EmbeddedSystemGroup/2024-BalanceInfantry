#ifndef _RC_VTM_H_
#define _RC_VTM_H_

/**
 * @attention   采用UTF-8字符集编码
 */

#include "stm32f4xx_hal.h"
#include "stdint.h"

typedef struct
{
    struct
    {
        int16_t X;
        int16_t Y;
        int16_t Z;  //鼠标滚轮 十分不灵敏
        uint8_t L;
        uint8_t R;
    } Mouse;
    union
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
        } Code;

        uint16_t Raw;
    } Key;

    uint32_t LastOnlineTick;

    int UpdateCounter;
    uint32_t LastFreqUpdateTick;
    int Freq;
} RC_VTM_t;

void RC_VTM_Init(void);
void RC_VTM_RxCallback(void);

extern RC_VTM_t RC_VTM;

#endif
