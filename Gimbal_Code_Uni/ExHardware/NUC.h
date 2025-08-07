#ifndef _NUC_H_
#define _NUC_H_

/**
 * @attention   采用UTF-8字符集编码
 */

#include "stm32f4xx_hal.h"
#include "stdint.h"

typedef struct
{
    float PitchAngle;
    float YawAngle;
    float YawAngleRaw;
    float Distance;
    struct
    {
        uint8_t X;
        uint8_t Y;
    } Position;
    int LossFrame;
    int TargetID;

    struct
    {
        struct
        {
            int16_t X;
            int16_t Y;
            int16_t Z;
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
    } RC;

    int UpdateCounter;
    uint32_t LastFreqUpdateTick;
    int Freq;

    int ErrorCount;
    uint32_t LastOnlineTick;
} NUC_t;

void NUC_Init(void);
void NUC_SendConfig(void);
void NUC_RxCallback(int isFromUart);

extern uint8_t NucRxBuf[32];
extern NUC_t NUC;

#endif
