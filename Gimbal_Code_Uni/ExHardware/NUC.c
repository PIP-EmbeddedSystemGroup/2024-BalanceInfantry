/**
 * @attention   采用UTF-8字符集编码
 * @brief       NUC数据收发
 * @authors     北工大PIP战队 樊郡捷
 * @version     v1.0
 * @date        2024-10-24
 * @details
 */

#include "header.h"

#define NUC_PACKAGE_LENGTH 32
uint8_t NucRxBuf[NUC_PACKAGE_LENGTH] = { 0 };
uint8_t NucTxBuf[8] = { 0 };
NUC_t NUC;

float tempy = 0;//4.f;	
float tempp = 0;

extern osThreadId VisionHandle;

void NUC_Init(void)
{
    NUC.UpdateCounter = 0;
    NUC.LastFreqUpdateTick = 0;
    NUC.Freq = 100;

    NUC.ErrorCount = 0;
    NUC.LastOnlineTick = 0;

    HAL_UART_Receive_DMA(&huart1, NucRxBuf, NUC_PACKAGE_LENGTH);
}

void NUC_SendConfig(void)
{
    NucTxBuf[0] = 0xFF;
    NucTxBuf[1] = 0xFF;
    NucTxBuf[2] = Controller.VisionMode;
    NucTxBuf[3] = Controller.AimingInvRotateEnable ? 0xE9 : 0x53;
    NucTxBuf[5] = 0x00;   //?
    NucTxBuf[6] = 0x53;   //53为瞄准蓝色，E9瞄准红色
    NucTxBuf[7] = Controller.AimingOn ? 0xE9 : 0x53;

    HAL_UART_Transmit_IT(&huart1, NucTxBuf, 8);
    CDC_Transmit_FS(NucTxBuf, 8);
}

void NUC_RC_RxCallback(void)
{
    uint32_t NowTick = HAL_GetTick();

    if (NowTick - RC_VTM.LastOnlineTick > 100)
    {
        NUC.RC.Mouse.X = NucRxBuf[22] | (NucRxBuf[23] << 8);	//Mouse X axis
        NUC.RC.Mouse.Y = NucRxBuf[24] | (NucRxBuf[25] << 8);	//Mouse Y axis
        NUC.RC.Mouse.Z = NucRxBuf[26] | (NucRxBuf[27] << 8);	//Mouse Z axis
        NUC.RC.Mouse.L = NucRxBuf[28];							//Mouse Left Is Press
        NUC.RC.Mouse.R = NucRxBuf[29];							//Mouse Right Is Press
        NUC.RC.Key.Raw = NucRxBuf[30] | (NucRxBuf[31] << 8);	//KeyBoard value

        Mouse.X = NUC.RC.Mouse.X * 0.5f;
        Mouse.Y = NUC.RC.Mouse.Y * 0.5f;
        Mouse.Z = NUC.RC.Mouse.Z;
        Mouse.L.Value = NUC.RC.Mouse.L;
        Mouse.R.Value = NUC.RC.Mouse.R;

        Keyboard.W.Value = NUC.RC.Key.Code.W;
        Keyboard.S.Value = NUC.RC.Key.Code.S;
        Keyboard.A.Value = NUC.RC.Key.Code.A;
        Keyboard.D.Value = NUC.RC.Key.Code.D;
        Keyboard.Shift.Value = NUC.RC.Key.Code.Shift;
        Keyboard.Ctrl.Value = NUC.RC.Key.Code.Ctrl;
        Keyboard.Q.Value = NUC.RC.Key.Code.Q;
        Keyboard.E.Value = NUC.RC.Key.Code.E;
        Keyboard.R.Value = NUC.RC.Key.Code.R;
        Keyboard.F.Value = NUC.RC.Key.Code.F;
        Keyboard.G.Value = NUC.RC.Key.Code.G;
        Keyboard.Z.Value = NUC.RC.Key.Code.Z;
        Keyboard.X.Value = NUC.RC.Key.Code.X;
        Keyboard.C.Value = NUC.RC.Key.Code.C;
        Keyboard.V.Value = NUC.RC.Key.Code.V;
        Keyboard.B.Value = NUC.RC.Key.Code.B;
    }

    ChassisBoard_SendRcVision();
}

void NUC_RxCallback(int isFromUart)
{
    int FrameHeaderOffset = 0;
    uint32_t NowTick = HAL_GetTick();

    for (FrameHeaderOffset = 0; FrameHeaderOffset < NUC_PACKAGE_LENGTH; FrameHeaderOffset++)
        if (NucRxBuf[FrameHeaderOffset] == 0xFF)
            break;

    if (FrameHeaderOffset != 0) //出现数据包错位
    {
        NUC.ErrorCount++;

        if (FrameHeaderOffset != 0)
        {
            if (isFromUart)
            {
                HAL_UART_DMAStop(&huart1);
                HAL_UART_Receive(&huart1, NucRxBuf, FrameHeaderOffset, 1);
                HAL_UART_Receive_DMA(&huart1, NucRxBuf, NUC_PACKAGE_LENGTH);
            }
            return;
        }
    }

    NUC.PitchAngle = ((float)((NucRxBuf[2] << 8) | NucRxBuf[3]) - 32768) / 100 - tempp;// / 180 * NORMAL_PI;
    // NUC.YawAngle = ((float)((NucRxBuf[4] << 8) | NucRxBuf[5]) - 32768) / 100;// / 180 * NORMAL_PI;
    NUC.Distance = ((float)((NucRxBuf[6] << 8) | (NucRxBuf[7]))) / 1000.0f;
    if (NucRxBuf[8] != 0xB0 || NucRxBuf[10] != 0xD7)
        return;
    NUC.Position.X = NucRxBuf[9];
    NUC.Position.Y = NucRxBuf[11];
    NUC.YawAngleRaw = ((float)((NucRxBuf[12] << 8) | NucRxBuf[13]) - 32768) / 100;// / 180 * NORMAL_PI;
    NUC.YawAngle = ((float)((NucRxBuf[12] << 8) | NucRxBuf[13]) - 32768) / 100 - tempy;// / 180 * NORMAL_PI;

    NUC.LossFrame = NucRxBuf[14];
    NUC.TargetID = NucRxBuf[15] - 10;
    
    NUC_RC_RxCallback();

    NUC.UpdateCounter++;
    if (NowTick - NUC.LastFreqUpdateTick > 500)
    {
        NUC.Freq = NUC.UpdateCounter * 1000 / (NowTick - NUC.LastFreqUpdateTick);
        NUC.UpdateCounter = 0;
        NUC.LastFreqUpdateTick = NowTick;
    }

    NUC.LastOnlineTick = NowTick;

    if (VisionHandle)
        osThreadResume(VisionHandle);
}
