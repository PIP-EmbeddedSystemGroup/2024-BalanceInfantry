/**
 * @attention   采用UTF-8字符集编码
 * @brief       DT7 DBUS接收
 * @authors     北工大PIP战队 樊郡捷
 * @version     v1.0
 * @date        2024-10-14
 * @details     以后有新遥控器的话 应该再进一步做封装 继承这个类型
 */

#include "header.h"

uint8_t DbusBuf[18];
RC_DBUS_t RC_DBUS = { 0 };

void RC_DBUS_Init(void)
{
    RC_DBUS.Pad.CH0 = 1024;
    RC_DBUS.Pad.CH1 = 1024;
    RC_DBUS.Pad.CH2 = 1024;
    RC_DBUS.Pad.CH3 = 1024;
    RC_DBUS.Pad.CH4 = 1024;
    RC_DBUS.Pad.S1 = 3;
    RC_DBUS.Pad.S2 = 3;

    RC_DBUS.Key.Raw = 0;
    RC_DBUS.Mouse.L = 0;
    RC_DBUS.Mouse.R = 0;
    RC_DBUS.Mouse.X = 0;
    RC_DBUS.Mouse.Y = 0;
    RC_DBUS.Mouse.Z = 0;

    RC_DBUS.LastOnlineTick = 0;

    RC_DBUS.UpdateCounter = 0;
    RC_DBUS.LastFreqUpdateTick = 0;
    RC_DBUS.Freq = 0;

    HAL_UART_Receive_DMA(&huart3, DbusBuf, 18);
    //UART回调函数编写在usart.c文件中
}

void RC_DBUS_RxCallback(void)
{
    RC_DBUS.Pad.CH0 = (DbusBuf[0] | (DbusBuf[1] << 8)) & 0x07ff;                                //Channel 0
    RC_DBUS.Pad.CH1 = ((DbusBuf[1] >> 3) | (DbusBuf[2] << 5)) & 0x07ff;                         //Channel 1
    RC_DBUS.Pad.CH2 = ((DbusBuf[2] >> 6) | (DbusBuf[3] << 2) | (DbusBuf[4] << 10)) & 0x07ff;    //Channel 2
    RC_DBUS.Pad.CH3 = ((DbusBuf[4] >> 1) | (DbusBuf[5] << 7)) & 0x07ff;                         //Channel 3
    RC_DBUS.Pad.CH4 = DbusBuf[16] | (DbusBuf[17] << 8);
    RC_DBUS.Pad.S1 = ((DbusBuf[5] >> 4) & 0x000C) >> 2;                                         //SwitCH left
    RC_DBUS.Pad.S2 = ((DbusBuf[5] >> 4) & 0x0003);                                              //SwitCH right

    RC_DBUS.Mouse.X = DbusBuf[6] | (DbusBuf[7] << 8);                                           //Mouse X axis
    RC_DBUS.Mouse.Y = DbusBuf[8] | (DbusBuf[9] << 8);                                           //Mouse Y axis
    RC_DBUS.Mouse.Z = DbusBuf[10] | (DbusBuf[11] << 8);                                         //Mouse Z axis
    RC_DBUS.Mouse.L = DbusBuf[12];                                                              //Mouse Left Is Press
    RC_DBUS.Mouse.R = DbusBuf[13];                                                              //Mouse Right Is Press
    RC_DBUS.Key.Raw = DbusBuf[14] | (DbusBuf[15] << 8);                                         //KeyBoard value

    uint32_t NowTick = HAL_GetTick();
    RC_DBUS.LastOnlineTick = NowTick;

    if (NowTick - RC_VTM.LastOnlineTick > 100 &&
        NowTick - NUC.LastOnlineTick > 100)
    {
        Mouse.X = RC_DBUS.Mouse.X;
        Mouse.Y = RC_DBUS.Mouse.Y;
        Mouse.Z = RC_DBUS.Mouse.Z;
        Mouse.L.Value = RC_DBUS.Mouse.L;
        Mouse.R.Value = RC_DBUS.Mouse.R;

        Keyboard.W.Value = RC_DBUS.Key.Code.W;
        Keyboard.S.Value = RC_DBUS.Key.Code.S;
        Keyboard.A.Value = RC_DBUS.Key.Code.A;
        Keyboard.D.Value = RC_DBUS.Key.Code.D;
        Keyboard.Shift.Value = RC_DBUS.Key.Code.Shift;
        Keyboard.Ctrl.Value = RC_DBUS.Key.Code.Ctrl;
        Keyboard.Q.Value = RC_DBUS.Key.Code.Q;
        Keyboard.E.Value = RC_DBUS.Key.Code.E;
        Keyboard.R.Value = RC_DBUS.Key.Code.R;
        Keyboard.F.Value = RC_DBUS.Key.Code.F;
        Keyboard.G.Value = RC_DBUS.Key.Code.G;
        Keyboard.Z.Value = RC_DBUS.Key.Code.Z;
        Keyboard.X.Value = RC_DBUS.Key.Code.X;
        Keyboard.C.Value = RC_DBUS.Key.Code.C;
        Keyboard.V.Value = RC_DBUS.Key.Code.V;
        Keyboard.B.Value = RC_DBUS.Key.Code.B;
    }

    RC_DBUS.UpdateCounter++;
    if (NowTick - RC_DBUS.LastFreqUpdateTick > 2000)
    {
        RC_DBUS.Freq = RC_DBUS.UpdateCounter * 1000 / (NowTick - RC_DBUS.LastFreqUpdateTick);
        RC_DBUS.UpdateCounter = 0;
        RC_DBUS.LastFreqUpdateTick = NowTick;
    }

    ChassisBoard_SendDBUS();
}
