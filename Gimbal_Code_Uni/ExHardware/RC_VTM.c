/**
 * @attention   采用UTF-8字符集编码
 * @brief       RM图传链路控制器(VTM) 接收函数
 * @authors     北工大PIP战队 樊郡捷
 * @version     v1.0
 * @date        2024-10-14
 * @details     
 */

#include "header.h"
#include "RC_VTM.h"

RC_VTM_t RC_VTM = { 0 };

void RC_VTM_Init(void)
{
    RC_VTM.Key.Raw = 0;
    RC_VTM.Mouse.L = 0;
    RC_VTM.Mouse.R = 0;
    RC_VTM.Mouse.X = 0;
    RC_VTM.Mouse.Y = 0;
    RC_VTM.Mouse.Z = 0;

    RC_VTM.LastOnlineTick = 0;
}

void RC_VTM_RxCallback(void)
{
    Mouse.X = RC_VTM.Mouse.X;
    Mouse.Y = RC_VTM.Mouse.Y;
    Mouse.Z = RC_VTM.Mouse.Z;
    Mouse.L.Value = RC_VTM.Mouse.L;
    Mouse.R.Value = RC_VTM.Mouse.R;

    Keyboard.W.Value = RC_VTM.Key.Code.W;
    Keyboard.S.Value = RC_VTM.Key.Code.S;
    Keyboard.A.Value = RC_VTM.Key.Code.A;
    Keyboard.D.Value = RC_VTM.Key.Code.D;
    Keyboard.Shift.Value = RC_VTM.Key.Code.Shift;
    Keyboard.Ctrl.Value = RC_VTM.Key.Code.Ctrl;
    Keyboard.Q.Value = RC_VTM.Key.Code.Q;
    Keyboard.E.Value = RC_VTM.Key.Code.E;
    Keyboard.R.Value = RC_VTM.Key.Code.R;
    Keyboard.F.Value = RC_VTM.Key.Code.F;
    Keyboard.G.Value = RC_VTM.Key.Code.G;
    Keyboard.Z.Value = RC_VTM.Key.Code.Z;
    Keyboard.X.Value = RC_VTM.Key.Code.X;
    Keyboard.C.Value = RC_VTM.Key.Code.C;
    Keyboard.V.Value = RC_VTM.Key.Code.V;
    Keyboard.B.Value = RC_VTM.Key.Code.B;

    uint32_t NowTick = HAL_GetTick();
    RC_VTM.LastOnlineTick = NowTick;

    RC_VTM.UpdateCounter++;
    if (NowTick - RC_VTM.LastFreqUpdateTick > 2000)
    {
        RC_VTM.Freq = RC_VTM.UpdateCounter * 1000 / (NowTick - RC_VTM.LastFreqUpdateTick);
        RC_VTM.UpdateCounter = 0;
        RC_VTM.LastFreqUpdateTick = NowTick;
    }

    ChassisBoard_SendVTM();
}

