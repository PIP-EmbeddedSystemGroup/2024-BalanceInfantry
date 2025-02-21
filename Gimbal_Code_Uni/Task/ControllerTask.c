/**
 * @attention   采用UTF-8字符集编码
 * @brief       机器人遥控任务
 * @authors     北工大PIP战队 樊郡捷
 * @version     v1.0
 * @date        2024-10-14
 * @details     采用统一的更新频率 统一了不同接收机的不同回传频率
 *              实现了整车使能和失能的归一化
 *              实现了不同控制模式的输出量的统一
 *              实现了控制量单位的统一
 */

#include "header.h"

#define CONTROLLER_TASK_UPDATE_TICK 10

Controller_t Controller = { 0 };

void Controller_Pad(void);
void Controller_MouseKeyboard(void);

void ControllerTask(void const* argument)
{
    uint32_t PreviousWakeTime = osKernelSysTick();

    Mouse_Init();
    Keyboard_Init();
    RC_DBUS_Init();
    RC_VTM_Init();

    Controller.isEnable = RESET;
    Daemon.ErrorFlag = 0;
    Controller.isSpinMode = RESET;
    Controller.FrictionOn = RESET;
    Controller.ShootingOn = RESET;
    Controller.AimingOn = RESET;
    Controller.VisionMode = VISION_MODE_ARMOR;
    Controller.AimingInvRotateEnable = SET;
    Controller.Move.FB.Set = 0;
    Controller.Move.LR.Set = 0;
    Controller.Move.FB.Real = 0;
    Controller.Move.LR.Real = 0;
    Controller.View.Pitch = 0;
    Controller.View.Yaw = 0;
    Controller.HeightSet = CTRL_HEIGHT_SET_LOW;

    Key_Init(&Controller.PadS1.Up, 0);
    Key_Init(&Controller.PadS1.Down, 0);

    while (1)
    {
        osDelayUntil(&PreviousWakeTime, CONTROLLER_TASK_UPDATE_TICK);

        uint32_t NowTick = HAL_GetTick();

        if ((RC_DBUS.Pad.S2 == 1 || RC_DBUS.Pad.S2 == 2) && //S2拨杆不在中位
            (NowTick - RC_DBUS.LastOnlineTick < 100) &&     //DBUS在线
            !Daemon.ErrorFlag)                          //底盘和云台均无错误标记
            Controller.isEnable = SET;
        else
            Controller.isEnable = RESET;

        switch (RC_DBUS.Pad.S2)
        {
        case RC_DBUS_S2_PAD_CTRL:       //上
            Controller_Pad();
            break;
        case RC_DBUS_S2_COMPUTER_CTRL:  //中
            Controller_MouseKeyboard();
            break;
        case RC_DBUS_S2_DISABLE_CTRL:   //下
        default:
            Controller.Move.FB.Set = 0;
            Controller.Move.LR.Set = 0;
            Controller.View.Pitch = 0;
            Controller.View.Yaw = Inertial.Yaw;
            Controller.isSpinMode = RESET;
            Controller.FrictionOn = RESET;
            Controller.ShootingOn = RESET;
            Controller.AimingOn = RESET;
            break;
        }
    }
}

void Controller_Pad(void)
{
    Controller.Move.FB.Set = (RC_DBUS.Pad.CH1 - 1024) / 660.f * CHASSIS_SPEED;  // m/s
    // Controller.Move.LR.Set = -(RC_DBUS.Pad.CH0 - 1024) / 660.f * CHASSIS_SPEED;
    Controller.Move.LR.Set = 0;

    if (!Controller.AimingOn)
    {
        Controller.View.Yaw += -(RC_DBUS.Pad.CH2 - 1024) / 660.f * YAW_SENSITY_PAD * (CONTROLLER_TASK_UPDATE_TICK / 1000.f);
        Controller.View.Pitch =
            PITCH_INIT_DEGREE +
            (((RC_DBUS.Pad.CH3 - 1024) > 0) ?
                ((RC_DBUS.Pad.CH3 - 1024) / 660.f * PITCH_MAX_DEGREE) :
                (-(RC_DBUS.Pad.CH3 - 1024) / 660.f * PITCH_MIN_DEGREE));
    }

    static int CH0SwitchLast = 3;
    int CH0Switch;
    if ((RC_DBUS.Pad.CH0 - 1024) > 400)
        CH0Switch = 1;
    else if ((RC_DBUS.Pad.CH0 - 1024) < -400)
        CH0Switch = 2;
    else
        CH0Switch = 3;
    switch (CH0Switch)
    {
    case 1:
        if (CH0SwitchLast == 3)
        {
            if (Controller.HeightSet == CTRL_HEIGHT_SET_HIGH)
                Controller.HeightSet = CTRL_HEIGHT_SET_MID;
            else if (Controller.HeightSet == CTRL_HEIGHT_SET_MID)
                Controller.HeightSet = CTRL_HEIGHT_SET_LOW;
            else
                Controller.HeightSet = CTRL_HEIGHT_SET_LOW;
        }
        CH0SwitchLast = 1;
        break;
    case 2:
        if (CH0SwitchLast == 3)
        {
            if (Controller.HeightSet == CTRL_HEIGHT_SET_LOW)
                Controller.HeightSet = CTRL_HEIGHT_SET_MID;
            else if (Controller.HeightSet == CTRL_HEIGHT_SET_MID)
                Controller.HeightSet = CTRL_HEIGHT_SET_HIGH;
            else
                Controller.HeightSet = CTRL_HEIGHT_SET_HIGH;
        }
        CH0SwitchLast = 2;
        break;
    case 3:
        CH0SwitchLast = 3;
        break;
    }

    Controller.isSpinMode = ((int)RC_DBUS.Pad.CH4 - 1024) > 500;
    Controller.AimingOn = ((int)RC_DBUS.Pad.CH4 - 1024) < -500;

    Controller.PadS1.Up.Value = RC_DBUS.Pad.S1 == 1;
    if (Key_Update(&Controller.PadS1.Up) == KEY_LONG_PRESS)
        Controller.FrictionOn = !Controller.FrictionOn;

    Controller.PadS1.Down.Value = RC_DBUS.Pad.S1 == 2;
    if (Key_Update(&Controller.PadS1.Down) == KEY_PRESSED && Controller.FrictionOn == SET)
        Controller.ShootingOn = SET;
    else
        Controller.ShootingOn = RESET;
}

extern int tempColor;
void Controller_MouseKeyboard(void)
{
    if (Key_Update(&Mouse.R) == KEY_PRESSED)
    {
        if (!Controller.AimingOn)
        {
            Controller.View.YawVisionOffset = 0;
            Controller.View.PitchVisionOffset = 0;
        }
        Controller.AimingOn = SET;
    }
    else
    {
        Controller.AimingOn = RESET;
    }

    if (!Controller.AimingOn)
    {
        Controller.View.Yaw += -Mouse.X * YAW_SENSITY_MOUSE;
        if (Gimbal.Pitch.Motor.Position.Real <= PITCH_MAX_ENCODER && Gimbal.Pitch.Motor.Position.Real >= PITCH_MIN_ENCODER)
            Controller.View.Pitch += -Mouse.Y * PITCH_SENSITY_MOUSE;
    }
    else
    {
        Controller.View.YawVisionOffset += -Mouse.X * YAW_SENSITY_MOUSE * 0.1f;
        Controller.View.PitchVisionOffset += -Mouse.Y * PITCH_SENSITY_MOUSE * 0.1f;
    }

    float MoveFB = 0, MoveLR = 0;
    if (Key_Update(&Keyboard.W) == KEY_PRESSED)
        MoveFB += CHASSIS_SPEED * 0.3f;
    if (Key_Update(&Keyboard.S) == KEY_PRESSED)
        MoveFB += -CHASSIS_SPEED * 0.3f;
    // if (Key_Update(&Keyboard.A) == KEY_PRESSED)
    //     MoveLR += CHASSIS_SPEED * 0.5f;
    // if (Key_Update(&Keyboard.D) == KEY_PRESSED)
    //     MoveLR += -CHASSIS_SPEED * 0.5f;
    // if (MoveFB != 0 && MoveLR != 0)
    // {
    //     MoveFB /= SQRT2;
    //     MoveLR /= SQRT2;
    // }
    // if (Key_Update(&Keyboard.Shift) == KEY_PRESSED)
    // {
    //     MoveFB *= 2;
    //     MoveLR *= 2;
    // }
    Controller.Move.FB.Set = MoveFB;
    Controller.Move.LR.Set = MoveLR;

    if (Key_Update(&Keyboard.Shift) == KEY_LONG_PRESS)
    {
        if (Controller.HeightSet == CTRL_HEIGHT_SET_MID)
            Controller.HeightSet = CTRL_HEIGHT_SET_HIGH;
        else
            Controller.HeightSet = CTRL_HEIGHT_SET_MID;
    }
    if (Key_Update(&Keyboard.Ctrl) == KEY_LONG_PRESS)
    {
        if (Controller.HeightSet == CTRL_HEIGHT_SET_MID)
            Controller.HeightSet = CTRL_HEIGHT_SET_LOW;
        else
            Controller.HeightSet = CTRL_HEIGHT_SET_MID;
    }

    if (Key_Update(&Keyboard.F) == KEY_LONG_PRESS)
        Controller.FrictionOn = !Controller.FrictionOn;

    if (Key_Update(&Mouse.L) == KEY_PRESSED && Controller.FrictionOn == SET)
        Controller.ShootingOn = SET;
    else
        Controller.ShootingOn = RESET;

    // if (Key_Update(&Keyboard.G) == KEY_LONG_PRESS)
    //     Controller.isSpinMode = !Controller.isSpinMode;

    if (Key_Update(&Keyboard.V) == KEY_LONG_PRESS)
    {
        if (Controller.VisionMode == VISION_MODE_ARMOR)
            Controller.VisionMode = VISION_MODE_POWER_RUNE_LITTLE;
        else if (Controller.VisionMode == VISION_MODE_POWER_RUNE_LITTLE)
            Controller.VisionMode = VISION_MODE_POWER_RUNE_BIG;
        else
            Controller.VisionMode = VISION_MODE_ARMOR;
    }

    if (Key_Update(&Keyboard.B) == KEY_LONG_PRESS)
        tempColor = tempColor == 0x53 ? 0xE9 : 0x53;
}
