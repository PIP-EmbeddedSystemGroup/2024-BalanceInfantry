/**
 * @attention   采用UTF-8字符集编码
 * @brief       键鼠库
 * @authors     北工大PIP战队 樊郡捷
 * @version     v1.0
 * @date        2024-10-14
 * @details     较为简单的实现了按键响应
 *              能够区分长短按 但是是抬键时判断的
 */

#include "header.h"

Mouse_t Mouse = { 0 };
Keyboard_t Keyboard;

void Key_Init(Key_t* key, unsigned longPressTick)
{
    key->State = KEY_UNPRESSED;
    key->Value = RESET;
    key->PrevValue = RESET;
    key->ClickDownTick = 0;
    key->LongPressTick = longPressTick;
}

KeyState_t Key_Update(Key_t* key)
{
    if (key->Value == SET && key->PrevValue == RESET)
    {
        key->ClickDownTick = HAL_GetTick();
    }
    else if (key->Value == RESET && key->PrevValue == SET)
    {
        key->PrevValue = key->Value;
        key->State = (HAL_GetTick() - key->ClickDownTick) > key->LongPressTick ? KEY_LONG_PRESS : KEY_SHORT_PRESS;
        return key->State;
    }
    key->PrevValue = key->Value;
    key->State = (key->Value == SET) ? KEY_PRESSED : KEY_UNPRESSED;
    return key->State;
}

void Mouse_Init(void)
{
    Mouse.X = 0;
    Mouse.Y = 0;
    Mouse.Z = 0;
    Key_Init(&Mouse.L, 0);
    Key_Init(&Mouse.R, 0);
}

void Keyboard_Init(void)
{
    Key_Init(&Keyboard.W, 0);
    Key_Init(&Keyboard.S, 0);
    Key_Init(&Keyboard.A, 0);
    Key_Init(&Keyboard.D, 0);
    Key_Init(&Keyboard.Shift, 0);
    Key_Init(&Keyboard.Ctrl, 0);
    Key_Init(&Keyboard.Q, 0);
    Key_Init(&Keyboard.E, 0);
    Key_Init(&Keyboard.R, 0);
    Key_Init(&Keyboard.F, 500);
    Key_Init(&Keyboard.G, 0);
    Key_Init(&Keyboard.Z, 0);
    Key_Init(&Keyboard.X, 500);
    Key_Init(&Keyboard.C, 0);
    Key_Init(&Keyboard.V, 500);
    Key_Init(&Keyboard.B, 500);
}
