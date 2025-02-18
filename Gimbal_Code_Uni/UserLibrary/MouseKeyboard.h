#ifndef _KEYBOARD_H_
#define _KEYBOARD_H_

/**
 * @attention   采用UTF-8字符集编码
 */

#include "stdint.h"

typedef enum
{
    KEY_UNPRESSED = 0,
    KEY_PRESSED = 1,
    KEY_SHORT_PRESS = 2,
    KEY_LONG_PRESS = 3,
} KeyState_t;

/// @brief 用来保存按键的状态
typedef struct
{
    KeyState_t State;      	//按键状态
    int Value;      		//按键是否按下
    int PrevValue;  		//上次更新时按键是否按下
    uint32_t ClickDownTick; //按下时的时间戳
    unsigned LongPressTick;	//长按时长
} Key_t;

/// @brief 保存鼠标状态的结构体
typedef struct
{
    int X;
    int Y;
    int Z;
    Key_t L;
    Key_t R;
} Mouse_t;

/// @brief 保存键盘状态的结构体
typedef struct
{
    Key_t W;
    Key_t S;
    Key_t A;
    Key_t D;
    Key_t Shift;
    Key_t Ctrl;
    Key_t Q;
    Key_t E;
    Key_t R;
    Key_t F;
    Key_t G;
    Key_t Z;
    Key_t X;
    Key_t C;
    Key_t V;
    Key_t B;
} Keyboard_t;

void Key_Init(Key_t* key, unsigned LongPressTick);
KeyState_t Key_Update(Key_t* key);
void Keyboard_Init(void);
void Mouse_Init(void);

extern Mouse_t Mouse;
extern Keyboard_t Keyboard;

#endif
