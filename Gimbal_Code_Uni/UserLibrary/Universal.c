/**
 * @attention   采用GB2312字符集编码
 * @brief       通用函数库
 * @authors     北工大PIP战队 樊郡捷
 * @version     v1.0
 * @date        2024-10-14
 * @details     
 */

#include "header.h"

int Constrain(int val, int max, int min)
{
    return val > max ? max : val < min ? min : val;
}

float ConstrainF(float val, float max, float min)
{
    return val > max ? max : val < min ? min : val;
}

/// @brief 相位卷绕
/// @param val 角度值
/// @param max 一圈对应的值
/// @return 卷绕后的角度值 例:AngleWrap(361, 360)=>1
int AngleWrap(int val, int max)
{
    return val > max ? val % max : (val < 0 ? val % max + max : val);
}

/// @brief 相位卷绕
/// @param val 角度值
/// @param max 一圈对应的值
/// @return 卷绕后的角度值 例:AngleWrap(361, 360)=>1
float AngleWrapF(float val, float max)
{
    return val > max ? fmodf(val, max) : (val < 0 ? fmodf(val, max) + max : val);
}

/// @brief 计算两角之间的角度差a-b 返回劣弧
/// @param a    被减数
/// @param b    减数
/// @param max  一圈对应的值
/// @return a-b 劣弧
int AngleDiff(int a, int b, int max)
{
    int err = AngleWrap(a, max) - AngleWrap(b, max);
    int halfMax = max / 2;
    if (err > halfMax)
        err -= max;
    else if (err < -halfMax)
        err += max;
    return err;
}

/// @brief 计算两角之间的角度差a-b 返回劣弧
/// @param a    被减数
/// @param b    减数
/// @param max  一圈对应的值
/// @return a-b 劣弧
float AngleDiffF(float a, float b, float max)
{
    float err = AngleWrapF(a, max) - AngleWrapF(b, max);
    float halfMax = max / 2;
    if (err > halfMax)
        err -= max;
    else if (err < -halfMax)
        err += max;
    return err;
}
