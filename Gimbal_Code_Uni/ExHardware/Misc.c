/**
 * @attention   采用UTF-8字符集编码
 * @brief       杂项外设
 * @authors     北工大PIP战队 樊郡捷
 * @version     v1.0
 * @date        2024-10-24
 * @details     
 */

#include "header.h"

void Misc_Laser_On(void)
{
    HAL_GPIO_WritePin(LASER_GPIO_Port, LASER_Pin, GPIO_PIN_SET);
}

void Misc_Laser_Off(void)
{
    HAL_GPIO_WritePin(LASER_GPIO_Port, LASER_Pin, GPIO_PIN_RESET);
}
