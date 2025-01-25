/*
 * @Author: 静_火 lintianlang0918@qq.com
 * @Date: 2024-03-18 16:42:53
 * @LastEditors: 静_火 lintianlang0918@qq.com
 * @LastEditTime: 2024-03-20 00:09:50
 * @FilePath: \MDK-ARMg:\Robot_engeneering\24BALANCE-INFANTRY\MC-01_chassis\BSP\RS485.c
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
/*
 ____ _____  _    ______        _____  ____  _  ______  
/ ___|_   _|/ \  |  _ \ \      / / _ \|  _ \| |/ / ___| 
\___ \ | | / _ \ | |_) \ \ /\ / / | | | |_) | ' /\___ \ 
 ___) || |/ ___ \|  _ < \ V  V /| |_| |  _ <| . \ ___) |
|____/ |_/_/   \_\_| \_\ \_/\_/  \___/|_| \_\_|\_\____/
*/
/**
  ******************************************************************************
  * @file    RS485.c
  * @author  北京工业大学-林天朗
  * @version V1.0
  * @date    2024/3/19
  * @brief   RS485数据的接收
  ******************************************************************************
  * @attention

  ******************************************************************************
  */                                       

#include "usart.h"
#include "stdint.h"
#include "A1_Driver.h"

uint8_t A1_Receive_Data_Left[78] = {0};//宇树A1电机数据接收缓冲区
uint8_t A1_Receive_Data_Right[78] = {0};//宇树A1电机数据接收缓冲区
uint8_t A1_Transmit_Data_Left[34] = {0};//宇树A1电机数据发送缓冲区
uint8_t A1_Transmit_Data_Right[34] = {0};//宇树A1电机数据发送缓冲区

void RS485_Start(void)
{
	HAL_UART_Receive_DMA(&huart1,A1_Receive_Data_Left,78);//左腿数据接收
	HAL_UART_Receive_DMA(&huart6,A1_Receive_Data_Right,78);//右腿数据接收

}
/**
 * @brief RS485接收完成中断回调函数
 * @param huart
 * @return NONE
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance == USART1)
    {
		A1_Data_Rx_Left();
    }
    if(huart->Instance == USART6)
    {
		A1_Data_Rx_Right();
    }
}

/**
 * @brief 串口接收空闲中断回调函数
 * @param huart 
 * @retval NONE
 */
void UART_RxIdleCallback(UART_HandleTypeDef *huart)
{
	if(__HAL_UART_GET_FLAG(huart,UART_FLAG_IDLE))
	{
		__HAL_UART_CLEAR_IDLEFLAG(huart);
  }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == USART1)
    {
		HAL_GPIO_WritePin(GPIOC,RS485_DIR1_Pin,GPIO_PIN_RESET);
    }
	
}