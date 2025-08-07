/*
 * @Author: frozen-fire 2812643217@qq.com
 * @Date: 2025-01-25 19:43:14
 * @LastEditors: frozen-fire 2812643217@qq.com
 * @LastEditTime: 2025-02-21 20:27:22
 * @FilePath: \2024-BalanceInfantry\Chassis_Code_MC01\Task\Communicate_Task.c
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "stdint.h"
#include "usart.h"
#include "string.h"

#include "Can_Bus.h"

#include "Communicate_Task.h"

Command_t Command ={0};
static uint16_t transmit_tick = 0;

float temp[6] ={0};
uint8_t Tx_buf[28] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0x80,0x7f};
    
void Communicate_Task(void const * argument)
{
    portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
	while(1)
	{
		vTaskDelayUntil(&xLastWakeTime,2);
        transmit_tick ++;//发送计时器自增
        
        if((transmit_tick % 1) == 0 )
        {
            //500Hz
            Gimbal_Communicate(&Robot_Status);
            
             temp[0] = Leg[0].theta;
            temp[1] = Leg[0].theta_dot_lpf;
			// temp[0] = LK9025_Motor[0].Speed.Real;
			// temp[1] = Leg[0].wheel_w;
            temp[2] = State_Variables.X;
            temp[3] = State_Variables.X_dot;
            temp[4]= State_Variables.Phi;
            temp[5] = State_Variables.Phi_dot;
            memcpy(Tx_buf,(uint8_t *)&temp,sizeof(temp));
            HAL_UART_Transmit_IT(&huart4,(uint8_t  *)Tx_buf,28);

        }
        if((transmit_tick % 10) == 0)
        {
            //50Hz
            HMI_Communicate(&Robot_Status);

        }
		
	}

}

