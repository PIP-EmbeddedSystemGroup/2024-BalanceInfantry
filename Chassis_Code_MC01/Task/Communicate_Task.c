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
            
             //temp[0] = State_Variables.Theta;
            //temp[1] = State_Variables.Theta_dot;
			temp[0] = LK9025_Motor[0].Speed.Real;
			temp[1] = Leg[0].wheel_w;
            temp[2] = State_Variables.X_dot;
            temp[3] = State_Variables.target_X;
            temp[4]= State_Variables.X;
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

