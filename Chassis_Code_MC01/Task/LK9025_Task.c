#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "stdint.h"

#include "Can_Bus.h"
#include "Configuration.h"
#include "PID.h"

#include "Communicate_Task.h"

void LK9025_Tx_Task(void const * argument)
{
	portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
	
	while(1)
	{
		vTaskDelayUntil(&xLastWakeTime,1);
		
        
		if(Command.Chassis_Power_Switch == ON)
		{
			LK9025_Output_Normal(-Leg[LEFT].U[0] + Balance_Yaw_Position_pid.Output, Leg[RIGHT].U[0] + Balance_Yaw_Position_pid.Output );
		}
		else if(Command.Chassis_Power_Switch == OFF)
			LK9025_Output_Zero();
	}
	
}