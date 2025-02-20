#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "stdint.h"

#include "Can_Bus.h"
#include "Configuration.h"
#include "PID.h"
#include "Universal.h"

#include "LK9025_Task.h"
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
			LK9025_Output_Normal(-Leg[LEFT].U[0] + Balance_Yaw_Position_pid.Output , Leg[RIGHT].U[0] + Balance_Yaw_Position_pid.Output );
		}
		else
			LK9025_Output_Damping();
	}
	
}


void LK9025_Output_Zero(void)
{ 
		LK9025_Tcurrent_Output(LEFT_WHEEL_ID,0);
		LK9025_Tcurrent_Output(RIGHT_WHEEL_ID,0);

}

void LK9025_Output_Damping(void)//将轮毂电机设置为阻尼模式
{
	PID_Calc(&Wheel_Damping_pid[0],LK9025_Motor[LEFT].Speed.Real,0);
	PID_Calc(&Wheel_Damping_pid[1],LK9025_Motor[RIGHT].Speed.Real,0);
	LK9025_Output_Normal(Wheel_Damping_pid[0].Output,Wheel_Damping_pid[1].Output);

}

void LK9025_Output_Normal(float output_left,float output_right)
{
		output_left = output_left * 403.44f - 36.746f;	
		output_left = Func_Limit(output_left,1800,-1800);
		LK9025_Tcurrent_Output(LEFT_WHEEL_ID,output_left);
		output_right = output_right * 403.44f - 36.746f;
		output_right = Func_Limit(output_right,1800,-1800);
		LK9025_Tcurrent_Output(RIGHT_WHEEL_ID,output_right);
}