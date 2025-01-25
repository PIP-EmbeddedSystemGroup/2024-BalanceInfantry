/**
 ******************************************************************************
 * @file    gimbal_task.c
 * @author  ������ҵ��ѧ-������
 * @version V1.0
 * @date    2023/6/1
 * @brief   ��̨����
 ******************************************************************************
 * @attention
 *
 *
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "Header.h"
#include "tim.h"
Motor_t GM6020[2] = {0}; // YAW 0 PITCH 1

// extern uint32_t time_count;//���Գ��������ٶ��õ���������
// uint32_t time[2] ={0};

void Gimbal_Task(void const *argument)
{
	portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
	while (1)
	{
		osDelayUntil(&xLastWakeTime, 1); // 1000HZ���˽�������ƽ����ʱС�ڵ���0.1ms
		//		time[0] = time_count;//���Գ��������ٶ��õ�

		// PITCH������㷨
		PITCH_PID_Control();

		// YAW������㷨
		YAW_PID_WithIMU();

		//    Gimbal_Output(0,0,0,0);������̨�����㷨���
		if (RC_Ctl.rc.s2 == 3)
		{
			Gimbal_Output(0, 0);
		}
		else
		{
			Gimbal_Output(Gimbal_Speed_pid[YAW].Output, Gimbal_Speed_pid[PITCH].Output);
		}
		//	  time[1] = time_count - time[0];//���Գ��������ٶ��õ�
	}
}

/**
 * @brief  YAW�����
 * @param  None
 * @retval ״ֵ̬
 */
void YAW_PID_WithIMU(void)
{
	Func_CircleRamp(GM6020[YAW].Position.Set, &GM6020[YAW].Position.Step, YAW_POSITION_RAMP_RATIO); // б����������

	PID_Calc(&Gimbal_Position_pid[YAW], INS.Yaw_8192, GM6020[YAW].Position.Step);	// λ�û�PID
	PID_Calc(&Gimbal_Speed_pid[YAW], INS.Gyro[2], Gimbal_Position_pid[YAW].Output); // �ٶȻ�PID
}

/**
 * @brief  PITCH�����
 * @param  None
 * @retval ״ֵ̬
 */
void PITCH_PID_Control(void)
{

	Func_CircleRamp(GM6020[PITCH].Position.Set, &GM6020[PITCH].Position.Step, PITCH_POSITION_RAMP_RATIO);
	PID_Calc(&Gimbal_Position_pid[PITCH], GM6020[PITCH].Position.Real, GM6020[PITCH].Position.Step);
	PID_Calc(&Gimbal_Speed_pid[PITCH], GM6020[PITCH].Speed.Real, Gimbal_Position_pid[PITCH].Output);
}

void Gimbal_Disable(void)
{
	GM6020[PITCH].Position.Set = GM6020[PITCH].Position.Real;
	GM6020[PITCH].Position.Step = GM6020[PITCH].Position.Real;

	GM6020[YAW].Position.Set = INS.Yaw_8192;
	GM6020[YAW].Position.Step = INS.Yaw_8192;

	PID_Clear(&Gimbal_Position_pid[PITCH]);
	PID_Clear(&Gimbal_Speed_pid[PITCH]);

	PID_Clear(&Gimbal_Position_pid[YAW]);
	PID_Clear(&Gimbal_Speed_pid[YAW]);

	Command.Mouse_x = INS.Yaw_8192;
	Command.Mouse_y = PITCH_POSITION_INIT;
}
