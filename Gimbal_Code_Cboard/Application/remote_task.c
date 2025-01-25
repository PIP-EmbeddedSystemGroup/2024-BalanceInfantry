/**
******************************************************************************
* @file    remote_task.c
* @author  ������ҵ��ѧ-��ͩ
* @version V1.0
* @date    2019/3/1
* @brief   ң�ؽ���
******************************************************************************
* @attention
*
*
******************************************************************************
*/

/* Includes ------------------------------------------------------------------*/
#include "Header.h"
#include "tim.h"
Command_t Command = {0};

void Remote_Task(void const *argument)
{
	portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
	while (1)
	{
		osDelayUntil(&xLastWakeTime, 2); // 500HZ

		switch (RC_Ctl.rc.s2) // ѡ�����ģʽ
		{
		case 1: //
		{
			Remote_Control();
			Command.Chassis_Power_Switch = ON;
			break;
		}
		case 2: // ���˴������·����Կ���
		{
			Computer_control();
			break;
			Command.Chassis_Power_Switch = ON;
		}
		default: // ���˴�����λ��δ��⵽ң�����źŵ�ʱ��ֹͣһ�������ź�
		{
			Gimbal_Disable();
			Command.Chassis_Power_Switch = OFF;
			break;
		}
		}
	}
}

/**
 * @brief  ң������������
 * @param  None
 * @retval None
 */
void Remote_Control(void)
{
	GM6020[PITCH].Position.Set = (RC_Ctl.rc.ch3 - 1024) + PITCH_POSITION_INIT;
	GM6020[YAW].Position.Set -= ((RC_Ctl.rc.ch2 - 1024) * 0.024f); // ��Ϊ��һ���ۼӵĹ��̣����Խ�����

	GM6020[PITCH].Position.Set = Func_Limit(GM6020[PITCH].Position.Set, PITCH_POSITION_MAX, PITCH_POSITION_MIN);
	GM6020[YAW].Position.Set = Func_ValueRannge(GM6020[YAW].Position.Set, 8192, 0);

	switch ((RC_Ctl.rc.ch4 == 1684) | (RC_Ctl.rc.ch4 == 364) << 4) // �������༫ֵ
	{
	case 0x01:
		Steering_Engine_OFF();
		Command.AimAssitFlag = MANUAL_ATTACK;
		break;
	case 0x10:
		Steering_Engine_ON();
		break;
	default:
		Steering_Engine_OFF();
		break;
	}

	// ��Ħ���ֿ���������£����²��������ӵ���Ϊ������
	switch (RC_Ctl.rc.s1)
	{
	case 1:
		if (Shooting.Last_s1 == 3)
			Shooting.Friction = !Shooting.Friction;
		Shooting.Last_s1 = 1;
		break;
	case 3:
		Shooting.Last_s1 = 3;
		break;
	case 2:
		if ((Shooting.Friction == 1) && (Shooting.Last_s1 == 3)) // ����
			// if(Shooting.Friction == 1)//����
			Shooting.PosSetCount = Shooting.Period;
		Shooting.Last_s1 = 2;
		break;
	default:
		break;
	}

	Command.Vx = (RC_Ctl.rc.ch1 - 1024) * 0.3f; // ��������λ����Ϊ�ٶȵ���ֵ

	if ((RC_Ctl.rc.ch0 - 1024) > 400)
    {
        RC_Ctl.rc.ch0_switch = 1;
    }
	else if ((RC_Ctl.rc.ch0 - 1024) < -400)
    {
		RC_Ctl.rc.ch0_switch = 2;
	}
    else
        RC_Ctl.rc.ch0_switch = 3;
    
     switch(RC_Ctl.rc.ch0_switch )
     {
         case 1:
         {
             if(RC_Ctl.rc.ch0_switch_last == 3)
             {
                if(Command.Height_Set == MID)
                    Command.Height_Set = LOW;
                else
                    Command.Height_Set = MID;
             }
             RC_Ctl.rc.ch0_switch_last = 1;
             break; 
         }
         
         case 2:
         {
            if(RC_Ctl.rc.ch0_switch_last == 3)
             {
                if(Command.Height_Set == MID)
                    Command.Height_Set = HIGH;
                else
                    Command.Height_Set = MID;
             }
             RC_Ctl.rc.ch0_switch_last = 2;
             break; 
         }
         
         case 3:
         {
            RC_Ctl.rc.ch0_switch_last = 3;
             break;
         }
     }
}

/**
 * @brief  ���Կ�������
 *         ��β��������ע����д
 * @param  None
 * @retval None
 */
void Computer_control(void)
{
	// ������Ҽ� ���� ��������
	Mouse_Control();

	// ���������������䣬��ס����
	if ((RC_Ctl.mouse.press_l == 1) && (Shooting.Friction == 1) && (Shooting.PosSetCount == 0) && (Shooting.PressTime == 0))
	{
		Shooting.PosSetCount = Shooting.Period;
	}
	else if (RC_Ctl.mouse.press_l == 1)
	{
		Shooting.PressTime++;
		if (Shooting.PressTime > 36)
			Shooting.PosSetCount = Shooting.Period * 0.4;
	}
	else
	{
		Shooting.PressTime = 0;
	}

	// X��ֵ��������Ħ����
	// ����1��ر�Ħ����
	// �̰�����Ħ����
	switch (Key_Press(Key.X1, 100))
	{
	case 2:
		Shooting.Friction = 0;
		break;
	case 1:
		Shooting.Friction = 1;
		break;
	default:
		break;
	}

	// R�����Ƶ��ָǣ�����R�����֣��̰�R�ص���
	switch (Key_Press(Key.R, 100))
	{
	case 2:
		Steering_Engine_ON();
		break;
	case 1:
		Steering_Engine_OFF();
		break;
	default:
		break;
	}

	// V�� ����С��һ�� ww
	switch (Key_Press(Key.V, 5))
	{
	case 1:
		Command.TriggerAnti = 15;
		break;
	case 2:
		break;
	default:
		break;
	}

	// E��
	switch (Key_Press(Key.E, 5))
	{
	case 1:
		Command.UIFlag = 1;
		break;
	default:
		Command.UIFlag = 0;
		break;
	}
	if (Key_Press(Key.Shift, 5))
		Command.Cap_Flag = ON;
	else
		Command.Cap_Flag = OFF;
}

void Mouse_Control(void)
{
	RC_Ctl.mouse.x = Func_Limit(RC_Ctl.mouse.x, 800, -800);
	RC_Ctl.mouse.y = Func_Limit(RC_Ctl.mouse.y, 800, -800);
	if (RC_Ctl.mouse.x > 0)
		Command.Mouse_x -= RC_Ctl.mouse.x / 15.0f + 0.2f;

	else if (RC_Ctl.mouse.x < 0)
		Command.Mouse_x -= RC_Ctl.mouse.x / 15.0f - 0.2f;

	if (RC_Ctl.mouse.y > 0)
		Command.Mouse_y -= RC_Ctl.mouse.y / 15.0f + 0.2f;

	else if (RC_Ctl.mouse.y < 0)
		Command.Mouse_y -= RC_Ctl.mouse.y / 15.0f - 0.2f;

	GM6020[YAW].Position.Set = Func_ValueRannge(Command.Mouse_x, 8192, 0);
	Command.Mouse_y = Func_Limit(Command.Mouse_y, PITCH_POSITION_MAX, PITCH_POSITION_MIN);
	GM6020[PITCH].Position.Set = Func_Limit(Command.Mouse_y, PITCH_POSITION_MAX, PITCH_POSITION_MIN);
}

/**
 * @brief  ң�������ӵ���ʱ����������
 * @param  None
 * @retval None
 */

void Key_Deal(void)
{
	Key.W[0] = RC_Ctl.key.v_l & 0x01;
	Key.S[0] = (RC_Ctl.key.v_l & 0x02) >> 1;
	Key.A[0] = (RC_Ctl.key.v_l & 0x04) >> 2;
	Key.D[0] = (RC_Ctl.key.v_l & 0x08) >> 3;
	Key.Shift[0] = (RC_Ctl.key.v_l & 0x10) >> 4;
	Key.Ctrl[0] = (RC_Ctl.key.v_l & 0x20) >> 5;
	Key.Q[0] = (RC_Ctl.key.v_l & 0x40) >> 6;
	Key.E[0] = (RC_Ctl.key.v_l & 0x80) >> 7;

	Key.R[0] = RC_Ctl.key.v_h & 0x01;
	Key.F[0] = (RC_Ctl.key.v_h & 0x02) >> 1;
	Key.G[0] = (RC_Ctl.key.v_h & 0x04) >> 2;
	Key.Z1[0] = (RC_Ctl.key.v_h & 0x08) >> 3;
	Key.X1[0] = (RC_Ctl.key.v_h & 0x10) >> 4;
	Key.C[0] = (RC_Ctl.key.v_h & 0x20) >> 5;
	Key.V[0] = (RC_Ctl.key.v_h & 0x40) >> 6;
	Key.B[0] = (RC_Ctl.key.v_h & 0x80) >> 7;
}
/**
 * @brief  �������
 * @param  ��ֵ������ʱ��
 * @retval �ް���0���̰�1������2
 */
u8 Key_Press(u16 key[], u16 SetTime)
{
	if (key[0] == 1 && key[1] == 0)
		return 2;
	else if (key[0] == 1 && key[1] != 0)
	{
		(key[1])--;
		return 1;
	}
	key[1] = SetTime;
	return 0;
}
