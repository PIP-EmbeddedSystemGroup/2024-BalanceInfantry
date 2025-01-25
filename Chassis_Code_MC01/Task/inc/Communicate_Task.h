
#include "stdint.h"

typedef struct              
{  
    int16_t Vx;	
	int16_t Vx_step;//移动速度设定值的斜坡值
	int16_t Yaw_Position_Move;
	float SpeedZoom;//移动速度缩放系数，用于调整移速设定值的数值范围
	uint8_t Chassis_Power_Switch;//底盘开关控制位
	uint8_t Rotate_Flag;//车体小陀螺旋转控制位
	uint8_t UIFlag;//UI刷新标志位
	uint8_t Gimbal_Follow;//车体跟随云台控制位
	uint8_t Height_Set;//车体高度控制位，蹲起PEEK
	uint8_t Height_Switch;//切换车体高度控制位，前后两次控制指令不同时置1
	uint8_t SideWay_Set;//侧身控制位，左右PEEK
	float Mouse_x;//鼠标X轴移动数据
	float Mouse_y;//鼠标Y轴移动数据
	uint8_t Storage_Flag;
	uint8_t AimAssit_Flag;//
	uint8_t GimbalFollow_OFF;//
  	uint8_t Cap_Flag;
	uint8_t Shooting_Friction;

}Command_t;    


extern Command_t Command;

void Communicate_Task(void const * argument);