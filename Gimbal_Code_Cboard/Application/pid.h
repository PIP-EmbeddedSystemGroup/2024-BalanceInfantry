#include "universal.h"

#define Incremental 0
#define Positional  1



typedef struct
{
    //PID 三参数
    float Kp;
    float Ki;
    float Kd;
		//最大输出 死区
		float Max_out;  //最大输出
		float Dead_band;//PID偏差死区
		float Intergral_band;//积分区
		float Max_input;//最大输入
    //PID输出值
		float Input;//输入
    float Output;//输出
		float I_max_out;//积分输出上限
    //误差
    float E[3];//2最新 1上一次 0上上次
		float D_last;
		float Intergral;//积分值
	  u8    Mode;//模式选择，位置PID，增量PID
} PidTypeDef;



void My_PID_Init(PidTypeDef * pid,float kp,float ki,float kd,float max_out,float dead_band,float i_band,float max_input,float i_max_out,u8 mode);
void PID_Calc(PidTypeDef * pid, float rel_val, float set_val);
void PID_Clear(PidTypeDef * pid);
s16 Yaw_Feedforward_Controler(s16 set);
s16 Pitch_Feedforward_Controler(s16 set);
void Chassis_Follow_PD_Calc(PidTypeDef * pid, float real_val, float set_val);

extern PidTypeDef Ammo_Speed_pid[2];
extern PidTypeDef Gimbal_Speed_pid[3];
extern PidTypeDef Gimbal_Position_pid[3];
extern PidTypeDef Gimbal_TCurrent_pid[3];

