#include "stdint.h"

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
    float Klpf;//低通滤波器滤波系数
	uint8_t Mode;//模式选择，位置PID，增量PID

} PidTypeDef;

extern PidTypeDef Balance_Yaw_Position_pid;
extern PidTypeDef Balance_Yaw_Speed_pid;
extern PidTypeDef Leg_theta_Harmonize_pid[2];
extern PidTypeDef Leg_ROLL_Compensate_pid[2];
extern PidTypeDef L0_pid[2];
extern PidTypeDef Theta0_pid[2];
extern PidTypeDef TempCtrl_pid;

void PID_Init(PidTypeDef * pid,float kp,float ki,float kd,float max_out,float dead_band,float i_band,float max_input,float i_max_out,float Klpf,uint8_t mode);
void PID_Calc(PidTypeDef * pid, float real_val, float set_val);
void Chassis_Follow_PD_Calc(PidTypeDef * pid, const float param[8] , float real_val, float set_val);
void PID_Clear(PidTypeDef * pid);
