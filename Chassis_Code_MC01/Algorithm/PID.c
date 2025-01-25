#include "PID.h"
#include "Configuration.h"
#include "universal.h"
#include "Balance_Task.h"

/**
 * @brief PID鎺у埗鍣ㄥ垵濮嬪寲鍑芥暟
 * @param PID缁撴瀯浣揚idTypeDef*pid
 * @param 姣斾緥绯绘暟kp
 * @param 绉垎绯绘暟ki
 * @param 寰垎绯绘暟kd
 * @param 鏈€澶ц緭鍑簃ax_out
 * @param 鎺у埗姝诲尯dead_band
 * @param 绉垎鍖哄煙i_band
 * @param 鏈€澶ц緭鍏ax_input
 * @param 绉垎鏈€澶ц緭鍑篿_max_out
 * @param 鎺у埗妯″紡mode
 * @retval NONE
*/
void PID_Init(PidTypeDef * pid,float kp,float ki,float kd,float max_out,float dead_band,float i_band,float max_input,float i_max_out,float Klpf,uint8_t mode)
{
	pid->Kp=kp;
	pid->Ki=ki;
	pid->Kd=kd;
    pid->Klpf=Klpf;
	pid->Max_out=max_out;
	pid->I_max_out=i_max_out;
	pid->Dead_band=dead_band;
	pid->Intergral_band=i_band;
	pid->Max_input=max_input;
	pid->Input=0;
	pid->Output=0;
	pid->Intergral=0;
	pid->Mode=mode;
	pid->E[0]=0;
	pid->E[1]=0;
	pid->E[2]=0;
	pid->D_last=0;
}


/**
  * @brief  PID鎺у埗鍣ㄨ绠楀嚱鏁�
  * @param  PID缁撴瀯浣揚idTypeDef*pid
  * @param 	鐪熷疄鍊紃eal_val
  * @param	璁惧畾鍊約et_val
  * @retval NONE
  */
void PID_Calc(PidTypeDef * pid, float real_val, float set_val)
{
	float p = 0,i = 0,d = 0;
	pid->E[2]=Func_Limit(set_val,pid->Max_input,-(pid->Max_input))-real_val;//杈撳叆闄愬箙	
	
	if(pid==&Balance_Yaw_Position_pid)//杞悜璧板姡寮�
	{
		if(pid->E[2]>4096)
			pid->E[2]=pid->E[2]-8192;
		else if((-pid->E[2])>4096)
			pid->E[2]=8192+pid->E[2];
	}
	if(Func_Abs(pid->E[2]) >=  pid->Dead_band)//鍒ゆ柇鏄惁杩涘叆姝诲尯
	{
		if(pid->Mode==POSITIONAL)
		{	
			if(Func_Abs(pid->E[2]) <= pid->Intergral_band)//鍒ゆ柇鏄惁杩涘叆绉垎鍖哄煙
			{
				pid->Intergral =pid->Intergral+ (pid->Ki) * (pid->E[2]);//锟斤拷锟斤拷值
				//pid->Intergral =pid->Intergral+ ((pid->Ki) * (pid->E[2]) + (pid->Ki) * (pid->E[1]))/2;//涓ゆ璇樊骞冲潎浠ユ秷闄ら儴鍒嗚宸烦鍙樼殑褰卞搷
			}
			else
			{
				pid->Intergral=0;//瓒呭嚭绉垎鍖哄煙鍒欏叧闂Н鍒嗕綔鐢ㄢ€斺€旂Н鍒嗗垎绂�
			}                
			pid->Intergral=Func_Limit(pid->Intergral,pid->I_max_out,-(pid->I_max_out));//绉垎鍣ㄨ緭鍑洪檺骞�
  		p = pid->Kp * (pid->E[2]);
			i = pid->Intergral;
			d = pid->Kd * (pid->E[2]-pid->E[1]);
			d = d * pid->Klpf + (1- pid->Klpf)* pid->D_last;
			pid->D_last=d;
			 
		}
		else if(pid->Mode==INCREMENTAL)
		{
			p=pid->Kp*((pid->E[2]-pid->E[1]));
			i=pid->Ki*pid->E[2];
			d=pid->Kd*(pid->E[2]-pid->E[1]*2+pid->E[0]);
		}
		
		pid->Output=p+i+d;
		pid->Output=Func_Limit(pid->Output,pid->Max_out,-(pid->Max_out));//杈撳嚭闄愬箙

	}
	else
		pid->Output=0;	
	pid->E[0] = pid->E[1];//璇樊杩唬
	pid->E[1] = pid->E[2];
}

void Chassis_Follow_PD_Calc(PidTypeDef * pid, const float param[8] , float real_val, float set_val)
{
	float Abs_Error = 0,
				p = 0,
				d = 0;

	pid->E[2] = Func_Limit(set_val,pid->Max_input,-(pid->Max_input))-real_val;
	
	if(pid->E[2]>4096)              
		pid->E[2]=pid->E[2]-8192;
	else if((-pid->E[2])>4096)
		pid->E[2]=8192+pid->E[2];
    
	Abs_Error = Func_Abs(pid->E[2]);
	if(Abs_Error >  1200)
	{
		pid->Kp = param[6];
		pid->Kd = param[7];
	}
	else if(Abs_Error >  800)
	{
		pid->Kp = param[4] ;
		pid->Kd = param[5];
	}
	else if(Abs_Error >  400)
	{
		pid->Kp = param[2];
		pid->Kd = param[3];
	}
	else 
	{
		pid->Kp = param[0];			
		pid->Kd = param[1];
	}			
						
	p = pid->Kp * (pid->E[2]);
	d = pid->Kd * (pid->E[2]-pid->E[1]);
	pid->Output = p+d;
	pid->Output = Func_Limit(pid->Output,pid->Max_out,-(pid->Max_out));
	
	pid->E[0] = pid->E[1];
	pid->E[1] = pid->E[2];
}

/**
  * @brief  PID鎺у埗鍣ㄦ竻绌哄嚱鏁�
  * @param PID缁撴瀯浣揚idTypeDef*pid
  * @retval NONE
  */
void PID_Clear(PidTypeDef * pid)
{
	pid->E[0] = 0;
	pid->E[1] = 0;
	pid->E[2] = 0;
	pid->Intergral = 0;
	pid->Output = 0;
}

