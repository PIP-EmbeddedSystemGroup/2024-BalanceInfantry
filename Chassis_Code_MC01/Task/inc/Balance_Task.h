/*
 * @Author: 闂傚倸鐗婇悰濠囨煟韫囥儲瀚� lintianlang0918@qq.com
 * @Date: 2024-03-07 19:46:45
 * @LastEditors: frozen-fire 2812643217@qq.com
 * @LastEditTime: 2024-08-30 17:08:33
 * @FilePath: \MC-01_chassis\Task\Chassis_Task.h
 * @Description: 闁哄鏅滈悷锕€危閸涘⿴娓舵俊顖涱儥閸氬洭鎮规担绋库挃闁汇倧鎷�,闁荤姴娲ㄩ弻澶愵敊閺囩姷纾鹃柣銏╂憽ustomMade`, 闂佺懓鐏氶幐鍝ユ閹辩釜roFileHeader闂佸搫琚崕鍐诧耿閸涘瓨鐓€鐎广儱娲ㄩ弸锟� 闁哄鏅滅粙鏍€侀幋鐘冲闁告挆鍛€�: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */

#include "stdint.h"
#include "INS_Task.h"

typedef struct//LK-9025闁哄鍎愰崰妤呮儊鐎ｎ喗鍋ㄩ柛顭戝亝缁ㄦ氨绱撴担瑙勫鞍闁诲寒鍨遍幏鍛存晸閿燂拷
{
	int16_t Temperature;//闂佹眹鍨归悘姘攦閳ь剚绻涢幘顖涙珔閻庣櫢鎷�
	
  struct
	{
		float Real;
		int16_t Step;
		int16_t Set;
	}Speed;//缂傚倸鍊归悧婊堟偉濠婂牆闂柕濞炬杹閸嬫捇鎮㈠畡閭﹀敽
	struct
	{
		int16_t Real;
		int16_t Step;
		int16_t Set;
	}Position;//缂傚倸鍊归悧婊堟偉濠婂牆闂柕濞垮€楃粔瀵哥磽閸愭寧瀚�
	struct
	{
		float Real;
		int16_t Set;
	}Tcurrent;//闂佺儵鏅涢幗婊堝极缁嬫５褰掓晸閿燂拷
	float output;

}LK9025_Motor_t;


typedef struct//RM闂佹眹鍨归悘姘攦閳ь剛绱撴担瑙勫鞍闁诲寒鍨遍幏鍛存晸閿燂拷
{
	struct
	{
		int16_t Set;
		int16_t Real;
		int16_t Step;
	}Speed;
	struct
	{
		int16_t Set;
		int16_t Real;
		float Real_fp32;
		int16_t Step;
		int16_t LastReal;
		int16_t Convert;
		int16_t InitConvert;
		int8_t  Count;
		int8_t  Flag;
	}Position;
	int16_t TCurrent;
	int16_t Temperature;
	int16_t Rotate;
	int16_t Output;
}Motor_t;


typedef struct 
{
	float Theta;
	float Theta_dot;
	float X;
	float X_dot;
	float X_dot_meansure;
	float X_dot_predict;
	float X_dot_cov;
	float Phi;
	float Phi_dot;
	float acc_m;
	float acc_last;
	float leg_length;
	float leg_len_pow2;
    float leg_len_pow3;
	float target_Theta;
	float target_X;
	float target_X_dot;
	float target_Phi;
	float target_Phi_dot;
	float target_Roll;
	float target_Height;
	float target_Leg_Length[2];
	
	
}State_Val_t;

typedef struct//闂佽法鍠愰弸濠氬箯閻戣姤鏅搁柡鍌樺€栫€氬綊鏌ㄩ悢绗哄仱缂備焦鎸婚悗顖炴煥閻斿憡鐏柟鍑ゆ嫹
{
	int16_t transmit_error_count;
	int16_t transmit_count;
	int16_t uart_feedback_count;
	int16_t inverse_error_count;
	float coord[6];//闁哄鏅濋崑鐐差焽瀹€鍕殟闁稿本绮屾禒顖炴煕瑜庨崝鏍偉椤ゆ卡 YB XC YC XD YD
	float theta0;
	float theta1;
	float theta2;
	float theta3;
	float theta4;
	float theta1_dot;
	float theta2_dot;
	float theta2_dot_lpf;
	float theta4_dot;
	float F;
	float Tp;
	float U[2];
    float T_LPF;
	float T0;
	float T1;
	float L0_set;
	float L0;
	float L0_dot;
	float L0_dot_lpf;
	float theta;
	float theta_dot;
	float theta_dot_lpf;
	float height;
	float wheel_w;//wheel rotate speed
	float w_ecd;//wheel encoder speed
	float body_v;
	//婵炲濮伴崕鎵箔閸涙潙鍙婃い鏍ㄨ壘椤棃鏌涘▎蹇曅фい鈺婂弮閺屽牓濡搁妷銉р偓锟�
	float A11;
	float A12;
	float A21;
	float A22;
	float A_matrix[4];
  	float A_inverse_matrix[4];
	float F_Feedback;
	float Tp_Feedback;
	float P;
}Leg_t;

typedef struct Robot_Status_Structure
{
	struct
	{
		uint8_t INS_Init_Status;
		uint8_t INS_Fail_Status;
	}INS;
    struct 
    {
        uint8_t Leg_Init_Status;
        uint8_t Leg_Height_Status;
    }Leg;
    struct
    {
        uint8_t Body_Upright_Status;
        uint8_t Body_Soar_Status;
    }Body;
	struct 
	{
		uint8_t Gimbal_Reposition_Status;
        int16_t YAW_POSITION_REAL;

	}Gimbal;
	uint8_t Param_Switch;//闂佸憡鐟ラ崐褰掑汲閻旂厧绀嗛柛銉ｅ妼鎼村﹪鏌″鍛悙缂佺粯姊归幏鍛存晸閿燂拷
	
}Robot_Status_t;

extern Robot_Status_t Robot_Status;


//閻庢鍠楀ú鏍矗閸℃ê绶為柡宥庡幖閸斻儵鏌涘▎鎰惰€块柛锝忔嫹
extern LK9025_Motor_t LK9025_Motor[2];
extern Motor_t GM6020_YAW;
extern Leg_t Leg[2];
extern State_Val_t State_Variables;

void Balance_Task(void const * argument);
void Jointmotor_Feedback_Update_Left(float dt);
void Jointmotor_Feedback_Update_Right(float dt);
void Odometry_Update(Leg_t* lp, Leg_t* rp,State_Val_t* state,LK9025_Motor_t* lw,LK9025_Motor_t* rw,INS_t* imu,float dt);
void State_Variables_Update(State_Val_t* state , float dt);
void LQR_Calc(State_Val_t* state,Leg_t* leg,Robot_Status_t* robot_status);
void Leg_Output(Leg_t * leg,float f,float tp);
float Leg_Theta0_Limit(Leg_t* leg);

