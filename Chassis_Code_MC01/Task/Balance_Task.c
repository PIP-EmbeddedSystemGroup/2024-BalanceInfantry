/*
 * @Author: 闂傚倸鍊搁悧濠囨偘婵犲洦鐓熼煫鍥ュ劜鐎氾拷 lintianlang0918@qq.com
 * @Date: 2024-03-07 19:46:25
 * @LastEditors: 静_火 lintianlang0918@qq.com
 * @LastEditTime: 2024-04-04 02:51:16
 * @FilePath: \MC-01_chassis\Task\Chassis_Task.c
 * @Description: 闂佸搫顦弲婊堟偡閿曗偓鍗遍柛娑樷看濞撹埖淇婇娑卞劌闁告艾娲幃瑙勬媴缁嬪簱鎸冮梺姹囧€ч幏锟�,闂佽崵濮村ú銊╁蓟婢舵劦鏁婇柡鍥╁Х绾鹃箖鏌ｉ姀鈺傛喗ustomMade`, 闂備胶鎳撻悘姘跺箰閸濄儲顫曢柟杈╅嚋roFileHeader闂備礁鎼悮顐﹀磿閸愯鑰块柛娑樼摠閻撯偓閻庡箍鍎卞ú銊╁几閿燂拷 闂佸搫顦弲婊呯矙閺嶎厹鈧線骞嬮悩鍐差€涢梺鍛婃寙閸涱喚鈧拷: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */

#include "stdint.h"
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "usart.h"


#include "Configuration.h"
#include "A1_Driver.h"
#include "universal.h"
#include "PID.h"
#include "Dwt_Timer.h"
#include "CRC.h"
#include "arm_math.h"

#include "Balance_Task.h"
#include "Monitor_Task.h"
#include "A1_Task.h"
#include "Communicate_Task.h"

//float a11[4] = {-155.9316,254.0555,-185.0108,1.2442};
//float a12[4] = {47.2934,-32.7831,-12.1635,0.2118};
//float a13[4] = {-10.1391,9.6784,-3.4424,-0.0200};
//float a14[4] = {-162.2113,155.2344,-55.5910,-0.3421};
//float a15[4] = {103.6842,9.5532,-61.9532,27.2285};
//float a16[4] = {18.7884,-4.4123,-6.4557,3.9953};
//float a21[4] = {1010.0438,-810.4388,205.8031,6.6265};
//float a22[4] = {75.9429,-74.9077,27.5461,0.3802};
//float a23[4] = {3.9794,0.7408,-2.6709,1.1003};
//float a24[4] = {64.5622,11.2439,-42.7203,17.7419};
//float a25[4] = {942.2017,-900.8558,322.1064,-0.9325};
//float a26[4] = {118.4404,-118.1467,44.7296,-2.0764};//保底

float a11[4] = {-113.4565,233.7880,-197.0249,1.7168};
float a12[4] = {57.7438,-41.3475,-13.4751,0.2739};
float a13[4] = {-10.1264,9.7490,-3.5194,0.0044};
float a14[4] = {-197.7084,190.7414,-69.2456,0.0643};
float a15[4] = {127.9262,-14.3882,-54.4930,27.2731};
float a16[4] = {23.3141,-8.6889,-5.1346,4.0241};
float a21[4] = {1139.9729,-936.6341,248.4732,6.5131};
float a22[4] = {89.6445,-90.7297,34.8253,0.2705};
float a23[4] = {4.8312,-0.1174,-2.4029,1.1017};
float a24[4] = {95.6563,-3.2846,-46.7845,21.6437};
float a25[4] = {939.8127,-907.4880,329.9648,-3.4998};
float a26[4] = {121.8077,-122.4618,47.0965,-2.6868};//好参数


State_Val_t State_Variables = {0};//闁哄牆鎼▍鎺撶閾忕懓笑闁诡兛绀佽ぐ澶愭煂韫囨洜娉㈤柡瀣缂嶏拷

PidTypeDef Balance_Yaw_Position_pid = {0};
PidTypeDef Balance_Yaw_Speed_pid = {0};
PidTypeDef Leg_ROLL_Compensate_pid[2] = {0};
PidTypeDef Leg_theta_Harmonize_pid[2] = {0};
PidTypeDef L0_pid[2] = {0};

LK9025_Motor_t LK9025_Motor[2]={0};
Motor_t GM6020_YAW = {0};
Leg_t Leg[2] = {0};

float yaw_ddwrNwwr, yaw_p_ddwrNwwr, pitch_ddwrNwwr;
float macc_x, macc_z;
float u, k;
float vel_prior, vel_measure, vel_cov;

float fai_offset = 0.0f;
float theta_offset = 0.00f;

float Theta_Klpf = 0.99f;

static uint32_t balance_dwt_cnt = 0; 
float BalanceTask_dt = 0;

extern float Support_F[2];
float Jump_F[2]={60.0f,60.0f};
float acc_avg =0;
int32_t avg_count = 3000;
void Balance_Task(void const * argument)
{
    portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();

	
	
	while(1)
	{
		//缁崵绮哄鑸垫1ms
		osDelayUntil(&xLastWakeTime,1);
		
		//缁楊剟娴傚銉礉閼惧嘲褰囨潻娑氣柤閹笛嗩攽閸涖劍婀t
		BalanceTask_dt = DWT_GetDeltaT(&balance_dwt_cnt);

		//缂佹鍏涚粩鏉戭潰閵夘垳绀夐柡宥堫潐瀹撲線宕楃€圭姴螡闁汇垹鐏氬┃鈧柛娆忕Ч椤╊參寮悧鍫濈ウ闁告粌鏈崕鑽も偓闈涘悑閺嗙喖骞戦娆惧悁缂佺姵顨呴懟鐔兼嚂閺冨洤鎮柣妯垮煐閳ь兛绶氶崳锟�
		Jointmotor_Feedback_Update_Left(BalanceTask_dt);
		Jointmotor_Feedback_Update_Right(BalanceTask_dt);
			
		//缂佹鍏涚花鈺侇潰閵夘垳绀夐柡鍥х摠閺屽﹪鏌屽畝鈧埢鑲╂媼閿燂拷
		Odometry_Update (&Leg[LEFT],&Leg[RIGHT],&State_Variables ,&LK9025_Motor[0],&LK9025_Motor[1],&Ins_Data ,BalanceTask_dt);
		
		//缂佹鍏涚粭浣割潰閵夘垳绀夐柡鍥х摠閺屽﹪鎮╅懜纰樺亾娴ｇǹ缍侀梺璇ф嫹
		State_Variables_Update(&State_Variables,BalanceTask_dt);
        
		//缂佹鍓欏ú鎾愁潰閵夘垳绀夐悹渚婄磿閻ｇ睈QR闁绘ǹ鍩栭埀顑跨瀵姤锛冮崼鐔蜂粯闁告帟娉涘▍锟�
		LQR_Calc(&State_Variables,&Leg[LEFT],&Robot_Status);
		LQR_Calc(&State_Variables,&Leg[RIGHT],&Robot_Status);
		
		if(Command.Storage_Flag == 0)
		{
			Support_F[0]+=Jump_F[0];
			Support_F[1]+=Jump_F[1];
		}else
		{
			Support_F[0]=62.0f;
			Support_F[1]=62.0f;
		}
		
			//Balance_Yaw_Position_pid.Max_out = 2 - Func_Abs(- Balance_fai_Position_pid.Output - Balance_fai_Speed_pid.Output + (Balance_theta_Position_pid[0].Output +Balance_theta_Position_pid[1].Output)*0.5f - Wheel_Speed_pid.Output);
		//if(Robot_Status.Body.Body_Upright_Status == YES)
		{
			if(Command.Rotate_Flag == 1)
			{
				Balance_Yaw_Position_pid.Output = Func_Limit(80,Balance_Yaw_Position_pid.Max_out,-Balance_Yaw_Position_pid.Max_out);
			}
			else
				//Chassis_Follow_PD_Calc(&Balance_Yaw_Position_pid,Fuzzy_PD_Param,GM6020_YAW.Position.Real_fp32,Func_ValueRannge(YAW_POSITION_INIT + Command.Yaw_Position_Move,8192,0)*0.043f);//PD
				PID_Calc(&Balance_Yaw_Position_pid,GM6020_YAW.Position.Real_fp32,Func_ValueRannge(YAW_POSITION_INIT + Command.Yaw_Position_Move,8192,0)*0.043f);//PD

			
			
		}
		
		Leg_Control_Cacl();
		//PID_Calc(&Leg_theta_Harmonize_pid[1],Leg[LEFT].theta - Leg[RIGHT].theta ,0);
		
//		L9015_Output_Zero();//缂佸倷鐒﹂娑欐姜椤旂晫妲嬮柣銏犵仛濠р偓閺夊牊鎸搁崵顓㈡晬瀹€鍐閻犲洦娲滈弫锟�
		
	}

}


//缂佹鍓熷ù鍌氼潰閵夘垳绀夐梺顐ｄ亢缁诲儊WT閻犱讲鍓濆鍌炲闯閵娿劌绠€电増顨夌换妯肩矙鐎ｎ偄鈷旈悶娑樿嫰閹冲棝寮甸敓锟�

//缂佹鍏涚粩鏉戭潰閵夘垳绀夐柡宥堫潐瀹撲線宕楃€圭姴螡闁汇垹鐏氬┃鈧柛娆忕Ч椤╊參寮悧鍫濈ウ闁告粌鏈崕鑽も偓闈涘悑閺嗙喖骞戦娆惧悁缂佺姵顨呴懟鐔兼嚂閺冨洤鎮柣妯垮煐閳ь兛绶氶崳锟�
/**
 * @brief 鐎归潻缂氶弲鍫曠嵁閹绮撻柤浣冩硶婵悂骞€娴ｇǹ缍侀梺鎻掔箺椤撳摜绮诲Δ鈧崵閬嶅极閿燂拷
 * @param NONE
 * @retval NONE
 * @author LTL
 */
void Jointmotor_Feedback_Update_Left(float dt)
{
	float xD, yD, xB, yB, A0, B0, C0 , xC , yC;//閻庤鐭粻鐔哥▔鐎涙ɑ顦ч柛娆愶耿閸ｏ拷
	static float L0_last ,theta2_last, theta_last;
	
	Leg->coord[0] = xB = -l5_Half + l1 * arm_cos_f32(Leg[LEFT].theta1);
	Leg->coord[1] = yB = l1 * arm_sin_f32(Leg[LEFT].theta1);
	Leg->coord[2] = xC = l1 * arm_cos_f32(Leg[LEFT].theta1) + l2 * arm_cos_f32(Leg[LEFT].theta2);
	Leg->coord[3] = yC = l1 * arm_sin_f32(Leg[LEFT].theta1) + l2 * arm_sin_f32(Leg[LEFT].theta2);
	Leg->coord[4] = xD = l5_Half + l4 * arm_cos_f32(Leg[LEFT].theta4);
	Leg->coord[5] = yD = l4 * arm_sin_f32(Leg[LEFT].theta4);
	
	A0 = 2.0f * l2 * (xD - xB);
	B0 = 2.0f * l2 * (yD - yB);
	C0 = powf(xD - xB,2) + powf(yD - yB,2);
	Leg[LEFT].theta2 = 2.0f * atan2f(B0 + sqrtf(powf(A0,2) + powf(B0,2) - powf(C0,2)),A0 + C0);
	
	Leg[LEFT].theta0 = atan2(yC,xC - l5_Half);//
	Leg[LEFT].theta3 = atan2(yC - yD,xC - xD);

	Leg[LEFT].theta =  NORMAL_HALF_PI - Ins_Data.Pitch  - Leg[LEFT].theta0; //闂佺妫勫Λ娆忣焽瀹€鍕剮缂佸顑欓崵鐘裁瑰⿰鍐劉闁稿骸顭烽獮鈧憸蹇涙焾閹绢喖鍨傞柛灞剧矋缁绢垶鏌￠崒婊勫殌闁诡喗绮撻幆鍐礋椤愮姳绮撮柣鐔哥懕閹凤拷
	Leg[LEFT].L0 = sqrtf(powf(xC - l5_Half,2) + powf(yC,2));//闂佺妫勫Λ娆忣焽瀹€鍕拹闁割煈鍠栭锟�
	Leg[LEFT].height = Leg[LEFT].L0 * arm_cos_f32(Leg[LEFT].theta);//闂佸憡顨嗗ú鎴﹀疾閸洖瀚夐弶鐐村缁夌厧螖閸屾耽顏嗏偓鐧告嫹


	Leg[LEFT].theta2_dot = (Leg[LEFT].theta2 - theta2_last) ;//
	One_Older_LPF(Leg[LEFT].theta2_dot,&Leg[LEFT].theta2_dot_lpf,0.5);
	
	Leg[LEFT].L0_dot = (Leg[LEFT].L0 - L0_last);
	One_Older_LPF(Leg[LEFT].L0_dot,&Leg[LEFT].L0_dot_lpf,0.5);
	Leg[LEFT].theta_dot = (Leg[LEFT].theta - theta_last)/0.001f;
	Leg[LEFT].theta_dot_lpf = Theta_Klpf*Leg[LEFT].theta_dot_lpf +(1- Theta_Klpf)*Leg[LEFT].theta_dot;
	//One_Older_LPF(Leg[LEFT].theta_dot,0.99);
	
	L0_last = Leg[LEFT].L0;
	theta2_last = Leg[LEFT].theta2;
	theta_last = Leg[LEFT].theta;
}

/**
 * @brief 闁告瑥鍘栭弲鍫曠嵁閹绮撻柤浣冩硶婵悂骞€娴ｇǹ缍侀梺鎻掔箺椤撳摜绮诲Δ鈧崵閬嶅极閿燂拷
 * @param NONE
 * @retval NONE
 * @author LTL
 */
void Jointmotor_Feedback_Update_Right(float dt)
{
	float xD, yD, xB, yB, A0, B0, C0 , xC , yC;//闁诲氦顫夐惌顔剧不閻斿摜鈻旈悗娑櫳戦ˇ褔鏌涘▎鎰惰€块柛锝忔嫹
	static float L0_last ,theta2_last, theta_last;
	Leg->coord[0] = xB = -l5_Half + l1 * arm_cos_f32(Leg[RIGHT].theta1);
	Leg->coord[1] = yB = l1 * arm_sin_f32(Leg[RIGHT].theta1);
	Leg->coord[2] = xC = l1 * arm_cos_f32(Leg[RIGHT].theta1) + l2 * arm_cos_f32(Leg[RIGHT].theta2);
	Leg->coord[3] = yC = l1 * arm_sin_f32(Leg[RIGHT].theta1) + l2 * arm_sin_f32(Leg[RIGHT].theta2);
	Leg->coord[4] = xD = l5_Half + l4 * arm_cos_f32(Leg[RIGHT].theta4);
	Leg->coord[5] = yD = l4 * arm_sin_f32(Leg[RIGHT].theta4);
	A0 = 2.0f * l2 * (xD - xB);
	B0 = 2.0f * l2 * (yD - yB);
	C0 = pow(xD - xB,2) + pow(yD - yB,2);
	Leg[RIGHT].theta2 = 2.0f * atan2(B0 + sqrt(pow(A0,2) + pow(B0,2) - pow(C0,2)),A0 + C0);
	
	Leg[RIGHT].theta0 = atan2(yC,xC - l5_Half);
	Leg[RIGHT].theta3 = atan2(yC - yD,xC - xD);
	
	
	Leg[RIGHT].theta =  -NORMAL_HALF_PI - Ins_Data.Pitch  + Leg[RIGHT].theta0;
	Leg[RIGHT].L0 = sqrt(pow(xC - l5_Half,2) + pow(yC,2));
	Leg[RIGHT].height = Leg[RIGHT].L0 * arm_cos_f32(Leg[RIGHT].theta);//闂佸憡顨嗗ú鎴﹀疾閸洖瀚夐弶鐐村缁夌厧螖閸屾耽顏嗏偓鐧告嫹


	Leg[RIGHT].theta2_dot = (Leg[RIGHT].theta2 - theta2_last);//
	One_Older_LPF(Leg[RIGHT].theta2_dot,&Leg[RIGHT].theta2_dot_lpf,0.5);
	Leg[RIGHT].L0_dot = (Leg[RIGHT].L0 - L0_last);
	One_Older_LPF(Leg[RIGHT].L0_dot,&Leg[RIGHT].L0_dot_lpf,0.5);
	Leg[RIGHT].theta_dot = (Leg[RIGHT].theta - theta_last)/0.001f;	
	Leg[RIGHT].theta_dot_lpf = Theta_Klpf*Leg[RIGHT].theta_dot_lpf +(1- Theta_Klpf)*Leg[RIGHT].theta_dot;
	L0_last = Leg[RIGHT].L0;
	theta2_last = Leg[RIGHT].theta2;
	theta_last = Leg[RIGHT].theta;
	
	//湖大跃鹿的关节状态预测算法，在这个车上效果不是很好，可能还需调试，暂未采用
	// 妫板嫭绁存稉瀣╃娑擃亝妞傞崚锟�
//    static float predict_dt = 0.0001f;
//    float phi1_pred = Leg[RIGHT].theta1 + Leg[RIGHT].theta1_dot * predict_dt; // 妫板嫭绁存稉瀣╃閺冭泛鍩㈤惃鍕彠閼哄倽顫楁惔锟�(閸掆晝鏁ら崗瀹犲Ν鐟欐帡鈧喎瀹�)
//    float phi4_pred = Leg[RIGHT].theta4 + Leg[RIGHT].theta4_dot * predict_dt;

//    // 闁插秵鏌婄拋锛勭暬閼靛潡鏆遍崪宀冨悪鐟欐帒瀹�
//    xD = l5_Half + l4 * arm_cos_f32(phi4_pred);
//    yD = l4 * arm_sin_f32(phi4_pred);
//    xB = l1 * arm_sin_f32(phi1_pred);
//    yB = l1 * arm_sin_f32(phi1_pred);

//    C0 = powf(xD - xB, 2) + powf(yD - yB, 2);
//    A0 = 2.0f * l2 * (xD - xB);
//    B0 = 2.0f * l2 * (yD - yB);
//    float phi2_pred = 2.0f * atan2(B0 + sqrt(pow(A0,2) + pow(B0,2) - pow(C0,2)),A0 + C0);
//    xC = l1 * arm_cos_f32(phi1_pred) + l2 * arm_cos_f32(phi2_pred);
//    yC = l1 * arm_sin_f32(phi1_pred) + l2 * arm_sin_f32(phi2_pred);
//    float phi5_pred = atan2(yC,xC - l5_Half);

//    // 瀹割喖鍨庣拋锛勭暬閼靛潡鏆遍崣妯哄閻滃洤鎷伴懙鑳潡闁喎瀹�
//    Leg[RIGHT].theta2_dot = (phi2_pred - Leg[RIGHT].theta2) / predict_dt; // 缁嬪秴鎮楅悽銊ょ艾娣囶喗顒滄潪顕€鈧拷
//    Leg[RIGHT].L0_dot = (sqrt(pow(xC - l5_Half,2) + pow(yC,2)) - Leg[RIGHT].L0) / predict_dt;
//    Leg[RIGHT].theta_dot = ((-phi5_pred - 0.5 * PI - (Ins_Data.Pitch + Ins_Data.Gyro[0] * predict_dt) - Leg[RIGHT].theta) / predict_dt); // 閸欘垯浜掓稉宥堚偓鍐閺堣桨缍�? -predict_dt*chassis.pitch_w

}

//用一阶卡尔曼滤波器融合加速度计和轮速计数据来估计车体速度，目前效果尚可，希望可以进一步调整观测器参数以达到更好的效果
/**
 * @brief 闂佹彃鐬奸埢鑲╂媼閳╁啯绾柡鍌涙緲閸ら亶寮敓锟�
 * @param 闂佹彃鐬奸埢鑲╂媼閿涘嫮娉㈤柡瀣缂嶏拷
 * @retval NONE
 */
void Odometry_Update(Leg_t* lp, Leg_t* rp,State_Val_t* state,LK9025_Motor_t* lw,LK9025_Motor_t* rw,INS_t* imu,float dt)
{

	//閺堫剝婧呴梽鈧摶杞板崕閺佺増宓侀惃鍕壈娑斿顩ф稉瀣剁窗
	//Z鏉炵繝璐熺€圭偠婧匵鏉炶揪绱漍鏉炵繝璐熺€圭偠婧匶鏉炶揪绱漎鏉炵繝璐熺€圭偠婧匷鏉烇拷
	// 濞ｅ浂鍠楅婊勬姜椤曗偓閳ь剛鍠庨幏鎵崉濠靛牜鐎�
    lp->wheel_w = lw->Speed.Real - lp->theta2_dot + imu->Gyro[0]; // 闁告垵绻愰獮鎾诲椽鐏炵晫鏆伴悗娑欏姇濞存劖娼婚悙鍨暠phi2_w
    rp->wheel_w = rw->Speed.Real - rp->theta2_dot - imu->Gyro[0]; 

    // 濞寸姰鍎撮悿鍡欌偓娑欏姃鐠愮喖宕洪搹鐟颁化,閻犱緤绱曢悾濠氬嫉鏉炴壆绉煎☉鎾卞€撻弲鑸殿殗鐎ｎ亜褰犻柤鍝勫€搁ˇ鈺呮儍閸曨垪鍋撻悢宄邦唺
    lp->body_v = lp->wheel_w * WHEEL_RADIUS + lp->L0 * 0.001 * lp->theta_dot_lpf + lp->L0_dot_lpf* 0.001 * arm_sin_f32(lp->theta);
    rp->body_v = rp->wheel_w * WHEEL_RADIUS - lp->L0 * 0.001 * rp->theta_dot_lpf - rp->L0_dot_lpf* 0.001 * arm_sin_f32(rp->theta);
    state->X_dot_meansure = (lp->body_v - rp->body_v) / 2; // 闁哄牐妗ㄧ紞瀣焻閻斿嘲顔�(妤犵偛鍟挎慨锟�)濞戞捁妗ㄧ悮杈ㄧ瑹瑜旈埀顒傚枎鐎规娊鎯冮崟顐︽尙闁秆冩搐閳ь剨鎷�
    
    // 閹碉綁娅庨弮瀣祮鐎佃壈鍤ч惃鍕倻韫囧啫濮為柅鐔峰閸滃矁顫楅崝鐘烩偓鐔峰*R
    float *gyro = imu->Gyro, *dgyro = imu->dgyro;
    //static float yaw_ddwrNwwr, yaw_p_ddwrNwwr, pitch_ddwrNwwr;
    yaw_ddwrNwwr = -powf(gyro[1], 2) * CENTER_IMU_L + dgyro[1] * CENTER_IMU_W;     // yaw闁哄啫顑堝ù鍡欌偓浣冨閸ゎ湷otion_acc[1]闁汇劌瀚伴·鍌涘緞閺嵮冾潱闂侇偆鍠庣€癸拷(闁哄牐妗ㄧ紞瀣礈瀹ュ懏鍊甸柡鍌滄嚀閹拷)
    yaw_p_ddwrNwwr = -powf(gyro[0], 2) * CENTER_IMU_L - dgyro[0] * CENTER_IMU_H;   // pitch闁哄啫顑堝ù鍡欌偓浣冨閸ゎ湷otion_acc[1]闁汇劌瀚伴·鍌涘緞閺嵮冾潱闂侇偆鍠庣€癸拷(闁哄牐妗ㄧ紞瀣礈瀹ュ懏鍊甸柡鍌滄嚀閹拷)
    pitch_ddwrNwwr = -powf(gyro[0], 2) * CENTER_IMU_H + dgyro[0] * CENTER_IMU_L;   // pitch闁哄啫顑堝ù鍡欌偓浣冨閸ゎ湷otion_acc[2]闁汇劌瀚伴·鍌涘緞閺嵮冾潱闂侇偆鍠庣€癸拷(闁哄牐妗ㄧ紞瀣博閺嶎偅绾柡鍌滄嚀閹拷)

    // 閻炴稏鍎辨导鈺呭触鎼达絾鐣遍悗鍦仱濡绢垶鐛崘鎻捫楅柛鏃傚█閳ь剛鍠庣€癸拷,闁哄牐妗ㄧ紞瀣寲鐠囨彃顤呴弶鈺傜⊕閺岀喖宕ラ幋婵囧缂佹梹鐗滃ú鍧楀棘閻熺増鍊�
    //static float macc_x, macc_z;
    macc_x = imu->MotionAccel_b[2] - yaw_ddwrNwwr - yaw_p_ddwrNwwr;
    macc_z = imu->MotionAccel_b[1] - pitch_ddwrNwwr;

    // 闁哄牐妗ㄧ紞瀣礉閻樼儵鍋撻悢宄邦唺闁硅埖娲栨總鏍礆閻楀牊瀵滄鐐舵珪閺岀喖宕ラ幋婊呯憪
    static float pitch;
    pitch = imu->Pitch;
    state->acc_last = state->acc_m;
    state->acc_m = macc_x * arm_cos_f32(pitch) - macc_z * arm_sin_f32(pitch);
//	if (Func_Abs(state->acc_m)<0.03)
//		state->acc_m = 0;
	state->acc_m+=0.0526736666;
    // 闁捐绉撮幃搴ㄥ礉閻樼儵鍋撻悢宄邦唺閻犱緤绱曞▓鎴﹀极閻楀牆绁﹂柛婊冩湰濠р偓濞达絾鎹囬埀顒傚枎鐎癸拷
    //static float u, k;   // 閺夊牊鎸搁崣鍡涘椽鐏炶棄骞㈤悘蹇旀⒐濞存牗鏅堕悙鍨妱
    //static float vel_prior, vel_measure, vel_cov;     // 闁稿繐鐗撻悰娆愬閹峰矈鍚€闁靛棔鐒︾粊鎾煂韫囧鍋撴担绋垮弗濡ょ姴鑻畷妤呭棘閻熺増鈻�

    // 濡澘瀚粊锟�
    u = (state->acc_m + state->acc_last) / 2;         //闂勫秴娅�
	//static float u_lpf;
	//One_Older_LPF(u,&u_lpf,0.53);
    state->X_dot_predict= vel_prior = state->X_dot + dt * u;          // 閻樿埖鈧浇娴嗙粔浼欑礉闁喎瀹抽崗鍫ョ崣妫板嫭绁寸粵澶夌艾娑撳﹣绔撮弮璺哄煝闁喎瀹抽崥搴ㄧ崣妫板嫭绁撮崐鐓庡娑撳﹤缍嬮崜宥呭闁喎瀹虫稉宸噒閻ㄥ嫪绠荤粔锟�
    vel_cov = state->X_dot_cov + VEL_PROCESS_NOISE * dt;          //妫板嫭绁撮崡蹇旀煙瀹割喗娲块弬锟�

    // 闁哄秮鍓濋锟�
    vel_measure = state->X_dot_meansure;	//鐏忓棗鐔€娴滃海绱惍浣告珤閸滃矁绻嶉崝銊ヮ劅鐟欙絿鐣婚崙铏规畱閺堣桨缍嬮柅鐔峰娴ｆ粈璐熷ù瀣櫤閸婏拷
    k = vel_cov / (vel_cov + VEL_MEASURE_NOISE);            // 闁告せ鈧磭姣滈柡鍥╁帶椤ゅ啴鎯勯敓锟�
    state->X_dot = vel_prior + k * (vel_measure - vel_prior);    // 闁告艾閰ｉ悰娆愬閹峰矈鍚€
    state->X_dot_cov = (1 - k) * vel_cov;                        // 闁告艾閰ｉ悰娆撳础韫囨梹鐓欑€归潻鎷�

    Func_Limit(state->X_dot_cov, 0.01, 100);       //对速度协方差进行限幅，防止观测器发散
	
//	 if(Robot_Status.Body.Body_Upright_Status == YES)
//	// //在机器人直立后对速度进行积分求得车体位移，之后会改为速度低于阈值后的速度积分项
//	// //据现在的测试结果，在全部运动状态下都进行积分会导致加速度计在激烈运动下的误差被累计，导致机器人运动稳定性下降
//	 	state->X = state->X + state->X_dot * dt;
	if(Robot_Status.Body.Body_Upright_Status == YES && Func_Abs(State_Variables.X_dot) <= 0.2)
		state->X = state->X + state->X_dot * dt;
	else
		state->X = 0;

	//閻€劋绨弽鈥冲櫙
//	if(avg_count >0 && Robot_Status.Body.Body_Upright_Status ==YES && Command.Chassis_Power_Switch == ON)
//	{
//		acc_avg +=state->acc_m;
//		avg_count--;
//	}
	
}

//
void State_Variables_Update(State_Val_t* state , float dt)
{
	state ->Theta = (Leg[LEFT].theta + Leg[RIGHT].theta) /2;
	state ->Theta_dot = (Leg[LEFT].theta_dot + Leg[RIGHT].theta_dot) / 2;
	state ->Phi = Ins_Data.Pitch;
	state ->Phi_dot = Ins_Data.Gyro[0];
	state->leg_length = (Leg[LEFT].L0 + Leg[RIGHT].L0)* 0.0005f;//閻犱緤绱曢悾濠氭惞濮橆厼鐝柡澶婃閺嗭拷
	state->leg_len_pow2 = powf(state->leg_length ,2);//閻犱緤绱曢悾濠氭惞濮橆厼鐝柡澶婃閺嗛亶鎯冮崟顐︽尙闁哄倻娅㈢槐婵嬫偨閵娿倗鑹鹃梺顐㈩槹鐎氥劑宕ｅ澶樻疮闁活厸鏅犲Ο鈧�
	state ->target_Theta = theta_offset;
	state->target_Phi = fai_offset;
	state->target_Phi_dot = 0;

	switch (Command.SideWay_Set)
	{
		case LEFT:
		{
			state->target_Roll = 0;
			break;
		}
			
		case RIGHT:
		{
			state->target_Roll = 0;
			break;
		}
		default:
		{
			state->target_Roll = 0;
			break;
		}
	}

	if(Robot_Status.Body.Body_Upright_Status == YES)
	{
		switch(Command.Height_Set)
		{
			case 1:
			{
				state->target_Leg_Length[0] = L0_LOW;
                state->target_Leg_Length[1] = L0_LOW;
				//Command.Height_Switch =0;
				break;
			}
			case 2:
			{
				state->target_Leg_Length[0] = L0_MID;
                state->target_Leg_Length[1] = L0_MID;
				//Command.Height_Switch =0;
				break;
			}
			case 3:
			{
				state->target_Leg_Length[0] = L0_HIGH;
                state->target_Leg_Length[1] = L0_HIGH;
				//Command.Height_Switch =0;
				break;
			}
			default:
				break;
		}
	}else
		state->target_Height = 0;
	Func_Ramp((float)(-Command.Vx * 10),&Command.Vx_step,1);
	state->target_X_dot = Command.Vx_step * 0.001f;
	
	//当机器人实际速度小于阈值时开启位移设定值的计算，与位移反馈值的计算同步进行
	//避免加速度计噪声和误差导致机器人运动稳定性下降
	if(Robot_Status.Body.Body_Upright_Status == YES && Func_Abs(state->X_dot) <= 0.2 )
		state->target_X +=( state->target_X_dot) * dt;
//	else
//		state->target_X = 0;//在阈值外时将设定值清零

//	if(Robot_Status.Body.Body_Upright_Status == YES)
//		state->X += (state->target_X_dot - state->X_dot) * dt;

}

void LQR_Calc(State_Val_t* state,Leg_t* leg,Robot_Status_t* robot_status)
{
	
		leg->U[0]= (a11[0] * state->leg_len_pow3 +a11[1] * state->leg_len_pow2 + a11[2] * state->leg_length + a11[3]) * (state ->target_Theta-leg->theta)\
            +(a12[0] * state->leg_len_pow3 +a12[1] * state->leg_len_pow2 + a12[2] * state->leg_length + a12[3]) * (-leg->theta_dot_lpf)\
            -(a14[0] * state->leg_len_pow3 +a14[1] * state->leg_len_pow2 + a14[2] * state->leg_length + a14[3]) * (state->target_X_dot - state->X_dot)\
			+(a15[0] * state->leg_len_pow3 +a15[1] * state->leg_len_pow2 + a15[2] * state->leg_length + a15[3]) * (state->target_Phi - state->Phi)\
            +(a16[0] * state->leg_len_pow3 +a16[1] * state->leg_len_pow2 + a16[2] * state->leg_length + a16[3]) * (-state->Phi_dot);
		if(Func_Abs(state->X_dot)<=0.2f)
			leg->U[0]-=(a13[0] * state->leg_len_pow3 +a13[1] * state->leg_len_pow2 + a13[2] * state->leg_length + a13[3]) * (state->target_X - state->X);
	
//	if(leg->P <=0.05 && robot_status->Body.Body_Upright_Status == YES) //触地不良时关闭轮毂输出，可考虑改为根据触地力进行动态调整的限幅
//		leg->U[0] = 0;

	if(robot_status->Body.Body_Upright_Status == YES)
	{
		//腿触地良好时全量控制
		{
		leg->U[1]= (a21[0] * state->leg_len_pow3 +a21[1] * state->leg_len_pow2 + a21[2] * state->leg_length + a21[3]) * (state ->target_Theta-leg->theta)\
            +(a22[0] * state->leg_len_pow3 +a22[1] * state->leg_len_pow2 + a22[2] * state->leg_length + a22[3]) * (-leg->theta_dot_lpf)\
            -(a24[0] * state->leg_len_pow3 +a24[1] * state->leg_len_pow2 + a24[2] * state->leg_length + a24[3]) * (state->target_X_dot - state->X_dot)\
			+(a25[0] * state->leg_len_pow3 +a25[1] * state->leg_len_pow2 + a25[2] * state->leg_length + a25[3]) * (state->target_Phi - state->Phi)\
        	+(a26[0] * state->leg_len_pow3 +a26[1] * state->leg_len_pow2 + a26[2] * state->leg_length + a26[3]) * (-state->Phi_dot);
		if(Func_Abs(state->X_dot)<=0.2f)						
			leg->U[1]-=(a23[0] * state->leg_len_pow3 +a23[1] * state->leg_len_pow2 + a23[2] * state->leg_length + a23[3]) * (state->target_X - state->X);
				//(1-Func_Abs(state->target_X_dot*0.51f))
		}
//		if(leg->P<=0.05)//腿触地不良时保持腿垂直于水平方向
//		{
//			leg->U[1]= (a21[0] * state->leg_len_pow3 +a21[1] * state->leg_len_pow2 + a21[2] * state->leg_length + a21[3]) * (state ->target_Theta-leg->theta)\
//            +(a22[0] * state->leg_len_pow3 +a22[1] * state->leg_len_pow2 + a22[2] * state->leg_length + a22[3]) * (-leg->theta_dot_lpf);
//		}

	}

	
}


float Leg_L0_Set(void)
{
	if(Command.Height_Set == LOW)
		return L0_LOW;
  	if(Command.Height_Set == HIGH)
		return L0_HIGH;
	return L0_MID;
}


float Leg_Theta0_Limit(Leg_t* leg)
{
	if(leg->L0 < 0.94f)
		return 0;
	float theta0_limit;
	theta0_limit = leg->L0 * 0.65f - 0.56f;
	  return theta0_limit;
}

void Leg_Control_Cacl(void)
{
	//PID_Calc(&Leg_ROLL_Compensate_pid[1],Ins_Data.Roll,State_Variables.target_Roll);
	PID_Calc(&L0_pid[0],Leg[0].L0,State_Variables.target_Leg_Length[0] - Leg_ROLL_Compensate_pid[1].Output);
	PID_Calc(&L0_pid[1],Leg[1].L0,State_Variables.target_Leg_Length[1] + Leg_ROLL_Compensate_pid[1].Output);
	
	PID_Calc(&Leg_theta_Harmonize_pid[0],Leg[LEFT].theta - Leg[RIGHT].theta ,0);
	PID_Calc(&Leg_ROLL_Compensate_pid[0],Ins_Data.Roll,State_Variables.target_Roll);
}

void Leg_Output(Leg_t * leg,float f,float tp)
{
	leg->F = f;
	leg->Tp = tp;
}

void Leg_Fall_Mode(void)
{
	Leg[0].L0_set     = 120;

	Leg[1].L0_set     = 120;

}



	


