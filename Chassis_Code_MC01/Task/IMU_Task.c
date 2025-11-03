#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "usart.h"
#include "tim.h"
#include "arm_math.h"
#include "stdint.h"

#include "BMI088driver.h"
#include "BMI088Middleware.h"
#include "BMI088reg.h"
#include "PID.h"
#include "QuaternionEKF.h"
#include "universal.h"
#include "Dwt_Timer.h"
#include "Timer.h"

#include "INS_Task.h"
#include "Configuration.h"


#define sampleFreq 1000.0f
#define X 0
#define Y 1
#define Z 2

PidTypeDef TempCtrl_pid ={0};
IMU_Param_t IMU_Param = {0};

float RefTemp = 40;//bmi088温控设定值
float  twoKp =(2*3.0f);// 2 * proportional gain (Kp)
float twoKi =(2*0.001f);// 2 * integral gain (Ki)
float twoKd =(2*0.000f);

volatile float q0 = 1.0f, q1 = -1.0f, q2 = 0.0f, q3 = 0.0f;					// quaternion of sensor frame relative to auxiliary frame
volatile float integralFBx = 0.0f,  integralFBy = 0.0f, integralFBz = 0.0f;	// integral error terms scaled by Ki

//static 
float InsTask_dt = 0;
static uint32_t operate_count = 0;
uint32_t INS_DWT_Count = 0;
const float xb[3] = {1, 0, 0};
const float yb[3] = {0, 1, 0};
const float zb[3] = {0, 0, 1};
float gravity_b[3];

float Lpf_State_Val = 0;
float K_Lpf = 0.9f;
float Accel_Data_Avg = 0;
float X_accel=0;
//uint32_t count[2] ={0};
IMU_Data_t BMI_088 = {0};
INS_t Ins_Data = {0};

void INS_Task(void const * argument)
{
    portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
	
	const float gravity[3] = {0, 0, 9.81f};
	
    while(1)
    {
        vTaskDelayUntil(&xLastWakeTime,1);
		operate_count++;
		//通过MCU底层计时器获取进程运行周期，保证算法微分、积分运算的准确
		InsTask_dt = DWT_GetDeltaT(&INS_DWT_Count);

		//SPI读取BMI088数据
        BMI088_Read(&BMI088);
		
		// for(uint8_t count =0;count<3;count++)
		// 	One_Older_LPF(Ins_Data.Gyro[count],&Ins_Data.Gyro_lpf[count],Ins_Data.Gyro_Klpf);
		//mahony算法更新四元数后反解欧拉角
        MahonyAHRSupdateIMU(&Ins_Data,BMI088.Gyro[0],BMI088.Gyro[1],BMI088.Gyro[2],BMI088.Accel[0],BMI088.Accel[1],BMI088.Accel[2]);
		//更新惯导结构体数据
        Ins_Data.Accel[0] = BMI088.Accel[0];
		Ins_Data.Accel[1] = BMI088.Accel[1];
		Ins_Data.Accel[2] = BMI088.Accel[2];
		Ins_Data.Gyro[0] = BMI088.Gyro[0];
		Ins_Data.Gyro[1] = BMI088.Gyro[1];
		Ins_Data.Gyro[2] = BMI088.Gyro[2];

		//计算角加速度，以便里程计算法使用
        Ins_Data.dgyro[X] = (BMI088.Gyro[X] - Ins_Data.Gyro[X])/ (Ins_Data.DGyroLPF + InsTask_dt) + Ins_Data.dgyro[X] * Ins_Data.DGyroLPF / (Ins_Data.DGyroLPF + InsTask_dt);
        Ins_Data.dgyro[Y] = (BMI088.Gyro[Y] - Ins_Data.Gyro[Y])/ (Ins_Data.DGyroLPF + InsTask_dt) + Ins_Data.dgyro[Y] * Ins_Data.DGyroLPF / (Ins_Data.DGyroLPF + InsTask_dt);
        Ins_Data.dgyro[Z] = (BMI088.Gyro[Z] - Ins_Data.Gyro[Z])/ (Ins_Data.DGyroLPF + InsTask_dt) + Ins_Data.dgyro[Z] * Ins_Data.DGyroLPF / (Ins_Data.DGyroLPF + InsTask_dt);

		//计算惯性系下机器人机体加速度，以便里程计算法使用
					//目前还没彻底吃透、调明白，速度
					// 机体系基向量转换到导航坐标系，本例选取惯性系为导航系
					BodyFrameToEarthFrame(xb, Ins_Data.xn, Ins_Data.q);
					BodyFrameToEarthFrame(yb, Ins_Data.yn, Ins_Data.q);
					BodyFrameToEarthFrame(zb, Ins_Data.zn, Ins_Data.q);

					// 将重力从导航坐标系n转换到机体系b,随后根据加速度计数据计算运动加速度
					
					EarthFrameToBodyFrame(gravity, gravity_b, Ins_Data.q);
					for (uint8_t i = 0; i < 3; i++) // 同样过一个低通滤波
					{
							Ins_Data.MotionAccel_b[i] = (Ins_Data.Accel[i] - gravity_b[i]) * InsTask_dt / (Ins_Data.AccelLPF + InsTask_dt) + Ins_Data.MotionAccel_b[i] * Ins_Data.AccelLPF / (Ins_Data.AccelLPF + InsTask_dt);
					}
					BodyFrameToEarthFrame(Ins_Data.MotionAccel_b, Ins_Data.MotionAccel_n, Ins_Data.q); // 转换回导航系n


		if(operate_count%2 ==0)
			IMU_Temperature_Ctrl();
//        if(count[0] !=60000&&count[1]==0)
//        {
//            Accel_Data_Avg+= BMI088.Accel[2];
//            count[0]++;
//        }
//            else if(count[1] ==0)
//            {
//            Accel_Data_Avg *= 0.0000167f;
//                count[1] = 1;
//            }
		
//        X_accel= BMI088.Accel[2] + arm_sin_f32(Ins_Data.Pitch)*9.82f;
		
//		HAL_GPIO_TogglePin(GPIOC,RS485_VCC_Pin);


//					BMI088_Read(&BMI088);

//					Ins_Data.Accel[X] = BMI088.Accel[X];
//					Ins_Data.Accel[Y] = BMI088.Accel[Y];
//					Ins_Data.Accel[Z] = BMI088.Accel[Z];
//					Ins_Data.Gyro[X] = BMI088.Gyro[X];
//					Ins_Data.Gyro[Y] = BMI088.Gyro[Y];
//					Ins_Data.Gyro[Z] = BMI088.Gyro[Z];

					// 计算重力加速度矢量和b系的XY两轴的夹角,可用作功能扩展,本demo暂时没用
					//INS.atanxz = -atan2f(INS.Accel[X], INS.Accel[Z]) * 180 / PI;
					//INS.atanyz = atan2f(INS.Accel[Y], INS.Accel[Z]) * 180 / PI;

					// 核心函数,EKF更新四元数
					//IMU_QuaternionEKF_Update(Ins_Data.Gyro[X], Ins_Data.Gyro[Y], Ins_Data.Gyro[Z], Ins_Data.Accel[X], Ins_Data.Accel[Y], Ins_Data.Accel[Z], dt);

					//memcpy(Ins_Data.q, QEKF_INS.q, sizeof(QEKF_INS.q));

					
    }
    
}

void INS_Init(void)
{
    IMU_Param.scale[X] = 1;
    IMU_Param.scale[Y] = 1;
    IMU_Param.scale[Z] = 1;
    IMU_Param.Yaw = 0;
    IMU_Param.Pitch = 0;
    IMU_Param.Roll = 0;
    IMU_Param.flag = 1;

    IMU_QuaternionEKF_Init(10, 0.001, 10000000, 1, 0);

    Ins_Data.AccelLPF = 0.5;//0.0085
	Ins_Data.DGyroLPF = 0.009;
	Ins_Data.Gyro_Klpf = 0.6;
}


uint8_t One_Older_LPF(float input, float* lpf_output ,float Klpf)
{
    if(Klpf >= 1.0f)//防呆用
        return 1;
    float X0 = input;
    *lpf_output= Klpf  * (*lpf_output) + (1 - Klpf) * X0;
    return 0;
}

//---------------------------------------------------------------------------------------------------
// IMU algorithm update

static float halfex[2], halfey[2], halfez[2];

void MahonyAHRSupdateIMU(INS_t* INS,float gx, float gy, float gz, float ax, float ay, float az) {
	float recipNorm;
	float halfvx, halfvy, halfvz;
	
	float qa, qb, qc;

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

		// Normalise accelerometer measurement
		recipNorm = invSqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;        

		// Estimated direction of gravity and vector perpendicular to magnetic flux
		halfvx = q1 * q3 - q0 * q2;
		halfvy = q0 * q1 + q2 * q3;
		halfvz = (q0 * q0 - q1*q1 - q2*q2 + q3 * q3) *0.5f;
	
		// Error is sum of cross product between estimated and measured direction of gravity
		halfex[0] = (ay * halfvz - az * halfvy);
		halfey[0] = (az * halfvx - ax * halfvz);
		halfez[0] = (ax * halfvy - ay * halfvx);

		// Compute and apply integral feedback if enabled
		if(twoKi > 0.0f) {
			integralFBx += twoKi * halfex[0] * (1.0f / sampleFreq);	// integral error scaled by Ki
			integralFBy += twoKi * halfey[0] * (1.0f / sampleFreq);
			integralFBz += twoKi * halfez[0] * (1.0f / sampleFreq);
			gx += integralFBx;	// apply integral feedback
			gy += integralFBy;
			gz += integralFBz;
		}
		else {
			integralFBx = 0.0f;	// prevent integral windup
			integralFBy = 0.0f;
			integralFBz = 0.0f;
		}

		// Apply proportional feedback
		gx += twoKp * halfex[0];
		gy += twoKp * halfey[0];
		gz += twoKp * halfez[0];
		
		gx +=twoKd * (halfex[0] - halfex[1]);
		gy +=twoKd * (halfey[0] - halfey[1]);
		gz +=twoKd * (halfez[0] - halfez[1]);
		
		halfex[1] = halfex[0];
		halfey[1] = halfey[0];
		halfez[1] = halfez[0];
	}
	
	// Integrate rate of change of quaternion
	gx *= (0.5f * (1.0f / sampleFreq));		// pre-multiply common factors
	gy *= (0.5f * (1.0f / sampleFreq));
	gz *= (0.5f * (1.0f / sampleFreq));
	qa = q0;
	qb = q1;
	qc = q2;
	q0 += (-qb * gx - qc * gy - q3 * gz);
	q1 += (qa * gx + qc * gz - q3 * gy);
	q2 += (qa * gy - qb * gz + q3 * gx);
	q3 += (qa * gz + qb * gy - qc * gx); 
	
	// Normalise quaternion
	recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= recipNorm;
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;
	
	INS->q[0] = q0;
	INS->q[1] = q1;
	INS->q[2] = q2;
	INS->q[3] = q3;
	
	INS->Yaw = atan2f(2.0f*(q0*q3+q1*q2), 2.0f*(q0*q0+q1*q1)-1.0f);
	INS->Roll = (asinf(-2.0f*(q1*q3-q0*q2))) - 0.021f;
	INS->Pitch = ((atan2f(2.0f*(q0*q1+q2*q3),2.0f*(q0*q0+q3*q3)-1.0f)) + NORMAL_HALF_PI);
}

//---------------------------------------------------------------------------------------------------
// Fast inverse square-root
// See: http://en.wikipedia.org/wiki/Fast_inverse_square_root

float invSqrt(float x) {
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}

/**
 * @brief          Transform 3dvector from BodyFrame to EarthFrame
 * @param[1]       vector in BodyFrame
 * @param[2]       vector in EarthFrame
 * @param[3]       quaternion
 */
void BodyFrameToEarthFrame(const float *vecBF, float *vecEF, float *q)
{
    vecEF[0] = 2.0f * ((0.5f - q[2] * q[2] - q[3] * q[3]) * vecBF[0] +
                       (q[1] * q[2] - q[0] * q[3]) * vecBF[1] +
                       (q[1] * q[3] + q[0] * q[2]) * vecBF[2]);

    vecEF[1] = 2.0f * ((q[1] * q[2] + q[0] * q[3]) * vecBF[0] +
                       (0.5f - q[1] * q[1] - q[3] * q[3]) * vecBF[1] +
                       (q[2] * q[3] - q[0] * q[1]) * vecBF[2]);

    vecEF[2] = 2.0f * ((q[1] * q[3] - q[0] * q[2]) * vecBF[0] +
                       (q[2] * q[3] + q[0] * q[1]) * vecBF[1] +
                       (0.5f - q[1] * q[1] - q[2] * q[2]) * vecBF[2]);
}

/**
 * @brief          Transform 3dvector from EarthFrame to BodyFrame
 * @param[1]       vector in EarthFrame
 * @param[2]       vector in BodyFrame
 * @param[3]       quaternion
 */
void EarthFrameToBodyFrame(const float *vecEF, float *vecBF, float *q)
{
    vecBF[0] = 2.0f * ((0.5f - q[2] * q[2] - q[3] * q[3]) * vecEF[0] +
                       (q[1] * q[2] + q[0] * q[3]) * vecEF[1] +
                       (q[1] * q[3] - q[0] * q[2]) * vecEF[2]);

    vecBF[1] = 2.0f * ((q[1] * q[2] - q[0] * q[3]) * vecEF[0] +
                       (0.5f - q[1] * q[1] - q[3] * q[3]) * vecEF[1] +
                       (q[2] * q[3] + q[0] * q[1]) * vecEF[2]);

    vecBF[2] = 2.0f * ((q[1] * q[3] + q[0] * q[2]) * vecEF[0] +
                       (q[2] * q[3] - q[0] * q[1]) * vecEF[1] +
                       (0.5f - q[1] * q[1] - q[2] * q[2]) * vecEF[2]);
}



/**
 * @brief 温度控制
 * 
 */
void IMU_Temperature_Ctrl(void)
{
    PID_Calc(&TempCtrl_pid, BMI088.Temperature, RefTemp);
	
    TIM_Set_PWM(&htim3, TIM_CHANNEL_2, (uint16_t)Func_Limit(float_rounding(TempCtrl_pid.Output), 0, (float) UINT32_MAX));
}

//------------------------------------functions below are not used in this demo-------------------------------------------------
//----------------------------------you can read them for learning or programming-----------------------------------------------
//----------------------------------they could also be helpful for further design-----------------------------------------------

/**
 * @brief        Update quaternion
 */
void QuaternionUpdate(float *q, float gx, float gy, float gz, float dt)
{
    float qa, qb, qc;

    gx *= 0.5f * dt;
    gy *= 0.5f * dt;
    gz *= 0.5f * dt;
    qa = q[0];
    qb = q[1];
    qc = q[2];
    q[0] += (-qb * gx - qc * gy - q[3] * gz);
    q[1] += (qa * gx + qc * gz - q[3] * gy);
    q[2] += (qa * gy - qb * gz + q[3] * gx);
    q[3] += (qa * gz + qb * gy - qc * gx);
}

/**
 * @brief        Convert quaternion to eular angle
 */
void QuaternionToEularAngle(float *q, float *Yaw, float *Pitch, float *Roll)
{
    *Yaw = atan2f(2.0f * (q[0] * q[3] + q[1] * q[2]), 2.0f * (q[0] * q[0] + q[1] * q[1]) - 1.0f) * 57.295779513f;
    *Pitch = atan2f(2.0f * (q[0] * q[1] + q[2] * q[3]), 2.0f * (q[0] * q[0] + q[3] * q[3]) - 1.0f) * 57.295779513f;
    *Roll = asinf(2.0f * (q[0] * q[2] - q[1] * q[3])) * 57.295779513f;
}

/**
 * @brief        Convert eular angle to quaternion
 */
void EularAngleToQuaternion(float Yaw, float Pitch, float Roll, float *q)
{
    float cosPitch, cosYaw, cosRoll, sinPitch, sinYaw, sinRoll;
    Yaw /= 57.295779513f;
    Pitch /= 57.295779513f;
    Roll /= 57.295779513f;
    cosPitch = arm_cos_f32(Pitch / 2);
    cosYaw = arm_cos_f32(Yaw / 2);
    cosRoll = arm_cos_f32(Roll / 2);
    sinPitch = arm_sin_f32(Pitch / 2);
    sinYaw = arm_sin_f32(Yaw / 2);
    sinRoll = arm_sin_f32(Roll / 2);
    q[0] = cosPitch * cosRoll * cosYaw + sinPitch * sinRoll * sinYaw;
    q[1] = sinPitch * cosRoll * cosYaw - cosPitch * sinRoll * sinYaw;
    q[2] = sinPitch * cosRoll * sinYaw + cosPitch * sinRoll * cosYaw;
    q[3] = cosPitch * cosRoll * sinYaw - sinPitch * sinRoll * cosYaw;
}