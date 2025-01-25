/*
 * @Author: 静_火 lintianlang0918@qq.com
 * @Date: 2024-03-26 18:11:12
 * @LastEditors: 静_火 lintianlang0918@qq.com
 * @LastEditTime: 2024-04-03 22:38:12
 * @FilePath: \MC-01_chassis_copy\Task\inc\INS_Task.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "stdint.h"

#ifdef __cplusplus
extern "C"
{
#endif

typedef struct
{
    float q[4]; // 四元数估计值

    float Gyro[3];  // 角速度
    float dgyro[3];
    float Accel[3]; // 加速度
    float MotionAccel_b[3]; // 机体坐标加速度
    float MotionAccel_n[3]; // 绝对系加速度
	
	float Gyro_lpf[3];
    float AccelLPF; // 加速度低通滤波系数
    float DGyroLPF;
	float Gyro_Klpf;
    // 加速度在绝对系的向量表示
    float xn[3];
    float yn[3];
    float zn[3];

    float atanxz;
    float atanyz;

    // 位姿
    float Roll;
    float Pitch;
    float Yaw;
    float YawTotalAngle;
		int16_t   Yaw_8192;
		int16_t   Pitch_8192;
} INS_t;

typedef struct
{
    uint8_t flag;

    float scale[3];

    float Yaw;
    float Pitch;
    float Roll;
} IMU_Param_t;

extern INS_t Ins_Data;

void INS_Task(void const * argument);

void INS_Init(void);
uint8_t One_Older_LPF(float input, float* lpf_output ,float Klpf);
float invSqrt(float x);
void MahonyAHRSupdateIMU(INS_t* INS,float gx, float gy, float gz, float ax, float ay, float az);
void BodyFrameToEarthFrame(const float *vecBF, float *vecEF, float *q);
void EarthFrameToBodyFrame(const float *vecEF, float *vecBF, float *q);
void IMU_Temperature_Ctrl(void);
#ifdef __cplusplus
}
#endif