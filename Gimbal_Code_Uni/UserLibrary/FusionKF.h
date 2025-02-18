#ifndef _FUSION_KF_H_
#define _FUSION_KF_H_

/**
 * @attention   采用UTF-8字符集编码
 */

#include "Universal.h"

typedef struct
{
    matrix_t x[3];  //状态向量 x[1]<=>上次的状态 x[2]<=>测量到的状态
    matrix_t z;     //测量向量
    matrix_t F;     //状态转移矩阵
    matrix_t FT;    //状态转移矩阵的转置
    matrix_t u;     //输入向量
    matrix_t G;     //控制矩阵
    matrix_t P[2];  //估计协方差 P[1]<=>上次的协方差
    matrix_t Q;     //过程噪声协方差矩阵
    matrix_t R;     //测量噪声协方差矩阵
    matrix_t H;     //观测矩阵
    matrix_t HT;    //观测矩阵的转置
    matrix_t K;     //卡尔曼增益

    matrix_t temp[2];   //计算过程临时变量

    arm_status MatStatus;   //保存矩阵运算过程中的返回值

    int isMeasurementUpdated;
} FusionKF_t;

void FusionKF_Init(FusionKF_t* kf, float deltaTime, float accelVar, float odomVar);
void FusionKF_UpdateMeasurement(FusionKF_t* kf, float odomSpeedFB, float odomSpeedLR);
void FusionKF_Update(FusionKF_t* kf, float imuAccelFB, float imuAccelLR);

#endif
