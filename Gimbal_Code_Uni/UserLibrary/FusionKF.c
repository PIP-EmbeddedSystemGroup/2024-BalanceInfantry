/**
 * @attention   采用UTF-8字符集编码
 * @brief       惯导融合滤波器
 * @authors     北工大PIP战队 樊郡捷
 * @version     v1.0
 * @date        2024-10-26
 * @details     加速度计和里程计数据融合滤波器
 */

#include "header.h"

/// @brief 初始化卡尔曼融合滤波器
/// @param kf 滤波器对象
/// @param deltaTime 更新时间
/// @param accelVar 加速度计方差
/// @param odomVar 里程计方差
void FusionKF_Init(FusionKF_t* kf, float deltaTime, float accelVar, float odomVar)
{
    MAT_INIT(kf->temp[0], 4, 4);
    MAT_INIT(kf->temp[1], 4, 4);

    MAT_INIT(kf->x[0], 2, 1);
    MAT_INIT(kf->x[1], 2, 1);
    MAT_INIT(kf->x[2], 2, 1);

    MAT_INIT(kf->z, 2, 1);

    MAT_INIT(kf->F, 2, 2);
    MAT_ELEM(kf->F, 1, 1) = 1;  MAT_ELEM(kf->F, 1, 2) = 0;
    MAT_ELEM(kf->F, 2, 1) = 0;  MAT_ELEM(kf->F, 2, 2) = 1;

    MAT_INIT(kf->FT, 2, 2);
    kf->MatStatus = MAT_TRS(kf->F, &kf->FT);

    MAT_INIT(kf->u, 2, 1);

    MAT_INIT(kf->G, 2, 2);
    MAT_ELEM(kf->G, 1, 1) = deltaTime;  MAT_ELEM(kf->G, 1, 2) = 0;
    MAT_ELEM(kf->G, 2, 1) = 0;          MAT_ELEM(kf->G, 2, 2) = deltaTime;

    MAT_INIT(kf->P[0], 2, 2);
    MAT_INIT(kf->P[1], 2, 2);

    MAT_INIT(kf->Q, 2, 2);
    MAT_ELEM(kf->Q, 1, 1) = deltaTime * deltaTime * accelVar;   MAT_ELEM(kf->Q, 1, 2) = 0;
    MAT_ELEM(kf->Q, 2, 1) = 0;                                  MAT_ELEM(kf->Q, 2, 2) = deltaTime * deltaTime * accelVar;

    MAT_INIT(kf->R, 2, 2);
    MAT_ELEM(kf->R, 1, 1) = odomVar;    MAT_ELEM(kf->R, 1, 2) = 0;
    MAT_ELEM(kf->R, 2, 1) = 0;          MAT_ELEM(kf->R, 2, 2) = odomVar;

    MAT_INIT(kf->H, 2, 2);
    MAT_ELEM(kf->H, 1, 1) = 1;  MAT_ELEM(kf->H, 1, 2) = 0;
    MAT_ELEM(kf->H, 2, 1) = 0;  MAT_ELEM(kf->H, 2, 2) = 1;

    MAT_INIT(kf->HT, 2, 2);
    kf->MatStatus = MAT_TRS(kf->H, &kf->HT);

    MAT_INIT(kf->K, 2, 2);

    kf->isMeasurementUpdated = RESET;
}

/// @brief 将里程计测得的速度值用于卡尔曼融合滤波器的测量更新
/// @param kf 滤波器对象
/// @param odomSpeedFB 里程计顺向速度
/// @param odomSpeedLR 里程计横向速度
void FusionKF_UpdateMeasurement(FusionKF_t* kf, float odomSpeedFB, float odomSpeedLR)
{
    MAT_ELEM(kf->x[2], 1, 1) = odomSpeedFB;
    MAT_ELEM(kf->x[2], 2, 1) = odomSpeedLR;
    kf->isMeasurementUpdated = SET;
}

/// @brief 更新卡尔曼融合滤波器
/// @param imuAccelFB 陀螺仪顺向加速度
/// @param imuAccelLR 陀螺仪横向加速度
void FusionKF_Update(FusionKF_t* kf, float imuAccelFB, float imuAccelLR)
{
    //更新输入向量
    MAT_ELEM(kf->u, 1, 1) = imuAccelFB;
    MAT_ELEM(kf->u, 2, 1) = imuAccelLR;
    
    //数值迭代
    MAT_CPY(kf->x[0], kf->x[1]);
    MAT_CPY(kf->P[0], kf->P[1]);

    ////预测更新////
    //状态外插方程: x(n+1,n) = F * x(n,n) + G * u
    MAT_SET_SIZE(kf->temp[0], 2, 1);
    kf->MatStatus = MAT_MPY(kf->F, kf->x[1], &kf->temp[0]); //temp0 = F * x(n,n)
    MAT_SET_SIZE(kf->temp[1], 2, 1);
    kf->MatStatus = MAT_MPY(kf->G, kf->u, &kf->temp[1]);    //temp1 = G * u
    kf->MatStatus = MAT_ADD(kf->temp[0], kf->temp[1], &kf->x[1]); //暂时在x[1]上原地更新
    //协方差外插方程: P(n+1,n) = F * P(n,n) * FT + Q
    MAT_SET_SIZE(kf->temp[0], 2, 2);
    kf->MatStatus = MAT_MPY(kf->F, kf->P[1], &kf->temp[0]); //temp0 = F * P(n,n)
    MAT_SET_SIZE(kf->temp[1], 2, 2);
    kf->MatStatus = MAT_MPY(kf->temp[0], kf->FT, &kf->temp[1]); //temp1 = F * P(n,n) * FT
    kf->MatStatus = MAT_ADD(kf->temp[1], kf->Q, &kf->P[1]); //暂时在P[1]上原地更新

    if (!kf->isMeasurementUpdated)
    {
        MAT_CPY(kf->x[1], kf->x[0]);
        MAT_CPY(kf->P[1], kf->P[0]);
        return;
    }

    ////测量更新////
    //测量方程: z = H * x
    kf->MatStatus = MAT_MPY(kf->H, kf->x[2], &kf->z);
    //卡尔曼增益: K = P(n,n-1) * HT * (H * P(n,n-1) * HT + R)^-1
    MAT_SET_SIZE(kf->temp[0], 2, 2);
    kf->MatStatus = MAT_MPY(kf->H, kf->P[1], &kf->temp[0]); //temp0 = H * P(n,n-1)
    MAT_SET_SIZE(kf->temp[1], 2, 2);
    kf->MatStatus = MAT_MPY(kf->temp[0], kf->HT, &kf->temp[1]); //temp1 = H * P(n,n-1) * HT
    MAT_SET_SIZE(kf->temp[0], 2, 2);
    kf->MatStatus = MAT_ADD(kf->temp[1], kf->R, &kf->temp[0]); //temp0 = H * P(n,n-1) * HT + R
    MAT_SET_SIZE(kf->temp[1], 2, 2);
    kf->MatStatus = MAT_INV(kf->temp[0], &kf->temp[1]); //temp1 = (H * P(n,n-1) * HT + R)^-1
    MAT_SET_SIZE(kf->temp[0], 2, 2);
    kf->MatStatus = MAT_MPY(kf->P[1], kf->HT, &kf->temp[0]); //temp0 = P(n,n-1) * HT
    kf->MatStatus = MAT_MPY(kf->temp[0], kf->temp[1], &kf->K); //K = P(n,n-1) * HT * (H * P(n,n-1) * HT + R)^-1
    //状态更新方程: x(n,n) = x(n,n-1) + K * (z - H * x(n,n-1))
    MAT_SET_SIZE(kf->temp[0], 2, 1);
    kf->MatStatus = MAT_MPY(kf->H, kf->x[1], &kf->temp[0]); //temp0 = H * x(n,n-1)
    MAT_SET_SIZE(kf->temp[1], 2, 1);
    kf->MatStatus = MAT_SUB(kf->z, kf->temp[0], &kf->temp[1]); //temp1 = z - H * x(n,n-1)
    MAT_SET_SIZE(kf->temp[0], 2, 1);
    kf->MatStatus = MAT_MPY(kf->K, kf->temp[1], &kf->temp[0]); //temp0 = K * (z - H * x(n,n-1))
    kf->MatStatus = MAT_ADD(kf->x[1], kf->temp[0], &kf->x[0]); //x(n,n) = x(n,n-1) + K * (z - H * x(n,n-1))
    //协方差更新方程: P(n,n) = (I - K * H) * P(n,n-1) = P(n,n-1) - K * H * P(n,n-1)
    MAT_SET_SIZE(kf->temp[0], 2, 2);
    kf->MatStatus = MAT_MPY(kf->K, kf->H, &kf->temp[0]); //temp0 = K * H
    MAT_SET_SIZE(kf->temp[1], 2, 2);
    kf->MatStatus = MAT_MPY(kf->temp[0], kf->P[1], &kf->temp[1]); //temp1 = K * H * P(n,n-1)
    kf->MatStatus = MAT_SUB(kf->P[1], kf->temp[1], &kf->P[0]); //P(n,n) = P(n,n-1) - K * H * P(n,n-1)

    kf->isMeasurementUpdated = RESET;
}
