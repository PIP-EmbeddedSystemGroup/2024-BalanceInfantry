/**
 * @attention   采用UTF-8字符集编码
 * @brief       融合惯导任务
 * @authors     北工大PIP战队 樊郡捷
 * @version     v1.0
 * @date        2024-10-26
 * @details     融合加速度计和里程计在水平面进行惯性导航
 */

#include "header.h"

#define FUSION_INS_TASK_UPDATE_TICK 10

FusionINS_t FusionINS;

void FusionInsTask(void const* argument)
{
    uint32_t PreviousWakeTime = osKernelSysTick();

    FusionKF_Init(&FusionINS.KF, FUSION_INS_TASK_UPDATE_TICK / 1000.f, 0.00075f, 0.0001f);

    while (1)
    {
        osDelayUntil(&PreviousWakeTime, FUSION_INS_TASK_UPDATE_TICK);

        float PitchSin, PitchCos;
        arm_sin_cos_f32(Inertial.Pitch, &PitchSin, &PitchCos);
        float RollSin, RollCos;
        arm_sin_cos_f32(Inertial.Roll, &RollSin, &RollCos);
        float YawSin, YawCos;
        arm_sin_cos_f32(Inertial.Yaw, &YawSin, &YawCos);

        float AccelFB = 0;
        if (fabsf(Inertial.Gyro[AY]) < 0.2f)    //高速转动时由于EKF相位滞后 重力会耦合进机体系加速度 因此高速转动时丢弃加速度
        {
            //运动向心力修正
            AccelFB += 0.7f * Inertial.GyroLPF[AZ].Output * Controller.Move.LR.Real;
            //安装位置造成的向心力修正
            AccelFB += powf(Inertial.GyroLPF[AZ].Output, 2) * GIMBAL_BOARD_OFFSET_X;
            //水平加速度
            AccelFB += (Inertial.MotionAccel_b[AY] + Inertial.dGyroLPF[AX].Output * GIMBAL_BOARD_OFFSET_Z) * PitchCos - (Inertial.MotionAccel_b[AZ] - Inertial.dGyroLPF[AX].Output * GIMBAL_BOARD_OFFSET_X) * PitchSin;
        }
        float AccelLR = 0;
        if (fabsf(Inertial.Gyro[AX]) < 0.2f)
        {
            AccelLR += -Inertial.GyroLPF[AZ].Output * Controller.Move.FB.Real;
            AccelLR += powf(Inertial.GyroLPF[AZ].Output, 2) * GIMBAL_BOARD_OFFSET_Y;
            AccelLR += -((Inertial.MotionAccel_b[AX] + Inertial.dGyroLPF[AZ].Output * GIMBAL_BOARD_OFFSET_X) * RollCos + Inertial.MotionAccel_b[AZ] * RollSin);
        }

        FusionKF_Update(&FusionINS.KF, AccelFB, AccelLR);

        FusionINS.Robot.Speed.FB = 0;//MAT_ELEM(FusionINS.KF.x[0], 1, 1);
        FusionINS.Robot.Speed.LR = 0;//MAT_ELEM(FusionINS.KF.x[0], 2, 1);

        FusionINS.World.Speed.X = FusionINS.Robot.Speed.FB * YawCos + FusionINS.Robot.Speed.LR * -YawSin;
        FusionINS.World.Speed.Y = FusionINS.Robot.Speed.FB * YawSin + FusionINS.Robot.Speed.LR * YawCos;

        if (fabsf(FusionINS.World.Speed.X) > 0.025f)
            FusionINS.World.Position.X += FusionINS.World.Speed.X * (FUSION_INS_TASK_UPDATE_TICK / 1000.f);
        if (fabsf(FusionINS.World.Speed.Y) > 0.025f)
            FusionINS.World.Position.Y += FusionINS.World.Speed.Y * (FUSION_INS_TASK_UPDATE_TICK / 1000.f);

        ChassisBoard_SendFusionInsSpeed();
    }
}
