/**
 * @attention   采用UTF-8字符集编码
 * @brief       惯性导航任务
 * @authors     北工大PIP战队 樊郡捷
 * @version     v1.0
 * @date        2024-10-14
 * @details     实现了陀螺仪温度闭环控制
 *              实现了陀螺仪校准
 *              实现了陀螺仪校准数据在Flash中的保存和读取(通过修改header.h中的校准时间戳来触发校准)
 *              目前只实现了EKF扩展卡尔曼姿态解算(玺佬的开源)
 * @cite        韭菜的菜(王洪玺) - 四元数&向量外积补偿IMU姿态解算: https://bbs.robomaster.com/article/7419
 * @todo        C板上有磁力计 但是咱们一直没用过 玺佬后来的这篇开源里有9轴融合算法: https://bbs.robomaster.com/wiki/4574/7420
 */

#include "header.h"

#define INERTIAL_TASK_UPDATE_TICK 2

void BodyFrameToEarthFrame(const float *vecBF, float *vecEF, float *q);
void EarthFrameToBodyFrame(const float *vecEF, float *vecBF, float *q);

typedef struct
{
    uint32_t TimeStamp;    // 校准时的时间戳
    uint32_t TempWhenCali; // 校准时的温度 4字节对齐
    union
    {
        uint32_t Raw;
        float Value;
    } GyroOffset[3];
    union
    {
        uint32_t Raw;
        float Value;
    } AccelScale; // 和9.81标准重力加速度相差的系数
} IMU_CaliData_t;

Inertial_t Inertial = {0};

const IMU_CaliData_t *const IMU_FlashCaliData = (void *)IMU_CALI_DATA_FLASH_ADDR;

void InertialCalibrateTask(void const *argument)
{
    float TempErrorFadingFilter = 10;
    const float FadingFactor = 0.9f;

    Inertial.IMU->TempWhenCali = IMU_HEATER_TEMP_SET; // 作为温度PID的设置值
    osDelay(4000);                                    // 先等待温度基本稳定

    while (1)
    {
        osDelay(100);

        // 计算一段时间内误差非线性累计值
        TempErrorFadingFilter *= FadingFactor;
        TempErrorFadingFilter += fabsf(Inertial.ImuHeaterPID.Error[0]) * (1 - FadingFactor);

        if (TempErrorFadingFilter < 0.1f)
            break;
    }

    while (BMI088_init(&hspi1, 1) != BMI088_NO_ERROR)
        ; // 重新初始化 并执行校准

    Inertial.IMU->TempWhenCali = IMU_HEATER_TEMP_SET; // 作为温度PID的设置值

    IMU_CaliData_t IMU_CaliData;
    IMU_CaliData.TimeStamp = IMU_CALI_TIMESTAMP;

    IMU_CaliData.TempWhenCali = IMU_HEATER_TEMP_SET;
    for (int i = 0; i < 3; i++)
        IMU_CaliData.GyroOffset[i].Value = Inertial.IMU->GyroOffset[i];
    IMU_CaliData.AccelScale.Value = Inertial.IMU->AccelScale;

    HAL_FLASH_Unlock();
    FLASH_WaitForLastOperation(3000);

    FLASH_EraseInitTypeDef FlashEraseInit = {0};
    FlashEraseInit.Banks = FLASH_BANK_1;
    FlashEraseInit.Sector = FLASH_SECTOR_11; // 1MB Flash空间中的最后128kB 见F4参考手册P75
    FlashEraseInit.TypeErase = FLASH_TYPEERASE_SECTORS;
    FlashEraseInit.VoltageRange = FLASH_VOLTAGE_RANGE_3;
    FlashEraseInit.NbSectors = 1;
    uint32_t SectorError;
    HAL_FLASHEx_Erase(&FlashEraseInit, &SectorError); // Flash写入操作只能把1变为0 因此需要先擦除(全部置1)
    FLASH_WaitForLastOperation(3000);

    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, IMU_CALI_DATA_FLASH_ADDR, IMU_CaliData.TimeStamp);
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, IMU_CALI_DATA_FLASH_ADDR + 4, IMU_CaliData.TempWhenCali);
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, IMU_CALI_DATA_FLASH_ADDR + 8, IMU_CaliData.GyroOffset[0].Raw);
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, IMU_CALI_DATA_FLASH_ADDR + 12, IMU_CaliData.GyroOffset[1].Raw);
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, IMU_CALI_DATA_FLASH_ADDR + 16, IMU_CaliData.GyroOffset[2].Raw);
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, IMU_CALI_DATA_FLASH_ADDR + 20, IMU_CaliData.AccelScale.Raw);
    FLASH_WaitForLastOperation(3000);

    HAL_FLASH_Lock();
    FLASH_WaitForLastOperation(3000);

    CLEAR_BIT(Daemon.ErrorFlag, ERR_GIMBAL_INERTIAL_INIT);

    osThreadTerminate(osThreadGetId());
}

/// @brief 初始化惯导的IMU部分 尝试从Flash中读取校准参数 若出错则重新校准并保存到Flash
/// @param
void Inertial_ImuInit(void)
{
    SET_BIT(Daemon.ErrorFlag, ERR_GIMBAL_INERTIAL_INIT);

    Inertial.IMU = &BMI088;
    while (BMI088_init(&hspi1, 0) != BMI088_NO_ERROR)
        ; // 即使需要校准 也需要先初始化芯片才能读取芯片温度

    if (IMU_FlashCaliData->TimeStamp == IMU_CALI_TIMESTAMP)
    {
        Inertial.IMU->TempWhenCali = Constrain(IMU_FlashCaliData->TempWhenCali, 70, 40);
        for (int i = 0; i < 3; i++)
            Inertial.IMU->GyroOffset[i] = IMU_FlashCaliData->GyroOffset[i].Value;
        Inertial.IMU->AccelScale = IMU_FlashCaliData->AccelScale.Value;

        CLEAR_BIT(Daemon.ErrorFlag, ERR_GIMBAL_INERTIAL_INIT);
    }
    else
    {
        osThreadDef(InertialCalibrate, InertialCalibrateTask, osPriorityAboveNormal, 0, 256);
        osThreadCreate(osThread(InertialCalibrate), NULL);

        SET_BIT(Daemon.ErrorFlag, ERR_GIMBAL_INERTIAL_INIT);
    }
}
float gravity_b[3];

void InertialTask(void const *argument)
{
    uint32_t PreviousWakeTime = osKernelSysTick();

    const float Gravity[3] = {0, 0, 9.81f};
    const float xb[3] = {1, 0, 0};
    const float yb[3] = {0, 1, 0};
    const float zb[3] = {0, 0, 1};

    PID_InitStruct_t PID_InitParam = {0};
    PID_InitParam.kP = 500;
    PID_InitParam.kI = 40;
    PID_InitParam.kD = 0;
    PID_InitParam.DeltaTime = INERTIAL_TASK_UPDATE_TICK / 1000.f;
    PID_InitParam.MaxError = 50;
    PID_InitParam.MaxOutput = 1500;
    PID_InitParam.I_Max = 1000;
    PID_InitParam.DeadBand = 0;
    PID_InitParam.IntegralBand = 8;
    PID_Init(&Inertial.ImuHeaterPID, &PID_InitParam);
    HAL_TIM_PWM_Start(&htim10, TIM_CHANNEL_1);

    Inertial_ImuInit();

    for (int i = 0; i < 3; i++)
    {
        LPF_1stOrderIIR_Init(&Inertial.AccelLPF[i], INERTIAL_TASK_UPDATE_TICK / 1000.f, 20);
        LPF_1stOrderIIR_Init(&Inertial.GyroLPF[i], INERTIAL_TASK_UPDATE_TICK / 1000.f, 10);
        LPF_1stOrderIIR_Init(&Inertial.dGyroLPF[i], INERTIAL_TASK_UPDATE_TICK / 1000.f, 50);
    }

    IMU_QuaternionEKF_Init(10, 0.001f, 1000000, 0.9996, 0);

    while (1)
    {
        osDelayUntil(&PreviousWakeTime, INERTIAL_TASK_UPDATE_TICK);

        BMI088_Read(Inertial.IMU);

        PID_Calc(&Inertial.ImuHeaterPID, Inertial.IMU->Temperature, Inertial.IMU->TempWhenCali);
        __HAL_TIM_SetCompare(&htim10, TIM_CHANNEL_1, Constrain(Inertial.ImuHeaterPID.Output, 2000, 0));

        if (READ_BIT(Daemon.ErrorFlag, ERR_GIMBAL_INERTIAL_INIT)) // 陀螺仪校准好之前只计算温度闭环PID
            continue;

        float DeltaTime = INERTIAL_TASK_UPDATE_TICK / 1000.f;

        Inertial.Accel[AX] = Inertial.IMU->Accel[AX];
        Inertial.Accel[AY] = Inertial.IMU->Accel[AY];
        Inertial.Accel[AZ] = Inertial.IMU->Accel[AZ];
        Inertial.Gyro[AX] = Inertial.IMU->Gyro[AX];
        Inertial.Gyro[AY] = Inertial.IMU->Gyro[AY];
        Inertial.Gyro[AZ] = Inertial.IMU->Gyro[AZ];

        // demo function,用于修正安装误差,可以不管,本demo暂时没用
        // IMU_Param_Correction(&Inertial_Param, Inertial.Gyro, Inertial.Accel);

        // 计算重力加速度矢量和b系的XY两轴的夹角,可用作功能扩展,本demo暂时没用
        // Inertial.atanxz = -atan2f(Inertial.Accel[AX], Inertial.Accel[AZ]) * 180 / PI;
        // Inertial.atanyz = atan2f(Inertial.Accel[AY], Inertial.Accel[AZ]) * 180 / PI;

        // 核心函数,EKF更新四元数
        IMU_QuaternionEKF_Update(Inertial.Gyro[AX], Inertial.Gyro[AY], Inertial.Gyro[AZ], Inertial.Accel[AX], Inertial.Accel[AY], Inertial.Accel[AZ], DeltaTime);

        memcpy(Inertial.Q, QEKF_INERTIAL.q, sizeof(QEKF_INERTIAL.q));

        // 机体系基向量转换到导航坐标系，本例选取惯性系为导航系
        BodyFrameToEarthFrame(xb, Inertial.xn, Inertial.Q);
        BodyFrameToEarthFrame(yb, Inertial.yn, Inertial.Q);
        BodyFrameToEarthFrame(zb, Inertial.zn, Inertial.Q);

        // 将重力从导航坐标系n转换到机体系b,随后根据加速度计数据计算运动加速度
        EarthFrameToBodyFrame(Gravity, gravity_b, Inertial.Q);
        for (uint8_t i = 0; i < 3; i++) // 同样过一个低通滤波
        {
            LPF_1stOrderIIR_Update(&Inertial.AccelLPF[i], Inertial.Accel[i] - gravity_b[i]);
            Inertial.MotionAccel_b[i] = Inertial.AccelLPF[i].Output;
        }
        BodyFrameToEarthFrame(Inertial.MotionAccel_b, Inertial.MotionAccel_n, Inertial.Q); // 转换回导航系n

        for (int i = 0; i < 3; i++)
        {
            float GypoLpfPrev = Inertial.GyroLPF[i].Output;
            LPF_1stOrderIIR_Update(&Inertial.GyroLPF[i], Inertial.Gyro[i]);
            Inertial.dGyro[i] = (Inertial.GyroLPF[i].Output - GypoLpfPrev) / DeltaTime;
            LPF_1stOrderIIR_Update(&Inertial.dGyroLPF[i], Inertial.dGyro[i]);
        }

        // 获取最终数据
        Inertial.Yaw = QEKF_INERTIAL.Yaw;
        Inertial.Pitch = QEKF_INERTIAL.Pitch;
        Inertial.Roll = QEKF_INERTIAL.Roll;
        Inertial.YawTotalAngle = QEKF_INERTIAL.YawTotalAngle;
    }
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
