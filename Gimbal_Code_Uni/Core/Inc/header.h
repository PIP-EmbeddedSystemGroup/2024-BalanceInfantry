/**
 * @attention   采用UTF-8字符集编码
 * @brief       UniversalRobot的头文件集合
 * @authors     北工大PIP战队 樊郡捷
 * @version     v1.0
 * @date        2024-10-14
 * @details     适合在每个.c文件开头调用 注意本文件没有ifndef 要防止递归调用(不要在任何头文件中包含此文件)
 *              虽然降低编译效率 但可以避免手动解耦调用的麻烦（
 */

//系统
#include "stm32f4xx_hal.h"
#include "stdint.h"
#include "math.h"
#include "cmsis_os.h"
#include "arm_math.h"

//宏
#include "RobotConfig.h"
#ifndef PI
#define PI 3.14159265358979f
#endif
#ifndef SQRT2
#define SQRT2 1.414213562373095f
#endif
#ifndef SQRT3
#define SQRT3 1.732050807568877f
#endif
#ifndef FLOAT_MAX_VAL
#define FLOAT_MAX_VAL 3.4028234663852886e+38f
#endif

#define AXIS_YAW 0
#define AXIS_PITCH 1

#define IMU_CALI_DATA_FLASH_ADDR (0x08000000 + (1024 * 1024) - 2048)    //1M Flash空间中的最后一页
#define IMU_CALI_TIMESTAMP 2410212126u  //将此数改为另一个值即可触发一次陀螺仪校准 此处可以将其设置为时间戳 注意该值要满足32位宽度
#define IMU_HEATER_TEMP_SET 50  //摄氏度 会优先使用从Flash中读取到的数值 仅修改此处无效 该值在校准后被写入Flash

//外部硬件
#include "RC_DBUS.h"
#include "RC_VTM.h"
#include "Chassis.h"
#include "BMI088driver.h"
#include "UniversalMotor.h"
#include "DjiMotor.h"
#include "DM4310.h"
#include "NUC.h"
#include "Misc.h"
#include "Referee.h"

//外设
#include "usart.h"
#include "can.h"
#include "tim.h"
#include "spi.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"

//软件
#include "Universal.h"
#include "UniversalPID.h"
#include "UniversalFilter.h"
#include "UniversalCRC.h"
#include "MouseKeyboard.h"
#include "QuaternionEKF.h"
#include "FusionKF.h"

//任务
#include "DaemonTask.h"
#include "ControllerTask.h"
#include "InertialTask.h"
#include "DebugTask.h"
#include "BoardCommTask.h"
#include "GimbalTask.h"
#include "ShootingTask.h"
#include "VisionTask.h"
#include "FusionInsTask.h"
