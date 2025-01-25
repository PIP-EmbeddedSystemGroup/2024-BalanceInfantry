/**
  ******************************************************************************
  * @file    Header.h
  * @author  北京工业大学-贾桐
  * @version V1.0
  * @date    2019/3/1
  * @brief   头文件存储
  ******************************************************************************
  * @attention
  *
  *
  ******************************************************************************
  */

/* Includes --系统--------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "stdint.h"
#include "main.h"
#include "math.h"
#include "FREERtos.h"
#include "cmsis_os.h"

/* Includes --硬件--------------------------------------------------------------*/
#include "gpio.h"
#include "can.h"
#include "usart.h"
#include "universal.h"
#include "bsp_can.h"
#include "bsp_PWM.h"
#include "bsp_dwt.h"
#include "bsp_uart.h"
#include "BMI088driver.h"
#include "controller.h"
#include "QuaternionEKF.h"

/* Includes --进程--------------------------------------------------------------*/
#include "Config.h"
#include "start_task.h"
#include "detect_task.h"
#include "gimbal_task.h"
#include "remote_task.h"
#include "shoot_task.h"
#include "com_task.h"
#include "pid.h"
#include "adrc.h"
#include "can_receive.h"
#include "ins_task.h"


//一些用法的宏定义
#define OFF   0
#define ON    1
#define RC    0
#define PC    1
#define OK    1
#define NO    0
#define READY    2
#define SHOOTING 1
#define WAITING  0
#define ERROR  1
#define NORMAL 0
#define INCREMENTAL 0
#define POSITIONAL  1
#define YAW 0
#define PITCH 1 
#define TRIGGER 2
#define FRI_LEFT 0
#define FRI_RIGHT 1
