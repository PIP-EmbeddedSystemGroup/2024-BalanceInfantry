/*
 * @Author: 静_火 lintianlang0918@qq.com
 * @Date: 2024-03-26 18:11:12
 * @LastEditors: 静_火 lintianlang0918@qq.com
 * @LastEditTime: 2024-04-03 22:42:24
 * @FilePath: \MC-01_chassis_copy\Task\inc\Configuration.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
//标志位宏定义
#define OFF   0
#define ON    1
#define RC    0
#define PC    1
#define OK    1
#define YES   1
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
#define BIG 0
#define SMALL 1
#define LEFT 0
#define RIGHT 1
#define LEG_LOW 0
#define LEG_MID 1
#define LEG_HIGH 2



//圆周率宏定义
#define NORMAL_PI 3.14159265f
#define NORMAL_HALF_PI 1.5708f

//CAN总线ID宏定义
#define LEFT_WHEEL_ID 0x141
#define RIGHT_WHEEL_ID 0x142

//并联腿电机初始位置宏定义
#define A1_MOTOR0_POSITION_INIT -168
#define A1_MOTOR1_POSITION_INIT 226
#define A1_MOTOR2_POSITION_INIT 205
#define A1_MOTOR3_POSITION_INIT 467

//并联腿参数设置
#define LOW 1
#define MID 2
#define HIGH 3
#define L0_LOW  150.0f
#define L0_MID  190.0f
#define L0_HIGH 270.0f
#define THETA0_MID  1.5708f

//腿部连杆的长度参数
#define l5 110 //mm
#define l5_Half 55
#define l1 140
#define l2 240
#define l3 240
#define l4 140
#define WHEEL_RADIUS 0.1 //m
//云台参数设置
#define YAW_POSITION_INIT   4790

//里程计一阶卡尔曼滤波器参数
#define VEL_PROCESS_NOISE 100	//速度过程噪声
#define VEL_MEASURE_NOISE 2500	//速度测量噪声

//底盘IMU安装位置
#define CENTER_IMU_L 0.05743//机体前后向偏差
#define CENTER_IMU_W 0.03782//机体左右向偏差
#define CENTER_IMU_H 0.11416//机体竖直向偏差