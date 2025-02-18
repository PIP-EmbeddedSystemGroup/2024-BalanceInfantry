#ifndef _DJI_MOTOR_H_
#define _DJI_MOTOR_H_

/**
 * @attention   采用UTF-8字符集编码
 */

#include "UniversalMotor.h"

typedef Motor_t DjiMotor_t;
typedef DjiMotor_t M3508_t;
typedef DjiMotor_t M2006_t;
typedef DjiMotor_t GM6020_t;

void M3508_Init(M3508_t* motor, int ID);
void M3508P14_Init(M3508_t* motor, int ID);
void M2006_Init(M2006_t* motor, int ID);
void GM6020_Init(GM6020_t* motor, int ID);
void DjiMotor_Update(DjiMotor_t* motor, uint8_t data[8]);
void DjiMotor_CanID_0x1FF_Output(void);
void DjiMotor_CanID_0x200_Output(void);
void DjiMotor_CanID_0x2FF_Output(void);

extern Motor_t* DjiMotor_CanID_0x1FF_List[4];
extern Motor_t* DjiMotor_CanID_0x200_List[4];
extern Motor_t* DjiMotor_CanID_0x2FF_List[3];

#endif
