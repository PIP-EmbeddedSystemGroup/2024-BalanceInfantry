#ifndef _DM4310_H_
#define _DM4310_H_

/**
 * @attention   采用UTF-8字符集编码
 */

#include "UniversalMotor.h"

typedef Motor_t DM4310_t;

void DM4310_Init(DM4310_t* motor, int RxID, int TxID, float kP, float kD);
void DM4310_TryUpdate(int ID, uint8_t data[8]);
void DM4310_Update(DM4310_t* motor, uint8_t data[8]);
void DM4310_MitOutput(DM4310_t* motor);
void DM4310_Enable(DM4310_t* motor);
void DM4310_Disable(DM4310_t* motor);
void DM4310_SetZero(DM4310_t* motor);

#endif
