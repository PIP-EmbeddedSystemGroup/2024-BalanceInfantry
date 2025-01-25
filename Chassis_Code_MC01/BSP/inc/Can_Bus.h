#include "stdint.h"
#include "Balance_Task.h"
#include "Monitor_Task.h"

void LK9025_Data_Deal(LK9025_Motor_t *Receive,uint8_t Data[]);
void LK9025_Tcurrent_Output(uint32_t ID,int16_t Tcurrent);
void LK9025_Output_Normal(float output_left,float output_right);
void LK9025_Output_Zero(void);
void HMI_Communicate(Robot_Status_t* status);
void Gimbal_Communicate(Robot_Status_t* status);
void Gimbal_Data_Receive(void);
void CAN1_Receive(void);
void Cap_Output(uint16_t Power);
void CAN_Start(void);
void CAN1_Filter_Init(void);
void CAN2_Filter_Init(void);
