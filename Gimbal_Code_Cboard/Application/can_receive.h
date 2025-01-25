#include "universal.h"

typedef struct Robot_Status_Structure
{
	struct
	{
		uint8_t INS_Init_Status;
		uint8_t INS_Fail_Status;
	}INS;
    struct 
    {
        uint8_t Leg_Init_Status;
        uint8_t Leg_Height_Status;
    }Leg;
    struct
    {
        uint8_t Body_Upright_Status;
        uint8_t Body_Soar_Status;
    }Body;
	struct 
	{
		uint8_t Gimbal_Reposition_Status;
        int16_t YAW_POSITION_REAL;

	}Gimbal;
	uint8_t Param_Switch;//闂佸憡鐟ラ崐褰掑汲閻旂厧绀嗛柛銉ｅ妼鎼村﹪鏌″鍛悙缂佺粯姊归幏鍛存晸閿燂拷
	
}Robot_Status_t;

void Ammo_Output(s16 gm1_iq, s16 gm2_iq, s16 gm3_iq);
void Gimbal_Output(s16 gm4_iq, s16 gm2_iq);
void CAN1_Receive(void);
void CAN2_Receive(void);
void Motor_Data_Deal(Motor_t *Receive, u8 Data[]);
void Chassis_Data_Deal(Robot_Status_t* status, u8 Data[]);
void GM2006_Position_Deal(Motor_t *Receive);
void Cap_Output(uint16_t temPower);

extern float PowerData[4];
extern u8 convertflag[2];
