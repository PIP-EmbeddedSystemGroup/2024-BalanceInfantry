#include "stdint.h"

typedef struct//A1电机数据结构体
{
	uint8_t id;            //
	uint8_t mode;          //
	//K_P*delta_Pos + K_W*delta_W + T
	struct
	{
		int16_t Origin;
		float Real;
	}T;          //
	float W;          //
	float K_P;        //
	float K_W;  
	struct
	{
		int32_t Origin;
		float Real;
		float Realf;
		float Step;
		float Set;
	}Position;
	
	struct
	{
		int16_t Origin;
		float real;
	}Speed;
}A1_Motor_t;

typedef struct {
	// ?? ????
    unsigned char  start[2];
	unsigned char  motorID;
	unsigned char  reserved;
}COMHead;

typedef union{
        int32_t           L;
        uint8_t       uint8_t[4];
       uint16_t      u16[2];
       uint32_t         u32;
          float           F;
}COMData32;

typedef struct {  // ? 4??????? ,????????
	// ?? ??
    uint8_t  mode;        // ??????
    uint8_t  ModifyBit;   // ?????????
    uint8_t  ReadBit;     // ?????????
    uint8_t  reserved;

    COMData32  Modify;     // ?????? ??? 
    //???FOC??????:
    //K_P*delta_Pos + K_W*delta_W + T
    int16_t     T;      // ?????????(???????)x256, 7 + 8 ??
    int16_t     W;      // ?????? (???????) x128,       8 + 7??	
    int32_t   Pos;      // ?????? x 16384/6.2832, 14????(??0???,??????????0???)

    int16_t    K_P;      // ?????? x2048  4+11 ??
    int16_t    K_W;      // ?????? x1024  5+10 ??

    uint8_t LowHzMotorCmdIndex;     // ????????????, 0-7, ????LowHzMotorCmd??8???
    uint8_t LowHzMotorCmdByte;      // ????????????
	
    COMData32  Res[1];    // ?? ????  ????????????
	
}MasterComdV3;   // ???????? ?CRC 34??

typedef struct {	
    COMHead head;    
    MasterComdV3 Mdata;
    COMData32 CRCdata;
}MasterComdDataV3;

typedef struct {
	//
    MasterComdDataV3  motor_send_data;  //???????????,??motor_msg.h

	  int hex_len ;                  //???16????????, 34
    // long long send_time;        //??(us)
    unsigned short id;             //??ID,0xBB??????
    unsigned short mode;           //0:??, 5:????, 10:??FOC??
    //FOC:
    //K_P*delta_Pos + K_W*delta_W + T
    float T;                       //(???????)(Nm)
    float W;                       //(???????)(rad/s)
    float Pos;                     //(rad)
    float K_P;                     //
    float K_W;                     //
    COMData32 Res;                 // 
}A1_CMD_Tx_t;

extern A1_Motor_t A1_Motor[4];
extern uint8_t A1_Transmit_Data_Left[34];//宇树A1电机数据发送缓冲区
extern uint8_t A1_Transmit_Data_Right[34];//宇树A1电机数据发送缓冲区
extern A1_Motor_t A1_Motor[4];
extern A1_CMD_Tx_t A1_Control[4];

void A1_Data_Rx_Left(void);
void A1_Data_Rx_Right(void);

void A1_Modify_Data(A1_CMD_Tx_t* motor_s);
void A1_Transmit_Data_Deal_Left(A1_CMD_Tx_t *motor_s);
void A1_Transmit_Data_Deal_Right(A1_CMD_Tx_t *motor_s);