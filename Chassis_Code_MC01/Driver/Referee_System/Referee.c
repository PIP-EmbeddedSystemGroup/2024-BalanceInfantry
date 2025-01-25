#include "stdint.h"
#include "usart.h"

#include "Referee.h"
#include "Configuration.h"

#include "Communicate_Task.h"

InfantryJudge_t InfantryJudge={0};
FLAOT_UNION P_deal;//实时功率
FLAOT_UNION V_deal;//实时射速

FLAOT_UNION DataA;
FLAOT_UNION DataB;
FLAOT_UNION DataC;

uint8_t JudgeHeadBuffer[7]={0};
uint8_t JudgeCheckBuffer[80]={0};
uint8_t JudgeDataBuffer[80]={0};
uint8_t JudgeSendBuff[150]={0};

/**
  * @brief  裁判系统数据接收
	*         通过CRC校验和数据接收帧率对数据实时性进行判断
	*         有些数据没有进行接收，如果需要可以查看手册
  * @param  None
  * @retval None
  */
void Referee_Receive(void)
{
	uint8_t group_count=0;
	if((JudgeDataBuffer[0] == 0XA5) && (Verify_CRC8_Check_Sum(JudgeDataBuffer, 5) == 1))//检验帧头是否是0XA5，CRC校验
	{
		InfantryJudge.datasize = (JudgeDataBuffer[1])|(JudgeDataBuffer[2]<<8);//读取数据字节长度		
		HAL_UART_DMAStop(&huart6);
		HAL_UART_Receive_DMA(&huart6,JudgeDataBuffer,InfantryJudge.datasize+2);
		InfantryJudge.Referee_count[2] = JudgeDataBuffer[3];//数据发送次数，用来检测是否失真		
		InfantryJudge.CmdID = (JudgeDataBuffer[5])|(JudgeDataBuffer[6]<<8);//获取数据帧ID
		//根据接收次数检验，检验是否丢失
		if((InfantryJudge.Referee_count[2] - InfantryJudge.Referee_count[1] != 1) && (InfantryJudge.Referee_count[2] - InfantryJudge.Referee_count[1] != (-255)))
		{
			InfantryJudge.ErrorFlag = ERROR;
			InfantryJudge.ErrorCount++;
		}
		InfantryJudge.Referee_count[1] = InfantryJudge.Referee_count[2];//数据迭代
		InfantryJudge.DMA_flag = 0;//DMA标志位
		for(group_count = 0;group_count < 7;group_count++)//因为数据拆分接收，CRC校验需要完整数据，在此进行前半部分数据重组
			{JudgeCheckBuffer[group_count] = JudgeDataBuffer[group_count];}
	}
	else if(InfantryJudge.DMA_flag == 0)
	{
		InfantryJudge.DMA_flag = 1;
		HAL_UART_DMAStop(&huart6);
		HAL_UART_Receive_DMA(&huart6,JudgeDataBuffer,7);
		for(group_count = 0;group_count < InfantryJudge.datasize + 2;group_count++)//后半部分数据重组
			JudgeCheckBuffer[7 + group_count] = JudgeDataBuffer[group_count];
		if(Verify_CRC16_Check_Sum(JudgeCheckBuffer, InfantryJudge.datasize + 9) == 1)//校验CRC
		{
			RefereeCount++;
			switch(InfantryJudge.CmdID)
			{
				case 0x0001://比赛状态数据 3bytes
                {
                    if(JudgeDataBuffer[0] >= 64&&JudgeDataBuffer[0]<80)
                        Command.GameStartFlag = YES;
                    else if(JudgeDataBuffer[0] >= 84)
                        Command.GameStartFlag = NO;
                    
                    break;
                }
					
				case 0x02://比赛结果数据
					break;
				case 0x03://比赛机器人存活数据
					break;
				case 0x11://场地事件数据
					break;
				case 0x12://补给站标识动作
					break;
				case 0x13://预约子弹数据
					break;
				case 0x14://裁判警告信息
					break;
				case 0x0201://机器人状态数据 10Hz 15bytes  
				{
					if(JudgeDataBuffer[0]<100)
                        Command.RivalColor = 0xFF;
                    else
                        Command.RivalColor = 0xAA;
                    
					InfantryJudge.RemainBlood       = (JudgeDataBuffer[2]) | (JudgeDataBuffer[3] << 8);
					InfantryJudge.Shoot17mm1CoolingRate  = (JudgeDataBuffer[6])  | (JudgeDataBuffer[7] << 8);
					InfantryJudge.Shoot17mm1CoolingLimit = (JudgeDataBuffer[8])  | (JudgeDataBuffer[9] << 8);
					InfantryJudge.Shoot17mm1SpeedLimit = (JudgeDataBuffer[10])  | (JudgeDataBuffer[11] << 8);
					InfantryJudge.Shoot17mm2CoolingRate  = (JudgeDataBuffer[12]) | (JudgeDataBuffer[13] << 8);
					InfantryJudge.Shoot17mm2CoolingLimit = (JudgeDataBuffer[14]) | (JudgeDataBuffer[15] << 8);
					InfantryJudge.ChassisPowerLimit = (JudgeDataBuffer[24])  | (JudgeDataBuffer[25] << 8);//底盘功率限制
					break;
				}
				case 0x0202://实时功率热量数据 50Hz 14bytes
				{
					InfantryJudge.RealVoltage    = ((float)((JudgeDataBuffer[0]) | (JudgeDataBuffer[1] << 8)) / 1000);
					InfantryJudge.RealCurrent    = ((float)((JudgeDataBuffer[2]) | (JudgeDataBuffer[3] << 8)) / 1000);
					InfantryJudge.RemainPower    =          (JudgeDataBuffer[8]) | (JudgeDataBuffer[9] << 8);
					InfantryJudge.Real17mm1Heating   =          (JudgeDataBuffer[10])| (JudgeDataBuffer[11]<< 8);
                    InfantryJudge.Real17mm2Heating   =          (JudgeDataBuffer[12])| (JudgeDataBuffer[13]<< 8);
					P_deal.float_byte.low_byte   = JudgeDataBuffer[4];
					P_deal.float_byte.mlow_byte  = JudgeDataBuffer[5];
					P_deal.float_byte.mhigh_byte = JudgeDataBuffer[6];
					P_deal.float_byte.high_byte  = JudgeDataBuffer[7];
					InfantryJudge.RealPower      = P_deal.value;
					break;
				}
				case 0x0203://实时位置数据 10Hz 16bytes
					break;
				case 0x24://增益数据 1byte
					break;
				case 0x25://空中机器人能量数据 10Hz 3bytes
					break;
//				case 0x0206://伤害状态数据 1byte
//				{
//          if(Command.RotateFlag == 0)
//					{
//                        InfantryJudge.Injure = YES;
//						if(JudgeDataBuffer[0] == 0x00)
//							{
//		            
//								InfantryJudge.InjureMark[0] =20;
//							}
//						else if(JudgeDataBuffer[0]  == 0x01)
//							{

//								InfantryJudge.InjureMark[1] = YES;
//								GM6020.Rotate = 2048;
//							}
//						else if(JudgeDataBuffer[0] == 0x02)
//							{

//								InfantryJudge.InjureMark[2] = YES;
//								GM6020.Rotate = 4096;
//							}
//						else if(JudgeDataBuffer[0] == 0x03)
//							{

//								InfantryJudge.InjureMark[3] = YES;
//								GM6020.Rotate = - 2048;
//							}
//					}
//					break;
//        }        				
				case 0x0207://实时射击数据 6bytes
				{
					InfantryJudge.ShootFrequency = JudgeDataBuffer[3];
					V_deal.float_byte.low_byte   = JudgeDataBuffer[4];
					V_deal.float_byte.mlow_byte  = JudgeDataBuffer[5];
					V_deal.float_byte.mhigh_byte = JudgeDataBuffer[6];
					V_deal.float_byte.high_byte  = JudgeDataBuffer[7];
					InfantryJudge.ShootSpeed     = V_deal.value;
					break;
				}
                case 0x0208://子弹剩余发送数量
                {
                    InfantryJudge.Bullet17RemainingNum = (JudgeDataBuffer[0]) | (JudgeDataBuffer[1] << 8);
                    break;
                }
				case 0x31://实时交互数据 10Hz n-byte		
					break;
				default:
					break;
			}
		}
		else//如果CRC没过，记为数据失真
		{
			InfantryJudge.ErrorFlag = ERROR;
			InfantryJudge.ErrorCount++;
		}
	}
	else
	{
		HAL_UART_DMAStop(&huart6);
		HAL_UART_Receive_DMA(&huart6,JudgeDataBuffer,7);
	}
}