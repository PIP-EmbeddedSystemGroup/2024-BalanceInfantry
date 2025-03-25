/**
 * @attention   采用UTF-8字符集编码
 * @brief       裁判系统读取
 * @authors     北工大PIP战队 李君豪 孙绍博 冯星侨
 * @version     v1.0
 * @date        2025-02-12
 * @details
 */

#include "header.h"

uint8_t RefereeRxBuf[192];
Referee_t Referee = { 0 };

void Referee_RxCallback_Parsing(uint8_t* rxBuf, int frameLen)
{
    // if (frameLen < 5)
    // {
    //     Referee.ErrorCount++;
    //     return;
    // }

    int dataLength = (rxBuf[1]) | (rxBuf[2] << 8);

    // if (frameLen < (dataLength + 9))
    // {
    //     Referee.ErrorCount++;
    //     return;
    // }

    if (rxBuf[0] != 0xA5 ||
        !CRC8_VerifyChecksum(rxBuf, 5) ||
        !CRC16_VerifyChecksum(rxBuf, 5 + 2 + dataLength + 2))
    {
        Referee.ErrorCount++;
        return;
    }
    Referee.PackageCount++;
    Referee.LastOnlineTick = HAL_GetTick();

    int cmdId = (rxBuf[5]) | (rxBuf[6] << 8);
    const uint8_t* pData = &rxBuf[7];

    switch (cmdId)
    {
    case 0x0001:
        memcpy(&Referee.GameStatus, pData, 11);
        break;
    case 0x0002:
        memcpy(&Referee.GameResult, pData, 1);
        break;
    case 0x0003:
        memcpy(&Referee.GameRobotHP, pData, 32);
        break;
    case 0x0101:
        memcpy(&Referee.EventData, pData, 4);
        break;
    case 0x0102:
        memcpy(&Referee.ExtSupplyProjectileAction, pData, 4);
        break;
    case 0x0104:
        memcpy(&Referee.RefereeWarning, pData, 3);
        break;
    case 0x0105:
        memcpy(&Referee.DartInfo, pData, 3);
        break;
    case 0x0201:
        memcpy(&Referee.RobotStatus, pData, 13);
        break;
    case 0x0202:
        memcpy(&Referee.PowerHeatData, pData, 16);
        break;
    case 0x0203:
        memcpy(&Referee.RobotPos, pData, 12);
        break;
    case 0x0204:
        memcpy(&Referee.Buff, pData, 6);
        break;
    case 0x0205:
        memcpy(&Referee.AirSupportData, pData, 2);
        break;
    case 0x0206:
        memcpy(&Referee.HurtData, pData, 1);
        if (Referee.HurtData.HPDeductionReason == 0)
        {
            switch (Referee.HurtData.ArmorId)
            {
            case 0:
                Referee.LastHurtTick.Front = HAL_GetTick();
                break;
            case 1:
                Referee.LastHurtTick.Left = HAL_GetTick();
                break;
            case 2:
                Referee.LastHurtTick.Back = HAL_GetTick();
                break;
            case 3:
                Referee.LastHurtTick.Right = HAL_GetTick();
                break;
            default:
                break;
            }
        }
        break;
    case 0x0207:
        memcpy(&Referee.ShootData, pData, 7);
        break;
    case 0x0208:
        memcpy(&Referee.ProjectileAllowance, pData, 6);
        break;
    case 0x0209:
        memcpy(&Referee.RfidStatus, pData, 4);
        break;
    case 0x020A:
        memcpy(&Referee.DartClientCmd, pData, 6);
        break;
    case 0x020B:
        memcpy(&Referee.GroundRobotPosition, pData, 40);
        break;
    case 0x020C:
        memcpy(&Referee.RadarMarkData, pData, 6);
        break;
    case 0x020D:
        memcpy(&Referee.SentryInfo, pData, 6);
        break;
    case 0x020E:
        memcpy(&Referee.RadarInfo, pData, 1);
        break;
    case 0x0301:
        break;
    case 0x0303:
        memcpy(&Referee.MapCommand, pData, 12);
        break;
    default:
        break;
    }
}
