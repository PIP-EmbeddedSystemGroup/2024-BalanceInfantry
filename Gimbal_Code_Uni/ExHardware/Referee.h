#ifndef _REFEREE_H_
#define _REFEREE_H_

/**
 * @attention   采用UTF-8字符集编码
 */

#include "stdint.h"
#include "cmsis_os.h"

typedef struct
{
    int PackageCount;   //接收到的数据包数量
    int ErrorCount;     //错误数据计数
    uint32_t LastOnlineTick;

    struct
    {
        uint32_t Front;
        uint32_t Left;
        uint32_t Back;
        uint32_t Right;
    } LastHurtTick;

    __packed struct
    {
        uint8_t GameType : 4;
        uint8_t GameProgress : 4;
        uint16_t StageRemainTime;
        uint64_t SyncTimeStamp;
    } GameStatus;

    __packed struct
    {
        uint8_t Winner;
    } GameResult;

    __packed struct
    {
        uint16_t Red1RobotHP;
        uint16_t Red2RobotHP;
        uint16_t Red3RobotHP;
        uint16_t Red4RobotHP;
        uint16_t Red5RobotHP;
        uint16_t Red7RobotHP;
        uint16_t RedOutpostHP;
        uint16_t RedBaseHP;
        uint16_t Blue1RobotHP;
        uint16_t Blue2RobotHP;
        uint16_t Blue3RobotHP;
        uint16_t Blue4RobotHP;
        uint16_t Blue5RobotHP;
        uint16_t Blue7RobotHP;
        uint16_t BlueOutpostHP;
        uint16_t BlueBaseHP;
    } GameRobotHP;

    __packed struct
    {
        uint32_t EventData;
    } EventData;

    __packed struct
    {
        uint8_t Reserved;
        uint8_t SupplyRobotId;
        uint8_t SupplyProjectileStep;
        uint8_t SupplyProjectileNum;
    } ExtSupplyProjectileAction;

    __packed struct
    {
        uint8_t Level;
        uint8_t OffendingRobotId;
        uint8_t Count;
    } RefereeWarning;

    __packed struct
    {
        uint8_t DartRemainingTime;
        uint16_t DartInfo;
    } DartInfo;

    __packed struct
    {
        uint8_t RobotId;
        uint8_t RobotLevel;
        uint16_t CurrentHP;
        uint16_t MaximumHP;
        uint16_t ShooterBarrelCoolingValue;
        uint16_t ShooterBarrelHeatLimit;
        uint16_t ChassisPowerLimit;
        uint8_t PowerManagementGimbalOutput : 1;
        uint8_t PowerManagementChassisOutput : 1;
        uint8_t PowerManagementShooterOutput : 1;
    } RobotStatus;

    __packed struct
    {
        uint16_t ChassisVoltage;
        uint16_t ChassisCurrent;
        float ChassisPower;
        uint16_t BufferEnergy;
        uint16_t Shooter17mm1BarrelHeat;
        uint16_t Shooter17mm2BarrelHeat;
        uint16_t Shooter42mmBarrelHeat;
    } PowerHeatData;

    __packed struct
    {
        float X;
        float Y;
        float Angle;
    } RobotPos;

    __packed struct
    {
        uint8_t RecoveryBuff;
        uint8_t CoolingBuff;
        uint8_t DefenceBuff;
        uint8_t VulnerabilityBuff;
        uint16_t AttackBuff;
    } Buff;

    __packed struct
    {
        uint8_t AirforceStatus;
        uint8_t TimeRemain;
    } AirSupportData;

    __packed struct
    {
        uint8_t ArmorId : 4;        //0 1 2 3 逆时针
        uint8_t HPDeductionReason : 4;
    } HurtData;

    __packed struct
    {
        uint8_t BulletType;
        uint8_t ShooterNumber;
        uint8_t LaunchingFrequency;
        float InitialSpeed;
    } ShootData;

    __packed struct
    {
        uint16_t ProjectileAllowance17mm;
        uint16_t ProjectileAllowance42mm;
        uint16_t RemainingGoldCoin;
    } ProjectileAllowance;

    __packed struct
    {
        uint32_t RfidStatus;
    } RfidStatus;

    __packed struct
    {
        uint8_t DartLaunchOpeningStatus;
        uint8_t Reserved;
        uint16_t TargetChangeTime;
        uint16_t LatestLaunchCmdTime;
    } DartClientCmd;

    __packed struct
    {
        float HeroX;
        float HeroY;
        float EngineerX;
        float EngineerY;
        float Standard3X;
        float Standard3Y;
        float Standard4X;
        float Standard4Y;
        float Standard5X;
        float Standard5Y;
    } GroundRobotPosition;

    __packed struct
    {
        uint8_t MarkHeroProgress;
        uint8_t MarkEngineerProgress;
        uint8_t MarkStandard3Progress;
        uint8_t MarkStandard4Progress;
        uint8_t MarkStandard5Progress;
        uint8_t MarkSentryProgress;
    } RadarMarkData;

    __packed struct
    {
        uint32_t SentryInfo;
        uint16_t SentryInfo2;
    } SentryInfo;

    __packed struct
    {
        uint8_t RadarInfo;
    } RadarInfo;

    __packed struct
    {
        uint16_t DataCmdId;
        uint16_t SenderId;
        uint16_t ReceiverId;
        uint8_t UserData[112];
    } RobotInteractionData;

    __packed struct
    {
        float TargetPositionX;
        float TargetPositionY;
        uint8_t CmdKeyboard;
        uint8_t TargetRobotId;
        uint16_t CmdSource;
    } MapCommand;
} Referee_t;

void Referee_RxCallback_Parsing(uint8_t *rxBuf, int frameLen);

extern uint8_t RefereeRxBuf[192];
extern Referee_t Referee;

#endif
