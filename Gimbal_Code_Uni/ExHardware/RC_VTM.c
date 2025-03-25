/**
 * @attention   采用UTF-8字符集编码
 * @brief       RM图传链路控制器(VTM) 接收函数
 * @authors     北工大PIP战队 樊郡捷
 * @version     v1.0
 * @date        2024-10-14
 * @details
 */

#include "header.h"
#include "RC_VTM.h"

uint8_t VtmBuf[64];
RC_VTM_t RC_VTM = { 0 };

extern DMA_HandleTypeDef hdma_usart6_rx;

void RC_VTM_Init(void)
{
    RC_VTM.Key.Raw = 0;
    RC_VTM.Mouse.L = 0;
    RC_VTM.Mouse.R = 0;
    RC_VTM.Mouse.X = 0;
    RC_VTM.Mouse.Y = 0;
    RC_VTM.Mouse.Z = 0;

    RC_VTM.PackageCount = 0;
    RC_VTM.StickyCount = 0;
    RC_VTM.ErrorCount = 0;
    RC_VTM.LastOnlineTick = 0;

    HAL_UARTEx_ReceiveToIdle_DMA(&huart6, VtmBuf, 64);
    __HAL_DMA_DISABLE_IT(&hdma_usart6_rx, DMA_IT_HT);
}

void RC_VTM_RxCallback(uint8_t* rxBuf, int rxSize)
{
    //寻找帧头
    if (rxBuf[0] != 0xA5)
    {
        for (int i = 0; i < rxSize; i++)
        {
            rxBuf++;
            rxSize--;
            if (rxBuf[0] == 0xA5)
                break;
        }
    }

    int DataLength = (rxBuf[1]) | (rxBuf[2] << 8);
    if (rxSize < 5 ||   //帧头至少5字节
        rxSize < 5 + 2 + DataLength + 2)
    {
        memcpy(VtmBuf, rxBuf, rxSize);
        HAL_UARTEx_ReceiveToIdle_DMA(&huart6, VtmBuf + rxSize, 64 - rxSize); //继续接收丢失的部分
        __HAL_DMA_DISABLE_IT(&hdma_usart6_rx, DMA_IT_HT);
        return;
    }

    if (!CRC8_VerifyChecksum(rxBuf, 5) ||                        //帧头CRC8
        !CRC16_VerifyChecksum(rxBuf, 5 + 2 + DataLength + 2))    //数据CRC16
    {
        RC_VTM.ErrorCount++;
        HAL_UARTEx_ReceiveToIdle_DMA(&huart6, VtmBuf, 64);
        __HAL_DMA_DISABLE_IT(&hdma_usart6_rx, DMA_IT_HT);
        return;
    }
    RC_VTM.PackageCount++;

    int cmdId = (rxBuf[5]) | (rxBuf[6] << 8);
    const uint8_t* pData = &rxBuf[7];

    switch (cmdId)
    {
    case 0x0302:
        memcpy(RC_VTM.CustomCtrlData, pData, 30);
        break;
    case 0x0304:
        RC_VTM.Mouse.X = pData[0] | (pData[1] << 8);
        RC_VTM.Mouse.Y = pData[2] | (pData[3] << 8);
        RC_VTM.Mouse.Z = pData[4] | (pData[5] << 8);
        RC_VTM.Mouse.L = pData[6];
        RC_VTM.Mouse.R = pData[7];
        RC_VTM.Key.Raw = pData[8] | (pData[9] << 8);

        Mouse.X = RC_VTM.Mouse.X;
        Mouse.Y = RC_VTM.Mouse.Y;
        Mouse.Z = RC_VTM.Mouse.Z;
        Mouse.L.Value = RC_VTM.Mouse.L;
        Mouse.R.Value = RC_VTM.Mouse.R;

        Keyboard.W.Value = RC_VTM.Key.Code.W;
        Keyboard.S.Value = RC_VTM.Key.Code.S;
        Keyboard.A.Value = RC_VTM.Key.Code.A;
        Keyboard.D.Value = RC_VTM.Key.Code.D;
        Keyboard.Shift.Value = RC_VTM.Key.Code.Shift;
        Keyboard.Ctrl.Value = RC_VTM.Key.Code.Ctrl;
        Keyboard.Q.Value = RC_VTM.Key.Code.Q;
        Keyboard.E.Value = RC_VTM.Key.Code.E;
        Keyboard.R.Value = RC_VTM.Key.Code.R;
        Keyboard.F.Value = RC_VTM.Key.Code.F;
        Keyboard.G.Value = RC_VTM.Key.Code.G;
        Keyboard.Z.Value = RC_VTM.Key.Code.Z;
        Keyboard.X.Value = RC_VTM.Key.Code.X;
        Keyboard.C.Value = RC_VTM.Key.Code.C;
        Keyboard.V.Value = RC_VTM.Key.Code.V;
        Keyboard.B.Value = RC_VTM.Key.Code.B;

        ChassisBoard_SendVTM();
        break;
    default:
        break;
    }

    uint32_t NowTick = HAL_GetTick();
    RC_VTM.LastOnlineTick = NowTick;

    RC_VTM.UpdateCounter++;
    if (NowTick - RC_VTM.LastFreqUpdateTick > 2000)
    {
        RC_VTM.Freq = RC_VTM.UpdateCounter * 1000 / (NowTick - RC_VTM.LastFreqUpdateTick);
        RC_VTM.UpdateCounter = 0;
        RC_VTM.LastFreqUpdateTick = NowTick;
    }

    rxBuf += 5 + 2 + DataLength + 2;
    rxSize -= 5 + 2 + DataLength + 2;
    if (rxSize > 0)
    {
        RC_VTM.StickyCount++;
        RC_VTM_RxCallback(rxBuf, rxSize);
    }
    else
    {
        HAL_UARTEx_ReceiveToIdle_DMA(&huart6, VtmBuf, 64);
        __HAL_DMA_DISABLE_IT(&hdma_usart6_rx, DMA_IT_HT);
    }
}
