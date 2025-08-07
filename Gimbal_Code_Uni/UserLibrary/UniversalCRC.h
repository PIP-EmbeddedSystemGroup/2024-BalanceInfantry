#ifndef _UNIVERSAL_CRC_H_
#define _UNIVERSAL_CRC_H_

/**
 * @attention   采用UTF-8字符集编码
 */

#include "stdint.h"

uint8_t CRC8_GetChecksum(uint8_t* pchMessage, uint32_t dwLength, uint8_t ucCRC8);
uint32_t CRC8_VerifyChecksum(uint8_t* pchMessage, uint32_t dwLength);
void CRC8_AppendChecksum(uint8_t* pchMessage, uint32_t dwLength);
uint16_t CRC16_GetChecksum(uint8_t* pchMessage, uint32_t dwLength, uint16_t wCRC);
uint32_t CRC16_VerifyChecksum(uint8_t* pchMessage, uint32_t dwLength);
void CRC16_AppendChecksum(uint8_t* pchMessage, uint32_t dwLength);

#endif
