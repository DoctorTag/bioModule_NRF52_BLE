/*******************************************************************************************************
 *
 *
 *
 *
 *
 * *****************************************************************************************************/
#include <stdint.h>
#include <string.h>

#include "boards.h"

uint8_t tPacket[PAYLOAD_MAX_SIZE];

//#include "sdk_config.h"
void BootL_Init(void)
{
    BootL_Comm_Init();
}

uint16_t crc16Make(tBootLPacket tPacket)
{
    uint8_t i;

    uint16_t _sum = 0;

    uint8_t* pmsg = tPacket.tPayload.ui8pData;

    for(i = 0 ; i < tPacket.ui8Length -3 ; i ++)
    {
        _sum += (*(pmsg+i));

    }
    _sum += tPacket.tPayload.ui8Command;
    _sum += tPacket.tPayload.ui8Addr_H;
    _sum += tPacket.tPayload.ui8Addr_L;
    return _sum;
}

uint8_t BootL_sendCommand(uint8_t cmd)
{
    
    tPacket[0] = BootL_HEADER;
    tPacket[1] = 1;
    tPacket[2] = cmd;
    tPacket[3] = cmd;
    tPacket[4] = 0;
    BootL_sendPacket(tPacket,PAYLOAD_CMD_LEN);

    if(cmd != BootL_JMP_APP_CMD)
        return BootL_getResponse();

    return 0;
}


uint8_t BootL_programMemorySegment(uint16_t addr, uint8_t* data,
                                   uint8_t len)
{
    uint16_t xferLen;
    uint8_t res;

    
        tBootLPacket tPacket;
        tPacket.tPayload.ui8Command = BootL_RX_APP_CMD;
        tPacket.tPayload.ui8pData = data;
        tPacket.ui8Length = xferLen + 3;
        tPacket.tPayload.ui8Addr_H = ((uint8_t) (addr >> 8));
        tPacket.tPayload.ui8Addr_L = ((uint8_t) (addr));
        tPacket.ui16Checksum = crc16Make(tPacket);
        BootL_sendPacket(tPacket);

        res = BootL_getResponse();

        if (res != BootL_OK_RES)
            break;

        len -= xferLen;
        addr += xferLen;
        data += xferLen;
   

    return res;
}

/*----------------------------------------------------------------------------+
| End of source file                                                          |
+----------------------------------------------------------------------------*/
/*------------------------ Nothing Below This Line --------------------------*/
