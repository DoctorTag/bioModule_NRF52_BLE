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

//#include "sdk_config.h"
#ifndef __BootL_H__
#define __BootL_H__

/*! Maximum size of Packet =  1 header + 1 length + 1 CMD + 2 ADDR + 128 data  + 2 checksum*/
#define PAYLOAD_DATA_LEN            128
#define PAYLOAD_MISC_LEN             7
#define PAYLOAD_MAX_SIZE            (PAYLOAD_DATA_LEN+PAYLOAD_MISC_LEN )

/*! size of CMD Packet =  1 header + 1 length + 1 CMD + 2 checksum*/
#define PAYLOAD_CMD_LEN             5

#define VBOOT_ENTRY_CMD     0xAA
#define MMED_BOOTL_VERSION       0xA1

#define BootL_VERSION_CMD     0x19
#define BootL_ERASE_APP_CMD   0x15
#define BootL_RX_APP_CMD      0x10
#define BootL_JMP_APP_CMD     0x1C
#define BootL_SOFTRST_CMD     0x1D

#define BootL_OK_RES          0x00

#define BootL_HEADER          0x80


/* Prototypes */
// Generic BootL
void BootL_Init(void);
uint8_t BootL_sendCommand(uint8_t cmd);
uint8_t BootL_programMemorySegment(uint32_t addr, const uint8_t* data,
        uint32_t len);
// Communication dependent
void BootL_Comm_Init(void);
uint8_t BootL_getResponse(void);
uint8_t BootL_slavePresent(void);
void BootL_sendSingleByte(uint8_t ui8Byte);
void BootL_sendPacket(tBootLPacket tPacket);
void BootL_flush(void);


#endif /* __BootL_H__ */

/*----------------------------------------------------------------------------+
| End of source file                                                          |
+----------------------------------------------------------------------------*/
/*------------------------ Nothing Below This Line --------------------------*/
