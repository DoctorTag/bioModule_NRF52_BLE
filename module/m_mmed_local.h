#ifndef M_MMED_LOCAL_H_
#define M_MMED_LOCAL_H_

typedef long	__S32_bit;
	typedef struct {
	    unsigned char b0:1;
	    unsigned char b1:1;
	    unsigned char b2:1;
	    unsigned char b3:1;
	    unsigned char b4:1;
	    unsigned char b5:1;
	    unsigned char b6:1;
	    unsigned char b7:1;
	    unsigned char b8:1;
	    unsigned char b9:1;
	    unsigned char b10:1;
	    unsigned char b11:1;
	    unsigned char b12:1;
	    unsigned char b13:1;
	    unsigned char b14:1;
	    unsigned char b15:1;
	    unsigned char b16:1;
	    unsigned char b17:1;
	    unsigned char b18:1;
	    unsigned char b19:1;
	    unsigned char b20:1;
	    unsigned char b21:1;
	    unsigned char b22:1;
	    unsigned char b23:1;
	    unsigned char b24:1;
	    unsigned char b25:1;
	    unsigned char b26:1;
	    unsigned char b27:1;
	    unsigned char b28:1;
	    unsigned char b29:1;
	    unsigned char b30:1;
	    unsigned char b31:1;

	} __32_bits;
	typedef union {
		struct
		{
			unsigned char byte0;
			unsigned char byte1;
			unsigned char byte2;
			unsigned char byte3;

		}byte;
	    __32_bits 		bits;
	    unsigned long 	u32;
	    long			s32;
	    
	} __32_type;


struct MMDTST_state{
	unsigned char state;
	unsigned char SamplingRate;
	unsigned char command;
};

typedef enum stMMDTST_RECORDER_STATE {
	
	IDLE_STATE =0,
	DATA_STREAMING_STATE,
	ACQUIRE_DATA_STATE,
	MMDTST_DOWNLOAD_STATE,
	MMDTST_RECORDING_STATE,
	FW_UPGRADING_STATE
}MMDTST_RECORDER_STATE;


typedef enum tBIO_SENSOR_STATE {
	
	BIO_NORMAL =0,
	BIO_BOOTL,
	BIO_LOSE	
}BIO_SENSOR_STATE;

#define START_DATA_HEADER			0x55
#define WRITE_REG_COMMAND			0x81
#define READ_REG_COMMAND			0x82
#define DATA_STREAMING_COMMAND	0x83
#define DATA_STREAMING_PACKET		0x83
#define ACQUIRE_DATA_COMMAND		0x84
#define ACQUIRE_DATA_PACKET 		0x84
#define DEVICE_ID_REQ                   	0x85
#define DATA_DOWNLOAD_COMMAND	0x86
#define FIRMWARE_UPGRADE_CMD	       0x87
#define FIRMWARE_UPGRADING_DATA	0x88
#define FIRMWARE_VERSION_REQ		0x89
#define STATUS_INFO_REQ 			0x8A
#define FILTER_SELECT_COMMAND		0x8B
//#define ERASE_MEMORY_COMMAND		0x8C
#define RESTART_COMMAND				0x8D
#define START_RECORDING_COMMAND		0x8E


#define END_DATA_HEADER				0x0A



#define NORMAL_RST				       0x01
#define FWUPGRADE_RST				0x02
#define FWUPGRADE_PWD		0x58

/* cmd respone info */

#define RSP_OK				        0x01
#define RSP_ERROR_UNKNOW	        0x02
#define RSP_ERROR_CRC		        0x03
#define RSP_ERROR_STATUS		 0x04


void m_mmed_local_init(void);
void m_local_uart_send_bytes(uint8_t *dbuf,uint16_t len,bool sceduled);
void m_mmed_local_hanlder(void * rdata,bool isanalog);

#endif /*MMDTST_MAIN_H_*/
