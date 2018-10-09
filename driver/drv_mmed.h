#ifndef DRV_MMED_H
#define DRV_MMED_H

#define RegNum_Addl                        0x00
#define RegNum_Addm                      0x01
#define RegNum_Addh                       0x02
#define RegNum_Ad_Type_Cnt          0x03
#define RegNum_Dev_Id                    0x04
#define RegNum_AP_VER                   0x05
#define RegNum_BL_VERSION	          0x06
#define RegNum_PStatus                   0x07

#define RegNum_Fun1                     0x08
#define RegNum_RMode_Fun2    0x09
#define BL_REG_DATA   0x0A
#define RegNum_MAX   11

//#define BL_REG_OFFSETH  RegNum_MAX    //RegNum_MAX

#define REG_ONLY_RD_LEN  8


#define SAMPLE_PPG_GDC	 0x00
#define SAMPLE_PPG_RDC     0x10
#define SAMPLE_PPG_IRDC	 0x20
#define SAMPLE_IMP		        0x30

#define SAMPLE_GSR		        0x40

#define SAMPLE_PPG_G	 0x70
#define SAMPLE_PPG_R	 0x80
#define SAMPLE_PPG_IR     0x90
#define SAMPLE_RESP		        0xa0

#define SAMPLE_ECG		        0xb0
#define SAMPLE_WH		        0xc0

#define SAMPLE_GSENSOR		 0xd0

#define FMUP_RSP		 0xF0


#define REG_FUN1_ECG_A             0x0001
#define REG_FUN1_PPG_R             0x0002
#define REG_FUN1_PPG_G             0x0004
#define REG_FUN1_PPG_IR            0x0008
//#define REG_FUN1_PPG_DARK       0x0010
#define REG_FUN1_IMP                  0x0020
#define REG_FUN1_RESP                  0x0040
#define REG_FUN1_GSR                   0x0080

#define REG_FUN2_ECG_D           0x0100
#define REG_FUN2_WH                   0x0200

#define FUN1_COMBO_STD                (REG_FUN1_PPG_R|REG_FUN1_PPG_IR|REG_FUN1_RESP|REG_FUN1_GSR|REG_FUN1_PPG_G)

//#define FUN1_COMBO_PPG                (REG_FUN1_PPG_R|REG_FUN1_PPG_IR|REG_FUN1_PPG_G)
#define FUN1_COMBO_HR_RESP        (REG_FUN1_PPG_G|REG_FUN1_RESP)
//#define FUN1_COMBO_HR_RESP        (REG_FUN1_PPG_G|REG_FUN1_RESP)

#define M_FUN_START        0x10

#define HALT_MODE 	    0xe0
#define OBEY_MODE       0xc0
#define WATCH_MODE    0xa0
#define CAL_MODE          0x80
#define RST_MODE        0x60

#define MODE_ALL_BITS        0xe0

#define OEM_MASK        0x60
#define AP_MASK          0x80

#define OEM_ID        0x20
#define FWUP_PWD        0x58
#define RESET_PWD        0x68


#define BL_CMD_REBOOT    0X60
//#define BL_CMD_UPGRADE_REQ    0X61
#define BL_CMD_ERASEAPP    0X62
#define BL_CMD_JMPAPP    0X63
#define BL_CMD_APRDY    0X64
#define BL_CMD_PROGRAM    0X6f


typedef void   (*mmed_drdy_evt_handler_t)(void *rdata,bool isanalog) ;

//void drv_mmed_init(void );

void drv_mmed_init(mmed_drdy_evt_handler_t handler );

uint8_t drv_mmed_rdreg( uint8_t reg);

void drv_mmed_wrreg( uint8_t reg,uint8_t data);

void drv_mmed_rdbytes(uint8_t *regbuf,uint8_t *rdata,uint8_t size);

//void drv_mmed_start(uint8_t type);

//void drv_mmed_stop(uint8_t type);

void drv_mmed_wrcmd(uint8_t reg_mfun2,uint8_t reg_fun1);
void drv_mmed_int_enable(bool istrue);
#if defined MMED_TEST_IND

void drv_mmed_test_ind(bool isset);


#endif
#endif

