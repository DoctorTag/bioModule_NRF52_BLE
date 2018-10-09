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
#include "nrf_delay.h"
//#include "sdk_config.h"
#include "app_uart.h"

#if defined (UART_PRESENT)
#include "nrf_uart.h"
#endif
#if defined (UARTE_PRESENT)
#include "nrf_uarte.h"
#endif
//#include "DSP_Filter.h"
#include "m_mmed_local.h"
// Function declarations
#include "drv_mmed.h"
#include "drv_mmed_adc.h"

#include "app_scheduler.h"
#include "nrf_pwr_mgmt.h"
#ifdef FEELKIT_WMMED_I2C
#include "drv_mmed_twi.h"

#endif

#define MAX_RDATA_LENGTH 130
/*1 Start+1CMD+1DLEN+2offset+128DATA+2CRC+1END*/

#define MAX_RX_LENGTH 136
#define CMD_LOC   1
#define DLEN_LOC  2


#define MAX_TDATA_LENGTH 48
/*1 Start+1CMD+1DLEN+48DATA+2CRC+1END*/

#define MAX_TX_LENGTH 54

unsigned char MMDTSTTxPacket[MAX_TX_LENGTH], MMDTSTTxCount ;
unsigned char MMDTSTRxPacket[MAX_RX_LENGTH ], MMDTSTRxCount, dumy ;

struct MMDTST_state MMDTST_Recoder_state;

#define UART_TX_BUF_SIZE 256                         /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE 256                         /**< UART RX buffer size. */
#define UDATA_LPLEN          32

uint8_t dev_id,fwbl_ver,fwap_ver,bio_status;


unsigned char rstate;
unsigned short offset,rcrc,lcrc;
unsigned char rlength;

void Stream_Data(unsigned char dtype,unsigned short rxdata);
static uint8_t freshBioSensorStatus();



unsigned char AdcFunType ;


void RecvInit();

void  RecvFrame( unsigned char src);

void Decode_Recieved_Command(void);

static void rxcmd_evt_sceduled(void * p_event_data, uint16_t event_size)
{
    Decode_Recieved_Command();
}

static void local_utx_evt_sceduled(void * p_event_data, uint16_t event_size)
{
    uint16_t i;
    // uint8_t * sdata = (uint8_t *)p_event_data;
    for(i=0; i<MMDTSTTxCount; i++)
        while (app_uart_put(MMDTSTTxPacket[i]) != NRF_SUCCESS);

}


void uart_event_handle(app_uart_evt_t * p_event)
{

    uint8_t udata;
    //  uint32_t       err_code;

    switch (p_event->evt_type)
    {
        case APP_UART_DATA_READY:
            UNUSED_VARIABLE(app_uart_get(&udata));
            RecvFrame( udata);

            break;

        case APP_UART_COMMUNICATION_ERROR:
            APP_ERROR_HANDLER(p_event->data.error_communication);
            break;

        case APP_UART_FIFO_ERROR:
            APP_ERROR_HANDLER(p_event->data.error_code);
            break;

        default:
            break;
    }
}


static void m_local_uart_init(void)
{
    uint32_t err_code;


    const app_uart_comm_params_t comm_params =
    {
        RX_PIN_NUMBER,
        TX_PIN_NUMBER,
        0xff, //RTS_PIN_NUMBER,
        0xff,// CTS_PIN_NUMBER,
        APP_UART_FLOW_CONTROL_DISABLED,
        false,
        NRF_UART_BAUDRATE_19200
    };

    APP_UART_FIFO_INIT(&comm_params,
                       UART_RX_BUF_SIZE,
                       UART_TX_BUF_SIZE,
                       uart_event_handle,
                       APP_IRQ_PRIORITY_LOWEST,
                       err_code);

    APP_ERROR_CHECK(err_code);

}
uint32_t err_codeD;
void m_local_uart_send_bytes(uint8_t *dbuf,uint16_t len,bool sceduled)
{
#if NRF_MODULE_ENABLED(NRF_PWR_MGMT)
    //    nrf_pwr_mgmt_feed();
#endif

    if(sceduled)
    {
        err_codeD = app_sched_event_put(0, 0, local_utx_evt_sceduled);
        APP_ERROR_CHECK(err_codeD  );
    }
    else
    {
        uint16_t i;

        for(i=0; i<len; i++)
            while (app_uart_put(dbuf[i]) != NRF_SUCCESS);

    }

}

static void generalRspFrame(uint8_t rspd0,uint8_t rspd1)
{
    uint16_t scrc = 0;
    scrc += rspd0;
    scrc += rspd1;
    MMDTSTTxPacket[0] = START_DATA_HEADER;
    MMDTSTTxPacket[1] = MMDTSTRxPacket[1];

    MMDTSTTxPacket[2] = 2;
    MMDTSTTxPacket[3] = rspd0;
    MMDTSTTxPacket[4] = rspd1;

    MMDTSTTxPacket[5] = (uint8_t)(scrc>>8);
    MMDTSTTxPacket[6] = (uint8_t)scrc;
    MMDTSTTxPacket[7] = END_DATA_HEADER;

    MMDTSTTxCount = 8;                              // number of bytes to send
    m_local_uart_send_bytes(MMDTSTTxPacket,MMDTSTTxCount,false);

}
/*******************************************************************************************************
 *
 *
 *
 * *****************************************************************************************************/
void Decode_Recieved_Command(void)
{
    if(lcrc != rcrc)
    {
        generalRspFrame( MMDTSTRxPacket[1],RSP_ERROR_CRC);
    }
    else
    {

        if (MMDTST_Recoder_state.state == IDLE_STATE)
        {
            switch(MMDTST_Recoder_state.command)
            {


                case DATA_STREAMING_COMMAND:    // Data streaming
                {
                    if(bio_status == BIO_NORMAL)
                    {
                        AdcFunType = MMDTSTRxPacket[4];
                        //  drv_mmed_wrreg(RegNum_Peripheral, 0x0a);
                        drv_mmed_wrcmd( MMDTSTRxPacket[3],MMDTSTRxPacket[4]);
                        MMDTST_Recoder_state.state = DATA_STREAMING_STATE;  // Set to Live Streaming state
                        generalRspFrame( bio_status,RSP_OK);
                    }
                    else
                        generalRspFrame( bio_status,RSP_ERROR_STATUS);

                }
                break;
                case FIRMWARE_UPGRADE_CMD:
                {
                    if(MMDTSTRxPacket[3] == BL_CMD_PROGRAM)
                    {
                        if(bio_status == BIO_BOOTL)
                        {
                            drv_mmed_wrreg(RegNum_RMode_Fun2,  BL_CMD_PROGRAM);
                            drv_mmed_int_enable(1);
                            MMDTST_Recoder_state.state = FW_UPGRADING_STATE;  // Set to firmware UPGRADING state

                        }
                        else
                            generalRspFrame( bio_status,RSP_ERROR_STATUS);
                    }
                }
                break;

                case DATA_DOWNLOAD_COMMAND:     // RAW DATA DUMP
                {

                }
                break;

                case START_RECORDING_COMMAND:   // Processed Data Dump
                {
                }
                break;


                case RESTART_COMMAND:  // firmware Version request
                {
                    if(MMDTSTRxPacket[3] == FWUPGRADE_RST)
                    {
                        drv_mmed_wrcmd( RST_MODE,FWUP_PWD);

                    }
                    if(MMDTSTRxPacket[3] == NORMAL_RST)
                    {
                        drv_mmed_wrcmd( RST_MODE,RESET_PWD);

                    }
                    generalRspFrame( MMDTSTRxPacket[4],RSP_OK);
                }
                break;

                case FIRMWARE_VERSION_REQ:  // firmware Version request
                {
                    if(MMDTSTRxPacket[3] == 0)
                        generalRspFrame(fwbl_ver,RSP_OK);
                    if(MMDTSTRxPacket[3] == 1)
                        generalRspFrame(fwap_ver,RSP_OK);
                }
                break;


                case DEVICE_ID_REQ:  // firmware Version request
                {
                    generalRspFrame(dev_id,RSP_OK);
                }
                break;
                case STATUS_INFO_REQ:   // Status Request
                {
                    freshBioSensorStatus();
                    generalRspFrame(bio_status,RSP_OK);
                }
                break;
                case FILTER_SELECT_COMMAND:     // Filter Select request
                {


                }
                break;
                default:

                    break;
            }
        }
        else
        {

            switch(MMDTST_Recoder_state.command)
            {

                case DATA_STREAMING_COMMAND:
                {
                    if(AdcFunType == MMDTSTRxPacket[4])
                    {
                        drv_mmed_wrcmd( MMDTSTRxPacket[3],MMDTSTRxPacket[4]);

                        //     drv_mmed_stop( MMDTSTRxPacket[3]);
                        MMDTST_Recoder_state.state = IDLE_STATE;    // Switch to Idle state
                        //MMDTST_Data_rdy = 0;
                        //Live_Streaming_flag = 0;            // Disable Live streaming flag
                    }

                }
                break;

                case FIRMWARE_UPGRADE_CMD:
                {

                    if(bio_status == BIO_BOOTL)
                    {

                        drv_mmed_wrreg(RegNum_RMode_Fun2, MMDTSTRxPacket[3]);

                    }
                    else
                        generalRspFrame( bio_status,RSP_ERROR_STATUS);

                }
                break;
                case FIRMWARE_UPGRADING_DATA:
                {

                    if(bio_status == BIO_BOOTL)
                    {
#ifdef FEELKIT_WMMED_I2C

                        MMDTSTRxPacket[2] = BL_REG_DATA;

                        mmed_twi_write(MMDTSTRxPacket+2,MAX_RDATA_LENGTH+3);


#endif

#ifdef FEELKIT_WMMED_SPI
                        for(uint8_t i=0; i<MAX_RDATA_LENGTH+2; i++)
                        {
                            // drv_mmed_wrreg(BL_REG_OFFSETH+i, MMDTSTRxPacket[i+3]);
                            drv_mmed_wrreg(BL_REG_DATA, MMDTSTRxPacket[i+3]);

                        }
#endif
                    }
                    else
                        generalRspFrame( bio_status,RSP_ERROR_STATUS);
                }
                break;
                default:

                    break;
            }
        }
    }


    MMDTST_Recoder_state.command = 0;
}

/*****************************************************************************************

*****************************************************************************************/
#define  RX_HEADER           0x00
#define  RX_CMD         0x01
#define  RX_LENGTH        0x02
#define  RX_DATA           0x03
#define  RX_CRC           0x04
#define  RX_END            0x05

#define  CMD_MASK            0xF0
#define  CMD_HIGH            0x80
void RecvInit()
{
    rstate = RX_HEADER;
    offset = 0;

}


void  RecvFrame( unsigned char src)
{
    MMDTSTRxPacket[ offset] = src ;
    offset ++;
    switch(rstate)
    {
        case RX_HEADER:
            if(MMDTSTRxPacket[offset-1] == START_DATA_HEADER)
            {
                rstate = RX_CMD;

            }
            else
            {
                RecvInit();
            }

            break;
        case RX_CMD:
            if((CMD_MASK & MMDTSTRxPacket[offset-1]) == CMD_HIGH)
            {
                rstate = RX_LENGTH;

            }
            else
            {
                RecvInit();
            }
            break;
        case RX_LENGTH:
            rlength = MMDTSTRxPacket[offset-1];
            if(rlength <= MAX_RDATA_LENGTH)
            {
                rstate = RX_DATA;
                lcrc = 0;
            }
            else
            {
                RecvInit();
            }
            break;
        case RX_DATA:
            if(offset >= (rlength+3))
            {
                rstate = RX_CRC;

            }
            lcrc += MMDTSTRxPacket[offset-1];
            break;
        case RX_CRC:
            if(offset == (rlength+4))
            {
                rcrc = MMDTSTRxPacket[offset-1];
                rcrc <<=8;
            }

            if(offset == (rlength+5))
            {
                rcrc |= MMDTSTRxPacket[offset-1];
                rstate = RX_END;

            }
            if(offset > (rlength+5) )
            {
                RecvInit();
            }
            break;
        case RX_END:
            if(MMDTSTRxPacket[offset-1] == 0x0a)
            {
                MMDTST_Recoder_state.command = MMDTSTRxPacket[1];
                //MMDTST_Recoder_state.command = readBytes;
                //   Decode_Recieved_Command();
                APP_ERROR_CHECK( app_sched_event_put(0, 0, rxcmd_evt_sceduled));


                RecvInit();
            }
            else
            {
                if(offset >  (rlength+6))
                {
                    RecvInit();
                }

            }
            break;
    }




}
static uint8_t freshBioSensorStatus()
{
    bio_status = BIO_LOSE;
    dev_id = drv_mmed_rdreg(RegNum_Dev_Id);

    if((dev_id&OEM_MASK) == OEM_ID)
    {
        fwbl_ver = drv_mmed_rdreg(RegNum_BL_VERSION);
        if(dev_id&AP_MASK)
        {
            fwap_ver = drv_mmed_rdreg(RegNum_AP_VER);
            if(fwbl_ver&AP_MASK)
                bio_status = BIO_NORMAL;
        }
        else if((fwbl_ver&AP_MASK) == 0)
            bio_status = BIO_BOOTL;

    }
    return bio_status;
//bio_status = BIO_NORMAL;
}

void m_mmed_local_init(void)
{
    uint8_t cnt=0,bio;
    MMDTST_Recoder_state.state = IDLE_STATE;
    MMDTST_Recoder_state.command = 0;

    drv_mmed_init(m_mmed_local_hanlder);


    drv_mmed_adc_init(m_mmed_local_hanlder);

    m_local_uart_init();
    RecvInit();
    do
    {
        bio = freshBioSensorStatus();
        nrf_delay_ms(100);
        cnt++;
    }
    while((bio == BIO_LOSE)&&(cnt < 10));

}


//dev_id = AP_MASK+OEM_ID+0X01;
//  fwbl_ver = AP_MASK+0X01;
//fwap_ver = 0xa0;




void m_mmed_local_hanlder(void * rdata,bool isanalog)
{
//    static uint8_t y0=0;

    switch(MMDTST_Recoder_state.state)
    {
        case IDLE_STATE:
        {
            if ( MMDTST_Recoder_state.command != 0)
            {
//                  Decode_Recieved_Command();
                MMDTST_Recoder_state.command = 0;
            }
        }
        break;
        case DATA_STREAMING_STATE:
        {


            if(isanalog)
            {
                uint8_t i;
                //uint16_t *tmpd = (uint16_t *)rdata;

                for(i=0; i<MMED_ADC_IN_BUFFER; i++)
                    //Stream_Adc_Data(SAMPLE_ECG,*(tmpd+i));
                    //   Stream_Adc_Data(SAMPLE_ECG,((uint16_t *)rdata)[i]);
                    Stream_Data(SAMPLE_ECG,((uint16_t *)rdata)[i]);
            }
            else
            {
                __32_type ad_val;
                uint8_t * tmpp = (uint8_t *)rdata;
                ad_val.byte.byte0 = tmpp[0];
                ad_val.byte.byte1 = tmpp[1];
                ad_val.byte.byte2 = tmpp[2];
                //    ad_val.byte.byte0 = 55;
                //  ad_val.byte.byte1 = 55;
                // ad_val.byte.byte2 = 55;

                ad_val.byte.byte3 = 0;
                // if((rxdata[3]   & 0xf0) == SAMPLE_TYPE)
                //  Stream_MMDTST_data_packets(ad_val.u32/256);
                Stream_Data(tmpp[3] & 0xf0,ad_val.u32/256);
            }




        }
        break;

        case FW_UPGRADING_STATE:
        {

            uint8_t * tmpp = (uint8_t *)rdata;

            generalRspFrame( tmpp[3],tmpp[0]);
        }

        break;
        case MMDTST_DOWNLOAD_STATE:

            // Send_Recorded_MMDTST_Samples_to_USB();

            break;

        case MMDTST_RECORDING_STATE:

            break;


        default:
            break;
    }




} //main()




#define PACK_SAMPLES 15
unsigned char StreamCount = 0;
static unsigned char sampleCNT = 0;
static short filterd;

void Stream_Data(unsigned char dtype,unsigned short rxdata)
{
    static uint16_t tmp_crc;
    if ( sampleCNT > PACK_SAMPLES) sampleCNT = 0;

    {
        filterd = rxdata;

        if ( sampleCNT == 0)
        {
            tmp_crc = 0;
            StreamCount = 0;
            MMDTSTTxPacket[StreamCount++] = START_DATA_HEADER;              // Packet start Header
            MMDTSTTxPacket[StreamCount++] = DATA_STREAMING_PACKET;          // Live MMDTST Streaming Header
            MMDTSTTxPacket[StreamCount++] = MAX_TDATA_LENGTH;                  // DATA LENGTH

            MMDTSTTxPacket[StreamCount++] = 0;                  // Heart Rate

            MMDTSTTxPacket[StreamCount++] = 0;              // Respiration Rate
            MMDTSTTxPacket[StreamCount++] = 0 ;                 // Lead Status
        }

        tmp_crc += dtype;
        MMDTSTTxPacket[StreamCount++] = dtype ;
        MMDTSTTxPacket[StreamCount] = (filterd & 0x00FF);         // High Byte B15-B8
        tmp_crc += MMDTSTTxPacket[StreamCount];
        StreamCount++;
        MMDTSTTxPacket[StreamCount] = ((filterd & 0xFF00) >> 8 );       // Low byte B7-B0
        tmp_crc += MMDTSTTxPacket[StreamCount];
        StreamCount++;

        sampleCNT++;
        if ( sampleCNT == PACK_SAMPLES)
        {
            sampleCNT = 0;
            MMDTSTTxPacket[StreamCount++] = (uint8_t)(tmp_crc>>8);
            MMDTSTTxPacket[StreamCount++] = (uint8_t)tmp_crc;
            MMDTSTTxPacket[StreamCount++] = '\n';
            //MMDTSTTxPacketRdy = 1;                      // Set packet ready flag after every 14th sample.
            MMDTSTTxCount = StreamCount;                    // Define number of bytes to send as 54.
            //   m_local_uart_send_bytes(MMDTSTTxPacket,StreamCount);
            m_local_uart_send_bytes(MMDTSTTxPacket,MMDTSTTxCount,true);
        }
        //MMDTST_Data_rdy = 0;
    }
}



/*----------------------------------------------------------------------------+
| End of source file                                                          |
+----------------------------------------------------------------------------*/
/*------------------------ Nothing Below This Line --------------------------*/
