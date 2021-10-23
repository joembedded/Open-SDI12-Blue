/****************************************************
* sdi12sensor_drv.h
*
* (C) joembedded@gmail.com - joembedded.de
*
***************************************************/


#define SDI_OBUFS 84+20           // 20 Reserve(Debug/Config)
extern char sdi_obuf[SDI_OBUFS]; // Der OUT-Buffer min 81 (75+6+1) 
#define SDI_IBUFS 84+20           // 20 Reserve(Debug/Config)
extern char sdi_ibuf[SDI_IBUFS]; // Der IN-Buffer 
extern int16_t sdi_ccnt;				// Counts incumming chars
extern uint8_t sdi_tx_delay;

#define ERROR_NO_REPLY      1000 // initial Timouts
#define ERROR_DATA_CORRUPT  1001
#define ERROR_DRV_ERROR     1002
#define ERROR_TOO_MUCH_DATA 1003
#define ERROR_INTERIM_TIMOUT 1004 // intermediate Timouts
#define ERROR_PARITY_ERROR  1005
#define ERROR_ILLEGAL_CHARS 1006
#define ERROR_UNEXPECTED_BREAK 1007
#define ERROR_FRAMING_ERROR 1008
#define ERROR_NOT_ENOUGH_DATA 1009
#define ERROR_DATA_CRC   1010
#define ERROR_PROTOCOL_TX   1011
#define ERROR_PROTOCOL_RX   1012
#define ERROR_PARAMETER      1013


int16_t  sdi_getcmd(uint16_t anz, int32_t wt);
void sdi_putc(uint8_t c);
int16_t sdi_send_reply_crlf(void);

int16_t sdi_uart_init(bool check);
int16_t sdi_uart_uninit(bool check);
uint16_t sdi_track_crc16(uint8_t *pdata, uint16_t wlen, uint16_t crc_run);

// ***