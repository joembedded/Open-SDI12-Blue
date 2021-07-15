/****************************************************
* sdi12sensor_drv.h
*
* (C) joembedded@gmail.com - joembedded.de
*
***************************************************/


#define SDI_OBUFS 80
#define SDI_IBUFS 80
extern char sdi_obuf[SDI_OBUFS]; // Der OUT-Buffer (2/14; 31+1)
extern char sdi_ibuf[SDI_IBUFS]; // Der IN-Buffer 80 (2/14: 79+1)
extern int16_t sdi_ccnt;				// Zaehlt Zeichen (alle)

#define ERROR_NO_REPLY      1000
#define ERROR_DATA_CORRUPT  1001
#define ERROR_DRV_ERROR     1002
#define ERROR_TOO_MUCH_DATA 1003

int16_t  sdi_getcmd(uint16_t anz, int32_t wt);
void sdi_putc(uint8_t c);
int16_t sdi_send_reply(char *pc);

int16_t sdi_uart_init(void);
int16_t sdi_uart_uninit(void);

// ***