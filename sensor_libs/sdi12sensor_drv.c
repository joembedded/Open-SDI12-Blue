/****************************************************
* sdi12sensor_drv.c
*
* (C) joembedded@gmail.com - joembedded.de
*
* UART is shared with tb_tools
***************************************************/

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include "nrf_drv_gpiote.h"

#include "./platform_nrf52/tb_pins_nrf52.h"
#include "osx_pins.h"

#include "tb_tools.h"
#include "osx_main.h"
#include "device.h"

#include "sdi12sensor_drv.h"

#include "app_uart.h"
#if defined (UART_PRESENT)
#include "nrf_uart.h"
#endif
#if defined (UARTE_PRESENT)
#include "nrf_uarte.h"
#endif

#if defined (UARTE_PRESENT)
 #define STD_SDI_BAUDRATE NRF_UARTE_BAUDRATE_1200
#else
 #define STD_MODEM_BAUDRATE NRF_UART_BAUDRATE_115200
#endif

static const app_uart_comm_params_t _sdi_uart_comm_params = {
          SDI_RX_PIN,
          SDI_TX_PIN,
          UART_PIN_DISCONNECTED  /*RTS_PIN_NUMBER*/,   
          UART_PIN_DISCONNECTED  /*CTS_PIN_NUMBER*/,
          /*APP_UART_FLOW_CONTROL_ENABLED */ APP_UART_FLOW_CONTROL_DISABLED,
          false,      // Not use Parity (bec. 8 Bits)
          STD_SDI_BAUDRATE
};

static bool _uart_already_init;
#ifdef USE_SDI_OBUF
char sdi_obuf[SDI_OBUFS]; // Der OUT-Buffer (2/14; 31+1)
#endif
char sdi_ibuf[SDI_IBUFS]; // Der IN-Buffer 80 (2/14: 79+1)
int16_t sdi_ccnt;				// Zaehlt Zeichen (alle)

// wt: msec lang String abholen versuchen, Ende bei '!' in jedem Fall
// Achtung: BREAK und ERRORs werden asynchron detektiert!

int16_t  sdi_getcmd(uint16_t anz, int32_t wt){
  int16_t res;
  char c;
  int32_t wt0=wt;
  sdi_ccnt=0;
  sdi_ibuf[0]=0;
  if(!anz) return 0;

  for(;;){
        res=tb_getc();
        if(res==-1){
          wt0-=1;  // msec
          if(wt0<=0) return -ERROR_NO_REPLY;  // Timeout, wenigstens CR fehlt
          tb_delay_ms(1);
        }else{
          wt0=wt; // Soft-Timer neu starten
          if(res<=0){ // (sent) BREAK reads <0> or ERROR
            sdi_ccnt=0;
            sdi_ibuf[0]=0;
          }else{
            c=(char)res;
            for(uint8_t i=0;i<7;i++) if(c&(1<<i)) c^=128; // Calc Parity
            if(c&128) return -ERROR_DATA_CORRUPT;
            // Char was OK
            sdi_ibuf[sdi_ccnt++]=c;
            sdi_ibuf[sdi_ccnt]=0; // Immer Terminieren
            if(sdi_ccnt==(SDI_IBUFS-1)) return -ERROR_TOO_MUCH_DATA; // Too much
            if(c=='!') break; // CMD End
            if(sdi_ccnt==anz) break; // Genau soviele lesen
          }
        }
  }
  return sdi_ccnt;
}

// SDI12: 7 Databits, Parity Even (Bsp: 'c' wird als 'c' uebertragen, 'e' mit Bit 7 gesetzt)
void sdi_putc(uint8_t c){
  c&=127;
  for(uint8_t i=0;i<7;i++) if(c&(1<<i)) c^=128; // Calc Parity
  tb_putc(c);
}

// SDI Send Reply - if PC not NULL: Points to cmd, else send sdi12_obuf (of defined)
int16_t sdi_send_reply_crlf(char *pc){
  int16_t h=0;
  
#ifdef USE_SDI_OBUF
  if(pc==NULL) pc=sdi_obuf;
#endif
  while(*pc){
    sdi_putc(*pc++);
    h++;
  }
  sdi_putc('\r');
  sdi_putc('\n');
  return h+2;	// sent LEN
}

/* Init SDI12-UART, save old status OPTIONAL: mehrere Schnittstellen als Parameter */
int16_t sdi_uart_init(void){
  int16_t res;
  _uart_already_init = tb_is_uart_init();
  if(_uart_already_init) tb_uart_uninit();
  res = tb_uart_init((void*)&_sdi_uart_comm_params, NULL, 0, NULL, 0, -1);
  return res;
}

/* Disable SDI-UART and opt. enable tb_uart */
int16_t sdi_uart_uninit(void){
  int16_t res;
  res = tb_uart_uninit();
  if(_uart_already_init) tb_uart_init(NULL, NULL, 0, NULL, 0, -1);
  return res;
}

/* Calculating the SDI12 CRC16: Also useful for external use. 
* SDI12: Init CRC_RUN with 0x0 (Same Polynom is used for MODBUS, but Init is 0xFFFF) */
#define POLY16 0xA001
uint16_t sdi_track_crc16(uint8_t *pdata, uint16_t wlen, uint16_t crc_run) {
    uint8_t j;
    while (wlen--) {
        crc_run ^= *pdata++;
        for (j = 0; j < 8; j++) {
            if (crc_run & 1)
                crc_run = (crc_run >> 1) ^ POLY16;
            else
                crc_run = crc_run >> 1;
        }
    }
    return crc_run;
}

// ***