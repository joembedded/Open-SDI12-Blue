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
#include "sdi12sensor_drv.h"

#include "device.h"
#include "osx_main.h"
#include "tb_tools.h"

#include "app_uart.h"
#if defined(UART_PRESENT)
#include "nrf_uart.h"
#endif
#if defined(UARTE_PRESENT)
#include "nrf_uarte.h"
#endif

#if defined(UARTE_PRESENT)
#define STD_SDI_BAUDRATE NRF_UARTE_BAUDRATE_1200
#else
#define STD_MODEM_BAUDRATE NRF_UART_BAUDRATE_115200
#endif

static const app_uart_comm_params_t _sdi_uart_comm_params = {
    SDI_RX_PIN,
    SDI_TX_PIN,
    UART_PIN_DISCONNECTED /*RTS_PIN_NUMBER*/,
    UART_PIN_DISCONNECTED /*CTS_PIN_NUMBER*/,
    /*APP_UART_FLOW_CONTROL_ENABLED */ APP_UART_FLOW_CONTROL_DISABLED,
    false, // Not use Parity (bec. 8 Bits)
    STD_SDI_BAUDRATE};

static bool _uart_already_init;

char sdi_obuf[SDI_OBUFS]; // Der OUT-Buffer (2/14; 31+1)
char sdi_ibuf[SDI_IBUFS]; // Der IN-Buffer 80 (2/14: 79+1)
int16_t sdi_ccnt;         // Zaehlt Zeichen (alle)
uint8_t sdi_tx_delay = 0; // Wait before TX

// wt: msec lang String abholen versuchen, Ende bei '!' in jedem Fall
// Achtung: BREAK und ERRORs werden asynchron detektiert!

int16_t sdi_getcmd(uint16_t anz, int32_t wt) {
  int16_t res;
  char c;
  int32_t wt0 = wt;
  sdi_ccnt = 0;
  sdi_ibuf[0] = 0;
  if (!anz)  return 0;

  while(tb_getc()>=0);

  for (;;) {
    res = tb_getc();
    if (res == -1) {
      wt0 -= 1; // msec
      if (wt0 <= 0)
        return -ERROR_NO_REPLY; // Timeout, wenigstens CR fehlt
      tb_delay_ms(1);
    } else {
      wt0 = wt;       // Soft-Timer neu starten
      if (res <= 0) { // (sent) BREAK reads <0> or ERROR
        // Theoretically Break Length (>12msec should be checked)
        sdi_ccnt=0;
        sdi_ibuf[0]=0;
      } else {
        c = (char)res;
        for (uint8_t i = 0; i < 7; i++)
          if (c & (1 << i))
            c ^= 128; // Calc Parity
        if (c & 128)
          return -ERROR_PARITY_ERROR;
        // Only allowed CR NL Space .. 0x7F
        if (c < 32 && (c != '\r' && c != '\n'))
          return -ERROR_ILLEGAL_CHARS;
        // Char was OK
        sdi_ibuf[sdi_ccnt++] = c;
        sdi_ibuf[sdi_ccnt] = 0; // Immer Terminieren
        if (sdi_ccnt == (SDI_IBUFS - 1))
          return -ERROR_TOO_MUCH_DATA; // Too much
        if (c == '!')
          break; // CMD End
        if (sdi_ccnt == anz)
          break; // Genau soviele lesen
      }
    }
  }
  return sdi_ccnt;
}

// SDI Send Reply - if PC not NULL: Points to cmd, else send sdi12_obuf (of defined)
// SDI12: 7 Databits, Parity Even (Bsp: 'c' wird als 'c' uebertragen, 'e' mit Bit 7 gesetzt)
int16_t sdi_send_reply_crlf(void) {
  uint8_t c;
  int16_t h = 0; // Len
  uint8_t *rbpc;
  int16_t rbc;
  uint8_t wt;

  // Optionally wait (marking 8.33 msec)
  if(sdi_tx_delay) tb_delay_ms(sdi_tx_delay);
  sdi_tx_delay=0;

  // Add Parity and CR NL
  char *pc = sdi_obuf;
  for (;;) {
    c = *pc;
    if (!c)
      break;
    // c&=127; Should not be necessary
    for (uint8_t i = 0; i < 7; i++)
      if (c & (1 << i))
        c ^= 128; // Calc Parity
    *pc++ = c;
  }
  *pc++ = 141; // '\r' with Parity
  *pc++ = 10;  // '\n' with Parity
  *pc = 0;
  h = pc - sdi_obuf;

  while(tb_getc()>=0);  // Clear Input

  pc = sdi_obuf;
  rbpc = pc; // Read Back pointer
  for (;;) {
    c = *pc++;
    if (!c)
      break;
    tb_putc(c);
    rbc = tb_getc();
    if (rbc !=-1) { // Found sth.
      if (!rbc) return -ERROR_UNEXPECTED_BREAK;
      if (rbc<0) return -ERROR_FRAMING_ERROR; 
      if ((uint8_t)rbc != *rbpc) return -ERROR_DATA_CORRUPT;
      rbpc++; // Expect next
    }
  }

  wt = 15;  // Initial wait for 1.st char
  for (;;) {
    rbc = tb_getc();
    if (rbc !=-1) { // Found sth.
      if (!rbc) return -ERROR_UNEXPECTED_BREAK;
      if (rbc<0) return -ERROR_FRAMING_ERROR; 
      if ((uint8_t)rbc != *rbpc) return -ERROR_DATA_CORRUPT;
      rbpc++; // Expect next
      if(!*rbpc) break; // All read
      wt = 10; // Wait again
      continue;
    }
    tb_delay_ms(1);
    if (!wt--) return -ERROR_INTERIM_TIMOUT; 
  }
  return h; // sent LEN
}

/* Init SDI12-UART, save old status OPTIONAL: mehrere Schnittstellen als Parameter */
int16_t sdi_uart_init(void) {
  int16_t res;
  _uart_already_init = tb_is_uart_init();
  if (_uart_already_init)
    tb_uart_uninit();
  res = tb_uart_init((void *)&_sdi_uart_comm_params, NULL, 0, NULL, 0, -1);
  return res;
}

/* Disable SDI-UART and opt. enable tb_uart */
int16_t sdi_uart_uninit(void) {
  int16_t res;
  res = tb_uart_uninit();
  if (_uart_already_init)
    tb_uart_init(NULL, NULL, 0, NULL, 0, -1);
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