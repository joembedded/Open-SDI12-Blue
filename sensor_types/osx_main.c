/****************************************************
* type_main.c - Test Sensor SDI12-Basics
*
* (C) joembedded@gmail.com - joembedded.de
*
* UART is shared with tb_tools
*
***************************************************/

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "nrf_drv_gpiote.h"

#include "./platform_nrf52/tb_pins_nrf52.h"
#include "osx_pins.h"

#include "device.h"
#include "osx_main.h"
#include "tb_tools.h"
#ifdef ENABLE_BLE
 #include "ltx_ble.h"
#endif
#include "intmem.h"
#include "sdi12sensor_drv.h"

// ---Globals---
#define ID_INTMEM_SDIADR 1 // Memory ID fuer Addresse
char my_sdi_adr = '0';     // Factory Default
#define MAX_OUTBUF SDI_OBUFS
char outrs_buf[MAX_OUTBUF];

// ===Sensor Part Start ===
// Id has fixed structure, max. 30+'\0'
//                      all cccccccc.8 mmmmmm.6 vvv.3 xx..xx.[0-13]
//                       13 JoEmbedd   Testse   OSX   (MAC.l)
char sensor_id[8 + 6 + 3 + 13 + 1] = "JoEmbedd"
                                     "Testse"
                                     "OSX"
                                     "Sno..";

// SDI12 allows max. 9 Chars or 7 Digits!
// E.g use 'snprintf(channel_val[].txt,10,"%+f",...)'
// and check for >< 10 Mio, '+9999999.' is not legal
#define MAX_CHAN 10 // Note: Logger-Driver can acept >10
typedef struct {
  int8_t didx;  // Index for 'D'/'R'-Commands, -1: None
  char txt[11]; // Output in SDI121 Format '+/-dd.ddddd', max. 10 char
  char *punit;  // OPt.
} CHANNEL_VAL;
CHANNEL_VAL channel_val[MAX_CHAN];

void sensor_init(void) {
  // Set SNO to Low Mac, so same Name as BLE Advertising
  sprintf(sensor_id + 17, "%08X", mac_addr_l);
}

int16_t sdi_send_reply_mux(uint8_t isrc){
  switch(isrc){
  case SRC_SDI:
    return sdi_send_reply_crlf(outrs_buf);           // send SDI_OBUF chars
  case SRC_CMDLINE:
    tb_printf("%s<CR><LF>",outrs_buf);
    break;
#ifdef ENABLE_BLE
  case SRC_BLE:
    ble_printf("%s<CR><LF>",outrs_buf);
    break;
#endif
  }
  return 1; // Something sent (len not important)
}


//---- Sensor CMDs Start ------------
int16_t sensor_cmd_m(uint8_t isrc, uint8_t carg) {
  if (carg > 0)
    return 0; // only !M supported!

  if(isrc == SRC_SDI) tb_delay_ms(9);                           // Default Delay after CMD
  sprintf(outrs_buf, "%c0012", my_sdi_adr); // 2 Measures in 1 secs
  sdi_send_reply_mux(isrc);           // send SDI_OBUF

  // Check for BRAK in 'M'... todo
  tb_delay_ms(500); // Measure... (faster than time above)
  snprintf(channel_val[0].txt, 11, "%+f", (float)tb_time_get() / 1.234);
  channel_val[0].punit = "xtime";

  snprintf(channel_val[1].txt, 11, "+%u", tb_get_ticks() % 1000000); // '+* only d/f
  channel_val[1].punit = "cnt";

  channel_val[2].txt[0] = 0;                                         // End

  // Build Outstring (Regarding max length 35/75.. todo)
  channel_val[0].didx = 0;
  channel_val[1].didx = 0;
  channel_val[2].didx = -1;

  sprintf(outrs_buf, "%c", my_sdi_adr); // Service Request
  sdi_send_reply_mux(isrc);       // send SDI_OBUF
  if(isrc == SRC_SDI) tb_delay_ms(28);                      // Needs 25 msec

  *outrs_buf = 0; // Assume no reply
  return 1; // Cmd was OK
}

//---- Sensor CMDs End ------------
// ===Sensor Part END ===

//----- SDI-RX-Pin-IRQ-Driver ------------
bool rxirq_active;
volatile uint32_t rxirq_zcnt;
/*irq*/ static void rxirq_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action) {
  rxirq_zcnt++;
}

// Set/Unset RX-Pin to "IRQ-on-BREAK", opt. deactivate SDI-UART, Details: see gpio_irq.c-Sample
static void rxirq_on(void) {
  uint32_t err_code;
  if (rxirq_active)
    return; // Already ON
  nrf_drv_gpiote_in_config_t in_config = NRFX_GPIOTE_CONFIG_IN_SENSE_HITOLO(false);
  err_code = nrf_drv_gpiote_in_init(SDI_RX_PIN, &in_config, rxirq_handler);
  APP_ERROR_CHECK(err_code);
  nrf_drv_gpiote_in_event_enable(SDI_RX_PIN, true);
  rxirq_active = true;
}
static void rxirq_off(void) {
  if (!rxirq_active)
    return; // Already OFF
  nrf_drv_gpiote_in_event_disable(SDI_RX_PIN);
  nrf_drv_gpiote_in_uninit(SDI_RX_PIN); // Disable Functins an Pullups
  rxirq_active = false;
}

//--- Init (only call once)
void type_init(void) {
  uint32_t err_code;
  err_code = nrf_drv_gpiote_init();
  APP_ERROR_CHECK(err_code);

  intpar_mem_read(ID_INTMEM_SDIADR, 1, (uint8_t *)&my_sdi_adr);
  sensor_init(); // ID etc..

  rxirq_on(); // SDI now active
}

// Static parts of CMDs
static bool cmdcrc_flag = false;

// Flexible cmd. 0: No Reply
int16_t sdi_process_cmd(uint8_t isrc, char *const ps_ibuf) {
  char *pc, arg_val0;
  int8_t i8h; // Temp
  uint16_t crc16;
  uint16_t len;

  *outrs_buf = 0; // Assume no reply
  len = strlen(ps_ibuf);
  if (len && ps_ibuf[len - 1] == '!' && // Only Commands (end with '!')
      (*ps_ibuf == '?' || *ps_ibuf == my_sdi_adr)) {

    // Fast scan CMD via switch() - reply only to valid CMDs
    switch (ps_ibuf[1]) {
    case '!': // Only "!\0"
      if (!ps_ibuf[2])
        sprintf(outrs_buf, "%c", my_sdi_adr);
      break;
    case 'I': // SDI V1.3
      if (!strcmp(ps_ibuf + 2, "!"))
        sprintf(outrs_buf, "%c13%s", my_sdi_adr, sensor_id);
      break;
    case 'A':
      arg_val0 = ps_ibuf[2];
      if (!strcmp(ps_ibuf + 3, "!") && ((arg_val0 >= '0' && arg_val0 <= '9') ||
                                           (arg_val0 >= 'A' && arg_val0 <= 'Z') ||
                                           (arg_val0 >= 'a' && arg_val0 <= 'z'))) {

        if (arg_val0 != my_sdi_adr) {
          intpar_mem_write(ID_INTMEM_SDIADR, 1, (uint8_t *)&arg_val0);
          intpar_mem_read(ID_INTMEM_SDIADR, 1, (uint8_t *)&my_sdi_adr);
        }

        sprintf(outrs_buf, "%c", my_sdi_adr);
      }
      break;
    case 'M': // aM!, aMx!, aMC!, aMCx!, pc points after 'M'
      pc = ps_ibuf + 2;
      if (*pc == 'C') {
        cmdcrc_flag = true;
        pc++;
      } else
        cmdcrc_flag = false;
      if (*pc == '!')
        arg_val0 = 0;
      else
        arg_val0 = (*pc++) - '0';
      if (!strcmp(pc, "!") && arg_val0 >= 0 && arg_val0 <= 9) {
        if(sensor_cmd_m(isrc, (uint8_t)arg_val0)) return 1; // Reply was sent
      }
      break;
    case 'D': // D0-D9
      arg_val0 = ps_ibuf[2] - '0';
      if (!strcmp(ps_ibuf + 3, "!") && arg_val0 >= 0 && arg_val0 <= 9) {
        pc = outrs_buf;
        *pc++ = my_sdi_adr;
        *pc = 0;
        for (uint16_t i=0; i < MAX_CHAN; i++) {
          i8h = channel_val[i].didx;
          if (i8h == -1)
            break;
          if (i8h == arg_val0) {
            strcpy(pc, channel_val[i].txt);
            pc += strlen(channel_val[i].txt);
          }
        }
        if (cmdcrc_flag) {
          crc16 = sdi_track_crc16(outrs_buf, (pc - outrs_buf), 0 /*Init*/);
          *pc++ = 64 + ((crc16 >> 12) & 63);
          *pc++ = 64 + ((crc16 >> 6) & 63);
          *pc++ = 64 + (crc16 & 63);
          *pc = 0;
        }
        // Now D-String is ready
      }
      break;

      // default: No Reply!
    } // switch
  }

  if (*outrs_buf) {
    if(isrc == SRC_SDI) tb_delay_ms(9);                        // Default Delay after CMD
    return sdi_send_reply_mux(isrc); // send SDI_OBUF
  } else {
    return 0;
  }
}

bool type_service(void) {
  int16_t res;
  int16_t txwait_chars = 0;

  // SDI12 Activity registered
  if (rxirq_zcnt) {
    rxirq_off();
    tb_putc('['); // Signal SDI12 Activity
    *outrs_buf = 0;
    tb_delay_ms(1);
    sdi_uart_init();
    for (;;) {
      res = sdi_getcmd(SDI_IBUFS, 100 /*ms*/); // Max. wait per Def.
      if (res == -ERROR_NO_REPLY) {
        txwait_chars = 0;
        break; // Timeout
      } else if (res > 0) {
        txwait_chars = sdi_process_cmd(SRC_SDI, sdi_ibuf);
      }
    } // for()

    if (txwait_chars)
      tb_delay_ms(txwait_chars * 9); // 1 Char needs 8.33 msec
    sdi_uart_uninit();
    rxirq_on();
    rxirq_zcnt = 0;

    tb_putc(']'); // Signal SDI12 Activity
    return true;  // No Periodic Service
  }
  return false; // Periodic Service
}

// Die Input String und Values
bool type_cmdline(uint8_t isrc, uint8_t *pc, uint32_t val) {
  int res;

tb_printf("CMD:'%s'",pc);

  switch (*pc) {
#if DEBUG // Test intmem functions 
  // !!! only if isrc==SRC_CMDLINE !!!
  case 'm':
    tb_printf("m: %d\n", intpar_mem_write(val, 0, NULL)); // Testparameter schreiben, check mit 'H'
    break;
  case 'l':
    tb_printf("l: %d\n", intpar_mem_write(val, strlen(pc + 1), pc + 1)); // Testparameter schreiben, check mit 'H'
    break;
  case 's':
    res = intpar_mem_read(val, 255, pc);
    tb_printf("l: %d:", res);
    if (res > 0)
      for (int i = 0; i < res; i++)
        tb_printf("%x ", pc[i]);
    tb_printf("\n");
    break;
  case 'k':
    intpar_mem_erase();
    break;
#endif

  case 'z':  // 'z' - SDI12 emulated
    if(isrc==SRC_CMDLINE){
      tb_printf("%s => ",pc+1);
      if(!sdi_process_cmd(SRC_CMDLINE, pc+1)) tb_printf("<NO REPLY>");
      tb_printf("\n");
    }
#ifdef ENABLE_BLE
    else if(isrc==SRC_BLE){
      bool oldr = ble_reliable_printf;
      ble_reliable_printf=true;
      if(!sdi_process_cmd(SRC_BLE, pc+1)) ble_printf("<NO REPLY>");
      ble_reliable_printf=oldr;
    }
#endif
    break;

  default:
    return false;
  }
  return true; // Command processed
}

//***