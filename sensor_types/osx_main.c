/****************************************************
* osx_main.c - Sensor SDI12-Driver
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
#include "sdi12sensor_drv.h"

#include "./platform_nrf52/tb_pins_nrf52.h"
#include "device.h"
#include "osx_main.h"
#include "osx_pins.h"
#include "saadc.h"
#include "tb_tools.h"

#ifdef ENABLE_BLE
#include "ltx_ble.h"
#endif
#include "intmem.h"

// ---Globals---
char my_sdi_adr = '0'; // Factory Default
char outrs_buf[MAX_OUTBUF];
SDI_VALIO sdi_valio;

// ===Sensor Part Start ===
// Id has fixed structure, max. 30+'\0'
//                      all cccccccc.8 mmmmmm.6 vvv.3 xx..xx.[0-13]
//                       13 JoEmbedd   Testse   OSX   (MAC.l)
char sensor_id[8 + 6 + 3 + 13 + 1] = "JoEmbedd"
                                     "Testse"
                                     "OSX"
                                     "Sno..";

int16_t sdi_send_reply_mux(uint8_t isrc) {
  switch (isrc) {
  case SRC_SDI:
    return sdi_send_reply_crlf(outrs_buf); // send SDI_OBUF chars
  case SRC_CMDLINE:
    tb_printf("%s<CR><LF>", outrs_buf);
    break;
#ifdef ENABLE_BLE
  case SRC_BLE: {
    bool oldr = ble_reliable_printf;
    ble_reliable_printf = true;
    ble_printf("%s<CR><LF>", outrs_buf);
    ble_reliable_printf = oldr;
  } break;
#endif
  }
  return 1; // Something sent (len not important)
}

// Build Outstring (Regarding max length 35/75.. todo)
void sensor_build_outstring(void) {
  uint8_t didx = 0;
  uint16_t rlen = 0, hlen;
  uint16_t cmd_max_len = 35; /** For M-CMD, othes: todo **/
  for (uint16_t i = 0; i < MAX_CHAN; i++) {
    if (i >= sdi_valio.anz_channels) {
      sdi_valio.channel_val[i].didx = -1;
      break;
    }
    hlen = strlen(sdi_valio.channel_val[i].txt);
    if (rlen + hlen > cmd_max_len) {
      rlen = 0;
      didx++;
    }
    sdi_valio.channel_val[i].didx = didx;
  }
}

//---- Sensor CMDs Start ------------
// return 0: CMD not valid, >0: Finished, -1: <BREAK> found
int16_t sensor_cmd_m(uint8_t isrc, uint8_t carg) {
  if (sensor_valio_input('M', carg) == false)
    return 0; // Cmd not supported

  if (isrc == SRC_SDI)
    tb_delay_ms(9); // Default Delay after CMD

  // CHannel '0'-'9' , then ASCII
  sprintf(outrs_buf, "%c%03u%c", my_sdi_adr, (sdi_valio.m_msec + 999) / 1000, sdi_valio.anz_channels + '0'); // M: y Measures in xxx secs
  sdi_send_reply_mux(isrc);                                                                                  // send SDI_OBUF

  if (sensor_valio_measure(isrc) < 0)
    return 0; // Aborted
  sensor_build_outstring();

  sprintf(outrs_buf, "%c", my_sdi_adr); // Service Request
  sdi_send_reply_mux(isrc);             // send SDI_OBUF

  *outrs_buf = 0; // Assume no reply
  return 1;       // Cmd was OK
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
        sensor_cmd_m(isrc, (uint8_t)arg_val0);
        return 1;
      }
      break;
    case 'D': // D0-D9
      arg_val0 = ps_ibuf[2] - '0';
      if (!strcmp(ps_ibuf + 3, "!") && arg_val0 >= 0 && arg_val0 <= 9) {
        pc = outrs_buf;
        *pc++ = my_sdi_adr;
        *pc = 0;
        for (uint16_t i = 0; i < MAX_CHAN; i++) {
          i8h = sdi_valio.channel_val[i].didx;
          if (i8h == -1)
            break;
          if (i8h == arg_val0) {
            strcpy(pc, sdi_valio.channel_val[i].txt);
            pc += strlen(sdi_valio.channel_val[i].txt);
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
    case 'X': // 'X'; Additional SDI12 - User Commands
      if (!strcmp(ps_ibuf+2, "FactoryReset!")){
        intpar_mem_erase();
        sprintf(outrs_buf, "%c", my_sdi_adr);
        tb_delay_ms(9);           // Default Delay after CMD
        sdi_send_reply_mux(isrc); // send SDI_OBUF
        tb_delay_ms(100);
        tb_system_reset();        // Reset...
      } // else 
      sensor_valio_xcmd(isrc,ps_ibuf+2);

      break;  

      // default: No Reply!
    } // switch
  }

  if (*outrs_buf) {
    if (isrc == SRC_SDI)
      tb_delay_ms(9);                // Default Delay after CMD
    return sdi_send_reply_mux(isrc); // send SDI_OBUF
  } else {
    return 0;
  }
}

bool type_service(void) {
  int16_t res;

  // SDI12 Activity registered
  if (rxirq_zcnt) {
    rxirq_off();
    tb_putc('['); // Signal SDI12 Activity
    *outrs_buf = 0;
    tb_delay_ms(1);
    sdi_uart_init();

    for (;;) {
      tb_board_led_on(0);
      res = sdi_getcmd(SDI_IBUFS, 100 /*ms*/); // Max. wait per Def.
      tb_board_led_off(0);
      if (res == -ERROR_NO_REPLY) {
        break;              // Timeout
      } else if (res > 0) { // Soemthing received
        sdi_process_cmd(SRC_SDI, sdi_ibuf);
        // And again until No Reply
      } // else: Only Break or Corrupt Data
    }   // for()

    sdi_uart_uninit();
    rxirq_on();
    rxirq_zcnt = 0;

    tb_putc(']'); // Signal SDI12 Activity
    return true;  // No Periodic Service
  }
  return false; // Periodic Service
}

void type_cmdprint_line(uint8_t isrc, char *pc) {
  if (isrc == SRC_CMDLINE) {
    tb_printf("%s\n", pc);
#ifdef ENABLE_BLE
  } else if (isrc == SRC_BLE) {
    bool oldr = ble_reliable_printf;
    ble_reliable_printf = true;
    ble_printf(pc);
    ble_reliable_printf = oldr;
#endif
  }
}

// Die Input String und Values
bool type_cmdline(uint8_t isrc, uint8_t *pc, uint32_t val) {
  int res;

  switch (*pc) {

  case 'z': // 'z' - SDI12 emulated
    if (!sdi_process_cmd(isrc, pc + 1))
      type_cmdprint_line(isrc, "<NO REPLY>");
    break;

  case 'e': // Measure

    //        ble_printf("~e:%u %u",highest_channel, measure_time_msec);
    if (sensor_valio_input('M', (uint8_t)val) == false) {
      type_cmdprint_line(isrc, "Error('e')");
      break;
    }
    sprintf(outrs_buf, "~e:%u %u", sdi_valio.anz_channels, sdi_valio.m_msec);
    type_cmdprint_line(isrc, outrs_buf);

    sensor_valio_measure(SRC_NONE); // SLient

    for (int16_t i = 0; i < sdi_valio.anz_channels; i++) {
      char *pc = sdi_valio.channel_val[i].txt;
      if (*pc == '+')
        pc++;
      sprintf(outrs_buf, "~#%u: %s %s", i, pc, sdi_valio.channel_val[i].punit);
      type_cmdprint_line(isrc, outrs_buf);
    }

    sprintf(outrs_buf, "~h:%u\n", 0); // Reset, Alarm, alter Alarm, Messwert, ..
    type_cmdprint_line(isrc, outrs_buf);

    break;

  default:
    return false;
  }
  return true; // Command processed
}

//***