/****************************************************
* 0900_testsensor.c - Test Sensor SDI12-Basics
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

#include "device.h"
#include "sdi12sensor_drv.h"
#include "osx_main.h"
#include "saadc.h"
#include "tb_tools.h"

#include "intmem.h"

//------------------- Implementation -----------
void sensor_init(void) {
  // Set SNO to Low Mac, so same Name as BLE Advertising
  sprintf(sensor_id + 17, "%08X", mac_addr_l);
}

bool sensor_valio_input(char cmd, uint8_t carg) {
  sdi_valio.measure_cmd = cmd;
  sdi_valio.measure_arg = carg;
  sdi_valio.channel_val[0].didx = -1; // Assume no Values in D-Buffer

  // For THIS sensor:
  if (cmd != 'M')
    return false; // This sensor only supports M, M1
  if (carg == 0) {
    sdi_valio.anz_channels = 2;
    sdi_valio.m_msec = 500;
  } else if (carg == 1) {
    sdi_valio.anz_channels = 3;
    sdi_valio.m_msec = 1000;
  } else
    return false;

  return true;
}
//
int16_t sensor_valio_measure(uint8_t isrc) {
  //---- 'M': While waiting: scan SDI for <BREAK> ----
  int16_t wt = 500;
  int16_t res;

  while (wt > 0) {
    if (wt & 1)
      tb_board_led_on(0);
    tb_delay_ms(25); // Measure... (faster than time above)
    tb_board_led_off(0);
    wt -= 25;

    if (isrc == SRC_SDI) {
      for (;;) { // Get
        res = tb_getc();
        if (res == -1)
          break;
        if (res <= 0)
          return -1; // Break FOund
                     // else: ignore other than break
      }
    }
  }
  // --- 'm' Wait end

  snprintf(sdi_valio.channel_val[0].txt, 11, "%+f", (float)tb_time_get() / 1.234);
  sdi_valio.channel_val[0].punit = "xtime";
  snprintf(sdi_valio.channel_val[1].txt, 11, "+%u", tb_get_ticks() % 1000000); // '+* only d/f
  sdi_valio.channel_val[1].punit = "cnt";

  if (sdi_valio.measure_arg) {
    float fval;
    saadc_init();
    saadc_setup(0);
    fval = saadc_get_vbat(true, 8); // Calibrate and 8 Averages
    saadc_uninit();

    snprintf(sdi_valio.channel_val[2].txt, 11, "%+.2f", fval); // Only 2
    sdi_valio.channel_val[2].punit = "VSup";
  }
  return 0;
}

//***