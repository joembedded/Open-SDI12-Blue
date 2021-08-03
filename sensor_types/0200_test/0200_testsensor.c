/****************************************************
* 0200_testsensor.c - Test Sensor SDI12-Basics
* DEVICE_TYP 200
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

#if DEVICE_TYP != 200
 #error "Wrong DEVICE_TYP, select other Source in AplicSensor"
#endif

// --------- Locals -------------
/* Parameters can be used e.g. as Offset/Multi per Channel etc...
* This is only a demo to manage 4 FLOAT Parameters
  // Note: this is GP/TT Standard; First MULTI, then sub OFFSET!
  fval*=param->factor; // Def. 1.0
  fval-=param->offset; // Def. 0.0
*/
#define ANZ_KOEFF 4
typedef struct{
  float koeff[ANZ_KOEFF];
} PARAM;
// Test Setup for Default Koeffs
PARAM param ={{1.0, 0.0, 1.001, 0.22}};


//------------------- Implementation -----------
void sensor_init(void) {
  // Id has fixed structure, max. 30+'\0'
  // all cccccccc.8 mmmmmm.6 vvv.3 xx..xx.[0-13]
  //  13 JoEmbedd   Testse   OSX   (MAC.l)
  // Set SNO to Low Mac, so same Name as BLE Advertising
  sprintf(sensor_id, "JoEmTest_0200_OSX%08X", mac_addr_l);

  // Try to read Parameters
  intpar_mem_read(ID_INTMEM_USER0, sizeof(param), (uint8_t *)&param);


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
          return -1; // Break Found
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
    snprintf(sdi_valio.channel_val[2].txt, 11, "%+.2f", get_vbat_aio() ); // Only 2 digits
    sdi_valio.channel_val[2].punit = "VSup";
  }
  return 0;
}

// 'X'; Additional SDI12 - User Commands points to 1.st char after 'X'
// Add here: 
// - Sensor specific Parameters Setup
// - I/O
// etc..
void sensor_valio_xcmd(uint8_t isrc, char *pc) {
  uint16_t pidx;
  float fval;

  if(*pc=='K'){   // Kn! or Kn=val!
    pidx=(uint16_t)strtoul(pc+1,&pc,0);
    if(pidx>ANZ_KOEFF) return;
    if(*pc=='='){  // Set Koeff
      fval=strtof(pc+1,&pc);
      param.koeff[pidx]=fval;
    }
    if(*pc!='!') return;  
    // Send Koeffs
    sprintf(outrs_buf, "%cK%d=%f", my_sdi_adr, pidx,param.koeff[pidx]);
  }else if(!strcmp(pc,"Write!")){  // Write SDI_Addr and Koefficients to Memory
       intpar_mem_erase();  // Compact Memory
       intpar_mem_write(ID_INTMEM_SDIADR, 1, (uint8_t *)&my_sdi_adr);
       intpar_mem_write(ID_INTMEM_USER0, sizeof(param), (uint8_t *)&param);
       sprintf(outrs_buf, "%c", my_sdi_adr);  // Standard Reply
  } // else 
}


//***