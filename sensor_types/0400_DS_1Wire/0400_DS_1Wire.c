/****************************************************
* 0400_DS_1Wire - 1 Wire Dallas
* DEVICE_TYP 400
*
* (C) joembedded@gmail.com - joembedded.de
*
* 
* DS18x20 needs about 1mA during Conversion/SPEE-Write
* and 3.0V-5.5V Supply.
*
***************************************************/

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "device.h"
#include "saadc.h"
#include "sdi12sensor_drv.h"
#include "tb_tools.h"

#include "osx_main.h"
#include "osx_pins.h"

#include "intmem.h"

#if DEBUG
 #include "nrf_drv_gpiote.h"
 #include "i2c.h"
 #include "nrfx_spim.h"
#endif

#include "onewire.h"


#if DEVICE_TYP != 400
#error "Wrong DEVICE_TYP, select other Source in AplicSensor"
#endif

// --------- Locals -------------
/* Parameters can be used e.g. as Offset/Multi per Channel etc...
* This is only a demo to manage 4 FLOAT Parameters
  // Note: this is GP/TT Standard; First MULTI, then sub OFFSET!
  fval*=param->factor; // Def. 1.0
  fval-=param->offset; // Def. 0.0
*/
#define ANZ_KOEFF (MAX_OW_SENS*2)
typedef struct {
  float koeff[ANZ_KOEFF];
} PARAM;
// Test Setup for Default Koeffs
PARAM param; // ={{1.0, 0.0, 1.0, 0..}};

//------------------- Implementation -----------
#if DEBUG
// === DEBUG START ===
//====== TEST COMMANDS FOR NEW SENSOR START_A ========
//====== TEST COMMANDS FOR NEW SENSOR END_A ========

// Additional Test-CMDs via TB_UART
// IMPORTANT: only SMALL capitals for User-CMDs!
bool debug_tb_cmdline(uint8_t *pc, uint32_t val){
  switch (*pc) {
  case 'S': // SCAN Often useful
    {
      // ....
    }
    break;
  //====== TEST COMMANDS FOR NEW SENSOR START_S ========
  case 'a':
    {
      // _bus_pwron(); 
      onewire_open(0);
      test_show_onewire(val);
      onewire_close();
      // _bus_pwroff(); 
    }
  break;
  case 'p':
    {
      uint16_t pee;
      if(val>=ow_anz){
        tb_printf("IDX?\n");
        break;
      }
      while(*pc && *pc!=',') pc++;
      if(*pc++!=','){
        tb_printf("VAL?\n");
        break;
      }
      pee=(uint16_t)atoi(pc); 
      // _bus_pwron(); 
      onewire_open(0);
      onewire_write_spee(ow_addr[val].addr,pee);
      test_show_onewire(2);
      onewire_close();
      // _bus_pwroff(); 
    }
  break;

  //====== TEST COMMANDS FOR NEW SENSOR END_S ========
  default:
    return false;
  }
  return true; // Command processed
}
// === DEBUG END
#endif


void sensor_init(void) {
  // Id has fixed structure, max. 30+'\0'
  // all cccccccc.8 mmmmmm.6 vvv.3 xx..xx.[0-13]
  //  13 JoEmbedd   Testse   OSX   (MAC.l)
  // Set SNO to Low Mac, so same Name as BLE Advertising
  sprintf(sensor_id, "TT_DS_1W_0400_OSX%08X", mac_addr_l);

  for(uint8_t i=0;i<ANZ_KOEFF;i+=2){ // set pairs of (1.0,0.0)
      param.koeff[i]=1.0; // 0.0 by default.
  }

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
#define WAIT_MS 500
int16_t sensor_valio_measure(uint8_t isrc) {
  //---- 'M': While waiting: scan SDI for <BREAK> ----
  int16_t res;

  res = sensor_wait_break(isrc, WAIT_MS);
  if (res)
    return res;

  // --- 'm' Wait end

  // Attention: Maximum allowed number of chars in value is 9, see Specs
  snprintf(sdi_valio.channel_val[0].txt, 11, "%+.1f", (float)tb_time_get() / 3333.3);
  sdi_valio.channel_val[0].punit = "xtime";
  snprintf(sdi_valio.channel_val[1].txt, 11, "+%u", tb_get_ticks() % 1000000); // '+* only d/f
  sdi_valio.channel_val[1].punit = "cnt";

  if (sdi_valio.measure_arg) {
    snprintf(sdi_valio.channel_val[2].txt, 11, "%+.2f", get_vbat_aio()); // Only 2 digits
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

  if (*pc == 'K') { // Kn! or Kn=val!
    pidx = (uint16_t)strtoul(pc + 1, &pc, 0);
    if (pidx >= ANZ_KOEFF)
      return;
    fval = param.koeff[pidx];
    if (*pc == '=') { // Set Koeff
      fval = strtof(pc + 1, &pc);
    }
    if (*pc != '!')  return;
    // Send Koeffs
    param.koeff[pidx] = fval;
    sprintf(sdi_obuf, "%cK%d=%f", my_sdi_adr, pidx, fval);

  } else if (!strcmp(pc, "Write!")) { // Write SDI_Addr and Koefficients to Memory
    intpar_mem_erase();               // Compact Memory
    intpar_mem_write(ID_INTMEM_USER0, sizeof(param), (uint8_t *)&param);
    intpar_mem_write(ID_INTMEM_SDIADR, 1, (uint8_t *)&my_sdi_adr);
    sprintf(sdi_obuf, "%c", my_sdi_adr);            // Standard Reply
  } else if (!strcmp(pc, "Sensor!")) {               // Identify Senor
    sprintf(sdi_obuf, "%cDS_1Wire!", my_sdi_adr); // Standard Reply
  }                                                  // else ..
}

//***