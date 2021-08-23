/****************************************************
* 0300_Pewatron_KKD18.c - CeramicPressure Sensor with I2C
* DEVICE_TYP 300
*
* (C) joembedded@gmail.com - joembedded.de
*
* UART is shared with tb_tools
*
* Info: KKD18 is a Pressure Sensor for Water Levels 
* with a Ceramic Membrane. Normally Piezo based sensors
* are used for Water levels, but the Ceramic Sensors are 
* more robust (to cleaning, e.g.), but Resoultion and 
* Accuracy is often not as good as Piezo based sensors.
*
* Connect: 
* - Sensor-Supply(White): I_X0 (Sensor need 1mA active)
* - Sensor-SCL(Green): I_SCL
* - Sensor-SDA(Yellow): I_SDA
* - Sensor-GND(Brown): I_GND
*
* Errors:
* -101 No Reply1
* -102 No Reply2
*
***************************************************/

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "device.h"
#include "nrf_drv_gpiote.h"
#include "saadc.h"
#include "sdi12sensor_drv.h"
#include "osx_main.h"
#include "tb_tools.h"

#include "i2c.h"
#include "intmem.h"
#include "osx_pins.h"

#if DEVICE_TYP != 300
#error "Wrong DEVICE_TYP, select other Source in AplicSensor"
#endif

//--- Local Paramaters for KKD Start
typedef struct {
  float p_min;  // here 0.0 BAR // unused
  float p_max;  // here 2.0 BAR
} KKD_KOEFFS;

typedef struct {
  int16_t err; // 0: OK (=res from fkts)
  float pressure;
  float temperature;
} KKD_VALS;

static KKD_KOEFFS kkd_koeffs;
static KKD_VALS kkd_vals;
//--- Local Paramaters for KKD End

// --------- Locals -------------
/* Parameters can be used e.g. as Offset/Multi per Channel etc...
  // Note: this is GP/TT Standard; First MULTI, then sub OFFSET!
  fval*=param->factor; // Def. 1.0
  fval-=param->offset; // Def. 0.0
*/
#define ANZ_KOEFF 4
typedef struct {
  float koeff[ANZ_KOEFF];
} PARAM;
// Test Setup for Default Koeffs (M O M O)
PARAM param = {{1.0, 0.0, 1.0, 0.0}};

//--- Local Functions for KKD Start
// Important: KKD has internal PullUps and max. FRQ is 100kHz
#define KKD_ADDR 81 // Addr-Raum 7-Bit

int32_t kkd_read_reg(uint8_t reg, int32_t *pval) {
  int32_t err_code, res;
  i2c_uni_txBuffer[0] = 0x40 + reg;
  err_code = i2c_readwrite_blk_wt(KKD_ADDR, 1, 3, 0); // 3 Bytes lesen, 0ms Warten
  if (!err_code) {
    // Erg. 24 Bit LE int24_t
    res = (i2c_uni_rxBuffer[2]) + (i2c_uni_rxBuffer[1] << 8) + (i2c_uni_rxBuffer[0] << 16);
    if (res & (1 << 23))
      res -= (1 << 24); // negative Werte
    *pval = res;
  }
  return err_code;
}

// Returns 0:OK or <0:Error
#define ZP 13107 // from AppNote
#define EP 117965
int16_t kkd_values_get(void) {
  int32_t val;
  int16_t res;
  float fval;

  ltx_i2c_init();

  if (!kkd_read_reg(0, &val)) { // Temp
    fval = (float)val / 4096.0;        // Convert to oC
    kkd_vals.temperature = fval;
    res=0;
  } else
    res = -101; // No Reply
  
  if (!res && !kkd_read_reg(1, &val)) { // Pressure 
    //fval = ((float)val - ZP) / (EP-ZP) * (kkd_koeffs.p_max - kkd_koeffs.p_min) + kkd_koeffs.p_min; // ???
    fval = ((float)val - ZP) / (EP-ZP) * (kkd_koeffs.p_max); //  - kkd_koeffs.p_min unused
    kkd_vals.pressure = fval;
  } else
    res = -102; // No Reply2

  ltx_i2c_uninit(false);       // KKD has int. PU
  kkd_vals.err = res;
  return res;
}
//--- Local Functions for KKD End

//------------------- Implementation -----------
#if DEBUG
// === DEBUG START ===
//====== TEST COMMANDS FOR NEW SENSOR START_A ========
//====== TEST COMMANDS FOR NEW SENSOR END_A ========

// Additional Test-CMDs via TB_UART
// IMPORTANT: only SMALL capitals for User-CMDs!
bool debug_tb_cmdline(uint8_t *pc, uint32_t val){
  switch (*pc) {
  case 'S': // Often useful
    ltx_i2c_scan((bool)val, false); // 0:W,1:R  NoPU */
    break;
  //====== TEST COMMANDS FOR NEW SENSOR START_S ========
  case 'a':
    {
      // ....
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
  sprintf(sensor_id, "TT_K18_A_0300_OSX%08X", mac_addr_l);
  // TT:'TerraTransfer', K18:'Pewatron KKD18 Ceramic', A:'Version 2 Bar Abs'

  // Try to read Parameters
  intpar_mem_read(ID_INTMEM_USER0, sizeof(param), (uint8_t *)&param);

  // KKD18 Fixed Koeffs for "Typ A":
  kkd_koeffs.p_min=0.0;  // unused
  kkd_koeffs.p_max=2.0;
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
    sdi_valio.m_msec = 2500;
  } else if (carg == 1) {
    sdi_valio.anz_channels = 3;
    sdi_valio.m_msec = 2600;
  } else
    return false;

  return true;
}

//The KKD has a WarmUp Time of 2 seconds!!!
#define WAIT_MS 2000
int16_t sensor_valio_measure(uint8_t isrc) {
  //---- 'M': While waiting: scan SDI for <BREAK> ----
  int16_t res;
  float fval;

  // Supply Sensor with Power
  nrf_gpio_cfg(
      IX_X0,
      NRF_GPIO_PIN_DIR_OUTPUT,
      NRF_GPIO_PIN_INPUT_DISCONNECT,
      NRF_GPIO_PIN_NOPULL,
      NRF_GPIO_PIN_S0H1, // High Out
      NRF_GPIO_PIN_NOSENSE);
  nrf_gpio_pin_set(IX_X0);

  res = sensor_wait_break(isrc, WAIT_MS);
  if (res){
    nrf_gpio_cfg_default(IX_X0); // Power OFF
    return res;
  }
  // --- 'm' Wait end

  // Read Data from Sensor
  res = kkd_values_get();
  nrf_gpio_cfg_default(IX_X0); // Power OFF

  // Prepare Output
  sdi_valio.channel_val[0].punit = "Bar";
  if (!res) {
    fval = kkd_vals.pressure;
    fval *= param.koeff[2]; // Def. 1.0
    fval -= param.koeff[3]; // Def. 0.0
  } else {
    fval = -1000 + res; // Error..
  }
  snprintf(sdi_valio.channel_val[0].txt, 11, "%+.5f", fval);

  sdi_valio.channel_val[1].punit = "oC";
  if (!res) {
    fval = kkd_vals.temperature;
    fval *= param.koeff[0]; // Def. 1.0
    fval -= param.koeff[1]; // Def. 0.0
  } else {
    fval = res; // Error..
  }
  snprintf(sdi_valio.channel_val[1].txt, 11, "%+.2f", fval);

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
    sprintf(outrs_buf, "%cK%d=%f", my_sdi_adr, pidx, fval);

  } else if (!strcmp(pc, "Write!")) { // Write SDI_Addr and Koefficients to Memory
    intpar_mem_erase();               // Compact Memory
    intpar_mem_write(ID_INTMEM_SDIADR, 1, (uint8_t *)&my_sdi_adr);
    intpar_mem_write(ID_INTMEM_USER0, sizeof(param), (uint8_t *)&param);
    sprintf(outrs_buf, "%c", my_sdi_adr); // Standard Reply
  } else if (!strcmp(pc, "Sensor!")) {                                                // Identify Senor
    sprintf(outrs_buf, "%cKKD18_A,P=%.1f;%.1f!", my_sdi_adr, kkd_koeffs.p_min, kkd_koeffs.p_max); // Standard Reply
  }                                       // else
}

//***