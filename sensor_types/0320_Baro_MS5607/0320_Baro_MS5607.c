/****************************************************
* 0320_Baro_MS5607.c Barometric Pressure Sensor with I2C
* DEVICE_TYP 320
*
* (C) joembedded@gmail.com - joembedded.de
*
* UART is shared with tb_tools
*
* Info: MS5607-02BA03 - Barometric Pressure Sensor
*
* Connect: 
* - Sensor-Supply(White): I_VCC (Sensor needs <1uA I_q. add a 100nF close to the Baro)
* - Sensor-SCL(Green): I_SCL with <=10k-Pullup to I_VCC(!)
* - Sensor-SDA(Yellow): I_SDA with <=10k-Pullup to I_VCC(!)
* - Sensor-GND(Brown): I_GND
* 
* Errors:
* -101 No Reply1 to Reset
* -102 Coeficient Error
* -103 Read AD Error
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
#include "tb_tools.h"

#include "i2c.h"
#include "intmem.h"
#include "osx_main.h"
#include "osx_pins.h"

#if DEVICE_TYP != 320
#error "Wrong DEVICE_TYP, select other Source in AplicSensor"
#endif

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

//--- Local Paramaters for Baro Start
#define BARO_I2C_ADDR 119
#define CMD_RESET 0x1E    // ADC reset command
#define CMD_ADC_READ 0x00 // ADC read command
#define CMD_ADC_D1 0x48   // ADC D1 conversion 4096
#define CMD_ADC_D2 0x58   // ADC D2 conversion 4096
#define CMD_PROM_RD 0xA0  // Prom read command

typedef struct {
  float fkoeff_1; // SENS Annahme: Wenn 0: Uninitialisiert
  float fkoeff_2; // OFF
  float fkoeff_3; // TCS
  float fkoeff_4; // TCO
  float fkoeff_5; // Tref
  float fkoeff_6; // Tempsens
} BARO_KOEFFS;

typedef struct {
  int16_t err;    // 0: OK (=res from fkts)
  float pressure; // Messwerte umgerechnet in oC und Bar
  float temperature;
  // Raw Values:
  float d1; // AD1 Pressure Unkomp vom AD
  float d2; // AD1 Temp
} BARO_VALS;

static BARO_KOEFFS baro_koeffs;
static BARO_VALS baro_vals;
//--- Local Paramaters for BARO End

//--- Local Functions for BARO Start

// Return 0: OK
int16_t baro_lcmd(uint8_t cmd) {
  int16_t res;
  i2c_uni_txBuffer[0] = cmd;
  res = (int16_t)i2c_write_blk(BARO_I2C_ADDR, 1);
  return res;
}

// Try 5 Times Reset
int16_t baro_reset(void) {
  int16_t res;
  for (uint8_t i = 0; i < 5; i++) {
    res = baro_lcmd(CMD_RESET);
    tb_delay_ms(4);
    if (!res)
      return 0; // Reset was OK
  }
  return -101; // No Reply to Reset
}

// Read 1 Coeficient
int32_t baro_lprom(uint8_t num) {
  int32_t res;
  i2c_uni_txBuffer[0] = (CMD_PROM_RD + (num * 2));
  res = i2c_write_blk(BARO_I2C_ADDR, 1);
  if (res)
    return -1;
  res = i2c_read_blk(BARO_I2C_ADDR, 2);
  if (res < 0)
    return -1; // Rd2 will return LE.16(!)
  // Baro need BE!
  res = (uint16_t)((i2c_uni_rxBuffer[0] << 8) + i2c_uni_rxBuffer[1]); // BE
  return res;
}

// Read ALL Coefficients
int16_t baro_lfkoeffs(void) {
  int32_t cf;

  cf = baro_lprom(1);
  if (cf < 0)
    return -102; // Coeff Error
  baro_koeffs.fkoeff_1 = (float)cf * 65536.0;

  cf = baro_lprom(2);
  if (cf < 0)
    return -102; // Coeff Error
  baro_koeffs.fkoeff_2 = (float)cf * 131072.0;

  cf = baro_lprom(3);
  if (cf < 0)
    return -102; // Coeff Error
  baro_koeffs.fkoeff_3 = (float)cf / 128.0;

  cf = baro_lprom(4);
  if (cf < 0)
    return -102; // Coeff Error
  baro_koeffs.fkoeff_4 = (float)cf / 64.0;

  cf = baro_lprom(5);
  if (cf < 0)
    return -102; // Coeff Error
  baro_koeffs.fkoeff_5 = -(float)cf * 256.0;

  cf = baro_lprom(6);
  if (cf < 0)
    return -102; // Coeff Error
  baro_koeffs.fkoeff_6 = (float)cf / 838860800.0;

  return 0;
}

int16_t baro_getkoeffs(void) {
  uint16_t mode16;
  int16_t res;
  baro_koeffs.fkoeff_1 = 0; // = Not Init

  res = baro_reset();
  if (res)
    return res;
  res = baro_lfkoeffs();
  return res;
}
// Get AD Values (>=0) or <0:Err
int16_t baro_ladc(uint8_t cmd, int32_t *phv) {
  int32_t v;
  if (baro_lcmd(cmd))
    return -1;
  tb_delay_ms(12); // Slightly more than DS..
  if (baro_lcmd(CMD_ADC_READ))
    return -2;
  if (i2c_read_blk(BARO_I2C_ADDR, 3))
    return -3;
  // Convert BE.24 -> LE.32
  v = i2c_uni_rxBuffer[2] + (i2c_uni_rxBuffer[1] << 8) + (i2c_uni_rxBuffer[0] << 16);
  *phv = v;
  return 0;
}
// D vals to Real values
void baro_lcalc(void) {
  float dt;
  float dOFF;
  float dSENS;

  dt = baro_vals.d2 + baro_koeffs.fkoeff_5;
  baro_vals.temperature = 20.0 + dt * baro_koeffs.fkoeff_6;

  dOFF = baro_koeffs.fkoeff_2 + dt * baro_koeffs.fkoeff_4;
  dSENS = baro_koeffs.fkoeff_1 + dt * baro_koeffs.fkoeff_3;
  baro_vals.pressure = ((baro_vals.d1 * dSENS) / 2097152.0 - dOFF) / 3276800000.0; // in BAR
}

int16_t baro_vals_get(void) {
  int16_t res;
  int32_t hv;
  float fval;
  ltx_i2c_init();
  // If necessary get Koeffs
  if (baro_koeffs.fkoeff_1 == 0) {
    res = baro_getkoeffs();
  } else {
    res = baro_reset();
  }

  if (!res) {                         // Get Raw
    res = baro_ladc(CMD_ADC_D1, &hv); // AD1 lesen
    if (!res)
      baro_vals.d1 = (float)hv;
    else
      res = -103;
    if (!res) {
      res = baro_ladc(CMD_ADC_D2, &hv); // AD2 lesen
      if (!res)
        baro_vals.d2 = (float)hv;
      else
        res = -103;
    }
  }
  ltx_i2c_uninit(false);

  if (!res)
    baro_lcalc();
  baro_vals.err = res;
  return (int16_t)res;
}
//--- Local Functions for BARO End

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

  sprintf(sensor_id, "TT_BRM_A_0320_OSX%08X", mac_addr_l);
  // TT:'TerraTransfer' BRM:'Baro MS5607', A:'Version: Range from Sensor'

  // Try to read Parameters
  intpar_mem_read(ID_INTMEM_USER0, sizeof(param), (uint8_t *)&param);

  // (try) to Get MinMax etc.
  ltx_i2c_init();
  baro_getkoeffs();
  ltx_i2c_uninit(false);
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
    sdi_valio.m_msec = 300;
  } else if (carg == 1) {
    sdi_valio.anz_channels = 3;
    sdi_valio.m_msec = 400;
  } else
    return false;

  return true;
}

//The BARO has no significant WarmUp Time, this is only to scan for Reps
#define WAIT_MS 50
int16_t sensor_valio_measure(uint8_t isrc) {
  //---- 'M': While waiting: scan SDI for <BREAK> ----
  int16_t res;
  int32_t val;
  float fval;

  res = sensor_wait_break(isrc, WAIT_MS);
  if (res)
    return res;
  // --- 'm' Wait end

  // Read Data from Sensor
  res = baro_vals_get();

  // Prepare Output
  sdi_valio.channel_val[0].punit = "Bar";
  if (!res) {
    fval = baro_vals.pressure;
    fval *= param.koeff[2]; // Def. 1.0
    fval -= param.koeff[3]; // Def. 0.0
  } else {
    fval = -1000 + res; // Error..
  }
  snprintf(sdi_valio.channel_val[0].txt, 11, "%+.5f", fval);

  sdi_valio.channel_val[1].punit = "oC";
  if (!res) {
    fval = baro_vals.temperature;
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
    sprintf(outrs_buf, "%c", my_sdi_adr);     // Standard Reply
  } else if (!strcmp(pc, "Sensor!")) {        // Identify Senor
    sprintf(outrs_buf, "%cBRM!", my_sdi_adr); // Standard Reply
  }                                           // else
}

//***