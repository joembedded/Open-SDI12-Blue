/****************************************************
* 0310_KELLER_LD.c - Piezo Pressure Sensor with I2C
* DEVICE_TYP 310
*
* (C) joembedded@gmail.com - joembedded.de
*
* UART is shared with tb_tools
*
* Info: LD is a Series of High Precision Pressure Sensor 
* for Water Levels from KELLER AG
*
* Connect: 
* - Sensor-Supply(White): I_VCC (Sensor needs <1uA I_q)
* - Sensor-SCL(Green): I_SCL with 10k-Pullup to I_VCC(!)
* - Sensor-SDA(Yellow): I_SDA with 10k-Pullup to I_VCC(!)
* - Sensor-GND(Brown): I_GND
*
* Important: LD Sensor can not use repeated 
* Start Condition ( i2c_readwrite_blk_wt())
* Either poll status flag or wait ( = less noise on signal lines)
*
* Koefficients: LD stores Float Koefficients in IEEE BE.32 Format,
* nRF52 in IEEE LE.32 Format
* e.g 10.0 is $41 20 00 00 on LD and $00 00 20 41 on nRF52
*
* Errors:
* -101 No Reply1
* -102 No Reply2
* -103 No Reply3/Timout
* -104 Still Busy?
* -105 Memory Error
* -106 No Reply4/Koeffs
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

#include "osx_main.h"
#include "i2c.h"
#include "intmem.h"
#include "osx_pins.h"

#if DEVICE_TYP != 310
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

//--- Local Paramaters for LD Start
#define LD_I2C_ADDR 64
typedef union {
  uint8_t bytes[4];
  float fval;
} BE_FLOAT32;

typedef struct {
  bool init_flag;
  uint8_t mode; // 0:PR 1:PA 2:PAA (3:Aux >3:???)
  float p_min;  // From Sensor (P 16384)
  float p_max;  // (P 49152)
  float delta;  // Calculated
  float offset;
} LD_KOEFFS;

typedef struct {
  int16_t err; // 0: OK (=res from fkts)
  float pressure;
  float temperature;
} LD_VALS;

static LD_KOEFFS ld_koeffs;
static LD_VALS ld_vals;
//--- Local Paramaters for LD End

//--- Local Functions for LD Start
// Get Single 16-Bit BE Koefficient
static int16_t ld_koeff16_get(uint8_t idx, uint8_t *pk_le) { // I2C init!
  int32_t res;
  i2c_uni_txBuffer[0] = idx;
  res = i2c_write_blk(LD_I2C_ADDR, 1);
  if (!res) {
    tb_delay_ms(1); // Memory Access needs 0.5 msec
    if (!res)
      res = i2c_read_blk(LD_I2C_ADDR, 3);
    if (!res) {
      if (i2c_uni_rxBuffer[0] & 4)
        res = -105; // Status Memory Error
      else {
        *pk_le++ = i2c_uni_rxBuffer[2];
        *pk_le = i2c_uni_rxBuffer[1];
      }
    }
  } else
    res = -102; // No Reply2
  return (int16_t)res;
}
// Get Float Koeff from LD
static int16_t ld_getfloat(uint8_t idx, float *pf) {
  BE_FLOAT32 hf;
  int16_t res;
  res = ld_koeff16_get(idx + 1, &hf.bytes[0]);
  if (res)
    return res;
  res = ld_koeff16_get(idx, &hf.bytes[2]);
  if (res)
    return res;
  *pf = hf.fval;
  return 0;
}

// Init All req. Koeffs.
int16_t ld_getkoeffs(void) {
  uint16_t mode16;
  int16_t res;
  ld_koeffs.init_flag = false;
  ld_koeffs.p_min=-106; // Unknown
  ld_koeffs.p_max=-106; // Unknown
  ld_koeffs.mode=255; // Not Set

  res = ld_koeff16_get(0x12, (uint8_t *)&mode16); // Mode
  if (res)
    return res;
  ld_koeffs.mode = mode16 & 3; // PR PA PAA Aux
  res = ld_getfloat(0x13, &ld_koeffs.p_min);
  if (res)
    return res;
  res = ld_getfloat(0x15, &ld_koeffs.p_max);
  if (res)
    return res;
  // Calculate Linearization Coeffs:
  ld_koeffs.delta = (ld_koeffs.p_max - ld_koeffs.p_min) / 32768.0;
  ld_koeffs.offset = ld_koeffs.p_min - ld_koeffs.delta * 16384.0;
  ld_koeffs.init_flag = true;
  return 0;
}

#define MODE_POLL     // if defined: (slightly) faster, but more noise on signal lines
#define WAIT_MS_MAX 8 // Datasheet:8 msec, own measures: ca. 5 msec
// Returns 0:OK or <0:Error
int16_t ld_values_get(void) {
  int32_t res;
  float fval;
  ltx_i2c_init();
  // If necessary get Koeffs
  if (ld_koeffs.init_flag == false) {
    res = ld_getkoeffs();
  } else
    res = 0;

  if (!res) { // Koeff valid
    i2c_uni_txBuffer[0] = 0xAC;
    res = i2c_write_blk(LD_I2C_ADDR, 1);
    if (!res) {
#ifdef MODE_POLL
      int16_t wt_ms;
      wt_ms = WAIT_MS_MAX;
      for (;;) {
        tb_delay_ms(1);
        wt_ms -= 1;
        res = i2c_read_blk(LD_I2C_ADDR, 1); // Return: 1.st Byte!
        if (res < 0) {
          break;
        }
        if (!(i2c_uni_rxBuffer[0] & 32)) { // Status Busy
          res = 0;
          break; // Value Ready
        }
        if (wt_ms <= 0) {
          res = -103; // No Reply3/Timout
          break;
        }
      }
#else
      tb_delay_ms(WAIT_MS_MAX);
      if (!res)
        res = i2c_read_blk(LD_I2C_ADDR, 5);
#endif
      res = i2c_read_blk(LD_I2C_ADDR, 5);
    } else
      res = -101; // No Reply1
    uint8_t i;

    if (!res) { // Now 5 Bytes S PH:PL TH:TL are ready in i2c_uni_rxBuffer
      if (i2c_uni_rxBuffer[0] & 32)
        res = -104; // Still Busy?
      else {
        // Calculate Values
        fval = (float)(i2c_uni_rxBuffer[2] + (i2c_uni_rxBuffer[1] << 8)); // Pressure Raw
        ld_vals.pressure = fval * ld_koeffs.delta + ld_koeffs.offset;
        fval = (float)(i2c_uni_rxBuffer[4] + (i2c_uni_rxBuffer[3] << 8)); // Tempemp Raw
        ld_vals.temperature = fval * 0.003125 - 51.2;
      }
    }
  } // Koeff valid

  ltx_i2c_uninit(false);
  ld_vals.err = res;
  return (int16_t)res;
}
//--- Local Functions for LD End

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
    ltx_i2c_scan(val, false); // 0:W,1:R  NoPU */
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
  sprintf(sensor_id, "TT_KLD_A_0310_OSX%08X", mac_addr_l);
  // TT:'TerraTransfer' KLD:'Keller LD', A:'Version: Range from Sensor'

  // Try to read Parameters
  intpar_mem_read(ID_INTMEM_USER0, sizeof(param), (uint8_t *)&param);

  // (try) to Get MinMax etc.
  ltx_i2c_init();
  ld_getkoeffs();
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

//The LD has no significant WarmUp Time, this is only to scan for Reps
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
  res = ld_values_get();

  // Prepare Output
  sdi_valio.channel_val[0].punit = "Bar";
  if (!res) {
    fval = ld_vals.pressure;
    fval *= param.koeff[0]; // Def. 1.0
    fval -= param.koeff[1]; // Def. 0.0
  } else {
    fval = -1000 + res; // Error..
  }
  snprintf(sdi_valio.channel_val[0].txt, 11, "%+.5f", fval);

  sdi_valio.channel_val[1].punit = "oC";
  if (!res) {
    fval = ld_vals.temperature;
    fval *= param.koeff[2]; // Def. 1.0
    fval -= param.koeff[3]; // Def. 0.0
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
    sprintf(sdi_obuf, "%cK%d=%f", my_sdi_adr, pidx, fval);

  } else if (!strcmp(pc, "Write!")) { // Write SDI_Addr and Koefficients to Memory
    intpar_mem_erase();               // Compact Memory
    intpar_mem_write(ID_INTMEM_SDIADR, 1, (uint8_t *)&my_sdi_adr);
    intpar_mem_write(ID_INTMEM_USER0, sizeof(param), (uint8_t *)&param);
    sprintf(sdi_obuf, "%c", my_sdi_adr);                                             // Standard Reply
  } else if (!strcmp(pc, "Sensor!")) {                                                // Identify Senor

    sprintf(sdi_obuf, "%cLD,P:%.1f;%.1f", my_sdi_adr, ld_koeffs.p_min, ld_koeffs.p_max); // Standard Reply
    switch (ld_koeffs.mode) {
    case 0:
      strcat(sdi_obuf, ",PR!");
      break;
    case 1:
      strcat(sdi_obuf, ",PA!");
      break;
    case 2:
      strcat(sdi_obuf, ",PAA!");
      break;
    // All other: Unknown
    default:
        strcat(sdi_obuf, "!");
    }
  } // else
}

//***