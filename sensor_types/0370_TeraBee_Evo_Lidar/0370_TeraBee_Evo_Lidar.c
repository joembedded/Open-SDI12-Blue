/****************************************************
* 0370_TeraBee_Evo_Lidar Distance Sensor with I22
* DEVICE_TYP 370
*
* (C) joembedded@gmail.com - joembedded.de
*
* UART is shared with tb_tools
*
* Info: The TeraRanger Evo 3m - 60m Lidar Sensors 
* are an excellent choice for measuring distances.
* TeraBee is European Company (France) https://www.terabee.com
* This Software was tested with "TeraBee Ranger Evo 15m"
*
* Connect (Pin 1 is "Red"): 
* - Sensor-Supply(Pin 7): Connect to 5V (90-330 mA for Evo15m), 
*   later a Power Swith/Regulator will be added 
*   (Switch ON/OFF via I_X2) for UltraLowPower
* - Sensor-SCL(5): I_SCL
* - Sensor-SDA(4): I_SDA
* - Sensor-GND(3 (and opt. 8)): I_GND
*
* Errors:
* -101 No Reply1 from Sensor
* -102 No Reply2 from Sensor
*
* -200 Target Too close
* -201 Unable to measure Distance
* -299 Target Too far
*
***************************************************/

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "device.h"
//#include "nrf_drv_gpiote.h"
#include "saadc.h"
#include "sdi12sensor_drv.h"
#include "osx_main.h"
#include "tb_tools.h"

#include "i2c.h"
#include "intmem.h"
#include "osx_pins.h"

#if DEVICE_TYP != 370
#error "Wrong DEVICE_TYP, select other Source in AplicSensor"
#endif

//--- Local Parameters for T_EVO Start
typedef struct {
  int16_t err; // 0: OK (=res from fkts)
  float distance; // in mm
} T_EVO_VALS;


static T_EVO_VALS tbe_vals;
//--- Local Paramaters for T_EVO End

// --------- Locals -------------
/* Parameters can be used e.g. as Offset/Multi per Channel etc...
  // Note: this is GP/TT Standard; First MULTI, then sub OFFSET!
  fval*=param->factor; // Def. 1.0
  fval-=param->offset; // Def. 0.0
*/
#define ANZ_KOEFF 2
typedef struct {
  float koeff[ANZ_KOEFF];
} PARAM;
// Test Setup for Default Koeffs (M O)
PARAM param = {{1.0, 0.0}};

//--- Local Functions for T_EVO Start
// Important: T_EVO has no internal PullUps and max. FRQ is 100kHz

//================== T_EVO =================================
// Create a Cyclic Redundancy Checks table used in the "crc8" function
static const uint8_t crc_table[] = {
    0x00, 0x07, 0x0e, 0x09, 0x1c, 0x1b, 0x12, 0x15, 0x38, 0x3f, 0x36, 0x31,
    0x24, 0x23, 0x2a, 0x2d, 0x70, 0x77, 0x7e, 0x79, 0x6c, 0x6b, 0x62, 0x65,
    0x48, 0x4f, 0x46, 0x41, 0x54, 0x53, 0x5a, 0x5d, 0xe0, 0xe7, 0xee, 0xe9,
    0xfc, 0xfb, 0xf2, 0xf5, 0xd8, 0xdf, 0xd6, 0xd1, 0xc4, 0xc3, 0xca, 0xcd,
    0x90, 0x97, 0x9e, 0x99, 0x8c, 0x8b, 0x82, 0x85, 0xa8, 0xaf, 0xa6, 0xa1,
    0xb4, 0xb3, 0xba, 0xbd, 0xc7, 0xc0, 0xc9, 0xce, 0xdb, 0xdc, 0xd5, 0xd2,
    0xff, 0xf8, 0xf1, 0xf6, 0xe3, 0xe4, 0xed, 0xea, 0xb7, 0xb0, 0xb9, 0xbe,
    0xab, 0xac, 0xa5, 0xa2, 0x8f, 0x88, 0x81, 0x86, 0x93, 0x94, 0x9d, 0x9a,
    0x27, 0x20, 0x29, 0x2e, 0x3b, 0x3c, 0x35, 0x32, 0x1f, 0x18, 0x11, 0x16,
    0x03, 0x04, 0x0d, 0x0a, 0x57, 0x50, 0x59, 0x5e, 0x4b, 0x4c, 0x45, 0x42,
    0x6f, 0x68, 0x61, 0x66, 0x73, 0x74, 0x7d, 0x7a, 0x89, 0x8e, 0x87, 0x80,
    0x95, 0x92, 0x9b, 0x9c, 0xb1, 0xb6, 0xbf, 0xb8, 0xad, 0xaa, 0xa3, 0xa4,
    0xf9, 0xfe, 0xf7, 0xf0, 0xe5, 0xe2, 0xeb, 0xec, 0xc1, 0xc6, 0xcf, 0xc8,
    0xdd, 0xda, 0xd3, 0xd4, 0x69, 0x6e, 0x67, 0x60, 0x75, 0x72, 0x7b, 0x7c,
    0x51, 0x56, 0x5f, 0x58, 0x4d, 0x4a, 0x43, 0x44, 0x19, 0x1e, 0x17, 0x10,
    0x05, 0x02, 0x0b, 0x0c, 0x21, 0x26, 0x2f, 0x28, 0x3d, 0x3a, 0x33, 0x34,
    0x4e, 0x49, 0x40, 0x47, 0x52, 0x55, 0x5c, 0x5b, 0x76, 0x71, 0x78, 0x7f,
    0x6a, 0x6d, 0x64, 0x63, 0x3e, 0x39, 0x30, 0x37, 0x22, 0x25, 0x2c, 0x2b,
    0x06, 0x01, 0x08, 0x0f, 0x1a, 0x1d, 0x14, 0x13, 0xae, 0xa9, 0xa0, 0xa7,
    0xb2, 0xb5, 0xbc, 0xbb, 0x96, 0x91, 0x98, 0x9f, 0x8a, 0x8d, 0x84, 0x83,
    0xde, 0xd9, 0xd0, 0xd7, 0xc2, 0xc5, 0xcc, 0xcb, 0xe6, 0xe1, 0xe8, 0xef,
    0xfa, 0xfd, 0xf4, 0xf3
};

/*
 * Brief : Calculate a Cyclic Redundancy Checks of 8 bits
 * Param1 : (*p) pointer to receive buffer
 * Param2 : (len) number of bytes returned by the TeraRanger
 * Return : (crc & 0xFF) checksum calculated locally
 */
uint8_t crc8(uint8_t *p, uint8_t len) {
  uint8_t i;
  uint8_t crc = 0x0;
  while (len--) {
    i = (crc ^ *p++) & 0xFF;
    crc = (crc_table[i] ^ (crc << 8)) & 0xFF;
  }
  return crc & 0xFF;
}

#define T_EVO_ADDR 0x31  
/*******************************************************************************
*
*********************************************************************************/
static int32_t tb_raw_read(void){
    uint16_t         res;
    int16_t         ires;
    uint8_t         tbe_crc;

    i2c_uni_txBuffer[0]=0; // Trigger Reading
    ires = i2c_write_blk(T_EVO_ADDR,1);
    if(ires) return ires;

    tb_delay_ms(1);	// Wait >= 500 usec

    ires= (int16_t)i2c_read_blk(T_EVO_ADDR, 3); 
    if(ires) return ires;
    tbe_crc=crc8(&i2c_uni_rxBuffer[0],2);
    if(tbe_crc!=i2c_uni_rxBuffer[2]) return -ERROR_DATA_CORRUPT; // Wrong CRC

    res=(i2c_uni_rxBuffer[0]<<8)+i2c_uni_rxBuffer[1]; // BE

    return res; // OK: 0..65535
}

//============== Global ======================
// Return NO_ERROR(0) or ERROR
int16_t tb_get_mm(float *perg){
    int32_t res;
    float mm;

    res=tb_raw_read();
    // tb_printf("T_EVO: %d\n",res);

    if(res<0) {   // Error from I2C
      *perg=(float)-101;  
	  return res;	
    }else if(res==0){ // Target Too close
      *perg=(float)-200;  
	  return -200;	
    }else if(res==1){ // Unable to measure Distance
      *perg=(float)-201;  
	  return -201;	
    }else if(res==65535){ // Target Too far
      *perg=(float)-299;  
	  return -299;	
    }else{
      mm= (float)res ; 
      *perg = mm;
	  return 0; // OK
    }
}

// Returns 0:OK or <0:Error
int16_t tbe_values_get(void) {
  int32_t val;
  int16_t res;
  float fval;

  ltx_i2c_init();

  res = tb_get_mm(&tbe_vals.distance);

  ltx_i2c_uninit(false);       // T_EVO has int. PU
  tbe_vals.err = res;
  
  return tbe_vals.err;
}
//--- Local Functions for T_EVO End

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
   ltx_i2c_init();
    for(;;){
      int32_t res;
      res=tb_raw_read();
      tb_printf("T_EVO: %dmm - %02x %02x %02x\n",res, i2c_uni_rxBuffer[0], i2c_uni_rxBuffer[1], i2c_uni_rxBuffer[2]);
      tb_delay_ms(200);
      if(tb_kbhit()>0) break;
    }
    ltx_i2c_uninit(false);
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
  // TT:'TerraTransfer', TBE:'Terabee T_EVO', 'A': T_EVO21a
  sprintf(sensor_id, "TT_TBE_A_0370_OSX%08X", mac_addr_l);

}

bool sensor_valio_input(char cmd, uint8_t carg) {
  sdi_valio.measure_cmd = cmd;
  sdi_valio.measure_arg = carg;
  sdi_valio.channel_val[0].didx = -1; // Assume no Values in D-Buffer

  // For THIS sensor:
  if (cmd != 'M')
    return false; // This sensor only supports M, M1
  if (carg == 0) {
    sdi_valio.anz_channels = 1;
    sdi_valio.m_msec = 300;
  } else if (carg == 1) {
    sdi_valio.anz_channels = 2;
    sdi_valio.m_msec = 400;
  } else
    return false;

  return true;
}

//The T_EVO (if powered constantly) no significant WarmUp Time, this is only to scan for Reps
#define WAIT_MS 50  
int16_t sensor_valio_measure(uint8_t isrc) {
  //---- 'M': While waiting: scan SDI for <BREAK> ----
  int16_t res;
  float fval;

  res = sensor_wait_break(isrc, WAIT_MS);
  if (res){
    return res;
  }
  // --- 'm' Wait end

  // Read Data from Sensor
  res = tbe_values_get();

  // Prepare Output
  sdi_valio.channel_val[0].punit = "mm";
  if (res>=0) {
    fval = tbe_vals.distance;
    fval *= param.koeff[0]; // Def. 1.0
    fval -= param.koeff[1]; // Def. 0.0
  } else {
    fval = res; // Error..
  }
  snprintf(sdi_valio.channel_val[0].txt, 11, "%+.0f", fval);

  if (sdi_valio.measure_arg) {
    snprintf(sdi_valio.channel_val[1].txt, 11, "%+.2f", get_vbat_aio()); // Only 2 digits
    sdi_valio.channel_val[1].punit = "VSup";
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
    sprintf(outrs_buf, "%cT_TB_EVO_A!", my_sdi_adr); // Standard Reply
  }                                       // else
}

//***