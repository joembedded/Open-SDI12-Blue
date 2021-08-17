/****************************************************
* 0340_Sensirion_SHT2x    Temperature/Humidity Sensor with I2C
* DEVICE_TYP 340
*
* (C) joembedded@gmail.com - joembedded.de
*
* UART is shared with tb_tools
*
* Info: The SHT21 is not the lastest generation, 
* but still very common. Because Open-SDI12-Blue supports individual
* Coefficients, it is possible to calibrate the sensor!
*
* Connect: 
* - Sensor-Supply(White): I_VCC (Sensor needs <0.15uA I_q. add a 100nF close to the SHT)
* - Sensor-SCL(Green): I_SCL with <=10k-Pullup to I_VCC(!)
* - Sensor-SDA(Yellow): I_SDA with <=10k-Pullup to I_VCC(!)
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
//#include "nrf_drv_gpiote.h"
#include "saadc.h"
#include "sdi12sensor_drv.h"
#include "osx_main.h"
#include "tb_tools.h"

#include "i2c.h"
#include "intmem.h"
#include "osx_pins.h"

#if DEVICE_TYP != 340
#error "Wrong DEVICE_TYP, select other Source in AplicSensor"
#endif

//--- Local Parameters for SHT Start
typedef struct {
  int16_t err; // 0: OK (=res from fkts)
  float humidity;
  float temperature;
} SHT_VALS;


static SHT_VALS sht_vals;
//--- Local Paramaters for SHT End

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

//--- Local Functions for SHT Start
// Important: SHT has no internal PullUps and max. FRQ is 100kHz
#define POLYNOMIAL   0x31  //P(x)=x^8+x^5+x^4+1 = (1)00110001 = 0x(1)31
/*******************************************************************************
* sht_crc(): 1 Byte zur CRC dazu. CRC startet mit 
* SHT2x:0, SHT3xDIS;0xFF muss am Ende identisch zur
* gelesenen crc sein. Running CRC wird jeweils uebergeben
*******************************************************************************/
static uint8_t sht_proc_crc(uint8_t sht_crc, uint8_t nbyte){
    uint8_t i;
    //calculates 8-Bit checksum with given polynomial
    sht_crc^=nbyte;
    for(i=8;i;i--){ // 8 Bits
        if (sht_crc & 0x80) {
            sht_crc = (sht_crc << 1) ^ POLYNOMIAL;
        }else sht_crc = (sht_crc << 1);
    }
    return sht_crc;
}

//================== SHT2x =================================
#define SHT2X_ADDR 0x40   // Ohne RW-Bit (7-Bit-Addresse)
#define SHT2X_CMD_MTEMP 0xF3 //0b11110011 //Messen Temp
#define SHT2X_CMD_MRRELHUM 0xF5 //0b11110101 //Messen RH
/*******************************************************************************
* Anscheinend laeuft der SHT21 mit 400kHz und auch ohne das 20usec-WAIT am Ende..
* Fehler:
* ERROR_HW_SEND_CMD
* ERROR_HW_READ_REPLY
* ERROR_DATA_CRC  // CRC alsch
* OK: >=0
*
* Umrechnungen SHT21
* RH=-6+125*S_rh/65536
* T=-46.85*175.72*S_t/65536
*
*********************************************************************************/
static int32_t sht2x_read(uint8_t cmd, uint16_t wt){
    uint16_t         res;
    int16_t         ires;
    uint8_t         sht_crc;

    i2c_uni_txBuffer[0]=cmd; // SHT2X_CMD_MTEMP; -> TEMP messen
    ires = i2c_write_blk(SHT2X_ADDR,1);
    if(ires) return ires;

    tb_delay_ms(wt);

    ires= (int16_t)i2c_read_blk(SHT2X_ADDR, 3); // HB nicht noetig
    if(ires) return ires;

    res=(i2c_uni_rxBuffer[0]<<8)+i2c_uni_rxBuffer[1]; // BE
    sht_crc=sht_proc_crc(0,i2c_uni_rxBuffer[0]);
    sht_crc=sht_proc_crc(sht_crc,i2c_uni_rxBuffer[1]);
    if(sht_crc!=i2c_uni_rxBuffer[2]) return -ERROR_DATA_CORRUPT; // Wrong CRC

    return res; // OK: 0..65535
}

//============== Global ======================
// Return NO_ERROR(0) or <0 ERROR
int16_t sht2x_get_temperature(float *perg){
    int32_t res;

    res=sht2x_read(SHT2X_CMD_MTEMP,86);
    // tb_printf("SHT-T: %d\n",res);
    if(res<0) {	
      *perg=(float)-101;
	  return res;	
    }else{
      *perg =(-46.85+(175.72/65536.0)*(float)res); // T=-46.85*175.72*S_t/65536 in °C
	  return 0; // OK
    }
}
// Return NO_ERROR(0) or ERROR
int16_t sht2x_get_humidity(float *perg){
    int32_t res;
    float humf;

    res=sht2x_read(SHT2X_CMD_MRRELHUM,30);
    // tb_printf("SHT-H: %d\n",res);

    if(res<0) {	
      *perg=(float)-101;
	  return res;	
    }else{
      humf= (-6.0+(125.0/65536.0)*(float)res) ; // RH=-6+125*S_rh/65536 in %rH
      if(humf>100) humf=100; // SHT2x need Clipping
      else if(humf<0) humf=0;
      *perg = humf;
	  return 0; // OK
    }
}

// Returns 0:OK or <0:Error
int16_t sht_values_get(void) {
  int32_t val;
  int16_t res_t,res_h;
  float fval;

  ltx_i2c_init();

  res_t = sht2x_get_temperature(&sht_vals.temperature);
  res_h = sht2x_get_humidity(&sht_vals.humidity);

  ltx_i2c_uninit(false);       // SHT has int. PU
  sht_vals.err = res_t?res_t:res_h;
  
  return sht_vals.err;
}
//--- Local Functions for SHT End

//------------------- Implementation -----------
void sensor_init(void) {
  // Id has fixed structure, max. 30+'\0'
  // all cccccccc.8 mmmmmm.6 vvv.3 xx..xx.[0-13]
  //  13 JoEmbedd   Testse   OSX   (MAC.l)
  // Set SNO to Low Mac, so same Name as BLE Advertising
  // TT:'TerraTransfer', SHT:'Sensirion SHT', 'A': SHT21a
  sprintf(sensor_id, "TT_SHT_A_0340_OSX%08X", mac_addr_l);

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

//The SHTs no significant WarmUp Time, this is only to scan for Reps
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
  res = sht_values_get();

  // Prepare Output
  sdi_valio.channel_val[0].punit = "%rH";
  if (!res) {
    fval = sht_vals.humidity;
    fval *= param.koeff[2]; // Def. 1.0
    fval -= param.koeff[3]; // Def. 0.0
  } else {
    fval = -1000 + res; // Error..
  }
  snprintf(sdi_valio.channel_val[0].txt, 11, "%+.1f", fval);

  sdi_valio.channel_val[1].punit = "oC";
  if (!res) {
    fval = sht_vals.temperature;
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
    if (pidx > ANZ_KOEFF)
      return;
    if (*pc == '=') { // Set Koeff
      fval = strtof(pc + 1, &pc);
      param.koeff[pidx] = fval;
    }
    if (*pc != '!')
      return;
    // Send Koeffs
    sprintf(outrs_buf, "%cK%d=%f", my_sdi_adr, pidx, param.koeff[pidx]);
  } else if (!strcmp(pc, "Write!")) { // Write SDI_Addr and Koefficients to Memory
    intpar_mem_erase();               // Compact Memory
    intpar_mem_write(ID_INTMEM_SDIADR, 1, (uint8_t *)&my_sdi_adr);
    intpar_mem_write(ID_INTMEM_USER0, sizeof(param), (uint8_t *)&param);
    sprintf(outrs_buf, "%c", my_sdi_adr); // Standard Reply
  } else if (!strcmp(pc, "Sensor!")) {                                                // Identify Senor
    sprintf(outrs_buf, "%cSHT21_A!", my_sdi_adr); // Standard Reply
  }                                       // else
}

//***