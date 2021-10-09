/****************************************************
* 0360_EplusE_EE08_TrH - Precise Temperature/Humidity Sensor
* DEVICE_TYP 360
*
* (C) joembedded@gmail.com - joembedded.de
*
* UART is shared with tb_tools
*
* The EE08 is a precise Temperature/Humidity Sensor
* A special coating against dust and tensides (used in agriculture)
* makes it ideal for outdoor usage. The interface is I2C-style,
* but only in the kHz range (set I2C_100KHZ_DIV in 'device.h')
*
* I2C_100KHZ_DIV with 5mtr cable, 10k Pullup:
* <10: Not OK, 10:(sometimes)OK 20: Selected (= ca. 5 kHz)
*
* The EE08 needs >= 4.5V! Current is ca. 0.5mA and ca. 1.5mA pulses 
* per measure (ca. each sec updated). 
* Add a switch for Ultra-Low-Power usage! Setup Time: *t.b.d*
* Here: EE08 is constantly powered!
*
* Connect: 
* - Red: 	Supply (>= 4.5V see Datasheet, max. 1.5mA)
* - Green:  X_SCL (use Pullup <= 10k)
* - Brown:  X_SDA (use Pullup <= 10k)
* - Pink:	GND
*
* Errors:
* -999: No Reply
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
 #include "nrf_drv_gpiote.h" // NRF_GPIO_PIN_MAP(x,y)
 #include "i2c.h" // EE08: Own Driver, because SLOW
 #include "nrfx_spim.h"
 #include "nrf_delay.h"
#endif

#if DEVICE_TYP != 360
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
typedef struct {
  float koeff[ANZ_KOEFF];
} PARAM;
// Test Setup for Default Koeffs
PARAM param = {{1.0, 0.0, 1.0, 0.0}};

//------------------- Implementation -----------
typedef union{
  uint8_t bytes[2];
  uint16_t izahl;
} IZAHL;

IZAHL ee08_hum;	// 1. Wert in 0.01% rHum
IZAHL ee08_temp; // 2. Wert in 0.01Grad +27315

/*****************************************************************
* ee08_getreg(reg). Fehler: <0, sonst: Wert: 0.255
* reg ist 8 Bit mit gesetztem Bit0(=W)
*****************************************************************/
int32_t ee08_getreg(uint8_t reg){
      int32_t res;	// Word
      uint8_t comp;	// Byte

      res = i2c_read_blk(reg>>1, 2); // REG -> ADDR|x
      // tb_printf("Reg $%x -> %i\n",reg,res);
      if(res<0) return res; // Hardware Error
      // tb_printf(" %u -> %u %u -> FCS:%u\n",reg,i2c_uni_rxBuffer[0],i2c_uni_rxBuffer[1],i2c_uni_rxBuffer[0]+reg);
      if(((uint8_t)(i2c_uni_rxBuffer[0]+reg))!=i2c_uni_rxBuffer[1]) return -999;  // FCS
      return i2c_uni_rxBuffer[0]; // >=0: OK
}

/*****************************************************************
* ee08_werte(); // >0: OK nac res REPS, sonst Fehler
* 
* Problem 9/2019: Detected some EE08 with strange spikes. Because
* the EE08 FCS is not very useful to detect errors, repeated readout
* was added as a test. 
* Seems to work now, but feedback from EE R&D still missing...
*****************************************************************/
int16_t ee08_werte(void){
	int32_t h;
	uint8_t i;
	ee08_hum.izahl=0xFFFF; 		// FFxx: Kein Wert
	ee08_temp.izahl=0xFFFF; 	// FFxx: Kein Wert
	for(i=0;i<5;i++){               // 5 Tries
                //tb_printf("*");
                if(i) tb_delay_ms(100);         
		h=ee08_getreg(0x11);
		if(h<0 || (h&0xFF)!=8) continue; // Typ 8 MUSS
		h=ee08_getreg(0x81);
		if(h<0) continue;	// Fehler
		ee08_hum.bytes[0]=(uint8_t)h;	// Hum-L (LE CPU)
		h=ee08_getreg(0x91);
		if(h<0) continue;	// Fehler
		ee08_hum.bytes[1]=(uint8_t)h;	// Hum-H (LE CPU)
		h=ee08_getreg(0xA1);
		if(h<0) continue;	// Fehler
		ee08_temp.bytes[0]=(uint8_t)h;	// Temp-L (LE CPU)
		h=ee08_getreg(0xB1);
		if(h<0) continue;	// Fehler
		ee08_temp.bytes[1]=(uint8_t)h;	// Temp-H (LE CPU)
		return i;	// OK 
	}
	return -1;	// Fehler!
}

#if DEBUG
// === DEBUG START ===
//====== TEST COMMANDS FOR NEW SENSOR START_A ========
void messen_ee08_werte(void){
        float fval;
        int16_t res;
	tb_printf("Measure EE08\n");
        res=ee08_werte();
	if(res<0) { 
		tb_printf("EE08 Read Error/No Reply!\n");
		return;
	}

        tb_printf("EE08-T:%u\n",ee08_temp.izahl);
        tb_printf("EE08-H:%u\n",ee08_hum.izahl);

        fval = (ee08_temp.izahl-27315)*0.01;
        tb_printf("T:%f, ",fval);
        fval = ee08_hum.izahl*0.01;
        tb_printf("H:%f\n",fval);
}

//====== TEST COMMANDS FOR NEW SENSOR END_A ========

// Additional Test-CMDs via TB_UART
// IMPORTANT: only SMALL capitals for User-CMDs!
bool debug_tb_cmdline(uint8_t *pc, uint32_t val){
  switch (*pc) {
  case 'S': // Often useful
    ltx_i2c_scan(val, false); // 0:W,1:RW 2:R  NoPU */
    break;
  //====== TEST COMMANDS FOR NEW SENSOR START_S ========
  case 'a':
    {
      ltx_i2c_init();
      messen_ee08_werte();
      ltx_i2c_uninit(false);
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
  // TT:'TerraTransfer', EE8:'EE08', 'A': Ver A
  sprintf(sensor_id, "TT_EE8_A_0360_OSX%08X", mac_addr_l);

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
    sdi_valio.m_msec = 900;
  } else if (carg == 1) {
    sdi_valio.anz_channels = 3;
    sdi_valio.m_msec = 950;
  } else
    return false;

  return true;
}

//The EE08 is WarmUp with Power ON, this is only to scan for Reps
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
  ltx_i2c_init();
  res = ee08_werte();
  ltx_i2c_uninit(false);

  // Prepare Output
  sdi_valio.channel_val[0].punit = "%rH";
  if (res>=0) {
    fval = (float)ee08_hum.izahl*0.01;
    fval *= param.koeff[2]; // Def. 1.0
    fval -= param.koeff[3]; // Def. 0.0
  } else {
    fval = -999; // Error..
  }
  snprintf(sdi_valio.channel_val[0].txt, 11, "%+.1f", fval);

  sdi_valio.channel_val[1].punit = "oC";
  if (res>=0) {
    fval = (float)(ee08_temp.izahl-27315)*0.01;
    fval *= param.koeff[0]; // Def. 1.0
    fval -= param.koeff[1]; // Def. 0.0
  } else {
    fval = -999;
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
    sprintf(sdi_obuf, "%c", my_sdi_adr);            // Standard Reply
  } else if (!strcmp(pc, "Sensor!")) {               // Identify Senor
    sprintf(sdi_obuf, "%cEE08_A!", my_sdi_adr); // Standard Reply
  }                                                  // else ..
}

//***