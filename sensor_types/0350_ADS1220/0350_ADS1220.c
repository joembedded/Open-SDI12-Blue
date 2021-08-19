/****************************************************
* 0350_ADS1220.c - Sensor with universal ADS1220
* DEVICE_TYP 200
*
* (C) joembedded@gmail.com - joembedded.de
*
* UART is shared with tb_tools
*
*
* The ADS1220 has either 4 single-ended or 2 differential
* very flexible inputs plus internal (precise) Temperature.
* The ADS1220 can provide 1 or 2 current sources.
* This all is coded in 4 configuration register (8 Bits),
* that are encoded here in an uint32 (in LE Format)
* See Docu!
*
* Errors:
* 0x80000000: AD Timeout
* 0x80000001: Verify AD Config
* 0x80000002: Wrong ad_physkan
* 0x80000003: Invalid or not enabled ad_physkan
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

#include "nrf_drv_gpiote.h"
#include "nrfx_spim.h"


#if DEVICE_TYP != 350
#error "Wrong DEVICE_TYP, select other Source in AplicSensor"
#endif

// --------- Locals -------------

// ===Some tested configurations Start ===
#define PT100_CONFIG 0x805624E6 // Config for internal Temperature Sensor in LE32
// 0:E6 AIN auf Ref/2 GAIN1 PGA_BYPASS
// 1:24 45SPS NORMAL TS_DISA CONTI=1  BURNOUT_DIS (20 SPS sind zu langsam)
// 2:56 EXT-REF FIR_5060 SW_OPN IDAC IDAC 1mA
// 3:80 IDAC1_AIN3 IDAC2_DIS DRDY_ONLY 0
// Setup fuer 2.00k Ref, PT100 max. 250 Grad: PGA8 AN IDAC1mA
#define PT100_DELAY 50
#define PT100_MITTEL 8
#define PT100_KALI true

#define BRIDGE01RPN_CONFIG 0x58240E // Config for ext. Bridge
  // 0:0E AIN auf A01 GAIN128 PGA_ENA
  // 1:24 45SPS NORMAL TS_ENA CONTI=1  BURNOUT_DIS  (20 SPS sind zu langsam)
  // 2:58 EXT-REF FIR_5060 SW_CLOSE T_OFF BO_OFF
  // 3:00 IDAC1_AIN3 IDAC2_DIS DRDY_ONLY 0
#define BRIDGE01RPN_DELAY 50
#define BRIDGE01RPN_MITTEL 8
#define BRIDGE01RPN_KALI true

#define ITEMP_CONFIG 0x5022E0 // Config for internal Temperature Sensor in LE32
  // 0:E0 AIN auf Ref/2 GAIN1 PGA_BYPASS
  // 1:22 45SPS NORMAL TS_ENA CONTI=0  BURNOUT_DIS  (20 SPS sind zu langsam)
  // 2:50 EXT-REF FIR_5060 SW_OPN T_OFF BO_OFF
  // 3:00 IDAC1_AIN3 IDAC2_DIS DRDY_ONLY 0
  // ftemp=(res/1024)*0.03125; bzw. ftemp=res/32768.0
#define ITEMP_DELAY 0
#define ITEMP_MITTEL 1
#define ITEMP_KALI false
// ===Some tested configurations End ===


#define ANZ_ad_physkan 8   // max. 8 Physical Channels, accesible via M2, M3, .. (max.) M9
typedef struct {
  uint8_t typ;          // 1 How to measure (0: not enabled)
  uint32_t ad_config;   // 4 Config-Registers
  uint16_t delay_ms;    // 2 Some setups (e.g. with IDAC) nees some msec until stable
  uint8_t mittel;       // 8 How many Samples to take? (1 - max 127)
  bool kali_flag;       // 1 Kalibrate AD with internal Short? (needs double time)
  char unit[4];         // 4 For Display, MAX 3 Chars (total size of ad_physkan[]<255!)
  float multi;          // 4 Calibration Params
  float offset;         // 4
  uint16_t true_msec;   // 2 Measure time (measured)
} AD_PHYSKAN;

#define P_TYP_ITEMP 1   // Internal Temperature
#define P_TYP_PT100_A 2  // PT100 via 2k REF and IDAC and Linearisation
#define P_TYP_STD 3     // Standard AD

AD_PHYSKAN ad_physkan[ANZ_ad_physkan]={
  {1, ITEMP_CONFIG, ITEMP_DELAY, ITEMP_MITTEL, ITEMP_KALI, "oC", 1.0, 0.0, 999 /*tbd.*/},
  {2, PT100_CONFIG, PT100_DELAY, PT100_MITTEL, PT100_KALI, "oC", 1.0, 0.0, 999 /*tbd.*/},
  {3, BRIDGE01RPN_CONFIG, BRIDGE01RPN_DELAY, BRIDGE01RPN_MITTEL, BRIDGE01RPN_KALI, "Cnt", 1.0, 0.0, 999 /*tbd.*/},
};

// ---User Part---
/* Parameters can be used e.g. as Offset/Multi per Channel etc...
* This is only a demo to manage 4 FLOAT Parameters
  // Note: this is GP/TT Standard; First MULTI, then sub OFFSET!
  fval*=param->factor; // Def. 1.0
  fval-=param->offset; // Def. 0.0
*/
#define ANZ_KOEFF (ANZ_ad_physkan*2)
typedef struct {
  float koeff[ANZ_KOEFF];
} PARAM;
//USER Parameters, individual Kalibrations!
PARAM param = {{1.0, 0.0, 1.0, 0.0, 1.0, 0.0, 1.0, 0.0, 1.0, 0.0, 1.0, 0.0, 1.0, 0.0, 1.0, 0.0}};


//------------------- Implementation -----------

// ADS1220 4-Channel, 2-kSPS, 24-Bit ADC with Integrated PGA and Reference
#define FPIN_SCK_PIN  IX_SCL   //  30 Clock
#define FPIN_MOSI_PIN IX_X2   //  25 AD:DIN
#define FPIN_MISO_PIN IX_SDA   //  29 AD:Dout
#define FPIN_DRDY_PIN IX_X1   //  27 AD:DRdy
#define FPIN_PWR_PIN IX_X0   //   28 AD:Vcc

#define SPI_INSTANCE  1      // 0:I2C, 1:SPI, 2/3:SPI-Flash CPU 32/40
static const nrfx_spim_t spi = NRFX_SPIM_INSTANCE(SPI_INSTANCE); 
static bool spi_init_flag=false;
static nrfx_spim_config_t spi_config = NRFX_SPIM_DEFAULT_CONFIG;

// 2 Buffers for AD-Data
#define MAX_SPI_IO 8
static uint8_t spi_out[MAX_SPI_IO];
static uint8_t spi_in[MAX_SPI_IO];

static int16_t ad_spi_init(void){
    if(spi_init_flag==true) return 0;  // Already init

    spi_config.frequency      = NRF_SPIM_FREQ_1M; // OK for AD1220
    spi_config.miso_pin       = FPIN_MISO_PIN;
    spi_config.mosi_pin       = FPIN_MOSI_PIN;
    spi_config.sck_pin        = FPIN_SCK_PIN;
    // 'Speical:'
    spi_config.mode =   NRF_SPIM_MODE_1;       //CPOL = 0 (Active High); CPHA = TRAILING (1)
    spi_config.orc = 0x0; // Default TX-Zeichen
    APP_ERROR_CHECK(nrfx_spim_init(&spi, &spi_config, NULL, NULL)); // Use Blocking transfer

    nrf_gpio_cfg_input(FPIN_DRDY_PIN, GPIO_PIN_CNF_PULL_Pullup);  
    // Supply Sensor with Power
    nrf_gpio_cfg(FPIN_PWR_PIN,NRF_GPIO_PIN_DIR_OUTPUT,NRF_GPIO_PIN_INPUT_DISCONNECT,NRF_GPIO_PIN_NOPULL,
      NRF_GPIO_PIN_S0H1 /*High Out*/, NRF_GPIO_PIN_NOSENSE);
    nrf_gpio_pin_set(FPIN_PWR_PIN);
    tb_delay_ms(50);  // Wait until stable
    spi_init_flag=true;
    return 0;
}
static void ad_spi_close(void){
    if(spi_init_flag==false) return;  // Already uninit
    nrfx_spim_uninit(&spi);
    nrf_gpio_cfg_default(FPIN_PWR_PIN); // Power OFF
    spi_init_flag=false;
}

static void ad_spi_read(uint16_t len){ 
    nrfx_spim_xfer_desc_t xfer_desc = NRFX_SPIM_XFER_TRX(NULL, 0, spi_in, len); 
    APP_ERROR_CHECK(nrfx_spim_xfer(&spi, &xfer_desc, 0));
}
static void ad_spi_write(uint16_t len){
    nrfx_spim_xfer_desc_t xfer_desc = NRFX_SPIM_XFER_TRX(spi_out, len, NULL, 0); 
    APP_ERROR_CHECK(nrfx_spim_xfer(&spi, &xfer_desc, 0));
}
// first RESET recommended for ADS1220 after PowerUP
static void ad_reset(void){
  spi_out[0]=0x06;  
  ad_spi_write(1);
  tb_delay_ms(1);
}
// The ADS1220 has 4 8-Bit Config-Registers, here seen as uint32t_LE
// Write/Read Back all Config. Registers as uint32 LE
static uint32_t ad_readback_config(void){
  spi_out[0]=0x23;  // RREG(0-3) Read 4 Registers
  ad_spi_write(1);
  memset(spi_in,0,4);
  ad_spi_read(4);
  return *(uint32_t*)spi_in;
}
static int32_t ad_write_config(uint32_t aconf, bool verify){
  spi_out[0]=0x43;  // WREG(0-3) Write 4 Registers
  ad_spi_write(1);
  *(uint32_t*)spi_out=aconf;
  ad_spi_write(4);
  if(verify){
    if(ad_readback_config()!=aconf) return 0x80000001;  // Verify ERROR
  }
  return 0;
}

/********************************************************
* messen_ad24(): Eine Messung ausloesen. Wenn Param
* ungleich 255: Register0 setzen (mit GAIN und PGA).
* soviele Werte Addieren, liefert *SUMME* der M Werte!
********************************************************/
static int32_t ad24_messen(uint8_t reg0, uint16_t m){
	int32_t res;
	int32_t sum;
        uint16_t dcnt;
        bool skip_flag=false;
	// START
	if(reg0!=255){
          spi_out[0]=0x40;  // WREG ab 0, 1 Bytes folgen
          spi_out[1]=reg0;  
          ad_spi_write(2);
	}
	sum=0;
        spi_out[0]=0x08;  
        ad_spi_write(1); // START CONTI=1  Einmal Starten
        if(m>1){          // If m>1: Skip first measure
          skip_flag=true;
          m++;    
        }
	while(m--){
                dcnt=0; 
		while(nrf_gpio_pin_read(FPIN_DRDY_PIN)) {
                  tb_delay_ms(1);	// Warten bis Messwert
                  dcnt++;
                  if(dcnt==200) return 0x80000000;   // Timeout 200msec, most neg. Number!
                }
                ad_spi_read(3);
                res=spi_in[2]+(spi_in[1]<<8)+(spi_in[0]<<16);
                if(spi_in[0]&128) res|=0xFF000000;  // Ext. Neg. Values-Bit

		if(!skip_flag) sum+=res;
                skip_flag=false;
	}
	return sum;
}

/*********************************************************************
* ad_pt100_wert: Aus dem Ver. PT100 hm eine Temp berechnen.
* Siehe PTXXX-Linearisierung.XLS. 
* Koeffizienten ermittelt fuer 2k GAIN 8 = 2e23
*********************************************************************/
#define pt_koeff_x0		-2.457390E+02
#define pt_koeff_x1		7.022650E-05
#define pt_koeff_x2		8.966090E-13
float ad_pt100_wert(long cnt){
	float sum,fx;
	
	if((cnt<2427000) || (cnt>4910000)) 
          return -99; // Begrenzen bei Schrott (<-70, >120)

	sum=pt_koeff_x0; // -245.7885;
	fx=(float) cnt;
	sum+=pt_koeff_x1*fx; // 23.5726
	fx*=fx;
	sum+=pt_koeff_x2*fx; // 0.1005893
	return sum;
}

// Universal Measure Routine
// delay in ms, mittel <128!
int32_t ad_measure(uint32_t confregs, uint16_t delay, uint16_t mittel,bool kaliflag){
  int32_t res,offset;
  uint8_t reg0;
  if(kaliflag){ // If kalibration: force Ain_p/n to A/ref/2 first
      reg0 = confregs & 0xFF; // Isolate Reg0
      confregs &= ~0xF0;  // And replace by Short
      confregs |= 0xE0;
  }else{
    reg0=255;
    offset=0;
  }
  if(ad_write_config(confregs,true)) return 0x80000001;
  if(delay) tb_delay_ms(delay);

  if(kaliflag){
    offset=ad24_messen(255,mittel);
  }

  res = (ad24_messen(reg0,mittel)-offset)/mittel;
  return res;
}

/* Try to get analog value from physical channel (0..max 8) (equal to M2..(max. M9) */
int32_t get_analog_channel(uint8_t pidx, float *perg){
  AD_PHYSKAN *pkan;
  int32_t res;
  float fres;
  if(pidx>=ANZ_ad_physkan) return 0x80000002; 
  pkan = &ad_physkan[pidx];
  switch(pkan->typ){
  case P_TYP_ITEMP: // 1 Internal Temperature
    res = ad_measure(pkan->ad_config, pkan->delay_ms, pkan->mittel , pkan->kali_flag);
    fres = (float)res / 32768.0;  // IN oC
    break;

  case P_TYP_PT100_A: // 2 PT100 via 2k REF and IDAC and Linearisation
    res = ad_measure(pkan->ad_config, pkan->delay_ms, pkan->mittel , pkan->kali_flag);
    fres = ad_pt100_wert(res);  // Error is -99 oC
    break;

  case P_TYP_STD: // 3 Standard AD
    res = ad_measure(pkan->ad_config, pkan->delay_ms, pkan->mittel , pkan->kali_flag);
    fres = (float)res;  // in Cnts
    break;

  default: 
    return 0x80000003;  // Invalid or not enabled
  }

  fres *= pkan->multi;  // Linearize as Factory
  fres -= pkan->offset; 
  *perg = fres;         // And Write
  return 0; // OK
}


#if DEBUG
// Additional Test-CMDs via TB_UART
// IMPORTANT: only SMALL capitals for User-CMDs!
bool debug_tb_cmdline(uint8_t *pc, uint32_t val){
  switch (*pc) {
  case 'a':
  {
  int32_t res;
  float fval;
  uint32_t t0;
  uint32_t ad_msec;
  
  ad_spi_init();
  ad_reset();

  while(1){
    fval=0;
    t0 = tb_get_ticks();
    res=get_analog_channel(val, &fval);
    if(res<-2147483392) tb_printf("ERR:%x\n",res); 
    else {
      ad_msec=tb_deltaticks_to_ms(t0, tb_get_ticks()); // Without Power ON
      tb_printf("Res:%d (P:%u/Real:%u msec) => %f %s\n",res,ad_physkan[val].true_msec, ad_msec ,fval,ad_physkan[val].unit );
    } 

    if(tb_getc()!=-1) break;
    tb_delay_ms(500);

  }

  ad_spi_close();
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
  // TT:'TerraTransfer', A24:'Analog_Sensor 24 Bit', 'A': ADC1220
  sprintf(sensor_id, "TT_A24_A_0350_OSX%08X", mac_addr_l);

  // Try to read FACTORY KALI Parameters
  intpar_mem_read(ID_INTMEM_USER0+1, sizeof(ad_physkan), (uint8_t *)&ad_physkan);

  // Try to read USER KALI Parameters
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

  snprintf(sdi_valio.channel_val[0].txt, 11, "%+f", (float)tb_time_get() / 3333.3);
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
    sprintf(outrs_buf, "%c", my_sdi_adr);            // Standard Reply
  } else if (!strcmp(pc, "Sensor!")) {               // Identify Senor
    sprintf(outrs_buf, "%cADS1220!", my_sdi_adr); // Standard Reply
  }                                                  // else ..
}

//***