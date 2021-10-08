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
* This Sensor is very flexible, but (could be) quite difficult for the setup.
* We use it for;
* - Very precise PT100 measures (-70..+120 oC), Precision <0.1 oC over complete range!
* - 0/4-20 mA
* - Bridge sensors (load/pressure cells)
* - (Solar) Radiation sensors
* - ...
*
* *todo* make Factory Parameters ad_physkan[] editable/NVM
*
* (internal) Errors:
* // 0x80000000 - 0x800000FF ('xxx < -2147483392' as int32) reserved for Error codes
* 0x80000000: AD Timeout
* 0x80000001: Verify AD Config
* 0x80000002: Wrong ad_physkan
* 0x80000003: Invalid or not enabled ad_physkan
*
* (external) Errors:
* -99 for PT100: OutOfRange or Sensor broken
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
#define PT100_CONFIG 0x80562406 // Config for internal Temperature Sensor in LE32
// 0:06 AIN0/AIN1 auf Ref/2 GAIN1 PGA_BYPASS
// 1:24 45SPS NORMAL TS_DISA CONTI=1  BURNOUT_DIS (20 SPS sind zu langsam)
// 2:56 EXT-REF FIR_5060 SW_OPN IDAC IDAC 1mA
// 3:80 IDAC1_AIN3 IDAC2_DIS DRDY_ONLY 0
// Setup fuer 2.00k Ref, PT100 max. 250 Grad: PGA8 AN IDAC1mA
#define PT100_DELAY 50
#define PT100_MITTEL 8
#define PT100_KALI true
#define PT100_TIME 470  // msec measured 453

#define BRIDGE01RPN_CONFIG 0x50240E // Config for ext. Bridge
  // 0:0E AIN auf A01 GAIN128 PGA_ENA
  // 1:24 45SPS NORMAL TS_ENA CONTI=1  BURNOUT_DIS  (20 SPS sind zu langsam)
  // 2:50 EXT-REF FIR_5060 SW_OPEN T_OFF BO_OFF
  // 3:00 IDAC1_AIN3 IDAC2_DIS DRDY_ONLY 0
#define BRIDGE01RPN_DELAY 50
#define BRIDGE01RPN_MITTEL 8
#define BRIDGE01RPN_KALI true

#define ITEMP_CONFIG 0x5022E0 // Config for internal Temperature Sensor in LE32
  // 0:E0 AIN auf Ref/2 GAIN1 PGA_BYPASS
  // 1:22 45SPS NORMAL TS_ENA CONTI=0  BURNOUT_DIS  (20 SPS sind zu langsam)
  // 2:50 EXT-REF(egal= FIR_5060 SW_OPN T_OFF BO_OFF
  // 3:00 IDAC1_AIN3 IDAC2_DIS DRDY_ONLY 0
  // ftemp=(res/1024)*0.03125; bzw. ftemp=res/32768.0
#define ITEMP_DELAY 0
#define ITEMP_MITTEL 1
#define ITEMP_KALI false
#define ITEMP_TIME 30 // measured 22 msec

#define SE_CONFIG 0x102481 // Config for SingleEnded chan 0
  // 0:81 AIN auf A0 GAIN1 PGA_DISA (Channel 0-3: 81-B1)
  // 1:24 45SPS NORMAL TS_ENA CONTI=1  BURNOUT_DIS  (20 SPS sind zu langsam)
  // 2:10 Int-REF FIR_5060 SW_OPEN T_OFF BO_OFF
  // 3:00 IDAC1_AIN3 IDAC2_DIS DRDY_ONLY 0
#define SE_DELAY 50
#define SE_MITTEL 4
#define SE_KALI true
#define SE_MULTI_G1 2.4414e-04 // Info: 2^3 Cnts equal 2048 mV for Gain 1
#define SE_TIME 290 // measured 275 msec

#define DE_CONFIG 0x10240E // Config for Differential 
  // 0:0E AIN auf A0 GAIN128 PGA_ENA (Channel 0-3: 81-B1) A01:0E, A02:1E, A23:5E
  // 1:24 45SPS NORMAL TS_ENA CONTI=1  BURNOUT_DIS  (20 SPS sind zu langsam)
  // 2:10 Int-REF FIR_5060 SW_OPEN T_OFF BO_OFF
  // 3:00 IDAC1_AIN3 IDAC2_DIS DRDY_ONLY 0
#define DE_DELAY 50
#define DE_MITTEL 8
#define DE_KALI true
#define DE_MULTI_G1 1.907e-06 // Info: 2^3 Cnts equal 16 mV for Gain 128
#define DE_TIME 470  // msec measured 453

// ===Some tested configurations End ===


#define ANZ_AD_PHYSKAN 8   // max. 8 Physical Channels, accesible via M2, M3, .. (max.) M9
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

#define P_TYP_ITEMP 1   // Internal Temperature, Res: oC
#define P_TYP_PT100_A 2  // PT100 via 2k REF and IDAC and Linearisation, Res: oC
#define P_TYP_STD 3     // Standard AD, Res; mV od CNTs

AD_PHYSKAN ad_physkan[ANZ_AD_PHYSKAN]={
/*0*/  {1, ITEMP_CONFIG, ITEMP_DELAY, ITEMP_MITTEL, ITEMP_KALI, "oC", 1.0, 0.0, ITEMP_TIME},
/*1*/  {2, PT100_CONFIG, PT100_DELAY, PT100_MITTEL, PT100_KALI, "oC", 1.0, 0.0, PT100_TIME},

/*2*/  {3, SE_CONFIG+0x00, SE_DELAY, SE_MITTEL, SE_KALI, "mV", SE_MULTI_G1, 0.0, SE_TIME},
/*3*/  {3, SE_CONFIG+0x10, SE_DELAY, SE_MITTEL, SE_KALI, "mV", SE_MULTI_G1, 0.0, SE_TIME},
/*4*/  {3, SE_CONFIG+0x20, SE_DELAY, SE_MITTEL, SE_KALI, "mV", SE_MULTI_G1, 0.0, SE_TIME},
/*5*/  {3, SE_CONFIG+0x30, SE_DELAY, SE_MITTEL, SE_KALI, "mV", SE_MULTI_G1, 0.0, SE_TIME},

/*6*/  {3, DE_CONFIG+0x00, DE_DELAY, DE_MITTEL, DE_KALI, "mV", DE_MULTI_G1, 0.0, DE_TIME},
/*7*/  {3, DE_CONFIG+0x50, DE_DELAY, DE_MITTEL, DE_KALI, "mV", DE_MULTI_G1, 0.0, DE_TIME},

// not important /*x*/  {3, DE_CONFIG+0x10, DE_DELAY, DE_MITTEL, DE_KALI, "mV", 1.0, 0.0, 999 /*tbd.*/},
// not important /*x*/  {3, BRIDGE01RPN_CONFIG, BRIDGE01RPN_DELAY, BRIDGE01RPN_MITTEL, BRIDGE01RPN_KALI, "Cnt", 1.0, 0.0, 999 /*tbd.*/},
};

// ---User Part---
/* Parameters can be used e.g. as Offset/Multi per Channel etc...
* This is only a demo to manage 4 FLOAT Parameters
  // Note: this is GP/TT Standard; First MULTI, then sub OFFSET!
  fval*=param->factor; // Def. 1.0
  fval-=param->offset; // Def. 0.0
*/
#define ANZ_KOEFF (ANZ_AD_PHYSKAN*2)
#define MAX_IUNIT 8
typedef struct {
  float koeff[ANZ_KOEFF]; // 2 Koeffs for each cahannel
  uint8_t m0_mask;  // Set One Bit for each Phys Channel (Bit 0 equals 'M2')
  uint8_t precision[ANZ_AD_PHYSKAN];  // No. of Digits 0..9 (only 0-6,9 relevant)
  struct{
    char s[MAX_IUNIT+1];      // Use only if LEN>0
  } ind_unit[ANZ_AD_PHYSKAN];
} PARAM;

// 10 Formats for sprintf
const char *s_formats[10]={"%+.0f","%+.1f","%+.2f","%+.3f","%+.4f","%+.5f","%+.6f","%+f","%+f","%+f"};

//USER Parameters, individual Kalibrations!
PARAM param = {{1.0, 0.0, 1.0, 0.0, 1.0, 0.0, 1.0, 0.0, 1.0, 0.0, 1.0, 0.0, 1.0, 0.0, 1.0, 0.0},
                79, // 0x4F 01001111: Measure iTemp, pt100, SE A0, SE A1, DE A1/A0
               {2,3,9,9,9,9,9,9},
               {"oC_i","oC_pt","","","","","",""},
              };


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

#define WAIT_ADINIT 50  // 50 msec
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
    tb_delay_ms(WAIT_ADINIT);  // Wait until stable
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
  if(pidx>=ANZ_AD_PHYSKAN) return 0x80000002; 
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
    if(res<-2147483392) tb_printf("ERR:%x\n",res); // '<0x80000100'
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

  // *todo* Try to read FACTORY KALI Parameters
  // *todo* intpar_mem_read(ID_INTMEM_USER0+1, sizeof(ad_physkan), (uint8_t *)&ad_physkan);

  // Try to read USER KALI Parameters
  intpar_mem_read(ID_INTMEM_USER0, sizeof(param), (uint8_t *)&param);

}

#define WAIT_MS 1 // At least 1 msec to Scan for REPS

bool sensor_valio_input(char cmd, uint8_t carg) {
  sdi_valio.measure_cmd = cmd;
  sdi_valio.measure_arg = carg;
  sdi_valio.channel_val[0].didx = -1; // Assume no Values in D-Buffer

  // For THIS sensor:
  if (cmd != 'M')
    return false; // This sensor only supports M, M1
  if (carg <2) {
    uint16_t sum_ms=WAIT_ADINIT+WAIT_MS;
    uint16_t anz_chans=0;
    for(uint16_t i=0;i<ANZ_AD_PHYSKAN;i++){
      if(param.m0_mask & (1<<i)){
        anz_chans++;
        sum_ms+=ad_physkan[i].delay_ms+ad_physkan[i].true_msec;
      }
    }
    if(carg == 1){  // With V_BAT
      anz_chans++;
      sum_ms += 10;
    }
    sdi_valio.anz_channels = anz_chans;
    sdi_valio.m_msec = sum_ms;
  } else if(carg>=2 && carg<=(2+ANZ_AD_PHYSKAN)){  // M2-(M9)
    carg -=2;
    sdi_valio.anz_channels = 1;
    sdi_valio.m_msec = WAIT_ADINIT+WAIT_MS + ad_physkan[carg].delay_ms+ad_physkan[carg].true_msec;

  } else  return false;

  return true;
}
//
int16_t sensor_valio_measure(uint8_t isrc) {
  //---- 'M': While waiting: scan SDI for <BREAK> ----
  float fval;
  int32_t res;

  ad_spi_init();
  ad_reset();

  res = sensor_wait_break(isrc, WAIT_MS);
  if (res){
    ad_spi_close(); // Measure aborted
    return res;
  }
  // --- 'm' Wait end

  if(sdi_valio.measure_arg>=2){ // Only 1 Channel for M2-M9
    fval=-9999;
    get_analog_channel(sdi_valio.measure_arg-2, &fval);
    snprintf(sdi_valio.channel_val[0].txt, 11, "%+f", fval);
    sdi_valio.channel_val[0].punit = ad_physkan[sdi_valio.measure_arg-2].unit;
    ad_spi_close(); // Done
  }else{ // M0, M1: Multichannel
    uint16_t chan_idx=0;
    const char* pformat;
    for(uint16_t i=0;i<ANZ_AD_PHYSKAN;i++){
      if(param.m0_mask & (1<<i)){
        fval=-9999;
        res = get_analog_channel(i, &fval);
        if(res>=-2147483392){  // Result OK, Use indivdual linearsation
            fval *= param.koeff[i*2]; // Def. 1.0
            fval -= param.koeff[i*2+1]; // Def. 0.0
        }
        pformat=s_formats[param.precision[i]];
        snprintf(sdi_valio.channel_val[chan_idx].txt, 11, pformat, fval);
        // Use Default or individual unit
        if(strlen(param.ind_unit[i].s)) sdi_valio.channel_val[chan_idx].punit = param.ind_unit[i].s;
        else sdi_valio.channel_val[chan_idx].punit = ad_physkan[i].unit;
        chan_idx++;
      }
    }
    if(sdi_valio.measure_arg == 1){  // With V_BAT
      snprintf(sdi_valio.channel_val[chan_idx].txt, 11, "%+.2f", get_vbat_aio()); // Only 2 digits
      sdi_valio.channel_val[chan_idx].punit = "VSup";
    }

    ad_spi_close(); // Done
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
  uint16_t mask;
  uint8_t prec;
  char* hpn;
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

  // ===Special to ADS1220-Sensor: Mask, Units and Resolution===
  }else if (*pc == 'B') { // Bitmask XB or XB=
    pc++;
    mask=param.m0_mask;
    if (*pc == '=') { // Set Mask
      mask = strtoul(pc + 1, &pc,10);
      if(!mask || mask>255) return; // Invalid
    }
    if (*pc != '!')
      return;
    param.m0_mask=mask;
    // Send Mask
    sprintf(sdi_obuf, "%cB=%u", my_sdi_adr, mask);
  }else if (*pc == 'U') { // Un! or Un=unit!
    pidx = (uint16_t)strtoul(pc + 1, &pc, 0);
    if (pidx >= ANZ_AD_PHYSKAN)
      return;
    if (*pc == '=') { // Set Koeff
       hpn=pc+1;
    }else hpn=NULL;
    while(*pc && *pc!='!') pc++;
    if (*pc != '!')
      return;
    if(hpn) {
      *pc=0;  // Remove trailing '!'
      strncpy(param.ind_unit[pidx].s,hpn,MAX_IUNIT);
    }
    // Show Unit or ''
    sprintf(sdi_obuf, "%cU%d='%s'", my_sdi_adr, pidx, param.ind_unit[pidx].s);
  }else if (*pc == 'P') { // Pn! or Pn=val! Precision
    pidx = (uint16_t)strtoul(pc + 1, &pc, 0);
    if (pidx >= ANZ_AD_PHYSKAN)
      return;
     prec = param.precision[pidx];
    if (*pc == '=') { // Set Koeff
      prec = strtoul(pc + 1, &pc,10);
      if(prec>9) return; // Invalid
    }
    if (*pc != '!')  return;
    // Send Koeffs
    param.precision[pidx] = prec;

    sprintf(sdi_obuf, "%cP%d=%u", my_sdi_adr, pidx, prec);
  } else if (!strcmp(pc, "Write!")) { // Write SDI_Addr and Koefficients to Memory
    intpar_mem_erase();               // Compact Memory
    intpar_mem_write(ID_INTMEM_SDIADR, 1, (uint8_t *)&my_sdi_adr);
    intpar_mem_write(ID_INTMEM_USER0, sizeof(param), (uint8_t *)&param);
    sprintf(sdi_obuf, "%c", my_sdi_adr);            // Standard Reply
  } else if (!strcmp(pc, "Sensor!")) {               // Identify Senor
    sprintf(sdi_obuf, "%cADS1220!", my_sdi_adr); // Standard Reply
  }                                                  // else ..
}

//***