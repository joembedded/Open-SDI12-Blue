/****************************************************
* osx_main.c - Sensor SDI12-Driver
*
* (C) joembedded@gmail.com - joembedded.de
*
* UART is shared with tb_tools
*
* For new Sensor-Types:
* - Set DEVICE_TYP (in device.h) close to the new Sensor or select DEVICE_TYP 200
* - Test Sensor Access in this Module in the area DEBUG (debug_tb_cmdline())
*   via local UART until it works
* - Save the results in in a new device (ApplicSensor in Project) with new DEVICE_TYP
*
***************************************************/

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "nrf_drv_gpiote.h"
#include "sdi12sensor_drv.h"

#include "./platform_nrf52/tb_pins_nrf52.h"
#include "device.h"
#include "osx_main.h"
#include "osx_pins.h"
#include "saadc.h"
#include "tb_tools.h"
#include "bootinfo.h"

#if DEBUG
 #include "i2c.h"
 #include "nrfx_spim.h"
#endif

#ifdef ENABLE_BLE
#include "ltx_ble.h"
#endif
#include "intmem.h"

// ---Globals---
bool type_irq_wake_flag=false;  // Flag indicates Wake by IRQ, set extern

char my_sdi_adr = '0'; // Factory Default
char outrs_buf[MAX_OUTBUF];
SDI_VALIO sdi_valio;

// ===Sensor Part Start ===
// Id has fixed structure, max. 30+'\0'
//                      all cccccccc.8 mmmmmm.6 vvv.3 xx..xx.[0-13]
//                       13 JoEmbedd   Testse   OSX   (MAC.l)
char sensor_id[8 + 6 + 3 + 13 + 1] = "JoEmbedd"
                                     "Testse"
                                     "OSX"
                                     "Sno..";

int16_t sdi_send_reply_mux(uint8_t isrc) {
  switch (isrc) {
  case SRC_SDI:
    return sdi_send_reply_crlf(outrs_buf); // send SDI_OBUF chars
  case SRC_CMDLINE:
    tb_printf("%s<CR><LF>", outrs_buf);
    break;
#ifdef ENABLE_BLE
  case SRC_BLE: {
    bool oldr = ble_reliable_printf;
    ble_reliable_printf = true;
    ble_printf("%s<CR><LF>", outrs_buf);
    ble_reliable_printf = oldr;
  } break;
#endif
  }
  return 1; // Something sent (len not important)
}

// Build Outstring (Regarding max length 35/75.. todo)
void sensor_build_outstring(void) {
  uint8_t didx = 0;
  uint16_t rlen = 0, hlen;
  uint16_t cmd_max_len = 35; /** For M-CMD, othes: todo **/
  for (uint16_t i = 0; i < MAX_CHAN; i++) {
    if (i >= sdi_valio.anz_channels) {
      sdi_valio.channel_val[i].didx = -1;
      break;
    }
    hlen = strlen(sdi_valio.channel_val[i].txt);
    if (rlen + hlen > cmd_max_len) {
      rlen = 0;
      didx++;
    }
    sdi_valio.channel_val[i].didx = didx;
  }
}

//---- Sensor CMDs Start ------------
// return 0: CMD not valid, >0: Finished, -1: <BREAK> found
int16_t sensor_cmd_m(uint8_t isrc, uint8_t carg) {
  if (sensor_valio_input('M', carg) == false)
    return 0; // Cmd not supported

  if (isrc == SRC_SDI)
    tb_delay_ms(9); // Default Delay after CMD

  // CHannel '0'-'9' , then ASCII
  sprintf(outrs_buf, "%c%03u%c", my_sdi_adr, (sdi_valio.m_msec + 999) / 1000, sdi_valio.anz_channels + '0'); // M: y Measures in xxx secs
  sdi_send_reply_mux(isrc);                                                                                  // send SDI_OBUF

  if (sensor_valio_measure(isrc) < 0)
    return 0; // Aborted
  sensor_build_outstring();

  sprintf(outrs_buf, "%c", my_sdi_adr); // Service Request
  sdi_send_reply_mux(isrc);             // send SDI_OBUF

  *outrs_buf = 0; // Assume no reply
  return 1;       // Cmd was OK
}

// Break can interrupt M. OPtional Wait in MS
int16_t sensor_wait_break(uint8_t isrc, int16_t wt){
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
  return 0; // OK, Continue
}


//---- Sensor CMDs End ------------
// ===Sensor Part END ===

//----- SDI-RX-Pin-IRQ-Driver ------------
bool rxirq_active;
volatile uint32_t rxirq_zcnt;
/*irq*/ static void rxirq_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action) {
  rxirq_zcnt++;
}

// Set/Unset RX-Pin to "IRQ-on-BREAK", opt. deactivate SDI-UART, Details: see gpio_irq.c-Sample
static void rxirq_on(void) {
  uint32_t err_code;
  if (rxirq_active)
    return; // Already ON
  nrf_drv_gpiote_in_config_t in_config = NRFX_GPIOTE_CONFIG_IN_SENSE_HITOLO(false);
  err_code = nrf_drv_gpiote_in_init(SDI_RX_PIN, &in_config, rxirq_handler);
  APP_ERROR_CHECK(err_code);
  nrf_drv_gpiote_in_event_enable(SDI_RX_PIN, true);
  rxirq_active = true;
}
static void rxirq_off(void) {
  if (!rxirq_active)
    return; // Already OFF
  nrf_drv_gpiote_in_event_disable(SDI_RX_PIN);
  nrf_drv_gpiote_in_uninit(SDI_RX_PIN); // Disable Functins an Pullups
  rxirq_active = false;
}

//--- Init (only call once)
void type_init(void) {
  uint32_t err_code;
  err_code = nrf_drv_gpiote_init();
  APP_ERROR_CHECK(err_code);

  intpar_mem_read(ID_INTMEM_SDIADR, 1, (uint8_t *)&my_sdi_adr);
  sensor_init(); // ID etc..

  rxirq_on(); // SDI now active
}

// Static parts of CMDs
static bool cmdcrc_flag = false;

// Flexible cmd. 0: No Reply
int16_t sdi_process_cmd(uint8_t isrc, char *const ps_ibuf) {
  char *pc, arg_val0;
  int8_t i8h; // Temp
  uint16_t crc16;
  uint16_t len;

  *outrs_buf = 0; // Assume no reply
  len = strlen(ps_ibuf);
  if (len && ps_ibuf[len - 1] == '!' && // Only Commands (end with '!')
      (*ps_ibuf == '?' || *ps_ibuf == my_sdi_adr)) {

    // Fast scan CMD via switch() - reply only to valid CMDs
    switch (ps_ibuf[1]) {
    case '!': // Only "!\0"
      if (!ps_ibuf[2])
        sprintf(outrs_buf, "%c", my_sdi_adr);
      break;
    case 'I': // SDI V1.3
      if (!strcmp(ps_ibuf + 2, "!"))
        sprintf(outrs_buf, "%c13%s", my_sdi_adr, sensor_id);
      break;
    case 'A':
      arg_val0 = ps_ibuf[2];
      if (!strcmp(ps_ibuf + 3, "!") && ((arg_val0 >= '0' && arg_val0 <= '9') ||
                                           (arg_val0 >= 'A' && arg_val0 <= 'Z') ||
                                           (arg_val0 >= 'a' && arg_val0 <= 'z'))) {

        if (arg_val0 != my_sdi_adr) {
          intpar_mem_write(ID_INTMEM_SDIADR, 1, (uint8_t *)&arg_val0);
          intpar_mem_read(ID_INTMEM_SDIADR, 1, (uint8_t *)&my_sdi_adr);
        }

        sprintf(outrs_buf, "%c", my_sdi_adr);
      }
      break;
    case 'M': // aM!, aMx!, aMC!, aMCx!, pc points after 'M'
      pc = ps_ibuf + 2;
      if (*pc == 'C') {
        cmdcrc_flag = true;
        pc++;
      } else
        cmdcrc_flag = false;
      if (*pc == '!')
        arg_val0 = 0;
      else
        arg_val0 = (*pc++) - '0';
      if (!strcmp(pc, "!") && arg_val0 >= 0 && arg_val0 <= 9) {
        sensor_cmd_m(isrc, (uint8_t)arg_val0);
        return 1;
      }
      break;
    case 'D': // D0-D9
      arg_val0 = ps_ibuf[2] - '0';
      if (!strcmp(ps_ibuf + 3, "!") && arg_val0 >= 0 && arg_val0 <= 9) {
        pc = outrs_buf;
        *pc++ = my_sdi_adr;
        *pc = 0;
        for (uint16_t i = 0; i < MAX_CHAN; i++) {
          i8h = sdi_valio.channel_val[i].didx;
          if (i8h == -1)
            break;
          if (i8h == arg_val0) {
            strcpy(pc, sdi_valio.channel_val[i].txt);
            pc += strlen(sdi_valio.channel_val[i].txt);
          }
        }
        if (cmdcrc_flag) {
          crc16 = sdi_track_crc16(outrs_buf, (pc - outrs_buf), 0 /*Init*/);
          *pc++ = 64 + ((crc16 >> 12) & 63);
          *pc++ = 64 + ((crc16 >> 6) & 63);
          *pc++ = 64 + (crc16 & 63);
          *pc = 0;
        }
        // Now D-String is ready
      }
      break;
    case 'X': // 'X'; Additional SDI12 - User Commands
      if (!strcmp(ps_ibuf+2, "FactoryReset!")){
        intpar_mem_erase();
        sprintf(outrs_buf, "%c", my_sdi_adr);
        tb_delay_ms(9);           // Default Delay after CMD
        sdi_send_reply_mux(isrc); // send SDI_OBUF
        tb_delay_ms(100);
        tb_system_reset();        // Reset...
      }else if (!strcmp(ps_ibuf+2, "Device!")){
        pc=outrs_buf;
        pc+=sprintf(pc, "%cM:%08X%08X,T:%u,V%u.%u", 
          my_sdi_adr, mac_addr_h, mac_addr_l,
          DEVICE_TYP,DEVICE_FW_VERSION/10,DEVICE_FW_VERSION%10);
        if(get_pin()) { // PIN found => Add it
          sprintf(pc,",P:%u!",get_pin());
        }else strcat(pc,"!"); // No Pin

      } // else 
      sensor_valio_xcmd(isrc,ps_ibuf+2);
      break;  

      // default: No Reply!
    } // switch
  }

  if (*outrs_buf) {
    if (isrc == SRC_SDI)
      tb_delay_ms(9);                // Default Delay after CMD
    return sdi_send_reply_mux(isrc); // send SDI_OBUF
  } else {
    return 0;
  }
}

bool type_service(void) {
  int16_t res;

  // SDI12 Activity registered
  if (rxirq_zcnt) {
    rxirq_off();
    tb_putc('['); // Signal SDI12 Activity
    *outrs_buf = 0;
    tb_delay_ms(1);
    sdi_uart_init();

    for (;;) {
      tb_board_led_on(0);
      res = sdi_getcmd(SDI_IBUFS, 100 /*ms*/); // Max. wait per Def.
      tb_board_led_off(0);
      if (res == -ERROR_NO_REPLY) {
        break;              // Timeout
      } else if (res > 0) { // Soemthing received
        sdi_process_cmd(SRC_SDI, sdi_ibuf);
        // And again until No Reply
      } // else: Only Break or Corrupt Data
    }   // for()

    sdi_uart_uninit();
    rxirq_on();
    rxirq_zcnt = 0;

    tb_putc(']'); // Signal SDI12 Activity
    return true;  // No Periodic Service
  }
  // Flag indicates IRQ in type implementation
  if(type_irq_wake_flag){ // If set: Skip PERIOD Serice once
      type_irq_wake_flag=false;
      return true;  // No Periodic Service
  }else  return false; // Periodic Service
}

void type_cmdprint_line(uint8_t isrc, char *pc) {
  if (isrc == SRC_CMDLINE) {
    tb_printf("%s\n", pc);
#ifdef ENABLE_BLE
  } else if (isrc == SRC_BLE) {
    bool oldr = ble_reliable_printf;
    ble_reliable_printf = true;
    ble_printf(pc);
    ble_reliable_printf = oldr;
#endif
  }
}

#if DEBUG
// === DEBUG START ===
//====== TEST COMMANDS FOR NEW SENSOR START_A ========

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
  int32_t res,tara;
  ad_spi_init();
  ad_reset();

  tara = ad_measure(BRIDGE01RPN_CONFIG,BRIDGE01RPN_DELAY,BRIDGE01RPN_MITTEL,BRIDGE01RPN_KALI);
  while(1){
    res=ad_measure(BRIDGE01RPN_CONFIG,BRIDGE01RPN_DELAY,BRIDGE01RPN_MITTEL,BRIDGE01RPN_KALI)-tara;
    tb_printf("Bridge-RES: %d  %x  -> %.2f\n",res,res,(float)(res)*1.69e-4);

    res=ad_measure(ITEMP_CONFIG,ITEMP_DELAY,ITEMP_MITTEL,ITEMP_KALI);
    tb_printf("Temp-RES: %d  %x  -> %.2f\n",res,res,(float)(res)/32768.0);

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


// Die Input String und Values
bool type_cmdline(uint8_t isrc, uint8_t *pc, uint32_t val) {
  int res;

  switch (*pc) {
  case 'z': // 'z' - SDI12 emulated
    if (!sdi_process_cmd(isrc, pc + 1))
      type_cmdprint_line(isrc, "<NO REPLY>");
    break;

  case 'e': // Measure
    //        ble_printf("~e:%u %u",highest_channel, measure_time_msec);
    if (sensor_valio_input('M', (uint8_t)val) == false) {
      type_cmdprint_line(isrc, "Error('e')");
      break;
    }
    sprintf(outrs_buf, "~e:%u %u", sdi_valio.anz_channels, sdi_valio.m_msec);
    type_cmdprint_line(isrc, outrs_buf);
    sensor_valio_measure(SRC_NONE); // SLient
    for (int16_t i = 0; i < sdi_valio.anz_channels; i++) {
      char *pc = sdi_valio.channel_val[i].txt;
      if (*pc == '+')
        pc++;
      sprintf(outrs_buf, "~#%u: %s %s", i, pc, sdi_valio.channel_val[i].punit);
      type_cmdprint_line(isrc, outrs_buf);
    }
    sprintf(outrs_buf, "~h:%u\n", 0); // Reset, Alarm, alter Alarm, Messwert, ..
    type_cmdprint_line(isrc, outrs_buf);

    break;

  default:
#if DEBUG
    // If DEBUG Additional Test-CMDs via TB_UART
    if(isrc == SRC_CMDLINE) return debug_tb_cmdline(pc, val);
#endif    
    return false;
  }
  return true; // Command processed
}

//***