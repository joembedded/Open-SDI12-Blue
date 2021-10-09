/****************************************************
* 0380_Rotronic_HC2 - Precise Temperature/Humidity Sensor
* DEVICE_TYP 380
*
* (C) joembedded@gmail.com - joembedded.de
*
* UART is shared with tb_tools
*
* The HC2 'Hydroclip_ is a precise Temperature/Humidity Sensor
* from Rotronic
*
* The  HC2 needs >= 3.2V! Current is ca. 4mA (constant)
* Power may be switched via IO or ext. Transistor if
* HC2_PWR_PIN is defined. 
* NOTE: Even in High-Power Mode GPIO voltage drops to 3.0V@4mA,
* HC2 works for Lab, but better to use external switch!
*
* Connect: 
* - Green:  Supply (>= 3.2V see Datasheet)
* - Red:    TX to sensor from CPU (*tdb* check for opt. Pullup?)
* - Blue:   RX from sensor to CPU (*tdb* check for opt. Pullup?)
* - Gray:   GND
*
* Errors:
* -999: No Reply from Sensor
* -998: Sensor not Ready (after Power On needs ca. 1500 msec)
* -997: Data Corrupt
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

#include "nrf_drv_gpiote.h"

#include "osx_main.h"
#include "osx_pins.h"

#include "intmem.h"

#include "app_uart.h"
#if defined(UART_PRESENT)
#include "nrf_uart.h"
#endif
#if defined(UARTE_PRESENT)
#include "nrf_uarte.h"
#endif

#if DEVICE_TYP != 380
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

float hc2_hum;	// 1. Wert rHum - Raw Values
float hc2_temp; // 2. Wert Temp

/*****************************************************************
* HC2 uart implement. Locals
*****************************************************************/
#define HC2_IBUFS 120 // Ca. 105 per Default
#if defined (UARTE_PRESENT)
 #define STD_HC2_BAUDRATE NRF_UARTE_BAUDRATE_19200
#else
 #define STD_HC2_BAUDRATE NRF_UART_BAUDRATE_19200
#endif

#define LOC_HC2_TX IX_SCL
#define LOC_HC2_RX IX_SDA

// undefine HC2_PWR_PIN for cosnt power
#define HC2_PWR_PIN   IX_X0 // If defined use THIS Pin for VCC
#define HC2_PWR_WAIT  1700  // msec

static const app_uart_comm_params_t _hc2_uart_comm_params = {
    LOC_HC2_RX,
    LOC_HC2_TX,
    UART_PIN_DISCONNECTED /*RTS_PIN_NUMBER*/,
    UART_PIN_DISCONNECTED /*CTS_PIN_NUMBER*/,
    /*APP_UART_FLOW_CONTROL_ENABLED */ APP_UART_FLOW_CONTROL_DISABLED,
    false, // Not use Parity (bec. 8 Bits)
    STD_HC2_BAUDRATE};
static bool _uart_tbdbg_init;   // tb_tools-uart was iniit

static char hc2_ibuf[HC2_IBUFS]; // Der IN-Buffer 80 (2/14: 79+1)
static int16_t hc2_ccnt;         // Zaehlt Zeichen (alle)

/*****************************************************************
* HC2 shares UART with tb_uart, SDI12
*****************************************************************/
static int16_t hc2_uart_init(uint8_t isrc) {
  int16_t res;
  // isrc == SDI12 comes with enabled UART, else check
  tb_delay_ms(10);  // Allow pending chars to flush
  if(isrc == SRC_SDI) sdi_uart_uninit(false);
  else {
    _uart_tbdbg_init = tb_is_uart_init();
    if (_uart_tbdbg_init) tb_uart_uninit();
  }
  res = tb_uart_init((void *)&_hc2_uart_comm_params, NULL, 0, NULL, 0, -1);
  return res;
}
static int16_t hc2_uart_uninit(uint8_t isrc) {
  int16_t res;
  res = tb_uart_uninit(); // Uninit UART for HC2
  // Enableing the right (old) depends on isrc or former init
  if(isrc == SRC_SDI) sdi_uart_init(false);
  else if (_uart_tbdbg_init) tb_uart_init(NULL, NULL, 0, NULL, 0, -1);
  return res;
}
// wt: msec lang String abholen versuchen, Ende bei CR/LF 13/10 in jedem Fall
// Achtung: BREAK und ERRORs werden asynchron detektiert!
// Blitzt 4/sek
int16_t hc2_gets(int32_t wt) {
  int16_t res;
  int16_t i;
  uint8_t c;
  int32_t wt0 = wt;
  hc2_ccnt = 0;
  hc2_ibuf[0] = 0;
  for (;;) {
    res = tb_getc();
    if (res == -1) {
      wt0 -= 10; // msec
      if (wt0 <= 0)
        return -ERROR_NO_REPLY; // Timeout, wenigstens CR fehlt
      if ((wt0 & 255) > 245)
        tb_board_led_on(0);
      else
        tb_board_led_off(0);
      tb_delay_ms(10);
    } else if (res < 0) {
      return -ERROR_DRV_ERROR; // Driver ERROR
    } else {
      c = (char)res;
      if (c < ' ')
        break;
      wt0 = wt; // Soft-Timer neu starten
      hc2_ibuf[hc2_ccnt++] = c;
      hc2_ibuf[hc2_ccnt] = 0; // Immer Terminieren
      if (hc2_ccnt == (HC2_IBUFS - 1))
        return -ERROR_TOO_MUCH_DATA; // Too much
    }
  }
  // 7 ist Minimum
  if (!hc2_ccnt)
    return -ERROR_NO_REPLY;
  if (hc2_ccnt < 7)
    return -ERROR_NOT_ENOUGH_DATA;
  c = 0;
  for (i = 0; i < hc2_ccnt - 1; i++) {
    c += hc2_ibuf[i];
  }
  c = (c & 0x3F) + 0x20;
  if (c != hc2_ibuf[hc2_ccnt - 1])
    return -ERROR_DATA_CRC;
  return hc2_ccnt; // Soviele Zeichen in Antwort
}

// hc2 Kommando senden, Zeichen # ist BREAK (logischerweise am Anfang)
// Wenn Parameter NULL: hc2_obug verwenden
static void hc2_send_cmd(char *pc) {
  for (;;) { // Opt. RX-Puffer leeren
    if (tb_getc() == -1)
      break;
  }
  while (*pc) {
    tb_putc(*pc++);
  }
}
/***********************************************************************
* hc2_scan_vals(): String scannen und Werte eintragen
* HC2 per Std. mit 19200 Bd, 8N1, Std.-Messkommando: "{F99RDD}\r"
* Das ist die Antwort vom Sensor (4 ist die FCS)
* {F00rdd 001; 41.18;%rh;000;-; 21.83;°C;000;=;nc;---.- ;°C;//
* //000; ;001;V2.0-2;0061114634;HC2         ;000;4 (+'\r', fehlt)
* ungueltiger Wert: '---.--' (during PowerUp ca. >1300 msex)
***********************************************************************/
static float hc2_get_fval(char *pbuf){
  if(!strncmp(pbuf,"--",2)){ // Sensor noch nicht ready
    return -998;  // Sensor not Ready
  }else{
    return atof(pbuf);
  }
}

// Kann einen hc2-String scannen
static int16_t hc2_scan_vals(void) {
  char *pbuf = hc2_ibuf;
  uint8_t c;
  uint8_t sc = 0;
  
  for (;;) {
    c = *pbuf++;
    if (!c)
      break;
    if (c == ';') {
      if (!sc) { // rH
          hc2_hum = hc2_get_fval(pbuf);
      } else {
        if (sc == 4) { // oC
          hc2_temp = hc2_get_fval(pbuf);
          return 2; // 2 Werte
        }
      }
      sc++;
    }
  }
  return -998; // Data Corrupt
}

/***********************************************************************
* hc2_measure: Komplette hc2-Messung durchfuehren
***********************************************************************/
int16_t hc2_measure(void) {
  int16_t res = -999;

  hc2_hum=-999; // No Reply
  hc2_temp=-999;

  // Optional *todo* Wecken und PowerOn Warten

  hc2_send_cmd("{F99RDD}\r");
  res = hc2_gets(200);  //20 msec auf Antwort warten

  if (res < 0)
    return res;
  return hc2_scan_vals(); // 0: OK!
}

#if DEBUG
// === DEBUG START ===
//====== TEST COMMANDS FOR NEW SENSOR START_A ========
void messen_hc2dbg_werte(void){
        float fval;
        int16_t res;
	tb_printf("Measure HC2\n");
        hc2_uart_init(SRC_CMDLINE);
        res=hc2_measure();
        hc2_uart_uninit(SRC_CMDLINE);
	if(res!=2) { 
		tb_printf("HC2 Read Error/No Reply! %d\n",res);
		return;
	}
        tb_printf("HC2-T(raw):%.2f oC\n",hc2_temp);
        tb_printf("HC2-H(raw):%.2f %%rH\n",hc2_hum);
}

//====== TEST COMMANDS FOR NEW SENSOR END_A ========

// Additional Test-CMDs via TB_UART
// IMPORTANT: only SMALL capitals for User-CMDs!
bool debug_tb_cmdline(uint8_t *pc, uint32_t val){
  switch (*pc) {
  //====== TEST COMMANDS FOR NEW SENSOR START_S ========
  case 'a':
    {
#ifdef HC2_PWR_PIN 
  // Supply Sensor with Power
  nrf_gpio_cfg(
      HC2_PWR_PIN,
      NRF_GPIO_PIN_DIR_OUTPUT,
      NRF_GPIO_PIN_INPUT_DISCONNECT,
      NRF_GPIO_PIN_NOPULL,
      NRF_GPIO_PIN_S0H1, // High Out
      NRF_GPIO_PIN_NOSENSE);
  nrf_gpio_pin_set(HC2_PWR_PIN);
  tb_delay_ms(HC2_PWR_WAIT);
#endif
      messen_hc2dbg_werte();
#ifdef HC2_PWR_PIN 
  nrf_gpio_cfg_default(HC2_PWR_PIN); // Power OFF
#endif
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
  // TT:'TerraTransfer', HC2:'HC2_', 'A': Ver A
  sprintf(sensor_id, "TT_HC2_A_0380_OSX%08X", mac_addr_l);

  // Try to read Parameters
  intpar_mem_read(ID_INTMEM_USER0, sizeof(param), (uint8_t *)&param);
}

#ifdef HC2_PWR_PIN 
 #define WAIT_MS HC2_PWR_WAIT
#else
 #define WAIT_MS 50  
#endif
bool sensor_valio_input(char cmd, uint8_t carg) {
  sdi_valio.measure_cmd = cmd;
  sdi_valio.measure_arg = carg;
  sdi_valio.channel_val[0].didx = -1; // Assume no Values in D-Buffer

  // For THIS sensor:
  if (cmd != 'M')
    return false; // This sensor only supports M, M1
  if (carg == 0) {
    sdi_valio.anz_channels = 2;
    sdi_valio.m_msec = WAIT_MS+200;
  } else if (carg == 1) {
    sdi_valio.anz_channels = 3;
    sdi_valio.m_msec = WAIT_MS+250;
  } else
    return false;

  return true;
}

int16_t sensor_valio_measure(uint8_t isrc) {
  //---- 'M': While waiting: scan SDI for <BREAK> ----
  int16_t res;
  float fval;

#ifdef HC2_PWR_PIN 
  // Supply Sensor with Power
  nrf_gpio_cfg(
      HC2_PWR_PIN,
      NRF_GPIO_PIN_DIR_OUTPUT,
      NRF_GPIO_PIN_INPUT_DISCONNECT,
      NRF_GPIO_PIN_NOPULL,
      NRF_GPIO_PIN_S0H1, // High Out
      NRF_GPIO_PIN_NOSENSE);
  nrf_gpio_pin_set(HC2_PWR_PIN);
#endif
  res = sensor_wait_break(isrc, WAIT_MS);
  if (res){
#ifdef HC2_PWR_PIN 
  nrf_gpio_cfg_default(HC2_PWR_PIN); // Power OFF
#endif
    return res;
  }
  // --- 'm' Wait end

  // Read Data from Sensor
  hc2_uart_init(isrc);
  res = hc2_measure();
  hc2_uart_uninit(isrc);
#ifdef HC2_PWR_PIN 
  nrf_gpio_cfg_default(HC2_PWR_PIN); // Power OFF
#endif

  // Prepare Output
  sdi_valio.channel_val[0].punit = "%rH";
  if (res>=0) {
    fval = hc2_hum;
    fval *= param.koeff[2]; // Def. 1.0
    fval -= param.koeff[3]; // Def. 0.0
  } else {
    fval = -999; // Error..
  }
  snprintf(sdi_valio.channel_val[0].txt, 11, "%+.1f", fval);

  sdi_valio.channel_val[1].punit = "oC";
  if (res>=0) {
    fval = hc2_temp;
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
    sprintf(sdi_obuf, "%cHC2_A!", my_sdi_adr); // Standard Reply
  }                                                  // else ..
}

//***