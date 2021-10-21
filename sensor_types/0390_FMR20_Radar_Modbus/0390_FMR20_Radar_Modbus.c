/****************************************************
* 0390_FMR20_Radar_Modbus.c - Radar Distance Sensor
* DEVICE_TYP 390
*
* (C) joembedded@gmail.com - joembedded.de
*
* UART is shared with tb_tools. This Module can be used for other
* MODBUS sensor too, see Comments...
*
* The FMR20 is a Radar Distance Sensor form Endress+Hauser
* It runs with Modbus and internally uses a nRF51822 BLE Soc.
*
* IMPORTANT: By Default the FMR20 uses "9600 8E1" at Addr. 200.d.
* This is not very commen, please set to "9600 8N1! 
*
* The  FMR20 needs 5-30V! Current is ca. 5 mA
* Power may be switched via IO or ext. Transistor if
* MBUS_PWR_PIN is defined. 
*
* Sample Modbus pakets:
* Sensor NOT ready:
* =================
* -> Read Input Reg. 0x1388/5000, Anz: 16 F
* <<TX: C8 03 13 88 00 10 D1 31>>
* <<RX: C8 3 20 0 0 0 0 0 0 0 0 0 0 0 0 41 A0 0 0 40 40 0 0 0 0 0 0 0 0 0 0 0 0 0 0 F7 A1 CRC_OK>>
* Result: 32 Bytes, Delay:47 msec
* Float: 0.000000 0.000000 0.000000 20.000000 3.000000 0.000000 0.000000 0.000000
*
* Sensor ready (after ca. 8-12 sec (t.b.d.)):
* -> Read Input Reg. 0x1388/5000, Anz: 16 F
* <<TX: C8 03 13 88 00 10 D1 31>>
* <<RX: C8 3 20 41 4C 69 FC 40 E 58 16 41 39 56 84 41 80 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 B5 52 CRC_OK>>
* Result: 32 Bytes, Delay:47 msec
* Float:  12.775875  2.224126  11.583622  16.000000  0.000000  0.000000  0.000000  0.000000
* (Expl.: 0:Level(x) 1:Dist(m) 2:Sig.(db) 3:Temp(oC) 4:Qual(-) 5:Diag(-) 6:GPS_Lat 7:GPS_Lng
*
* Trying to read trash:
* -> Read Input Reg. 0x1389/5001, Anz: 2 F
* <<TX: C8 03 13 89 00 02 00 FC>>
* <<RX: C8 83 2 10 CF CRC_OK>>
* Result: Delay:15 msec, Fkt.: 0x3/3 Error 2
*
*
* Connect: 
* Needs additional RX/TX<->RS485 PCB (t.b.d)
*
* Errors  (for >= -1000 see 'sdi12sensor_drv.h') 
* -700..-755: MODBUS Error Codes (700: Code 0..755: Code:255)
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

#if DEVICE_TYP != 390
#error "Wrong DEVICE_TYP, select other Source in ApplicSensor"
#endif

// --------- Locals -------------
/* Parameters can be used e.g. as Offset/Multi per Channel etc...
* This is only a demo to manage 4 FLOAT Parameters
  // Note: this is GP/TT Standard; First MULTI, then sub OFFSET!
  fval*=param->factor; // Def. 1.0
  fval-=param->offset; // Def. 0.0
*/
#define MB_RAW_VALS 9 // Maximum 9 Values
#define ANZ_KOEFF (MB_RAW_VALS * 2)
#define KONFIG_MAX 80 // Konfiguration String
typedef struct {
  float koeff[ANZ_KOEFF];
  char konfig[KONFIG_MAX];
} PARAM;
// Test Setup for Default Koeffs
PARAM param = {
    {1.0, 0.0,        // #0
        1.0, 0.0,     // #1
        1.0, 0.0,     // #2
        1.0, 0.0,     // #3
        1.0, 0.0,     // #4
        1.0, 0.0,     // #5
        1.0, 0.0,     // #6
        1.0, 0.0,     // #7
        1.0, 0.0},    // #8
    "r200 5000 FFFFF", // r:ReadInputReg(fkt3) 5 Regs. from Reg 5000 at MB Addr 200 to #0-#4 as Float
};

// Analyses Scan Data
typedef struct{
  // For current Request
  uint8_t addr;
  uint8_t fkt;
  uint16_t reg;
  uint8_t intCnt;
  // Working Vars
  char *pcon; // Points to Config
  char *pficon; // Points to FI Part
  uint8_t sdireg_cnt; // For Scan (D) (for Param)
} MODBUS_REQUEST;

static MODBUS_REQUEST mbr;

//------------------- Implementation -----------

/*****************************************************************
* Modbus Uart Implement. Locals
*****************************************************************/
float modbus_raw_fval[MB_RAW_VALS]; // Raw Values

#define MOB_IBUFS 60 // OK for 10 Floats
#define MOB_OBUFS 60

#if defined(UARTE_PRESENT)
#define STD_MOB_BAUDRATE NRF_UARTE_BAUDRATE_9600
#else
#define STD_MOB_BAUDRATE NRF_UART_BAUDRATE_9600
#endif
#define MODBUS_TRAILER_MS 5 // Trailertime(ms) afer UART ON (>3.5 chars)
#define MODBUS_REPLY_WAIT 200 // Wait 200 msec for Reply from MB

#define LOC_MOB_TX IX_SCL
#define LOC_MOB_RX IX_SDA

// Definition old Version: PCB <= V0.2 PCB >0.2: t.b.d
#define MOB_PWR_PIN IX_X0   // H:Pwr/Term ON L:Off
#define MOB_NRXEN_PIN IX_X1 // L:RX_on H:RX_off
#define MOB_TXEN_PIN IX_X2  // H:TX_on L:TX_off

// undefine MOB_PWR_WAIT for const power
//#define MOB_PWR_WAIT  15000  // msec

static const app_uart_comm_params_t _mob_uart_comm_params = {
    LOC_MOB_RX,
    LOC_MOB_TX,
    UART_PIN_DISCONNECTED /*RTS_PIN_NUMBER*/,
    UART_PIN_DISCONNECTED /*CTS_PIN_NUMBER*/,
    /*APP_UART_FLOW_CONTROL_ENABLED */ APP_UART_FLOW_CONTROL_DISABLED,
    false, // Not use Parity (bec. 8 Bits)
    STD_MOB_BAUDRATE};
static bool _uart_tbdbg_init; // tb_tools-uart was iniit

static uint8_t mob_ibuf[MOB_IBUFS]; // Der IN-Buffer
static int16_t mob_ccnt;            // Zaehlt Zeichen (alle)
static uint8_t mob_obuf[MOB_OBUFS]; // Der OUT-Buffer

/*****************************************************************
* MOB shares UART with tb_uart, SDI12
*****************************************************************/
static int16_t mob_uart_init(uint8_t isrc) {
  int16_t res;
  // isrc == SDI12 comes with enabled UART, else check
  tb_delay_ms(10); // Allow pending chars to flush
  if (isrc == SRC_SDI)
    sdi_uart_uninit(false);
  else {
    _uart_tbdbg_init = tb_is_uart_init();
    if (_uart_tbdbg_init)
      tb_uart_uninit();
  }
  res = tb_uart_init((void *)&_mob_uart_comm_params, NULL, 0, NULL, 0, -1);
  return res;
}
static int16_t mob_uart_uninit(uint8_t isrc) {
  int16_t res;
  res = tb_uart_uninit(); // Uninit UART for MOB
  // Enableing the right (old) depends on isrc or former init
  if (isrc == SRC_SDI)
    sdi_uart_init(false);
  else if (_uart_tbdbg_init)
    tb_uart_init(NULL, NULL, 0, NULL, 0, -1);
  return res;
}

// Modbus-Antwort lesen Result: Read-Len>4 oder: Error
// TX-Frame required for RX-Check
// Ret: Datalen(>0) or <0: ERROR
int16_t mob_get_reply(int16_t wt) {
  int16_t c;

  int32_t wt0 = wt; // Inital Timeout
  mob_ccnt = 0;
  // Data from TX Frame
  uint8_t addr = mob_obuf[0];
  uint8_t fkt = mob_obuf[1];
  uint16_t exp_len = mob_obuf[5] * 2; // Expected Bytelen ([4] MUST be 0, already checked)..
  if (!exp_len || exp_len >= (MOB_IBUFS - 5))
    return -ERROR_PROTOCOL_TX;

  exp_len += 5; // Header A F L .. CRC.16
  for (;;) {
    c = tb_getc();
    if (c >= 0) { // Framing Errors are asynchron, Ignore in Reply
      switch (mob_ccnt) {
      case 0: // Addr
        if ((uint8_t)c != addr)
          c = -1; // Forget Input until addr OK
        break;
      case 1: // Fkt or Error (== Fkt + 128)
        if ((uint8_t)c == (fkt | 128)) {
          exp_len = 5; // Receive Error A ErrF ErrC CRC.16
          fkt|=128; // Mark ERROR
        } else if ((uint8_t)c != fkt)
          return -ERROR_PROTOCOL_RX;
        break;
      case 2: // Bytelen or Error-Code at Pos 2
        if ( ((uint8_t)c != (exp_len - 5)) && !(fkt|128) )
          return -ERROR_PROTOCOL_RX;
        break;
      }

      if (c != -1) {
        mob_ibuf[mob_ccnt++] = (uint8_t)c;
        if (mob_ccnt == exp_len) {
          // Check CRC16 - Same POLYNOMAL than SDI12, but init with FFFF
          uint16_t crc16 = sdi_track_crc16(mob_ibuf, exp_len-2, 0xFFFF);
          // CRC is L:H
          if(c != (uint8_t)(crc16>>8) || mob_ibuf[mob_ccnt-2]!= (uint8_t)(crc16)) return -ERROR_DATA_CRC;
          if((fkt&128)) return -(700+mob_ibuf[2]); // OK, but: Modbus Error Code at Pos 2
          return mob_ccnt; // >4 *OK* Finished!
        }
        wt = MODBUS_TRAILER_MS; // Frame has started
        continue;
      }
    }
    tb_delay_ms(1);
    if (!wt--) {
      return -ERROR_NO_REPLY;
    }
  } // forever;
}

// mob Kommando senden (im Buffer) und ruecklesen
static int16_t mob_send_cmd(uint16_t len) {

  if (len < 8)
    return -ERROR_PROTOCOL_TX;
  if (mob_obuf[4])
    return -ERROR_PROTOCOL_TX; // HB of LEN MUST be 0

  nrf_gpio_pin_set(MOB_TXEN_PIN); // H:TX_on L:TX_off
  tb_delay_ms(MODBUS_TRAILER_MS); // Activate first,
  for (;;) {                      // then clear RX Buffer
    if (tb_getc() == -1)
      break;
  }

  uint8_t *pc = mob_obuf;
  uint16_t h = len;
  while (h--) {
    tb_putc(*pc++);
  }

  // Read Back TX
  h = len;
  pc = mob_obuf;
  int16_t res = 0; // OK
  int16_t c;
  uint16_t wt = MODBUS_TRAILER_MS * 3; // 3-Times GAP for echo max.

  for (;;) {
    c = tb_getc();
    if (c != -1) {
      if (c < 0 || *pc != (uint8_t)c) { // Framing Error or Wrong Echo
        res = -ERROR_DATA_CORRUPT;
        break;
      }
      pc++;
      wt = MODBUS_TRAILER_MS;
      if (!--h)
        break; // Done
      continue;
    }
    tb_delay_ms(1);
    if (!wt--) {
      res = -ERROR_DATA_CORRUPT;
      break;
    }
  }
  nrf_gpio_pin_clear(MOB_TXEN_PIN); // H:TX_on L:TX_off
  return res;
}
/***********************************************************************
* Scan Values to Float. Return >=0: Anzahl Param nach SDI12 D
***********************************************************************/
typedef union{
	unsigned char b[4];
	float fzahl;
} MVAL; // Konvert Bytes to FLOAT

// Res: Anz. Data
int16_t mob_scan_vals(void) {
  uint8_t *pmb = mob_ibuf+3; // Points to Data Block
  uint8_t idx=0;
  MVAL mv;
  char *pc=mbr.pficon; // Reuses old FI pointer
  char c;
  for(;;){
    c=*pc++;
    if(c=='f' || c=='F'){
      if(idx==MB_RAW_VALS) return -ERROR_PARAMETER;
      // ***LittleEndian*** Float Conversion:
      mv.b[3]=*pmb++;
      mv.b[2]=*pmb++;
      mv.b[1]=*pmb++;
      mv.b[0]=*pmb++;
      modbus_raw_fval[idx++]=mv.fzahl;
    }else if(c=='i' || c=='I'){
      // ***LittleEndian*** INt16 Conversion:
      if(idx==MB_RAW_VALS) return -ERROR_PARAMETER;
      modbus_raw_fval[idx++]=(pmb[1]<<8)+pmb[0];
      pmb+=2;
    }else return idx; // Alles OK
  }
}

// Built TX Frame form Read Reg end send it
int16_t modbus_34_request(uint8_t addr, uint8_t fkt, uint16_t reg, uint8_t intCnt){
  uint8_t *pmob_obuf = mob_obuf;
  *pmob_obuf++ = addr;
  if(fkt!=3 && fkt!= 4) return -ERROR_PARAMETER;
  *pmob_obuf++ = fkt;
  *pmob_obuf++ = (uint8_t)(reg>>8);
  *pmob_obuf++ = (uint8_t)(reg);
  *pmob_obuf++ = 0x00; // MUST be
  *pmob_obuf++ = intCnt;
  uint16_t crc16 = sdi_track_crc16(mob_obuf, 6, 0xFFFF);  // 6 Chars for CRC
  *pmob_obuf++ = (uint8_t)(crc16); // CRC L
  *pmob_obuf = (uint8_t)(crc16>>8); // CRC_h
  return mob_send_cmd(8); // Send 8
}

// Scan Konfig-String  (<0: Error, 0:Ende, >0: Kanaele) 
int16_t mbr_scan(void){
  char c, *pc=mbr.pcon;
  c=*pc++;
  if(c=='r') mbr.fkt=3; 
  else if(c=='h') mbr.fkt=4;
  else return -ERROR_PARAMETER; // only r oder h allowed
  mbr.addr=strtoul(pc,&pc,0); // 0 allowed as Broadcast
  mbr.reg=strtoul(pc,&pc,0); // Reg
  if(*pc==' ') pc++; // Opt. 1 WS!
  mbr.pficon=pc;  // Save ScanFfIi
  mbr.intCnt=0;
  int16_t anz=0;
  for(;;){
    c=*pc++;
    if(c=='f' || c=='F') mbr.intCnt+=2;
    else if(c=='i' || c=='I') mbr.intCnt++;
    else break; // EOL
    anz++;
  }
  mbr.pcon=pc;
  if(anz>MB_RAW_VALS) return -ERROR_PARAMETER; // Too many Regs!
  return anz;   // Anzahl Kanaele
}

/***********************************************************************
* mob_measure: Komplette Modbus-Messung durchfuehren
* Messprg steht in Konfig
***********************************************************************/
int16_t mob_measure(void) {
  int16_t res,anz;

  mbr.pcon=param.konfig; // Komplettmessung anhand Konfig-String
  mbr.sdireg_cnt=0;
  for(;;){
    anz=mbr_scan(); // Returns No of Values
    if(anz>0){
      nrf_gpio_pin_clear(MOB_NRXEN_PIN); // L:RX_on H:RX_off
      res = modbus_34_request(mbr.addr, mbr.fkt, mbr.reg, mbr.intCnt); 
      if (!res) {
        res = mob_get_reply(MODBUS_REPLY_WAIT); //(200) msec auf Antwort warten (Res <0 or >0)
      }
      nrf_gpio_pin_set(MOB_NRXEN_PIN); // L:RX_on H:RX_off

      if (res < 0){
        for (uint16_t i = 0; i < anz; i++)
        modbus_raw_fval[i] = res; 
      }else res=mob_scan_vals(); // Return >0 or <0

      uint16_t idx;
      float fval;
      for (uint16_t i = 0; i < anz; i++){
        // Werte nach  SDI12-Regs uebertragen
        idx=mbr.sdireg_cnt++; // In the End: mbr.sdireg_cnt<=anz_channels
        if(idx>=MAX_CHAN) return -ERROR_PARAMETER;
        sdi_valio.channel_val[idx].punit = "Uxxx";
        if (res>0) {
            fval = (float)modbus_raw_fval[idx];
            fval *= param.koeff[idx*2]; // Def. 1.0
            fval -= param.koeff[idx*2+1]; // Def. 0.0
        } else {
          fval = res; // Error..
        }
        snprintf(sdi_valio.channel_val[idx].txt, 11, "%+f", fval);
      }

      if(*mbr.pcon==0) break; // konfif ende erreicht
    }else break; // mbr_scan-Error
  }
  return res;
}

#if DEBUG
// === DEBUG START ===
//====== TEST COMMANDS FOR NEW SENSOR START_A ========
void messen_mobdbg_werte(void) {
  float fval;
  int16_t res;
  tb_printf("Measure MOB\n");
  mob_uart_init(SRC_CMDLINE);
  res = mob_measure();
  mob_uart_uninit(SRC_CMDLINE);
  if(res<=0) tb_printf("MOB Read Error/No Reply! %d\n", res);
  else  tb_printf("MOB Read OK: %d\n", res);    

  for(uint16_t i=0;i<mbr.sdireg_cnt;i++){
    tb_printf("#%d: %s %s\n", i,sdi_valio.channel_val[i].txt, sdi_valio.channel_val[i].punit);
  }
  return;
}

//====== TEST COMMANDS FOR NEW SENSOR END_A ========

// Additional Test-CMDs via TB_UART
// IMPORTANT: only SMALL capitals for User-CMDs!
bool debug_tb_cmdline(uint8_t *pc, uint32_t val) {
  switch (*pc) {
  //====== TEST COMMANDS FOR NEW SENSOR START_S ========
  case 'a': {
#ifdef MOB_PWR_WAIT
    nrf_gpio_pin_set(MOB_PWR_PIN);
    tb_delay_ms(MOB_PWR_WAIT);
#endif
    messen_mobdbg_werte();
#ifdef MOB_PWR_WAIT
    nrf_gpio_pin_clear(MOB_PWR_PIN);
#endif
  } break;
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
  // TT:'TerraTransfer', MOB:'Modbus', 'A': Ver A
  sprintf(sensor_id, "TT_MOB_A_0390_OSX%08X", mac_addr_l);

  // Try to read Parameters
  intpar_mem_read(ID_INTMEM_USER0, sizeof(param), (uint8_t *)&param);

  // Setup GPIOs
  nrf_gpio_cfg_output(MOB_PWR_PIN); // H:Pwr/Term ON L:Off
#ifdef MOB_PWR_WAIT
  nrf_gpio_pin_clear(MOB_PWR_PIN); // Normally OFF
#else
  nrf_gpio_pin_set(MOB_PWR_PIN); // Supply Sensor with Constant Power
#endif
  nrf_gpio_cfg_output(MOB_NRXEN_PIN); // L:RX_on H:RX_off
  nrf_gpio_pin_set(MOB_NRXEN_PIN);
  nrf_gpio_cfg_output(MOB_TXEN_PIN); // H:TX_on L:TX_off
  nrf_gpio_pin_clear(MOB_TXEN_PIN);
}

#ifdef MOB_PWR_WAIT
#define WAIT_MS MOB_PWR_WAIT
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
    sdi_valio.m_msec = WAIT_MS + MODBUS_REPLY_WAIT;
  } else if (carg == 1) {
    sdi_valio.anz_channels = 3;
    sdi_valio.m_msec = WAIT_MS + MODBUS_REPLY_WAIT + 50;
  } else
    return false;

  return true;
}

int16_t sensor_valio_measure(uint8_t isrc) {
  //---- 'M': While waiting: scan SDI for <BREAK> ----
  int16_t res;
  float fval;

#ifdef MOB_PWR_WAIT
  // Supply Sensor with Power
  nrf_gpio_pin_set(MOB_PWR_PIN);
#endif
  res = sensor_wait_break(isrc, WAIT_MS);
  if (res) {
#ifdef MOB_PWR_WAIT
    nrf_gpio_pin_clear(MOB_PWR_PIN);
#endif
    return res;
  }
  // --- 'm' Wait end

  // Read Data from Sensor
  mob_uart_init(isrc);
  res = mob_measure();
  mob_uart_uninit(isrc);
#ifdef MOB_PWR_WAIT
  nrf_gpio_pin_clear(MOB_PWR_PIN);
#endif

  // Prepare Output
  /*
  sdi_valio.channel_val[0].punit = "%rH";
  if (res>=0) {
    fval = mob_hum;
    fval *= param.koeff[2]; // Def. 1.0
    fval -= param.koeff[3]; // Def. 0.0
  } else {
    fval = res; // Error..
  }
  snprintf(sdi_valio.channel_val[0].txt, 11, "%+.1f", fval);

  sdi_valio.channel_val[1].punit = "oC";
  if (res>=0) {
    fval = mob_temp;
    fval *= param.koeff[0]; // Def. 1.0
    fval -= param.koeff[1]; // Def. 0.0
  } else {
    fval = res;
  }
  snprintf(sdi_valio.channel_val[1].txt, 11, "%+.2f", fval);
*/

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
    if (*pc != '!')
      return;
    // Send Koeffs
    param.koeff[pidx] = fval;
    sprintf(sdi_obuf, "%cK%d=%f", my_sdi_adr, pidx, fval);

  } else if (!strcmp(pc, "Write!")) { // Write SDI_Addr and Koefficients to Memory
    intpar_mem_erase();               // Compact Memory
    intpar_mem_write(ID_INTMEM_SDIADR, 1, (uint8_t *)&my_sdi_adr);
    intpar_mem_write(ID_INTMEM_USER0, sizeof(param), (uint8_t *)&param);
    sprintf(sdi_obuf, "%c", my_sdi_adr);       // Standard Reply
  } else if (!strcmp(pc, "Sensor!")) {         // Identify Senor
    sprintf(sdi_obuf, "%cMOB_A!", my_sdi_adr); // Standard Reply
  }                                            // else ..
}

//***