/****************************************************
* type_main.c - Test Sensor SDI12-Basics
*
* (C) joembedded@gmail.com - joembedded.de
*
* UART is shared with tb_tools
*
***************************************************/

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "nrf_drv_gpiote.h"

#include "./platform_nrf52/tb_pins_nrf52.h"
#include "osx_pins.h"

#include "tb_tools.h"
#include "osx_main.h"
#include "device.h"

#include "sdi12sensor_drv.h"
#include "intmem.h"

// ---Globals---
#define ID_INTMEM_SDIADR  1    // Memory ID fuer Addresse
char my_sdi_adr = '0'; // Factory Default
#define MAX_RESULT 80
char result_string[MAX_RESULT+1];


// ===Sensor Part Start ===
// Id has fixed structure, max. 30+'\0'
//                      all cccccccc.8 mmmmmm.6 vvv.3 xx..xx.[0-13]
char sensor_id[8+6+3+13+1]= "JoEmbedd" "Testse" "OSX" "Sno..";

// SDI12 allows max. 9 Chars or 7 Digits! 
// E.g use 'snprintf(channel_val[].txt,10,"%+f",...)' 
// and check for >< 10 Mio, '+9999999.' is not legal
#define MAX_CHAN 10 // Note: Logger-Driver can acept >10
typedef struct{
  int8_t didx; // Index for 'D'/'R'-Commands, -1: None
  char txt[11]; // Output in SDI121 Format '+/-dd.ddddd', max. 10 char
} CHANNEL_VAL;
CHANNEL_VAL channel_val[MAX_CHAN];


void sensor_init(void){
  // Set SNO to Low Mac, so same Name as BLE Advertising
  sprintf(sensor_id+17,"%08X",mac_addr_l);
}

//---- Sensor CMDs Start ------------
void sensor_cmd_m(uint8_t carg){
  if(carg>0) return;  // only !M supported!

  tb_delay_ms(9); // Default Delay after CMD
  sprintf(sdi_obuf,"%c0053\r\n",my_sdi_adr); // 3 Measures in 5 secs
  sdi_send_reply(NULL); // send SDI_OBUF
  
  // Check for BRAK in 'M'... todo
  tb_delay_ms(2000); // Measure... (faster than time above)
  snprintf(channel_val[0].txt,11,"%+f", (float)tb_time_get()/1.234);
  snprintf(channel_val[1].txt,11,"+%u", tb_get_ticks()%1000000); // '+* only d/f
  channel_val[2].txt[0]=0; // End

  // Build Outstring (Regarding max length 35/75.. todo)
  channel_val[0].didx=0;
  channel_val[1].didx=0;
  channel_val[2].didx=-1;

  sprintf(sdi_obuf,"%c\r\n",my_sdi_adr); // Service Request
  sdi_send_reply(NULL); // send SDI_OBUF
  tb_delay_ms(28); // Needs 25 msec
  *sdi_obuf=0; // Assume no reply
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
static void rxirq_on(void){
    uint32_t err_code;
    if(rxirq_active) return;  // Already ON
    nrf_drv_gpiote_in_config_t in_config = NRFX_GPIOTE_CONFIG_IN_SENSE_HITOLO(false);
    err_code = nrf_drv_gpiote_in_init(SDI_RX_PIN, &in_config, rxirq_handler);
    APP_ERROR_CHECK(err_code);
    nrf_drv_gpiote_in_event_enable(SDI_RX_PIN, true);
    rxirq_active = true;
}
static void rxirq_off(void){
    if(!rxirq_active) return;  // Already OFF
    nrf_drv_gpiote_in_event_disable(SDI_RX_PIN);
    nrf_drv_gpiote_in_uninit(SDI_RX_PIN); // Disable Functins an Pullups
    rxirq_active = false;
}


//--- Init (only call once)
void type_init(void){
    uint32_t err_code;
    err_code = nrf_drv_gpiote_init();
    APP_ERROR_CHECK(err_code);

    intpar_mem_read(ID_INTMEM_SDIADR,1,(uint8_t*)&my_sdi_adr);
    sensor_init();  // ID etc..

    rxirq_on(); // SDI now active
}

// Static parts of CMDs
static bool cmdcrc_flag=false;

bool type_service(void){
  int16_t res;
  int16_t txwait_chars = 0;
  char *pc,arg_val0;
  int8_t i8h; // Temp
  uint16_t crc16;

   // SDI12 Activity registered
   if(rxirq_zcnt){
    rxirq_off();
    tb_putc('S'); // Signal SDI12 Activity
    *result_string=0;
    tb_delay_ms(1);
    sdi_uart_init();
    for(;;){
      res=sdi_getcmd(SDI_IBUFS, 100 /*ms*/); // Max. wait per Def.
      if(res == -ERROR_NO_REPLY) {
        txwait_chars = 0;
        break; // Timeout
      }
      else if(res > 0 && sdi_ibuf[res-1]=='!'){ // Only Commands (end with '!')
        if(*sdi_ibuf=='?' || *sdi_ibuf==my_sdi_adr){

          // Save Request
          strncpy(result_string, sdi_ibuf, MAX_RESULT); //DSn
          // Fast scan CMD via switch() - reply only to valid CMDs
          *sdi_obuf=0; // Assume no reply
          pc=&sdi_ibuf[2];
          switch(sdi_ibuf[1]){  
          case '!': // Only "!\0"
            if(sdi_ibuf[2]) sprintf(sdi_obuf,"%c\r\n",my_sdi_adr);
            break;
          case 'I': // SDI V1.3
            if(!strcmp(sdi_ibuf+2,"!")) sprintf(sdi_obuf,"%c13%s\r\n",my_sdi_adr, sensor_id);
            break;
          case 'A':
            arg_val0=sdi_ibuf[2];
            if(!strcmp(sdi_ibuf+3,"!") && (
              (arg_val0>='0' && arg_val0<='9') ||
              (arg_val0>='A' && arg_val0<='Z') ||
              (arg_val0>='a' && arg_val0<='z') )){

              if(arg_val0!=my_sdi_adr){
                intpar_mem_write(ID_INTMEM_SDIADR,1,(uint8_t*)&arg_val0);
                intpar_mem_read(ID_INTMEM_SDIADR,1,(uint8_t*)&my_sdi_adr);
              }

              if(sdi_ibuf[2]) sprintf(sdi_obuf,"%c\r\n",my_sdi_adr);
            }
            break;
          case 'M': // aM!, aMx!, aMC!, aMCx!, pc points after 'M' 
            if(*pc=='C'){
              cmdcrc_flag=true;
              pc++;
            }else cmdcrc_flag=false;
            if(*pc=='!') arg_val0=0;
            else arg_val0 = (*pc++)-'0';
            if(!strcmp(pc,"!") && arg_val0>=0 && arg_val0<=9){
              sensor_cmd_m((uint8_t)arg_val0);
            }
            break;
          case 'D': // D0-D9
            arg_val0=sdi_ibuf[2]-'0';
            if(!strcmp(sdi_ibuf+3,"!") && arg_val0>=0 && arg_val0<=9){
              pc=sdi_obuf;
              *pc++=my_sdi_adr;
              *pc=0;
              for(uint16_t i;i<MAX_CHAN;i++){
                i8h = channel_val[i].didx;
                if(i8h==-1) break;
                if(i8h == arg_val0){
                  strcpy(pc,channel_val[i].txt);
                  pc+=strlen(channel_val[i].txt);
                }
              }
              if(cmdcrc_flag){
                crc16=sdi_track_crc16(sdi_obuf,(pc-sdi_obuf), 0 /*Init*/);
                *pc++=64+((crc16>>12)&63);
                *pc++=64+((crc16>>6)&63);
                *pc++=64+(crc16&63);
              }
              strcpy(pc,"\r\n");
              // Now D-String is ready
            }
            break;

            // default: No Reply!
          } // switch

          if(*sdi_obuf){ // Send Reply if present 
            tb_delay_ms(9); // Default Delay after CMD
            txwait_chars = sdi_send_reply(NULL); // send SDI_OBUF
          }
       }
      }
    } // for()
    if(txwait_chars) tb_delay_ms(txwait_chars*9); // 1 Char needs 8.33 msec
    sdi_uart_uninit();
    rxirq_on();
    rxirq_zcnt=0;

    if(*result_string) tb_printf("->R'%s'\r\n",result_string); 
    return true; // No Periodic Service
   }
   return false; // Periodic Service
}


// Die Input String und Values
void type_cmdline(uint8_t isrc, uint8_t *pc, uint32_t val, uint32_t val2){

  int res;

  switch(*pc){
  case 'm':
    tb_printf("m: %d\n",intpar_mem_write(val,0,NULL)); // Testparameter schreiben, check mit 'H'
    break;

  case 'l':
    tb_printf("l: %d\n",intpar_mem_write(val,strlen(pc+1),pc+1)); // Testparameter schreiben, check mit 'H'
    break;

  case 's':
    res=intpar_mem_read(val,255,pc);
    tb_printf("l: %d:",res);
    if(res>0) for(int i=0;i<res;i++) tb_printf("%x ",pc[i]);
    tb_printf("\n");
    break;

  case 'k':
    intpar_mem_erase(); 
    break;

  case 'd':
    break;

  default:
    tb_printf("???\n");
  }
}

//***