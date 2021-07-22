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
#include "bootinfo.h"

// ===Sensor Part Start ===
// Id has fixed structure, max. 30+'\0'
//                      all cccccccc.8 mmmmmm.6 vvv.3 xx..xx.[0-13]
char sensor_id[8+6+3+13+1]= "JoEmbedd" "Testse" "OSX" "Sno..";

void sensor_init(void){
  // Set SNO to Low Mac, so same Name as BLE Advertising
  sprintf(sensor_id+17,"%08X",mac_addr_l);
}
// ===Sensor Part END ===

// ---Globals---
char my_sdi_addr = '3';
#define MAX_RESULT 80
char result_string[MAX_RESULT+1];

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

/* Calculating the SDI12 CRC16: Also useful for external use. 
* SDI12: Init CRC_RUN with 0x0 (Same Polynom is used for MODBUS, but Init is 0xFFFF) */
#define POLY16 0xA001
uint16_t sdi_track_crc16(uint8_t *pdata, uint16_t wlen, uint16_t crc_run) {
    uint8_t j;
    while (wlen--) {
        crc_run ^= *pdata++;
        for (j = 0; j < 8; j++) {
            if (crc_run & 1)
                crc_run = (crc_run >> 1) ^ POLY16;
            else
                crc_run = crc_run >> 1;
        }
    }
    return crc_run;
}


#include "nrf_nvmc.h"

/* Memory Access - Defines SECTOR 1 Sized internal parameter access
  uint16_t crc16; // CRC of parid, qwords and all data bytes
  uint8_t parid;  // Allowed 0..xFE
  uint8_t qwords; // No of following data (as uint32 (can be 0))
  uint32 qdata[qwords];

  NRF52832 CPU:  Flash: 0x6F000 / 454656.d
  NRF52840 CPU:  Flash: 0xEF000 / 978944.d
*/
#define MAX_DW 16 // Maximum No of uint32 Params max. 0xFE
uint8_t intmem_buf[MAX_DW+1]; // Buffer to IntMem

#define IMTMEM_SIZE   CPU_SECTOR_SIZE // 4096 for NRF52
#define INTMEM_START  (IBOOT_FLASH_START+IBOOT_FLASH_SIZE-IMTMEM_SIZE) // Located at End of Boot-Memory

// Result >=0: Rel Pos. in INTMEM, <0: ERROR
int16_t intpar_mem_write(uint8_t parid, uint8_t qwords, uint8_t *pdata){
  uint16_t plen;  // Len of Parameter Blox in Bytes
  uint16_t crc;
  uint32_t mem_addr;
  intmem_buf[2]=parid;
  intmem_buf[3]=qwords;
  if(qwords){
    if(qwords>MAX_DW) return -1;  // Too much data
    plen=qwords*4;
    memcpy(intmem_buf+4,pdata,plen);  // DSN
  }else plen=0;

  mem_addr =  INTMEM_START;
  for(;;){
    if ((*(uint32_t*)mem_addr)==0xFFFFFFFF) break;
    if (mem_addr >= (INTMEM_START+IMTMEM_SIZE)) break;
    mem_addr+= ((*(uint8_t*)(mem_addr+3))*4)+4; // Next Entry
  }
  if((int32_t)((INTMEM_START+IMTMEM_SIZE) - mem_addr)<(plen+4)){
    return -2;  // Out of Memory
  }

  crc=sdi_track_crc16(intmem_buf+2,plen+2,/*Init*/ 0);
  *(uint16_t*)(intmem_buf)=crc; // Add CRC
  nrf_nvmc_write_words(mem_addr, (uint32_t *)intmem_buf, qwords+1);
  return (int16_t)(mem_addr-INTMEM_START);
}
// Read (last valid) Parameter to intmem_buf - Result 
int16_t intpar_mem_read(uint8_t parid){
  // todo
  return 0; 
}
// Clear internal Memory
void intpar_mem_erase(void){
  nrf_nvmc_page_erase(INTMEM_START);
}


//--- Init (only call once)
void type_init(void){
    uint32_t err_code;
    err_code = nrf_drv_gpiote_init();
    APP_ERROR_CHECK(err_code);

    sensor_init();  // ID etc..

    rxirq_on(); // SDI now active
}

bool type_service(void){
  int16_t res;
  int16_t txwait_chars = 0;
   // SDI12 Activity registered
   if(rxirq_zcnt){
    rxirq_off();
    tb_putc('S'); // Signal SDI12 Activity
    tb_delay_ms(1);
    sdi_uart_init();
    for(;;){
      res=sdi_getcmd(SDI_IBUFS, 100 /*ms*/); // Max. wait per Def.
      if(res == -ERROR_NO_REPLY) {
        txwait_chars = 0;
        break; // Timeout
      }
      else if(res > 0 && sdi_ibuf[res-1]=='!'){ // Only Commands
        if(*sdi_ibuf=='?' || *sdi_ibuf==my_sdi_addr){

          // Save Request
          strncpy(result_string, sdi_ibuf, MAX_RESULT); //DSn
          // Fast scan CMD via switch() - reply only to valid CMDs
          *sdi_obuf=0; // Assume no reply
          switch(sdi_ibuf[1]){  
          case '!': // Only "!\0"
            if(sdi_ibuf[2]) sprintf(sdi_obuf,"%c\r\n",my_sdi_addr);
            break;
          case 'I':
            if(!strcmp(sdi_ibuf+2,"!")) sprintf(sdi_obuf,"%c12%s\r\n",my_sdi_addr, sensor_id);
            break;
          } // switch
          if(*sdi_obuf){
            tb_delay_ms(9); 
            txwait_chars = sdi_send_reply(NULL); // send SDI_OBUF
          }
       }
      }
    } // for()
    if(txwait_chars) tb_delay_ms(txwait_chars*9); // OK for 6 Chars (1 Char needs 8.33 msec)
    sdi_uart_uninit();
    rxirq_on();
    rxirq_zcnt=0;

    tb_printf("->R'%s'\r\n",result_string); 

   }
  return false; // Service
}

// Die Input String und Values
void type_cmdline(uint8_t isrc, uint8_t *pc, uint32_t val, uint32_t val2){
  switch(*pc){
  case 'm':
    tb_printf("M: %d\n",intpar_mem_write(val,0,NULL)); // Testparameter schreiben, check mit 'H'
    break;

  case 'k':
    intpar_mem_erase(); 
    break;


  default:
    tb_printf("???\n");
  }
}

//***