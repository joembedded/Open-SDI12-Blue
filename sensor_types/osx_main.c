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
  uint8_t bytes; // No of following data in bytes (can be 0)
  uint32 qdata[qwords]; // Allocated words ((Bytes+3)&252)

  NRF52832 CPU:  Flash: 0x6F000 / 454656.d
  NRF52840 CPU:  Flash: 0xEF000 / 978944.d
*/

#define IMTMEM_SIZE   CPU_SECTOR_SIZE // 4096 for NRF52
#define INTMEM_START  (IBOOT_FLASH_START+IBOOT_FLASH_SIZE-IMTMEM_SIZE) // Located at End of Boot-Memory

// Result >=0: Rel Pos. in INTMEM, <0: ERROR
int16_t intpar_mem_write(uint8_t parid, uint8_t pbytes, uint8_t *pdata){
  uint16_t plen;  // Len of Parameter Block in Bytes M4
  uint32_t mem_addr;
  mem_addr =  INTMEM_START;
  for(;;){
    if ((*(uint32_t*)mem_addr)==0xFFFFFFFF) break;
    if (mem_addr >= (INTMEM_START+IMTMEM_SIZE)) break;
    plen=((*(uint8_t*)(mem_addr+3))+3)&252;
    mem_addr+= plen+4; // Next Entry
  }
  plen=(pbytes+3)&252; // No of bytes to allocate
  if((int32_t)((INTMEM_START+IMTMEM_SIZE) - mem_addr)<(plen+4)){
    return -2;  // Out of Memory
  }
  uint8_t hdr[4]; 
  hdr[2]=parid;
  hdr[3]=pbytes;

  uint16_t crc;
  crc=sdi_track_crc16(hdr+2,2,/*Init*/ 0);  // First 2 Bytes
  if(pbytes){
    crc=sdi_track_crc16(pdata,pbytes,crc);  // Data Block
  }
  *(uint16_t*)(hdr)=crc; // Add CRC
  nrf_nvmc_write_words(mem_addr, (uint32_t *)hdr, 1);
  mem_addr+=4;
  if(pbytes){
    nrf_nvmc_write_words(mem_addr, (uint32_t *)pdata, plen/4);
    mem_addr+=plen;
  };
  return (int16_t)(mem_addr-INTMEM_START); // Return End of Memory
}

// Read (last valid) Parameter to intmem_buf - Result Copy if pdata != NULL
// If strings must be written: Optionally regard the trailing 0!
int16_t intpar_mem_read(uint8_t parid, uint8_t *pdata){
  uint32_t mem_addr;
  mem_addr =  INTMEM_START;
  int16_t res =-1;  // Not Found
  uint8_t pbytes;
  uint8_t mparid;
  uint16_t plen;  // Len of Parameter Block in Bytes M4
  uint16_t crc;
  for(;;){
    if ((*(uint32_t*)mem_addr)==0xFFFFFFFF) break;
    if (mem_addr >= (INTMEM_START+IMTMEM_SIZE)) break;
    pbytes=(*(uint8_t*)(mem_addr+3));
    mparid=(*(uint8_t*)(mem_addr+2));
    
    if(mparid==parid){
      crc=sdi_track_crc16((uint8_t*)(mem_addr+2),2+pbytes,/*Init*/ 0);  // First 2 Bytes
      if(crc != *(uint16_t*)(mem_addr)){
        res=-3; // Found, but CRC not OK for this Par
      }else{
        if(pdata){  // Move Parameter to Dest..
          memcpy(pdata,(uint8_t*)(mem_addr+4), pbytes); // DSN
        }
        res=pbytes;
      }
    }
    plen=(pbytes+3)&252;
    mem_addr+= plen+4; // Next Entry
  }
  return res;
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

  int res;

  switch(*pc){
  case 'm':
    tb_printf("m: %d\n",intpar_mem_write(val,0,NULL)); // Testparameter schreiben, check mit 'H'
    break;

  case 'l':
    tb_printf("l: %d\n",intpar_mem_write(val,strlen(pc+1),pc+1)); // Testparameter schreiben, check mit 'H'
    break;

  case 's':
    res=intpar_mem_read(val,pc);
    tb_printf("l: %d:",res);
    if(res>0) for(int i=0;i<res;i++) tb_printf("%x ",pc[i]);
    tb_printf("\n");
    break;

  case 'k':
    intpar_mem_erase(); 
    break;


  default:
    tb_printf("???\n");
  }
}

//***