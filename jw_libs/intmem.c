/****************************************************
* intmem.c - Internal NVM Memory Helpers
*
* (C) joembedded@gmail.com - joembedded.de
*
*
***************************************************/

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "nrf_nvmc.h"

#include "./platform_nrf52/tb_pins_nrf52.h"

#include "tb_tools.h"
#include "device.h"

#include "sdi12sensor_drv.h"
#include "bootinfo.h"
#include "intmem.h"


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
int16_t intpar_mem_read(uint8_t parid, uint8_t pbytes_max, uint8_t *pdata){
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
        if(pdata && pbytes<=pbytes_max){  // Move Parameter to Dest.. Regard Maxiumum
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

//***