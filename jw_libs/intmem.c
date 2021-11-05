/****************************************************
* intmem.c - Internal NVM Memory Helpers
*
* (C) joembedded@gmail.com - joembedded.de
* V1.1 - >255 Bytes now possible
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
  uint16_t total_bytes; // No of following data in bytes (can be 0)
  uint32 qdata[qwords]; // Allocated words ((Bytes+3)&252)

  V1.1: Will write max 32k Bytes in Junks of max. 252 Bytes

  NRF52832 CPU:  Flash: 0x6F000 / 454656.d
  NRF52840 CPU:  Flash: 0xEF000 / 978944.d

  Usage (e.g.):
    - tb_printf("m: %d\n", intpar_mem_write(val, 0, NULL)); // Test

    - tb_printf("l: %d\n", intpar_mem_write(val, strlen(pc + 1), pc + 1));

    - res = intpar_mem_read(val, 255, pc);
      if (res > 0) for (int i = 0; i < res; i++) tb_printf("%x ", pc[i]);

    - intpar_mem_erase();
*/

#define IMTMEM_SIZE   CPU_SECTOR_SIZE // 4096 for NRF52
#define INTMEM_START  (IBOOT_FLASH_START+IBOOT_FLASH_SIZE-IMTMEM_SIZE) // Located at End of Boot-Memory

// Result >=0: Rel Pos. in INTMEM, <0: ERROR
// May write only parts, if avl. Memory is to low. So use only big blocks
// if enough Mem is available.
int16_t intpar_mem_write(uint8_t parid, uint16_t total_pbytes, uint8_t *pdata){
  uint16_t plen;  // Len of Parameter Block in Bytes M4
  uint32_t mem_addr;
  uint8_t pbytes;
  mem_addr =  INTMEM_START;
  for(;;){
    if ((*(uint32_t*)mem_addr)==0xFFFFFFFF) break;
    if (mem_addr >= (INTMEM_START+IMTMEM_SIZE)) break;
    plen=((*(uint8_t*)(mem_addr+3))+3)&252;
    mem_addr+= plen+4; // Next Entry
  }
  for(;;){
    if(total_pbytes>252) pbytes=252;
    else pbytes = total_pbytes;
    plen=(pbytes+3)&252; // No of bytes to allocate
    if((int32_t)((INTMEM_START+IMTMEM_SIZE) - mem_addr)<(plen+4)){
      return -2;  // Out of Memory (may only partly written!)
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
    };
    mem_addr+=plen; // Allocates Memory
    total_pbytes -= pbytes; // Transferred Memory
    pdata += pbytes; // Transferred Memory
    if(!total_pbytes) break;  // All written
  }
  return (int16_t)(mem_addr-INTMEM_START); // Return End of Memory
}

// Read (last valid) Parameter to intmem_buf - Result Copy if pdata != NULL
// If strings must be written: Optionally regard the trailing 0!
// Chained bigblocks (len >=252 ) will build 1 block
int16_t intpar_mem_read(uint8_t parid, uint16_t pbytes_max, uint8_t *pdata_p0){
  uint32_t mem_addr;
  mem_addr =  INTMEM_START;
  int16_t res =-1;  // Not Found
  uint8_t pbytes;
  uint8_t mparid;
  uint16_t plen;  // Len of Parameter Block in Bytes M4
  uint16_t crc;
  uint8_t *pdata_run=pdata_p0;
  int16_t pbytes_total=0;
  for(;;){
    if ((*(uint32_t*)mem_addr)==0xFFFFFFFF) break;
    if (mem_addr >= (INTMEM_START+IMTMEM_SIZE)) break;
    pbytes=(*(uint8_t*)(mem_addr+3)); // Real
    mparid=(*(uint8_t*)(mem_addr+2));
    //tb_printf("?%u - %x:ID:%u Plen:%u\n",parid,mem_addr,mparid,pbytes); // Debug
    if(mparid==parid){
      crc=sdi_track_crc16((uint8_t*)(mem_addr+2),2+pbytes,/*Init*/ 0);  // First 2 Bytes
      if(crc != *(uint16_t*)(mem_addr)){
        res=-3; // Found, but CRC not OK for this Par
      }else{
        if(pdata_p0 && pbytes_total<=pbytes_max){  // Move Parameter to Dest.. Regard Maxiumum
          memcpy(pdata_run,(uint8_t*)(mem_addr+4), pbytes); // DSN
          res=pbytes_total+pbytes;
        }else{
          res=-4; // Not enough Space
        }
      }
      if(pbytes==252){
        pdata_run+=pbytes;
        pbytes_total+=pbytes;
      }else{
        pdata_run=pdata_p0;
        pbytes_total=0;
      }
    }else{
      pdata_run=pdata_p0;
      pbytes_total=0;
    }
    plen=(pbytes+3)&252;
    mem_addr+= (plen+4); // Next Entry
  }
  return res; // If OK: Return total Nr. of Bytes (opt. Sum of all blocks)
}
// Clear internal Memory
void intpar_mem_erase(void){
  nrf_nvmc_page_erase(INTMEM_START);
}

//***