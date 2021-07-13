// Bootinfo.c - Get Firmware-Info DiesundDas, kann z.B verifizieren etc.

#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include "device.h"
#include "bootinfo.h"
#include "jesfs.h"
#include "jesfs_int.h"

// Header Type0 of JesFsHex2Bin FILE
#define HDR0_MAGIC 0xE79B9C4F
typedef struct {
    uint32_t hdrmagic;     // 0 MagicHeader Type0: HDR0_MAGIC
    uint32_t hdrsize;      // 1 Size in Bytes (Type0: 32 for 8 uint32 or 48 if with security_key)
    uint32_t binsize;      // 2 Size of following BinaryBlock
    uint32_t binload;      // 3 Adr0 of following BinaryBlock
    uint32_t crc32;        // 4 CRC32 of following BinaryBlock
    uint32_t timestamp;    // 5 UnixSeconds of this file
    uint32_t binary_start; // 6 StartAddress Binary (Parameter 2 of 'h')
    uint32_t resv0;        // 7 Reserved, 0xFFFFFFFF

    // Extra-Data
    uint8_t security_key[16];  // Optional AES-Key if Header Len >32 and Content not 16*FF
    uint32_t mac_addr_h;   // MAC detected by Bootloader (should match User Firmware MAC, else: 
    uint32_t mac_addr_l;   // Software was Cross-Copied (CPU Image)
    uint32_t pin;

} HDR0_TYPE;

// Possible, dass Extra-Data nicht dabei sind
static HDR0_TYPE* check_bootloader_present(void){
  HDR0_TYPE *pbl_memory = (HDR0_TYPE *)BOOTLOADER_SETTINGS_ADDRESS;
  if (pbl_memory->hdrmagic == HDR0_MAGIC && pbl_memory->hdrsize >= 32 ) return pbl_memory;
  else return NULL;
}

// Check the Bootloader Timestamp, returns Seconds if present
uint32_t get_firmware_bootloader_cookie(void){
  HDR0_TYPE *pbl_memory = check_bootloader_present();
  if(pbl_memory) return pbl_memory->timestamp; // Erstellungstimestamp des HEX-Files
  else return 0; // No Bootloader (<1526030617 or >2472110617: illegal)
}

// Nur wenn MACs gleich sind und Security-Key 
uint8_t* get_security_key_ptr(void){
  HDR0_TYPE *pbl_memory = check_bootloader_present();
  if(pbl_memory && pbl_memory->hdrsize>=48 &&  memcmp(pbl_memory->security_key,"\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFF",16)){
    // Neu im BL 
    if(pbl_memory->hdrsize>=64){
        if(pbl_memory->mac_addr_h!=mac_addr_h || pbl_memory->mac_addr_l!=mac_addr_l) return NULL;
    }
    return pbl_memory->security_key;
  }
  return NULL;
}

// No PIN if No Bootloader. Older Bootloader: Pin 0xFFFFFFFF ( 4294967295 )
uint32_t get_pin(void){
  HDR0_TYPE *pbl_memory = check_bootloader_present();
  if(pbl_memory) return pbl_memory->pin;
  else return 0;
}

// End
