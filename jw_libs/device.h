//**** Device.h  ***

// Typen 100-999 sind Bootloader oder >= 500 Sensoren! 1000..xx FS-Anwendungen
// ---Bootloader---
#define DEVICE_TYP  200 // Basisbootloader OHNE Serielles Flash

#if DEVICE_TYP == 200
  // LTX-Tracker/LTX-Pegel BLE-User-"Device for Bootloader" - Minimalversion fuer Sensoren OHNE Speicher
  // Der BLE-Bootloader kann garnix, NUR (opt.) Disk, Terminal und BLE, damit kann man aber andere Firmware nachladen
  //#define HAS_FS // 200 has NO Filesystem!

  //#define ENABLE_BLE // Wenn definiert SD anwerfen fuer IRQs

  #define DEVICE_FW_VERSION 1 // Release in Steps of 10 (35 == V3.5, 1: V0.1) // FW_VWEION uns TYP wird vei CONTENT mitgeschickt
#endif

#ifndef DEBUG
#if !defined(ENABLE_BLE)
  #warning "Release: BLE OFF?"
#endif
#endif

extern uint32_t mac_addr_h,mac_addr_l; 
extern uint8_t ledflash_flag;    // Radio-Active-Notification/Heartbeat/Etc
// **



