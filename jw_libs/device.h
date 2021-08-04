//**** Device.h - Defines Sensor Type and Features  ***

// *** Select DEVICE_TYP to build here: ***
// Typen 100-299 sind Bootloader/Inits 300-999 Sensoren! >=1000..xx FS-Anwendungen
// ---Bootloaders---
//#define DEVICE_TYP  200 // Basisbootloader OHNE Serielles Flash
// ---Sensors---
#define DEVICE_TYP  300 // *** Pewatron_Ceramic Pressure Sensor ***


// Features
#if DEVICE_TYP == 200
  // Div. LTX-Tracker/LTX-Pegel BLE-User-"Device for Bootloader" - Minimalversion fuer Sensoren OHNE Speicher
  // Der BLE-Bootloader kann garnix, NUR (opt.) Disk, Terminal und BLE, damit kann man aber andere Firmware nachladen
  //#define HAS_FS // 200 has NO Filesystem!
  //#define ENABLE_BLE // Wenn definiert SD anwerfen fuer IRQs

  #define DEVICE_FW_VERSION 1 // Release in Steps of 10 (35 == V3.5, 1: V0.1) // FW_VWEION uns TYP wird vei CONTENT mitgeschickt
#endif

#if DEVICE_TYP == 300
  // *** Pewatron_Ceramic Pressure Sensor KKD18 ***
  //#define HAS_FS // has NO Filesystem!
  #define ENABLE_BLE // Wenn definiert SD anwerfen fuer IRQs
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



