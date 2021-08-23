//**** Device.h - Defines Sensor Type and Features  ***

// *** Select DEVICE_TYP to build here: ***
// Typen 100-299 sind Bootloader/Inits 300-999 Sensoren! >=1000..xx FS-Anwendungen
// ---Bootloaders---
#define DEVICE_TYP  200 // Dummy For Tests or Bootloader
// ---Sensors---
//#define DEVICE_TYP  300 // *** Pewatron_Ceramic Pressure Sensor ***
//#define DEVICE_TYP  310 // *** Keller_LD Piezo Pressure Sensor ***
//#define DEVICE_TYP  320 // *** Baro MS5607 Sensor ***
//#define DEVICE_TYP  330 // *** Frequency Counter ***
//#define DEVICE_TYP  340 // *** Sensirion SHT2x Temperature/Humidity ***
//#define DEVICE_TYP  350 // *** 24Bit A/D-Converter ADS1220  ***


// Features
#if DEVICE_TYP == 200
  // Div. LTX-Tracker/LTX-Pegel BLE-User-"Device for Bootloader" - Minimalversion fuer Sensoren OHNE Speicher
  // Der BLE-Bootloader kann garnix, NUR (opt.) Disk, Terminal und BLE, damit kann man aber andere Firmware nachladen
  //#define HAS_FS // 200 has NO Filesystem!
  #define ENABLE_BLE // Wenn definiert SD anwerfen fuer IRQs
  #define DEVICE_FW_VERSION 1 // Release in Steps of 10 (35 == V3.5, 1: V0.1) // FW_VWEION uns TYP wird vei CONTENT mitgeschickt
#endif

#if DEVICE_TYP == 300
  // *** Pewatron_Ceramic Pressure Sensor KKD18 ***
  //#define HAS_FS // has NO Filesystem!
  #define ENABLE_BLE // Wenn definiert SD anwerfen fuer IRQs
  #define DEVICE_FW_VERSION 1 // Release in Steps of 10 (35 == V3.5, 1: V0.1) // FW_VWEION uns TYP wird vei CONTENT mitgeschickt
#endif

#if DEVICE_TYP == 310
  // *** Keller_LD Piezo Pressure Sensor ***
  //#define HAS_FS // has NO Filesystem!
  #define ENABLE_BLE // Wenn definiert SD anwerfen fuer IRQs
  #define DEVICE_FW_VERSION 1 // Release in Steps of 10 (35 == V3.5, 1: V0.1) // FW_VWEION uns TYP wird vei CONTENT mitgeschickt
#endif

#if DEVICE_TYP == 320
  // *** Baro MS5607 Sensor ***
  //#define HAS_FS // has NO Filesystem!
  #define ENABLE_BLE // Wenn definiert SD anwerfen fuer IRQs
  #define DEVICE_FW_VERSION 1 // Release in Steps of 10 (35 == V3.5, 1: V0.1) // FW_VWEION uns TYP wird vei CONTENT mitgeschickt
#endif

#if DEVICE_TYP == 330
  // *** Frequency Counter ***
  //#define HAS_FS // has NO Filesystem!
  #define ENABLE_BLE // Wenn definiert SD anwerfen fuer IRQs
  #define DEVICE_FW_VERSION 1 // Release in Steps of 10 (35 == V3.5, 1: V0.1) // FW_VWEION uns TYP wird vei CONTENT mitgeschickt
#endif

#if DEVICE_TYP == 340
  // *** Sensirion SHT2x Temperature/Humidity ***
  //#define HAS_FS // has NO Filesystem!
  #define ENABLE_BLE // Wenn definiert SD anwerfen fuer IRQs
  #define DEVICE_FW_VERSION 1 // Release in Steps of 10 (35 == V3.5, 1: V0.1) // FW_VWEION uns TYP wird vei CONTENT mitgeschickt
#endif

#if DEVICE_TYP == 350
  // *** 24Bit A/D-Converter ADS1220  ***
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



