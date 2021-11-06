/* device.c */

#include <stdint.h>

#include "device.h"


uint32_t mac_addr_h,mac_addr_l; // Muss von MAIN gesetzt werden, da das SD Zugriff auf Register blockt
const uint16_t device_type = DEVICE_TYP;
const uint16_t device_fw_version = DEVICE_FW_VERSION;


//**
