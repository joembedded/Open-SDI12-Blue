/*******************************************************************************
*  onewire.h - driver for 1Wire-devices
*
* Mainly designed for DS18S20 and DS18B20 Devices 
* Read also onewire.c for Details
*
*******************************************************************************/


/********************************************************************
* Globals and Defs
*********************************************************************/
#define OW_ERROR_NONE         0   // ***All is OK*** -- Normal communication Erros--
#define OW_ERROR_NOSENSOR     1   // No reply to ow_reset() or no Value
#define OW_ERROR_SHORTCIRCUIT 2   // Or Pullup tooo low..
#define OW_ERROR_CRC8         5   // CRC wrong (DataBlock)

#define OW_ERROR_BITERROR     3   // Noise on the bus? -- These Errors only during SCAN--
#define OW_ERROR_TOOMANY      4   // Increase MAX_OW_SENS... (not really an error)
#define OW_ERROR_UNKNID       6   // Wrong ID in Coefficients Only allowed 0x10/0x28 for S and B
#define OW_ERROR_COEFFCRC     7   // CRC wrong (Coefficients)
#define OW_ERROR_TIMESLOT     8   // ERROR Running Timeslot
#define OW_ERROR_CMD          9   // Unknown CMD
#define OW_ERROR_NOCONV      10   // No Conversion, this is PowerUp Value.
#define OW_ERROR_NOT_HIRES   11   // DS18B20 in LowRes Mode
#define OW_ERROR_NOT_OPEN    12   // 1W-Port not open
#define OW_ERROR_COEFFERR    13   // Coefficients Error: Bad Clone, WrongConfig(not12Bit) or Chip Defect



/**********************************************************************************
* Entry for the Sensor's Addresses:
*
* Each 1-Wire sensor hold a fixed and unique 64 Bit ID (address):
* - describing the sensor Type (first Byte: 0x10: DS18S20, 0x28: DS18B20, ..),
* - a 48 Bit MAC (unique) and
* - a 8-bit CRC.
*
* Each sensor has an internal EEPROM for a 16 bit value. New sensors hold a value, that I
* mapped to 65535. To get the EEPROM-Value "spee", the sensor must be read once.
***********************************************************************************/

typedef struct{
    uint8_t addr[8];    // The 64 Bit Addr of the sensor
    int16_t spee;     //  Value stored in the sensor's EEPROM
} OW_ADDR;

#define MAX_OW_SENS 15  // Expect max. XX sensors (each takes 8+2 byes RAM for the Addrs)

extern OW_ADDR ow_addr[MAX_OW_SENS]; // The 1 Wire ID and Addr. per sensor
extern uint8_t ow_anz;
extern uint8_t ow_last_error_code;

//---functions---
int16_t onewire_open(uint8_t io_kanal);
void onewire_close(void);
int16_t onewire_scan_bus(void);
int16_t onewire_tempread(uint8_t *addr, int16_t *pspee);
int16_t onewire_temptrigger(void);
int16_t onewire_write_spee(uint8_t *adr,int16_t val);
void onewire_sort_spee(void);
void test_show_onewire(uint8_t todo); // For Demo/Test

// END

