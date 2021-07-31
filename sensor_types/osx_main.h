/****************************************************
* type_main.h - Sensor Cmdline
*
***************************************************/
#define SRC_NONE 0 // No Output
#define SRC_CMDLINE 1 // Debug TB_UART
#define SRC_SDI 2 // SDI12
#define SRC_BLE 3 // BLE

void type_init(void);
bool type_cmdline(uint8_t isrc, uint8_t *pc, uint32_t val);
bool type_service(void); // Ret: false: Kein Periodic

//***