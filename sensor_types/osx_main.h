/****************************************************
* type_main.h - Sensor Cmdline
*
***************************************************/
#define SRC_NONE 0 // No Output
#define SRC_CMDLINE 1 // Debug TB_UART
#define SRC_SDI 2 // SDI12
#define SRC_BLE 3 // BLE

#define VMON_CHANNEL            NRF_SAADC_INPUT_AIN7
#define HK_VBAT_KOEFF           0.00346 // Div: 4M7-1M6 (0 - 14.2V)

void type_init(void);
bool type_cmdline(uint8_t isrc, uint8_t *pc, uint32_t val);
bool type_service(void); // Ret: false: Kein Periodic

//***