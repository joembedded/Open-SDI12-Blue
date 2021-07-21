/****************************************************
* type_main.h - Sensor Cmdline
*
***************************************************/

#define SRC_CMDLINE 0

void type_init(void);
void type_cmdline(uint8_t isrc, uint8_t *pc, uint32_t val, uint32_t val2);
bool type_service(void); // Ret: false: Kein Periodic

//***