/****************************************************
* intmem.h - Internal NVM Memory Helpers
*
* (C) joembedded@gmail.com - joembedded.de
*
***************************************************/

int16_t intpar_mem_write(uint8_t parid, uint16_t pbytes_total, uint8_t *pdata);
int16_t intpar_mem_read(uint8_t parid, uint16_t pbytes_max, uint8_t *pdata);
void intpar_mem_erase(void);

//***