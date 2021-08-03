/*******************************************************************************
* I2C.H - Lib fuer LTX-I2C-Sensorik
*******************************************************************************/

void ltx_i2c_driver_pwron_init(void); // Spater auslagern
int16_t ltx_i2c_init(void);
void ltx_i2c_uninit(void);	// Spater auslagern
void ltx_i2c_scan(void);

#define I2C_UNI_RXBUF 16          // ACHTUNG: Sollte gross genug sein, z.B. fuer UHR oder Sensoren
#define I2C_UNI_TXBUF 16          // dto. 
extern uint8_t i2c_uni_rxBuffer[I2C_UNI_RXBUF];
extern uint8_t i2c_uni_txBuffer[I2C_UNI_TXBUF];

int16_t i2c_write_blk(uint8_t i2c_addr, uint8_t anz );
int32_t i2c_read_blk(uint8_t i2c_addr, uint8_t anz);
int32_t i2c_readwrite_blk_wt(uint8_t i2c_addr, uint8_t anz_w, uint8_t anz_r, uint16_t wt_ms);

#define ERROR_HW_WRITE_NO_REPLY 1
#define ERROR_HW_READ_NO_REPLY  2
#define ERROR_HW_SEND_CMD 3

// ***
