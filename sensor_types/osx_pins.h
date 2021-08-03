/***************************************************************************************************************
* osx_pins.h - Pin definitins for my used HW platforms (similar to tb_pins_nrf52.h)
*
* (C) joembedded.de
***************************************************************************************************************/


//-------- NRF52840-CPUS -------------
#ifdef NINA_B3_EVK
#endif

#if defined(NINA_B3_LTX) || defined(NINA_B3_EPA)
#endif

//-------- NRF52832-CPUS -------------
#ifdef ANNA_SDI12 // ANNA-SDI12 Sensor platform with ANNA-B112
	#define SDI_RX_PIN  NRF_GPIO_PIN_MAP(0, 4) // SDI12: SDI_RX (needs ext. PULUP)
	#define SDI_TX_PIN  NRF_GPIO_PIN_MAP(0, 5) 
#endif

#ifdef YJ_NRF52832  // YJ_16048 from HolyIot (NO CE/FCC uncertified Low-Cost module)
	#define SDI_RX_PIN  NRF_GPIO_PIN_MAP(0, 6) // YJ_NRF52832 - SDI12: SDI_RX (has ext. PULUP)
	#define SDI_TX_PIN  NRF_GPIO_PIN_MAP(0, 7) 
#endif

#define B_I2CINTERN_SCL  NRF_GPIO_PIN_MAP(0, 30) 
#define B_I2CINTERN_SDA  NRF_GPIO_PIN_MAP(0, 29) 


//***