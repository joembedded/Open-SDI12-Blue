/*******************************************************************************
* SAADC.H - Lib fuer LTX
* Misst auch VBat
*******************************************************************************/

void saadc_init(void);
void saadc_setup(uint8_t setup_id);
void saadc_calibrate_offset(void);
void saadc_uninit(void);

float saadc_get_vbat(bool cali_flag, int32_t anz_mittel);

// all In One ANALOG;
float get_vbat_aio(void); 

#ifdef  PIN_MODEM_VINTCHAN
int16_t saadc_get_vmodem(bool cali_flag);
#endif

// ***