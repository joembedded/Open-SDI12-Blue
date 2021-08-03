/*******************************************************************************
* SAADC.C - Lib fuer LTX
* Measure VBAT - V1.1 (C) JoEmbedded.de
*******************************************************************************/

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "nrf.h"
#include "nrf_drv_saadc.h"

#include "boards.h"
#include "nrf_delay.h"

// JW Toolbox
#include "tb_tools.h"
#include "device.h"
#include "sdi12sensor_drv.h"
#include "osx_main.h"
#include "saadc.h"

// Pin und Setup extern 

// Kanal 0 immer HK_VMON
// (more channels optional)

static volatile bool saadc_cali_fertig_flag;
static volatile bool saadc_init_flag=false;

// Non-Blocking Wandeln loest Event NRF_DRV_SAADC_EVT_DONE aus,
// Kalibrieren:  NRFX_SAADC_EVT_CALIBRATEDONE ..DONE, ..RESULTDONE
static void saadc_callback(nrf_drv_saadc_evt_t const * p_event){
    if(p_event->type == NRFX_SAADC_EVT_CALIBRATEDONE){
        // NRF_LOG_INFO("Kalibrieren fertig\n");
        saadc_cali_fertig_flag=true;
    }
}

void saadc_init(void){
    ret_code_t err_code;
    if(saadc_init_flag) return;
    err_code = nrf_drv_saadc_init(NULL, saadc_callback);  // NRFX_SAADC_CONFIG_RESOLUTION, etc in Config
    APP_ERROR_CHECK(err_code);
    saadc_init_flag=true;
}
void saadc_uninit(void){
    nrf_drv_saadc_uninit();
    saadc_init_flag=false;
}


void saadc_setup(uint8_t setup_id){ // ID unused. Call after saadc_init()
    ret_code_t err_code;
    // Std-Einstellungen sind OK (10uS, Gain 1_6, Int.Ref) Single-Ended. Voller Bereich fuer 3.6V 4096 Cnts.
    nrf_saadc_channel_config_t channel_config0 = NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(VMON_CHANNEL); // 
    channel_config0.acq_time   = NRF_SAADC_ACQTIME_40US;  
    err_code = nrf_drv_saadc_channel_init(0, &channel_config0);
    APP_ERROR_CHECK(err_code);
/***
#ifdef  PIN_ADD_OPTIONAL
    nrf_saadc_channel_config_t channel_config1 = NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(PIN_ADD_OPTIONAL); // 
    channel_config1.acq_time   = NRF_SAADC_ACQTIME_10US;  
    err_code = nrf_drv_saadc_channel_init(1, &channel_config1);
    APP_ERROR_CHECK(err_code);
#endif
***/
}

// Empfohlen alle paar Grad Temperaturschwankung
void saadc_calibrate_offset(void){ 
    ret_code_t err_code;
    saadc_cali_fertig_flag=false;
    err_code = nrf_drv_saadc_calibrate_offset();
    APP_ERROR_CHECK(err_code);
    while(saadc_cali_fertig_flag==false); // bissl warten bis fertig... (wenn adc nicht init: forever)
}

//----------------- HK_VBAT------------------------------
// Analoge HK_VBAT Spannung holen - SAADC bereits initialisiert
float saadc_get_vbat(bool cali_flag, int32_t anz_mittel){
      if(cali_flag) saadc_calibrate_offset();
      int16_t value;
      int32_t sum=0;
      nrf_drv_saadc_sample_convert(0, &value); // Ignore First!
      for(uint32_t i=0;i<anz_mittel;i++){
              nrf_drv_saadc_sample_convert(0, &value); // Channel, pWert
              sum+=value;
      }
      sum/=anz_mittel;
      return ((float)sum)*HK_VBAT_KOEFF;
}
float get_vbat_aio(void){ // all In One
    float fval;
    saadc_init();
    saadc_setup(0);
    fval = saadc_get_vbat(true, 8); // Calibrate and 8 Averages
    saadc_uninit();
    return fval;
}

/***
#ifdef  PIN_ADD_OPTIONAL
// needs saadc_init(); saadc_setup(0); / saadc_uninit();
int16_t saadc_get_addopt(bool cali_flag){
      if(cali_flag) saadc_calibrate_offset();
      int16_t value;
      nrf_drv_saadc_sample_convert(1, &value); // Channel, pWert
      //tb_printf("%d\n",value);
      return value;
}
#endif
***/
//***
