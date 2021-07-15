/****************************************************
* type_main.c - Test Sensor SDI12-Basics
*
* (C) joembedded@gmail.com - joembedded.de
*
* UART is shared with tb_tools
*
***************************************************/

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "nrf_drv_gpiote.h"

#include "./platform_nrf52/tb_pins_nrf52.h"
#include "osx_pins.h"

#include "tb_tools.h"
#include "type_main.h"
#include "device.h"

#include "sdi12sensor_drv.h"


//----- Port-IRQ-Driver ------------
bool rxirq_active;
volatile uint32_t rxirq_zcnt; 
/*irq*/ static void rxirq_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action) {
  rxirq_zcnt++;
}

// Set/Unset RX-Pin to "IRQ-on-BREAK", opt. deactivate SDI-UART, Details: see gpio_irq.c-Sample
static void rxirq_on(void){
    uint32_t err_code;
    if(rxirq_active) return;  // Already ON
    nrf_drv_gpiote_in_config_t in_config = NRFX_GPIOTE_CONFIG_IN_SENSE_HITOLO(false);
    err_code = nrf_drv_gpiote_in_init(SDI_RX_PIN, &in_config, rxirq_handler);
    APP_ERROR_CHECK(err_code);
    nrf_drv_gpiote_in_event_enable(SDI_RX_PIN, true);
    rxirq_active = true;
}
static void rxirq_off(void){
    if(!rxirq_active) return;  // Already OFF
    nrf_drv_gpiote_in_event_disable(SDI_RX_PIN);
    nrf_drv_gpiote_in_uninit(SDI_RX_PIN); // Disable Functins an Pullups
    rxirq_active = false;
}

//--- Init (only call once)
void type_init(void){
    uint32_t err_code;
    err_code = nrf_drv_gpiote_init();
    APP_ERROR_CHECK(err_code);

    rxirq_on(); // SDI now active
}

char my_sdi_addr = '3';

#define MAX_RESULT 80
char result_string[MAX_RESULT+1];

bool type_service(void){
  int16_t res;
  int16_t txwait_chars = 0;
   // SDI12 Activity registered
   if(rxirq_zcnt){
    rxirq_off();
    tb_putc('S'); // Signal SDI12 Activity
    tb_delay_ms(1);
    sdi_uart_init();
    for(;;){
      res=sdi_getcmd(SDI_IBUFS, 100 /*ms*/); // Max. wait per Def.
      if(res == -ERROR_NO_REPLY) {
        txwait_chars = 0;
        break; // Timeout
      }
      else if(res > 0 && sdi_ibuf[res-1]=='!'){ // Only Commands
        if(*sdi_ibuf=='?' || *sdi_ibuf==my_sdi_addr){

          // Save Request
          strncpy(result_string, sdi_ibuf, MAX_RESULT); //DSn
          *sdi_obuf=0; // Assume no reply

          // Parse CMD and reply
          if(!strcmp(sdi_ibuf+1,"!")){
            sprintf(sdi_obuf,"%c\r\n",my_sdi_addr);
          }else if(!strcmp(sdi_ibuf+1,"I!")){
            sprintf(sdi_obuf,"%c012SensorAbc\r\n",my_sdi_addr);
          } //else unknown

          if(*sdi_obuf){
            tb_delay_ms(9); 
            txwait_chars = sdi_send_reply(NULL); // send SDI_OBUF
          }
       }
      }
    } // for()
    if(txwait_chars) tb_delay_ms(txwait_chars*9); // OK for 6 Chars (1 Char needs 8.33 msec)
    sdi_uart_uninit();
    rxirq_on();
    rxirq_zcnt=0;

    tb_printf("->R'%s'\r\n",result_string); 

   }
  return false; // Service
}

// Die Input String und Values
void type_cmdline(uint8_t isrc, uint8_t *pc, uint32_t val, uint32_t val2){
  switch(*pc){

  default:
    tb_printf("???\n");
  }
}

//***