/****************************************************
* gpio_irq.c - Test IRQ als ZAEHLEINGANG
*
* --- SAMPLE als Ausgangsbasis ---
*
* Schalter kann man detektieren, indem man misst, wielange er LOW->HIGH braucht
* Mit 10 nF kommen wir auf tau = 22msec, also 10 Hz. gehen alleweil 
***************************************************/

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include "tb_tools.h"
#include "nrf_drv_gpiote.h"
#include "gpio_irq.h"

//#define IRQ_TEST_PIN NRF_GPIO_PIN_MAP(0, 25) // ANNA_B112_EVK: Button and GREEN Led, needs PULLUP!
#define IRQ_TEST_PIN NRF_GPIO_PIN_MAP(0, 6) // YJ_NRF52832 - SDI12: SDI_RX (Ext. PULUP)

uint32_t gpio_zcnt; // Zaehlt
bool irqon;    // Flag
bool gpiote_driver_init;

void in_pin_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action) {
  gpio_zcnt++;
}

void test_the_irq(uint8_t val) {
  uint32_t err_code;

  if (!gpiote_driver_init) { // Einmalig
    err_code = nrf_drv_gpiote_init();
    APP_ERROR_CHECK(err_code);
    gpiote_driver_init = true;
  }

  if (!irqon) {
    tb_printf("IRQ_AN\n");
    // true: .high_accu - Std.-Wert fuer HighAccuracy, fuer SDI12 flsle auchOK zum Det. von BREAK
    nrf_drv_gpiote_in_config_t in_config = NRFX_GPIOTE_CONFIG_IN_SENSE_HITOLO(false /*true*/); // true: High_Accuracy
    // Std. ist NOPULL und man kan her auch die Flanke auswaehlen LOHI etc..
    //in_config.pull = NRF_GPIO_PIN_PULLUP; // Default ist NRF_GPIO_PIN_NOPULL
                                                                                         // in_config.pull = NRF_GPIO_PIN_PULLUP; // Std.
    err_code = nrf_drv_gpiote_in_init(IRQ_TEST_PIN, &in_config, in_pin_handler);          // un_init gibts auch
    APP_ERROR_CHECK(err_code);
    nrf_drv_gpiote_in_event_enable(IRQ_TEST_PIN, true);
    irqon = true;
  }

  for (;;) {
    tb_printf("ZC:%u\n", gpio_zcnt);
    tb_delay_ms(2000);
    if (tb_kbhit()) {
      tb_getc();
      break;
    }
  }

  if (!val) {
    tb_printf("IRQ_AUS\n");
    nrf_drv_gpiote_in_event_disable(IRQ_TEST_PIN);
    nrf_drv_gpiote_in_uninit(IRQ_TEST_PIN); // Disable Functins an Pullups
    irqon = 0;
  }
}


//***