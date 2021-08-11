/****************************************************
* 0330_FrqCounterC
* DEVICE_TYP 330
*
* (C) joembedded@gmail.com - joembedded.de
*
* UART is shared with tb_tools
*
* Info: Simple Counter and Freqency Measure
* Frequency is measured each CNT_2_FRQ_MS ms 
* and works reliable for frequencies up to 1000 Hz,
* Counter rolls over on XCOUNT_ROLLOVER
*
* Connect:
* - I_SCL (internal PU activated)
* - I_GND
*
*  (I_SCL)--+---+
*           |   |
*           |   o /
*  1-100nF ===   /  Contact
*           |   o
*           |   |
*  (I_GND)--+---+
*
* - I_SDA: Optional Piezo/Oszi for Feedback
*
* Errors:
* -101 No Frequency (needs CNT_2_FRQ_MS msec after Power ON)
*
***************************************************/

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "device.h"
#include "nrf_drv_gpiote.h"
#include "saadc.h"
#include "sdi12sensor_drv.h"
#include "tb_tools.h"

#include "app_timer.h" // APP_TIMER_TICKS

#include "i2c.h"
#include "intmem.h"
#include "osx_main.h"
#include "osx_pins.h"

#if DEVICE_TYP != 330
#error "Wrong DEVICE_TYP, select other Source in AplicSensor"
#endif

// --------- Locals -------------
/* Parameters can be used e.g. as Offset/Multi per Channel etc...
  // Note: this is GP/TT Standard; First MULTI, then sub OFFSET!
  fval*=param->factor; // Def. 1.0
  fval-=param->offset; // Def. 0.0
*/
#define ANZ_KOEFF 4
typedef struct {
  float koeff[ANZ_KOEFF];
} PARAM;
// Test Setup for Default Koeffs (M O M O)
PARAM param = {{1.0, 0.0, 1.0, 0.0}};

//--- Local Paramaters for Counter Start

typedef struct {
  float counter;  // Counters Pulses
  float frequency; // Frequency of the last valid CNT_2_FRQ_MS period
} CNT_VALS;

static CNT_VALS cnt_vals = {0, -101.0}; // Err
//--- Local Paramaters for CNT End

//--- Local Functions for CNT Start
// - Here Counter and Frequency on IX_SCL -
#define SCL_FEEDBACK            // If defined: Connect Piezo or Oszi to IX_SCL
#define XCOUNT_ROLLOVER 1000000 // Roll-Over Value
volatile uint32_t x2l_cnt;      // Read Only!
/*irq*/ static void x2l_irq_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action) {
  uint32_t h;
  h = x2l_cnt;
  if (++h == XCOUNT_ROLLOVER)
    h = 0;
  x2l_cnt = h;
#ifdef SCL_FEEDBACK
  if (h & 1)
    nrf_gpio_pin_set(IX_SCL);
  else
    nrf_gpio_pin_clear(IX_SCL);
#endif
}

static uint32_t xt_t0, xt_t1;
static uint32_t xt_x2l0, xt_x2l1;

#define CNT_2_FRQ_MS 8000
static void cnt_timer_handler(void *p_context) {
  int32_t delta_cnt;
  xt_t1 = tb_get_ticks();
  xt_x2l1 = x2l_cnt;

  delta_cnt = xt_x2l1 - xt_x2l0;
  if (delta_cnt < 0)
    delta_cnt += XCOUNT_ROLLOVER;

  cnt_vals.frequency = (float)delta_cnt / (float)tb_deltaticks_to_ms(xt_t0, xt_t1) * 1000.0;

  // Shift to old
  xt_t0 = xt_t1;
  xt_x2l0 = xt_x2l1;
}

APP_TIMER_DEF(cnt_timer_id);

void simple_cnt_timer(void) {
  uint32_t ticks = APP_TIMER_TICKS(CNT_2_FRQ_MS); // msec
  ret_code_t ret = app_timer_create(&cnt_timer_id,
      APP_TIMER_MODE_REPEATED,
      cnt_timer_handler);
  APP_ERROR_CHECK(ret);

  ret = app_timer_start(cnt_timer_id, ticks, NULL);
  APP_ERROR_CHECK(ret);
}

// Pin Driver
bool pint_irq_on = false;
void pint_sda_on(void) {
  uint32_t err_code;
  if (pint_irq_on)
    return;

  nrf_drv_gpiote_in_config_t in_config = NRFX_GPIOTE_CONFIG_IN_SENSE_HITOLO(true); // HiAcc
  in_config.pull = NRF_GPIO_PIN_PULLUP;                                            // Overwrite Default (= NRF_GPIO_PIN_NOPULL)

  err_code = nrf_drv_gpiote_in_init(IX_SDA, &in_config, x2l_irq_handler);
  APP_ERROR_CHECK(err_code);
  nrf_drv_gpiote_in_event_enable(IX_SDA, true);
  pint_irq_on = true;
#ifdef SCL_FEEDBACK
  nrf_gpio_cfg_output(IX_SCL);
#endif
}
void pint_sda_off(void) {
  if (!pint_irq_on)
    return;
  nrf_drv_gpiote_in_event_disable(IX_SDA);
  // Keep Pullup ON
  pint_irq_on = false;
}

/* **Fragment** to measure Frequency
* Works reliable (even with BLE / SDI12 on and active)
* for Frequencies up to 1000 Hz 
void messure_frequency(uint32_t v){
  uint32_t xt_t0,xt_t1; 
  uint32_t xt_x2l0,xt_x2l1;
  int32_t delta_cnt;
  uint32_t msec;
  float frq;
  
  if(v<100) v=100;  // Minimum Measure Time

  // place tb_get_ticks() and get x2l_cnt close together
  xt_t0=tb_get_ticks();
  xt_x2l0 = x2l_cnt;
  tb_delay_ms(v);
  xt_t1=tb_get_ticks();
  xt_x2l1 = x2l_cnt;
  delta_cnt=xt_x2l1-xt_x2l0;

  if(delta_cnt<0) delta_cnt+=XCOUNT_ROLLOVER;
  msec=tb_deltaticks_to_ms(xt_t0,xt_t1);
  frq= (float)delta_cnt/(float)msec * 1000.0;
  
  _printf("%u Cnts in %u msec, %f Hz\n",delta_cnt,msec, frq);
}
*/

int16_t cnt_vals_get(void) {
  cnt_vals.counter = x2l_cnt;
  return 0;
}
//--- Local Functions for CNT End

//------------------- Implementation -----------
void sensor_init(void) {
  // Id has fixed structure, max. 30+'\0'
  // all cccccccc.8 mmmmmm.6 vvv.3 xx..xx.[0-13]
  //  13 JoEmbedd   Testse   OSX   (MAC.l)
  // Set SNO to Low Mac, so same Name as BLE Advertising

  sprintf(sensor_id, "TT_CNT_A_0320_OSX%08X", mac_addr_l);
  // TT:'TerraTransfer' CNT:'FrqCounter', A:'Version: Range from Sensor'

  // Try to read Parameters
  intpar_mem_read(ID_INTMEM_USER0, sizeof(param), (uint8_t *)&param);

  // Init driver
  pint_sda_on();

  xt_t0 = tb_get_ticks(); // Set start point for FRQ
  xt_x2l0 = x2l_cnt;
  simple_cnt_timer();
}

bool sensor_valio_input(char cmd, uint8_t carg) {
  sdi_valio.measure_cmd = cmd;
  sdi_valio.measure_arg = carg;
  sdi_valio.channel_val[0].didx = -1; // Assume no Values in D-Buffer

  // For THIS sensor:
  if (cmd != 'M')
    return false; // This sensor only supports M, M1
  if (carg == 0) {
    sdi_valio.anz_channels = 2;
    sdi_valio.m_msec = 100;
  } else if (carg == 1) {
    sdi_valio.anz_channels = 3;
    sdi_valio.m_msec = 150;
  } else
    return false;

  return true;
}

//The CNT has no significant WarmUp Time, this is only to scan for Reps
#define WAIT_MS 50
int16_t sensor_valio_measure(uint8_t isrc) {
  //---- 'M': While waiting: scan SDI for <BREAK> ----
  int16_t res;
  int32_t val;
  float fval;

  res = sensor_wait_break(isrc, WAIT_MS);
  if (res)
    return res;
  // --- 'm' Wait end

  // Read Data from Sensor
  cnt_vals_get();

  // Prepare Output
  sdi_valio.channel_val[0].punit = "Cnt";
  fval = cnt_vals.counter;
  fval *= param.koeff[2]; // Def. 1.0
  fval -= param.koeff[3]; // Def. 0.0
  snprintf(sdi_valio.channel_val[0].txt, 11, "%+.0f", fval);

  sdi_valio.channel_val[1].punit = "Hz";
  fval = cnt_vals.frequency;
  if (fval >= 0) {
    fval *= param.koeff[0]; // Def. 1.0
    fval -= param.koeff[1]; // Def. 0.0
  }                         // else Error..
  snprintf(sdi_valio.channel_val[1].txt, 11, "%+.2f", fval);

  if (sdi_valio.measure_arg) {
    snprintf(sdi_valio.channel_val[2].txt, 11, "%+.2f", get_vbat_aio()); // Only 2 digits
    sdi_valio.channel_val[2].punit = "VSup";
  }
  return 0;
}

// 'X'; Additional SDI12 - User Commands points to 1.st char after 'X'
// Add here:
// - Sensor specific Parameters Setup
// - I/O
// etc..
void sensor_valio_xcmd(uint8_t isrc, char *pc) {
  uint16_t pidx;
  float fval;

  if (*pc == 'K') { // Kn! or Kn=val!
    pidx = (uint16_t)strtoul(pc + 1, &pc, 0);
    if (pidx > ANZ_KOEFF)
      return;
    if (*pc == '=') { // Set Koeff
      fval = strtof(pc + 1, &pc);
      param.koeff[pidx] = fval;
    }
    if (*pc != '!')
      return;
    // Send Koeffs
    sprintf(outrs_buf, "%cK%d=%f", my_sdi_adr, pidx, param.koeff[pidx]);
  } else if (!strcmp(pc, "Write!")) { // Write SDI_Addr and Koefficients to Memory
    intpar_mem_erase();               // Compact Memory
    intpar_mem_write(ID_INTMEM_SDIADR, 1, (uint8_t *)&my_sdi_adr);
    intpar_mem_write(ID_INTMEM_USER0, sizeof(param), (uint8_t *)&param);
    sprintf(outrs_buf, "%c", my_sdi_adr);     // Standard Reply
  } else if (!strcmp(pc, "Sensor!")) {        // Identify Senor
    sprintf(outrs_buf, "%cCNT!", my_sdi_adr); // Standard Reply
  }                                           // else
}

//***