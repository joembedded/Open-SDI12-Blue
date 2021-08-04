/****************************************************
* 0300_Pewatron_KKD18.c - CeramicPressure Sensor with I2C
* DEVICE_TYP 300
*
* (C) joembedded@gmail.com - joembedded.de
*
* UART is shared with tb_tools
*
* Info: KKD18 is a Pressure Sensor for Water Levels 
* with a Ceramic Membrane. Normally Piezo based sensors
* are used for Water levels, but the Ceramic Sensors are 
* more robust (to cleaning, e.g.), but Resoultion and 
* Accuracy is often not as good as Piezo based sensors.
*
* Connect: 
* - Sensor-Supply(White): I_X0 (Sensor need 1mA active)
* - Sensor-SCL(Green): I_SCL
* - Sensor-SDA(Yellow): I_SDA
* - Sensor-GND(Brown): I_GND
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
#include "osx_main.h"
#include "tb_tools.h"

#include "i2c.h"
#include "intmem.h"
#include "osx_pins.h"

#if DEVICE_TYP != 300
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

//------------------- Implementation -----------
void sensor_init(void) {
  // Id has fixed structure, max. 30+'\0'
  // all cccccccc.8 mmmmmm.6 vvv.3 xx..xx.[0-13]
  //  13 JoEmbedd   Testse   OSX   (MAC.l)
  // Set SNO to Low Mac, so same Name as BLE Advertising
  sprintf(sensor_id, "CerKKD18_0300_OSX%08X", mac_addr_l);

  // Try to read Parameters
  intpar_mem_read(ID_INTMEM_USER0, sizeof(param), (uint8_t *)&param);
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
    sdi_valio.m_msec = 2500;
  } else if (carg == 1) {
    sdi_valio.anz_channels = 3;
    sdi_valio.m_msec = 2600;
  } else
    return false;

  return true;
}

// Important: KKD has internal PullUps and max. FRQ is 100kHz
#define KKD_ADDR 81 // Addr-Raum 7-Bit
#define KKD_BUS 7
int32_t kkd_read_reg(uint8_t reg, int32_t *pval) {
  int32_t err_code, res;
  i2c_uni_txBuffer[0] = 0x40 + reg;
  err_code = i2c_readwrite_blk_wt(KKD_ADDR, 1, 3, 0); // 3 Bytes lesen, 0ms Warten
  if (!err_code) {
    // Erg. 24 Bit LE int24_t
    res = (i2c_uni_rxBuffer[2]) + (i2c_uni_rxBuffer[1] << 8) + (i2c_uni_rxBuffer[0] << 16);
    if (res & (1 << 23))
      res -= (1 << 24); // negative Werte
    *pval = res;
  }
  return err_code;
}

//The KKD has a WarmUp Time of 2 seconds!!!
int16_t sensor_valio_measure(uint8_t isrc) {
  //---- 'M': While waiting: scan SDI for <BREAK> ----
  int16_t wt = 2000;
  int16_t res;
  int32_t val;
  float fval;

  // Supply Sensor with Power
  nrf_gpio_cfg(
      IX_X0,
      NRF_GPIO_PIN_DIR_OUTPUT,
      NRF_GPIO_PIN_INPUT_DISCONNECT,
      NRF_GPIO_PIN_NOPULL,
      NRF_GPIO_PIN_S0H1, // High Out
      NRF_GPIO_PIN_NOSENSE);
  nrf_gpio_pin_set(IX_X0);

  while (wt > 0) {
    if (wt & 1)
      tb_board_led_on(0);
    tb_delay_ms(25); // Measure... (faster than time above)
    tb_board_led_off(0);
    wt -= 25;

    if (isrc == SRC_SDI) {
      for (;;) { // Get
        res = tb_getc();
        if (res == -1)
          break;
        if (res <= 0)
          nrf_gpio_cfg_default(IX_X0); // Power OFF
        return -1;                     // Break Found
                                       // else: ignore other than break
      }
    }
  }
  // --- 'm' Wait end

  // Read Data from Sensor
  ltx_i2c_init();

  sdi_valio.channel_val[0].punit = "oC";
  if (!kkd_read_reg(0, &val)) { // Temp
    fval = (float)val / 4096.0;        // Convert to oC
    fval *= param.koeff[0];     // Def. 1.0
    fval -= param.koeff[1];     // Def. 0.0
  } else
    fval = -99.9; // Error..
  snprintf(sdi_valio.channel_val[0].txt, 11, "%+.2f", fval);

  sdi_valio.channel_val[1].punit = "mBar";
  if (!kkd_read_reg(1, &val)) { // Pressure 104.8576 Counts are equal to 2 mBar
    fval = (float)val / 104.8576 * 2;
    fval *= param.koeff[2]; // Def. 1.0
    fval -= param.koeff[3]; // Def. 0.0
  } else
    fval = -999.9; // Error..
  snprintf(sdi_valio.channel_val[1].txt, 11, "%+.2f", fval);

  ltx_i2c_uninit(false);       // KKD has int. PU
  nrf_gpio_cfg_default(IX_X0); // Power OFF

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
    sprintf(outrs_buf, "%c", my_sdi_adr); // Standard Reply
  }                                       // else
}

//***