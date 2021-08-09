/*******************************************************************************
* LTX_ERRORS.C
*
*******************************************************************************/

#include "ltx_errors.h"

#include "nordic_common.h"
#include "nrf.h"
#include "nrf_ble_gatt.h"
#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
#include "nrf_sdh_soc.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "nrf_pwr_mgmt.h"
#include "nrf_queue.h"

#include "nrf_delay.h"
// JW Toolbox
#include "tb_tools.h"
#include "bootinfo.h"


// -------------------------- Error Handlers -------------------------------
#define NRF_ERROR_OWN_BASE_NUM (0xF000) // Ab hier eigene Fehler (spaeter auslagern)
#define OWN_ERR_BUFFERSIZE NRF_ERROR_OWN_BASE_NUM + 1

//void app_error_handler(ret_code_t error_code, uint32_t line_num, const uint8_t * p_file_name)
void app_error_handler(uint32_t err_code, uint32_t line_num, const uint8_t *pfile) {
    NRF_LOG_FINAL_FLUSH();
    tb_printf("FATAL: Err%X L:%u '%s'\n", err_code, line_num, pfile);
    nrf_delay_ms(900);
#ifdef DEBUG
    NRF_BREAKPOINT_COND;
#endif
    NVIC_SystemReset();
}
#define DEAD_BEEF 0xDEADBEEF /* Value used as error code on stack dump, can be used to identify stack location on stack unwind. */
void assert_nrf_callback(uint16_t line_num, const uint8_t *p_file_name) {
    NRF_LOG_ERROR("assert_nrf_callback()\n");
    NRF_LOG_FLUSH();
    app_error_handler(DEAD_BEEF, line_num, (uint8_t *)p_file_name);
}

//

