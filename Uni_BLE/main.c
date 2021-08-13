/*******************************************************************************
* OPEN-OSX-BLE Minimal -  Mit S112/S113, Achtung: das geht nur bis 4db TX 
*
* Hinweise:
* Defaults (NUS_BASE_UUID): wird in ble_nus_init() gesetzt
* Details siehe 'ble_uart_wrk_orig'
*
* BLE-Transfer in binaeren Bloecken:
* LEN TOKEN Daten[LEN]  LEN kann 0...(MTU-3) sein
* TOKEN: 

*  BB_BLE_INFO:       Unsolicited (RSSI) BLE an PC, ohne Retries
*  BB_BLE_REPLY_END:  ASCII-Terminal Ende-der-Antwort, mit '~' am Anfang: "Unsichtbare/Interpretierte Antwort, mit Retries
*  BB_BLE_BINBLK_OUT: asBinaerblock, mit Retries
*  xx 0x12: ASCII-Terminal Zwischenzeilen, mit Retries
*
* AENDERUNGEN ausserhalb EIGENE: Suchen nach 'JuergenWickenh'
* 2 Files betroffen: Kopiert nach "nordic_mods":
* - components/ble/ble_advertising/ble_advertising.c ca. Zeile 590
* - components/ble/ble_services/ble_nus/ble_nus.c (2-fach) ca. Zeile 70

*
* (C) joembedded@gmail.com - joembedded.de
* Version: 
*
*******************************************************************************/

#define GID 1 // Guard-ID fuer dieses Modul

#include <stdarg.h> // for var_args
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>

#include "device.h"

//#define USE_NOTIFICATIONS   // BLE-Notifications sind nicht unbedingt noetig, ausser SIMPLETIMER wird nicht benutzt.
// SIMPLETIMER: Bei 1 sec. ca. 4uA
#define USE_SIMPLETIMER 4000  // Wenn KEIN BLE oder keine Notifications ist der Timer noetig, mit Timeout


#include "ltx_ble.h"
#include "ltx_errors.h"

#include "nordic_common.h"
#include "nrf.h"
#include "nrf_ble_gatt.h"
#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
#include "nrf_sdh_soc.h"

#include "app_timer.h" // APP_TIMER_TICKS
#include "app_util_platform.h"

#include "boards.h"

#include "nrf_delay.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "nrf_pwr_mgmt.h"
#include "nrf_queue.h"

#include "nrf_nvmc.h"

// JW Toolbox
#include "tb_tools.h"
#include "bootinfo.h"


//#include "ltx_defs.h"
#include "jesfs_int.h" // fs_track_crc32()
#include "filepool.h"

// Sensorspecific
#include "sdi12sensor_drv.h"
#include "osx_main.h"
#include "saadc.h"
#include "i2c.h"

//---------------- Periodic_sec Verwaltungsvariablen -----------------

static uint32_t periodic_old_secs; // Letzte Sekunden (gemerkt)

uint32_t reset_reason_bits;        // Backup Flags nach Reset vor Memeory Protection
uint8_t ledflash_flag = 0;    // Radio-Active-Notification/Heartbeat/Etc

#if DEBUG
uint32_t dbg_idle_cnt;  // Zaehlt Wakeups
#endif

// Jedes Quellfile kriegt das einzeln, falls LOC_ERROR_CHECK verwendet wird
static void local_error(uint32_t error, uint16_t line_num) {
    NRF_LOG_ERROR("ERROR %u in Line %u\n", error, line_num);
    NRF_LOG_FLUSH();
    app_error_handler(error, line_num, (uint8_t *)__FILE__);
}


//=== Platform specific, Helpers for JesFs-time ===
uint32_t _time_get(void) {
    return tb_time_get();
}


/* Immer aufrufen, aber nur max. 1 mal/sec Sekunden durchlaufen
* Diese Routine soll die regelmaessigen Tasks erledigen:
* Messen/Alarme/Display/Timeouts
*/
void periodic_secs(void) {
    uint32_t nsecs;

    char stat;
    int16_t res;

    // --- Periodic each second ---
    nsecs = tb_time_get();
    if (nsecs == periodic_old_secs) {
        return; // War schon in dieser Sekunde.
    }
    periodic_old_secs = nsecs;

    // Feed Watchdog 1nce/sec
    tb_watchdog_feed(1);

    ledflash_flag=1;

    stat = '.';

#ifdef ENABLE_BLE
    if (ble_connected_flag == true) {
        stat = 'o';
        // Zaehlt alle Sekunde hoch
        if (nus_stat.comm_started_flag == true) {
            // Nur wenn Notificatiions auch erlaubt sind
            ble_rssi_report_cnt++;
            stat = '*';
        }
        last_ble_cmd_cnt++;

        if (last_ble_cmd_cnt == BLE_MAX_CONFAST_IDLE && act_conn.max_interval < CONN_INTERVAL_MITTE) {
            ble_reliable_printf=false;
            ble_printf("AUTO:Slow");
            tb_printf("AUTO:Slow\n");

            conn_interval_change(CONN_INTERVAL_STD);
        }else if (last_ble_cmd_cnt == (MAX_BLE_SESSION_SECS-15)) { // Timeout abgelaufen -> Disconnect
            ble_printf("~X"); // Disconnect soon/Request for Disconnect

        }else if (last_ble_cmd_cnt >= MAX_BLE_SESSION_SECS) { // Timeout abgelaufen -> Disconnect
            ble_disconnect();
            last_ble_cmd_cnt = 0;
        }

    }
#endif

    tb_putc(stat); // 1-2 Sekunden-Check
}

// Init UART and Filesystem, set FS then to sleep
void jw_drv_init(void) {
    int16_t res;

    tb_init(); // Init Toolbox (ohne Watchdog) und Minimal (wie Std. pca10056)

    tb_watchdog_init();
    tb_watchdog_feed(1);

    conv_secs_to_date_buffer(get_firmware_bootloader_cookie());
    tb_printf("\n\n*** OSX-BLE Type:%u V%u.%u(Built:%s) (C)JoEmbedded.de\n\n",DEVICE_TYP,DEVICE_FW_VERSION/10,DEVICE_FW_VERSION%10,date_buffer);
#ifdef NRF52832_XXAA
    tb_printf("CPU: nRF52832\n");
#endif
#ifdef NRF52840_XXAA
    tb_printf("CPU: nRF52840\n");
#endif

#ifdef DEBUG
    uint32_t uval = reset_reason_bits;

    tb_printf("Reset-Reason: 0x%X ", uval);
    if (uval & 1)
        tb_printf("(Pin-Reset)"); // The low Nibble Reasons
    if (uval & 2)
        tb_printf("(Watchdog)");
    if (uval & 4)
        tb_printf("(Soft-Reset)");
    if (uval & 8)
        tb_printf("(CPU Lockup)");
    tb_printf(", Bootcode: 0x%x\n", tb_get_bootcode(false));
    tb_printf("Time: %u\n", tb_time_get());

    uint8_t *psec=get_security_key_ptr();
    int16_t i;
    if(psec){
      tb_printf("KEY: ");
      for(i=0;i<16;i++){
	tb_printf("%02X",psec[i]);
      }
      tb_printf("\n");
    }

#endif
    GUARD(GID); // GUARD: Save THIS line as last visited line in Module GID

    mac_addr_h = NRF_FICR->DEVICEADDR[1];
    mac_addr_l = NRF_FICR->DEVICEADDR[0];

    srand(mac_addr_l);

    tb_printf("MAC: %08X%08X\n", mac_addr_h, mac_addr_l);
    // Erstmalige Annahme fuer den Advertising-Namen
#ifdef ENABLE_BLE
    sprintf(ble_device_name, "OSX%08X", mac_addr_l); // LowWord of Device Addr is ID !! LEN!
#endif

}

// Systemdaten OSX von Disk laden/initialisieren
void osx_system_init(void) {  
  // e.g. Advertising Data
}

//========== Arbeitsflaeche===============


/************************** Lokale UART-Kommando-Schnittstelle ***************************/

// Input Line (UNI)
#define INPUT_LEN 80
uint8_t input[INPUT_LEN + 1];

//=== cmdline ===
void uart_cmdline(void) {
    int32_t i;
    int16_t cmdres;
    uint32_t val, val2;
    char *pc;

#ifdef DEBUG
    float fval; // Debug
#endif

    tb_printf(">");
    cmdres = tb_gets(input, INPUT_LEN, 15000, 1); // Max. 15 secs for input, echo
    tb_printf("\n");
    if (cmdres < 0) {
        tb_printf("<UART ERROR>\n");
        tb_uninit();
        tb_uart_sec_counter = 0;
        return;
    } else if (cmdres >= 0) {
        tb_uart_sec_counter = TB_UART_ON_SECS; // Jede Zeile laedt
        uint32_t t0 = tb_get_ticks();
        pc = input + 1;
        val = strtol(pc, &pc, 0);  // Parameter 1
        val2 = strtol(pc, &pc, 0); // Parameter 2
        switch (*input) {

        // --------- Commands -------------------

#ifdef DEBUG
        //==== Memory-Zeugs Anf - OK under BLE active ====
        case 'H': // Adresse, 256 Bytes zeigen
            tb_printf("Memory:");
            for(i=0;i<256;i++){
              if(!(i%16)) tb_printf("\n%x:",val);
              tb_printf(" %02x",*(uint8_t*)val++);
            }
            tb_printf("\n");
            break;
       case 'K':
            if(val&4095) tb_printf("Error: Sector Adr.\n");
            else {
              tb_printf("Erase %x\n",val);
              nrf_nvmc_page_erase(val);
            }
            break;
      case 'I':
          i=(strlen(pc)+3)/4;
          tb_printf("Write %u(%u) Bytes at %x:'%s'\n",i,strlen(pc),val,pc);
          nrf_nvmc_write_words(val, (uint32_t *)pc, i);
          break;
      case 'J':
          tb_printf("CRC32 %x[%d]: %x\n",val,val2,fs_track_crc32((uint8_t *)val,val2,0xFFFFFFFF));
          break;
      //==== Memory-Zeugs End  ====

      // === Div Info
        case 'N':
            tb_printf("Novo: %x %x %x %x\n", _tb_novo[0], _tb_novo[1], _tb_novo[2], _tb_novo[3]);
            tb_printf("Button: %u\n", tb_board_button_state(0));
            tb_printf("Idle: %u\n", dbg_idle_cnt);
            // AD(val avg.)
            if(val<1) val=1; // val Averages (8 is more than enough)
            saadc_init();
            saadc_setup(0);  
            fval = saadc_get_vbat(true, val); 
            tb_printf("HK-BAT(%d): %f V\n", val,fval);
            saadc_uninit();
            break;
        case '!':
            if (val) { // An P0.0 normalerweise das Quarz
                tb_dbg_pinview(val);
            }
            break;

#endif

        case '#':
            tb_printf("UnInit UART!\n");
            tb_uninit();
            tb_uart_sec_counter = 0;
            break;

        case 'T': // ZEIT holen oder setzen (muss regelmaessig sein)
          if (val) tb_time_set(val);
          val = tb_time_get();
          tb_printf("Time: %u\n", val); // Interpret to Date
          break;

        case 'R':
            tb_printf("Reset...\n");
            tb_delay_ms(1000);
            tb_system_reset();
            break;

        case 0:
            tb_printf("OK\n");
            break;

        default:  // Type Specific -Command?
            if(type_cmdline(SRC_CMDLINE, input, val)==false){
              tb_printf("???\n");
            }
        }
        // Measure runtime
        tb_printf("(Run: %u msec)\n", tb_deltaticks_to_ms(t0, tb_get_ticks()));
    } // if (cmdlen)
}

/*** Start Softdevice - Benoetigt von der Toolbox! ***/

/* Function for initializing the nrf log module. */
static void log_init(void) {
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    LOC_ERROR_CHECK(err_code);
    NRF_LOG_DEFAULT_BACKENDS_INIT();
}

/* Function for initializing power management. */
static void power_management_init(void) {
    ret_code_t err_code;
    err_code = nrf_pwr_mgmt_init();
    LOC_ERROR_CHECK(err_code);
}

/* Function for handling the idle state (main loop).
If there is no pending log operation, then sleep until next the next event occurs. */
static void idle_state_handle(void) {
    if (NRF_LOG_PROCESS() == false) {
        nrf_pwr_mgmt_run();
    }
}

#ifdef USE_SIMPLETIMER
//-------------- NO_BLE_TIMER (simuliert BLE IRQ) ---------------
static void no_ble_timeout_handler(void *p_context) {
    ledflash_flag = 1;
}

APP_TIMER_DEF(no_ble_timer_id);

void simple_no_ble_timer(void) {
    uint32_t ticks = APP_TIMER_TICKS(USE_SIMPLETIMER); // msec
    ret_code_t ret = app_timer_create(&no_ble_timer_id,
        APP_TIMER_MODE_REPEATED,
        no_ble_timeout_handler);
    LOC_ERROR_CHECK(ret);

    ret = app_timer_start(no_ble_timer_id, ticks, NULL);
    LOC_ERROR_CHECK(ret);
}
#endif

/***************** M A I N *************************/
void main(void) {

    log_init(); // the nrf_log

    // Get/prepare Reset Reason, not possible after SD started
    reset_reason_bits = (NRF_POWER->RESETREAS);
    (NRF_POWER->RESETREAS) = 0xFFFFFFFF; // Clear with '1'


    power_management_init();
    ble_stack_init(); // Startet Timer! Zuerst starten

#ifdef HAS_EXTERNAL32KHZ_CLK // Init AFTER SD! 
    NRF_CLOCK->LFCLKSRC |= (1<<16)|(1<<17);
#endif
    jw_drv_init();    // Systemtreiber

    
    // Vor BLE/Advertising evtl. HW aktivieren
    osx_system_init();

#ifdef ENABLE_BLE
#ifdef DEBUG
    tb_printf("***** BLUETOOTH ON ****\n");
    tb_printf("Advertising Name: '%s'\n", ble_device_name);
#endif
    

#ifdef USE_NOTIFICATIONS
    radio_notification_init(); // Ausm Forum
#endif

    gap_params_init();
    gatt_init();
    services_init();
    advertising_init();
    conn_params_init();
    advertising_start();
#endif

#ifdef USE_SIMPLETIMER
    //tb_printf("***** SIMPLETIMER ****\n");
    simple_no_ble_timer();
#endif

#ifdef NRF_LOG
    NRF_LOG_INFO("Debug logging RTT started.\n");
#else
    NRF_LOG_INFO("Minimum Debug logging RTT started.\n");
#endif

#ifdef DEBUG
    tb_printf("*** DEBUG - " __DATE__ " "__TIME__ " ***\n");
    if (tb_is_wd_init())
        tb_printf("ACHTUNG: WD ist an!\n");
    tb_uart_sec_counter = TB_UART_ON_SECS;
#else
    tb_uart_sec_counter = 30; // 30 sec lang chance fuer Kommu nach Reset
#endif

    type_init();  // Typspezifisch initialisieren


    // Enter main loop.
    for (;;) {

        // periodische Aufgaben?
        GUARD(GID); // GUARD: Save THIS line as last visited line in Module GID

        if(!type_service()){
#ifdef ENABLE_BLE
          // --- BLE Data Input Scanner (nur wenn connected, aber max. Prio) ---
          if (ble_connected_flag == true) {
              GUARD(GID); // GUARD: Save THIS line as last visited line in Module GID
              ble_periodic_connected_service();
              ledflash_flag = 1;  // Connected: in jedem Fall Blinken
          }
#endif
          periodic_secs();

          // ---- UART-Service (or UART-ERROR) Wichtig zu pollen, sonst evtl. 500uA Iq---
          if (tb_kbhit()) {
              GUARD(GID); // GUARD: Save THIS line as last visited line in Module GID
              uart_cmdline();
          }
        } // sensor_service

        // -------------- Epilog (spaeter kummulieren auf z.B. 1 Blink/5 sec) -----------
        if (ledflash_flag) {
            tb_board_led_on(0);
            nrf_delay_us(100);
            tb_board_led_off(0);
            ledflash_flag = 0;
        }
        // 
#if DEBUG
        dbg_idle_cnt++;
#endif

        idle_state_handle();
    }
}
// ***