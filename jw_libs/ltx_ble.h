/*******************************************************************************
* LTX_BLE.h
*
*******************************************************************************/

#ifdef ENABLE_BLE
// Ankommend
#define BB_BLE_CMD 0x10       // Terminal-Kommando , erfordert einer Antwort
#define BB_BLE_BINBLK_IN 0x11 // Ankommender Binaerblock
#define BB_BLE_PLING 0x12     // Pling am BLE ankommend (wird wenn aktiviert alle 60 Sekunden geschickt)
//#define BB_BLE_USERCMD 0x13   // Usercommand - Aktuell nicht verwendet

// Ausgehend
#define BB_BLE_INFO 0x20         // Info-Block z.B. RSSI, wird nicht-blockierend geschickt
#define BB_BLE_REPLY_END 0x21    // ASCII-Terminal Ende-der-Antwort, mit '~' am Anfang: "Unsichtbare/Interpretierte Antwort, mit Retries
#define BB_BLE_BINBLK_OUT 0x22   // Asuegehnder Binblock, mit Retries
#define BB_BLE_REPLY_INTERM 0x23 // Intermediate Part of Reply, more follows, optionally mit Retries

#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "ble_hci.h"
#include "ble_nus.h"
#include "ble_radio_notification.h" // Radio Notifications
#endif

// ---Was immer verwendet wird---
//**** FAST INTERVALL *****
#define APP_ADV_DEFAULT_DURATION BLE_GAP_ADV_TIMEOUT_GENERAL_UNLIMITED             /* The advertising duration in units of 10 milliseconds. */
//#define APP_ADV_FAST_INTERVAL 682                                                  // 426 msec (1600/sec) dto. 46uA 3.0V/nrF52832
#define APP_ADV_FAST_INTERVAL 960                                                  // 600 msec (1600/sec) dto. 33uA 3.0V/nrF52832
#define APP_ADV_FAST_DURATION 1500                                                 // fuer 15 sec The advertising duration in units of 10 milliseconds. */

#define CONN_INTERVAL_STD 82   // in Units of 1.25 msec. (eq. 153 msec) idle ca. 45uA
#define CONN_INTERVAL_MITTE 50 /// Rein informativ, conn_intervall.max: darueber ist es S, darunter F
#define CONN_INTERVAL_START 20 // Zaehlt noch als "relativ schnell" zum Starten (ca. 25-37msec) ca. 180-130uA
#define CONN_INTERVAL_FAST 6   // in Units of 1.25 msec (6: eq 7.5msec-11.25) idle ca 620uA-420uA

#define MAX_BLE_SESSION_SECS 300  // sec, wie lange Verb. gehalten wird, wird von User-Aktivitaet und PLING gesetzt
#define BLE_MAX_CONFAST_IDLE 15   // Nach wieviel Sekunden IDLE kein FAST mehr erlaubt (Auto-Check)
#define BLE_IDLE_CNT 6            // Nach x sec idle darf System-Info (RSSI) geschickt werden
#define BLE_REPORT_LIMIT 2        // Alle y Sekunden Report

#if DEBUG
#define TB_UART_ON_SECS 86400 // 1d
extern uint32_t dbg_scan_cnt; 
#else
#define TB_UART_ON_SECS 20 // Nach xx idle secs ende
#endif

typedef struct {
    uint16_t min_interval; // in units of 1.25 msec
    uint16_t max_interval;
    bool updated; //  true if updated
} CON_PARAM;
extern CON_PARAM act_conn;

typedef struct {
    bool comm_started_flag; // State of NUS (ready = true)
                            //uint32_t rx_in_cnt;     // Counts incomming Packets
                            //uint32_t tx_out_cnt;    // Counts outgoing TX packets (there may be >1 pending packets)
                            //uint32_t tx_ready_cnt;  // Counts confirmed TX packets (with pending delay)
} NUS_STAT;
extern NUS_STAT nus_stat;
extern uint16_t m_ble_nus_max_data_len;

extern uint16_t ble_rssi_report_cnt;

#define BLE_DEVICE_NAME_MAXLEN 11                                    // 11: Max. Anz Chars for Fitting in 31-Bytes SRDATA with UUIDs
extern uint8_t ble_device_name[BLE_DEVICE_NAME_MAXLEN + 1];   // "LTX00000000"; // Short MAC-Name

extern bool  pin_ok; // True wenn OK
extern bool ble_connected_flag; // True wenn Connected

extern uint32_t last_ble_cmd_cnt;
extern uint32_t tb_uart_sec_counter;
extern bool fast_advertising_flag;

extern bool ble_reliable_printf;  // If true: Send only in 1 line

// Functions
void advertising_change_name(char *newname); 
uint32_t ble_printf(char *fmt, ...);
void ble_stack_init(void);

uint32_t conn_interval_change(uint32_t newcon_min);
int32_t get_con_rssi(int32_t *p_anz);
void gap_params_init(void);
void gatt_init(void);
void services_init(void);
void advertising_init(void);
void conn_params_init(void);
void advertising_start(void);
uint32_t ble_disconnect(void);


void ble_periodic_connected_service(void);

//

