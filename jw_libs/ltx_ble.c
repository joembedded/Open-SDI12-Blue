/*******************************************************************************
* OSX-BLE LIGHT -  Mit S112,S113, Achtung: das geht nur bis 4db TX, andere bis 8db TX
* Nur BLE-Teil
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
* - components/ble/ble_services/ble_nus/ble_nus.c ca. Zeile 70
*
* WENN Filesystem dabei ist, aber nicht verfuegbar gibt es Fehlermeldungen,
* auch beim Download ins interne CPU-Flash. Noch evtl. *todd* oder als
* Defines auslagern...
*
* (C) joembedded@gmail.com - joembedded.de
* Version: 
*
*******************************************************************************/

#define GID 100 // Guard-ID fuer dieses Modul

#include <stdarg.h> // for var_args
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>


#include "device.h"

#include "ltx_ble.h"
#if DEVICE_TYP >= 300  
  // *** Real Sensors should use a PIN or Password. ***
  // *** SECRET Backdoor Password, defined in ltx_ble.secret ***
  // e.g.:
  // #define SECRET_GLOBALPIN "GlobalPin"
  #include "ltx_ble.secret"
#endif

#include "ltx_errors.h"

#include "nordic_common.h"
#include "nrf.h"
#include "nrf_ble_gatt.h"
#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
#include "nrf_sdh_soc.h"

#include "app_timer.h" // APP_TIMER_TICKS
#include "app_util_platform.h"


#include "nrf_delay.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "nrf_pwr_mgmt.h"
#include "nrf_queue.h"

#include "nrf_nvmc.h"
#include "bootinfo.h"

// JW Toolbox
#include "tb_tools.h"
#include "bootinfo.h"
#include "sdi12sensor_drv.h"
#include "osx_main.h"

#include "filepool.h"


#ifdef ENABLE_BLE
#define NUS_SERVICE_UUID_TYPE BLE_UUID_TYPE_VENDOR_BEGIN /* UUID type for the Nordic UART Service (vendor specific). */
#define APP_BLE_OBSERVER_PRIO 3                          /* Application's BLE observer priority. You shouldn't need to modify this value. */

//---- SOnstige Globals ---------------
uint8_t ble_device_name[BLE_DEVICE_NAME_MAXLEN + 1] = "LTX00000000"; // Short MAC-Name

// Transfer Buffer fuer Filesystem. Gute Idee den exakt "NRF_SDH_BLE_GATT_MAX_MTU_SIZE-3-2" zu machen oder DEUTLICH groesser
// Getesteter Durchsatz am PC ca. 16kB/sec mit Minimalbuffer, bei 8 nur unwesentlich schneller (SPIM 16MHz)
#define SBUF_MULTI 1                                                     // Empf.: 1 oder >= 8 Speed up , bringt aber quasi nix (selbst bei Conn-Int. 6) nur schnellerer Disk-Check, daher: 1 ist OK
#define SBUF_SIZE ((NRF_SDH_BLE_GATT_MAX_MTU_SIZE - 3 - 2) * SBUF_MULTI) // Binary Blocks 2 Bytes smaller!
uint8_t sbuffer[SBUF_SIZE];                                              // Hilfs-Zwischen-Puffer fuer diesunddas (ca. 247)


// ---------Advertsing Typen-------------
#define TX_POWER 4 // S113: Anscheinend setzbar von -40..+4 (S140 bis +8)

//**** DEFAULT INTERVALL *****
#ifdef DEBUG  // ***0.625 msec/Tick***
 #define NRF_LOG // if undefined: Only Minimum LOG
 #define APP_ADV_DEFAULT_INTERVAL 1706 // 1065 msec (1600/sec, max. 16383) **DEBUG SCHNELLER* 
 //#define APP_ADV_DEFAULT_INTERVAL 500 // 312 msec (1600/sec, max. 16383) **DEBUG NOCH SCHNELLER*
 uint32_t dbg_scan_cnt;  // Zaehlt Scans
#else
 #define APP_ADV_DEFAULT_INTERVAL 3412 // 2133 msec (1600/sec max. 16383) mit 1 128Bit UUID (21 Bytes TX) und 3.0V 4dBm 12uA NrF52832 Avg
#endif


#if 0 // Set to 1 for ULTRA LOW POWER-Test
#undef APP_ADV_DEFAULT_INTERVAL
#define APP_ADV_DEFAULT_INTERVAL 16383
#endif

static ble_uuid_t m_adv_uuids[] = {{BLE_UUID_NUS_SERVICE, NUS_SERVICE_UUID_TYPE}}; /** Universally unique service identifier. */

// ------------ Std. Connection -  LowPower: ca. 20-30 uA Avg. ----------------------
// CONN Intervalles ext. def.

#define SLAVE_LATENCY 0
// Da gibt es eine Regel, bei Deklaration 'ble_gap_conn_params_t' erklaert, 10 sec sollten immer OK sein
#define CONN_SUP_TIMEOUT MSEC_TO_UNITS(10000, UNIT_10_MS) /* Connection supervisory timeout (10 seconds), Supervision Timeout uses 10 ms units. */

// 0 Wiederholungen sind ok, wenn Verbindung selbst verwaltet wird
#define FIRST_CONN_PARAMS_UPDATE_DELAY APP_TIMER_TICKS(12500) /* Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called  */
#define NEXT_CONN_PARAMS_UPDATE_DELAY APP_TIMER_TICKS(12500)  /* Time between each call to sd_ble_gap_conn_param_update after the first call  */
#define MAX_CONN_PARAMS_UPDATE_COUNT 0                       /* Number of attempts before giving up the connection parameter negotiation. */

// ------ RX/TX-Buffers (nicht extern verwenden!) ---
static uint8_t tx_wrk_blk[NRF_SDH_BLE_GATT_MAX_MTU_SIZE - 3];     // Temp-Buffer NUR fuer bl_tx_block_xxx();
static uint8_t rx_wrk_blk[NRF_SDH_BLE_GATT_MAX_MTU_SIZE - 3 + 1]; // Temp-Buffer NUR fuer Daten aus RX-Queue  // Inkl. Platz fuer ending 0

NRF_BLE_GATT_DEF(m_gatt); /* GATT module instance. */

#define BLRX_QUEUE_SIZE 2560 // ca. 10 frames!
NRF_QUEUE_DEF(uint8_t, m_blrx_queue, BLRX_QUEUE_SIZE, NRF_QUEUE_MODE_NO_OVERFLOW);

BLE_ADVERTISING_DEF(m_advertising); /* Advertising module instance. */

// Only 1 connection accepted
#if NRF_SDH_BLE_TOTAL_LINK_COUNT != 1
#error "Designed for 1 link only"
#endif
BLE_NUS_DEF(m_nus, NRF_SDH_BLE_TOTAL_LINK_COUNT);

bool  pin_ok = false; // Annahme
bool  ble_connected_flag = false; // gemappt m_conn_handle
static uint16_t m_conn_handle = BLE_CONN_HANDLE_INVALID;               /* Handle of the current connection. */

/*static *todo* kapseln*/ bool fast_advertising_flag = false;
/*static *todo* kapseln*/ uint16_t m_ble_nus_max_data_len = BLE_GATT_ATT_MTU_DEFAULT - 3; /* Maximum length of data (in bytes) that can be transmitted to the peer by the Nordic UART service module. */
/*static *todo* in Flag*/ NUS_STAT nus_stat = {false}; // NUS states (only informative)

CON_PARAM act_conn; // Current Connection Parameters extern

typedef struct {
    int32_t rssi_sum;
    volatile int32_t anz; // Anzahl RSSIs (dynamisch, muss int sein wg. Mittelwert
} RSSI_STAT;
static RSSI_STAT rssi_stat;

// zaehlt inaktivitaet BLE-CMDs, wird von jedem RX/TX auf 0 gesetzt
/*static *todo*/ uint32_t last_ble_cmd_cnt; // Zaehlt inaktivitaet BLE (32 bit hoch/sec)
/*static *todo* kapseln*/ uint16_t ble_rssi_report_cnt;
#endif

//--------------- Allg. Defines, immer praesent

uint32_t tb_uart_sec_counter = TB_UART_ON_SECS; // Zaehlt runter wenn DEBUG

#define APP_BLE_CONN_CFG_TAG 1 /* tag identifying the SoftDevice BLE configuration. */
// Jedes Quellfile kriegt das einzeln, falls LOC_ERROR_CHECK verwendet wird
static void local_error(uint32_t error, uint16_t line_num) {
    NRF_LOG_ERROR("ERROR %u in Line %u\n", error, line_num);
    NRF_LOG_FLUSH();
    app_error_handler(error, line_num, (uint8_t *)__FILE__);
}


#ifdef ENABLE_BLE

static void conn_params_error_handler(uint32_t nrf_error) {
    NRF_LOG_ERROR("conn_params_error_handler()\n");
    NRF_LOG_FLUSH();
    app_error_handler(nrf_error, __LINE__, (uint8_t *)__FILE__);
}


//-------------- BLE Stack --------------------
static void set_device_name(void) {
    uint32_t err_code;
    ble_gap_conn_sec_mode_t sec_mode;
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);
    err_code = sd_ble_gap_device_name_set(&sec_mode, ble_device_name, strlen(ble_device_name));
    LOC_ERROR_CHECK(err_code);
}

// Achtung: Nur erlaubt, wenn auch Advertising moeglich ist, sonst Error
static void trigger_fast_advertising_mode(void) {
    ret_code_t err_code;
    // if already active or already connected: ignore
    if (fast_advertising_flag == true || ble_connected_flag == true)
        return;
    err_code = sd_ble_gap_adv_stop(m_advertising.adv_handle);
    LOC_ERROR_CHECK(err_code);
    err_code = ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);
    LOC_ERROR_CHECK(err_code);
    fast_advertising_flag = true;
#ifdef NRF_LOG
    NRF_LOG_INFO("Fast Advertsing\n");
#endif
}

// Change with 'uint32_t sd_ble_gap_conn_param_update(uint16_t 	conn_handle,ble_gap_conn_params_t const *p_conn_params)'
void gap_params_init(void) {
    uint32_t err_code;
    ble_gap_conn_params_t gap_conn_params;

    set_device_name();
    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = CONN_INTERVAL_START;
    gap_conn_params.max_conn_interval = CONN_INTERVAL_START + CONN_INTERVAL_START / 2 + 3;
    gap_conn_params.slave_latency = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    LOC_ERROR_CHECK(err_code);
}

// if newcon_max=0: set to newcon_min*1.5 + 3
// Return ERR_CODE
uint32_t conn_interval_change(uint32_t newcon_min) {
    uint32_t err_code;
    uint32_t newcon_max;
    ble_gap_conn_params_t gap_conn_params;

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));
    if (newcon_min > 2000)
        newcon_min = 2000; // 2.5 sec
    newcon_max = newcon_min + newcon_min / 2 + 3;
    if (newcon_min < 6)
        newcon_min = 6;
    if (newcon_max < newcon_min)
        newcon_max = newcon_min;

    // If already OK (overlapping intervals)
    if (act_conn.updated == true &&
        ((newcon_min >= act_conn.min_interval && newcon_min <= act_conn.max_interval) ||
            (newcon_max >= act_conn.min_interval && newcon_max <= act_conn.max_interval))) {
#ifdef NRF_LOG
        NRF_LOG_INFO("Set Connection Interval to %u/%u-> Schon OK\n", newcon_min, newcon_max);
#endif
        return NRF_SUCCESS;
    }

    gap_conn_params.min_conn_interval = newcon_min;
    gap_conn_params.max_conn_interval = newcon_max; // Es wird wohl immer nur MAX uebernommen?
    gap_conn_params.slave_latency = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout = CONN_SUP_TIMEOUT;

    act_conn.updated = false;
    err_code = ble_conn_params_change_conn_params(m_conn_handle, &gap_conn_params);
    //LOC_ERROR_CHECK(err_code);
#ifdef NRF_LOG
    NRF_LOG_INFO("Set Connection Interval to %u/%u-> GAP:%u\n", newcon_min, newcon_max, err_code);
#endif
    if (err_code) // err_code = 17: Busy!
        return err_code;
    uint16_t msec_wait = 30000;
    for (;;) {
        tb_delay_ms(1);
        msec_wait--;
        if (!msec_wait)
            return NRF_ERROR_TIMEOUT; //  NRF_ERROR_TIMEOUT = 13
        if (act_conn.updated == true)
            break;
    }
    return NRF_SUCCESS; // 0
}

/*handling the data from the Nordic UART Service.
* This function will process the data received from the Nordic UART BLE Service
* p_evt       Nordic UART Service event. */
static void nus_data_handler(ble_nus_evt_t *p_evt) {

    switch (p_evt->type) {
    case BLE_NUS_EVT_RX_DATA: {
        // Connection Handle fehlt noch
        ret_code_t err_code;
        uint8_t const *p_data = p_evt->params.rx_data.p_data;
        size_t data_len = p_evt->params.rx_data.length;

        // NRF_LOG_DEBUG("BLE_NUS->UART: %u Bytes...\n",data_len);
        // (nrf_queue_in() returns no of written chars), nrf_queue_write() safe for 0-len
        err_code = nrf_queue_write(&m_blrx_queue, p_data, data_len);
        // Check fuer Overflow NRF_ERROR_NO_MEM
        /* FATAL: Kann pasieren wenn CPU zu beschaeftigt und gleichzeitig BLE-Transfer */
        LOC_ERROR_CHECK(err_code);
        //nus_stat.rx_in_cnt++;
    } break;
    case BLE_NUS_EVT_TX_RDY:
        // NRF_LOG_DEBUG("BLE_NUS: TX_RDY\n");       /* Service is ready to accept new data to be transmitted. */
        //nus_stat.tx_ready_cnt++;
        break;
    case BLE_NUS_EVT_COMM_STARTED:
#ifdef NRF_LOG
        NRF_LOG_DEBUG("BLE_NUS: COMM_STARTED\n");
#endif
        nus_stat.comm_started_flag = true;
        break;
    case BLE_NUS_EVT_COMM_STOPPED:
#ifdef NRF_LOG
        NRF_LOG_DEBUG("BLE_NUS: COMM_STOPPED\n");
#endif
        nus_stat.comm_started_flag = false;
        break;
    default:
#ifdef NRF_LOG
        NRF_LOG_DEBUG("nus_data_handler(): %u\n", p_evt->type); // Was ist das??? - Eingebaut JW
#endif
        break;
    }
}

void services_init(void) {
    uint32_t err_code;
    ble_nus_init_t nus_init;

    // Initialize NUS.
    memset(&nus_init, 0, sizeof(nus_init));
    nus_init.data_handler = nus_data_handler;
    err_code = ble_nus_init(&m_nus, &nus_init); // Setzt UUID
    LOC_ERROR_CHECK(err_code);
}

void conn_params_init(void) {
    uint32_t err_code;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail = false; // true; // false; // Also wenn Verbindung nicht upgedated werden kann: true = disconnect
    cp_init.evt_handler = NULL;         // on_conn_params_evt; (Handler entfernt, siehe BLE_NUS-Demo falls noetig)
    cp_init.error_handler = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    LOC_ERROR_CHECK(err_code);
}

/** handling BLE events.
* p_ble_evt   Bluetooth stack event.   p_context   Unused. */
static void ble_evt_handler(ble_evt_t const *p_ble_evt, void *p_context) {
    uint32_t err_code;
    uint16_t handle;
    switch (p_ble_evt->header.evt_id) {
    case BLE_GAP_EVT_CONNECTED:
        handle = p_ble_evt->evt.gap_evt.conn_handle;
#ifdef NRF_LOG
        NRF_LOG_INFO("Connected (%u)\n", handle);
        NRF_LOG_INFO("PARAM: Mi/Ma:%u/%u\n", p_ble_evt->evt.gap_evt.params.connected.conn_params.min_conn_interval,
            p_ble_evt->evt.gap_evt.params.connected.conn_params.max_conn_interval);
#endif
        m_conn_handle = handle;
        ble_connected_flag = true;
        m_ble_nus_max_data_len = BLE_GATT_ATT_MTU_DEFAULT - 3; // Erste Annahme
        act_conn.min_interval = p_ble_evt->evt.gap_evt.params.connected.conn_params.min_conn_interval;
        act_conn.max_interval = p_ble_evt->evt.gap_evt.params.connected.conn_params.max_conn_interval;
        act_conn.updated = true;

        // Div. Flags loeschen!
        f_info.cmd = 0;

        nrf_queue_reset(&m_blrx_queue); // Queue reset
        nus_stat.comm_started_flag = false;
        rssi_stat.rssi_sum = 0;
        rssi_stat.anz = 0;

        last_ble_cmd_cnt = 0; // Action!
        ble_rssi_report_cnt = 0;
        // Initialisierungen: Noch was vergessen?

        // Initialisieren! Und erst schreiben, wenn Notifications gesetzt wurden
        // Scan RSSI (0,0: zu viel aber macht nix)
        err_code = sd_ble_gap_rssi_start(m_conn_handle, 0 /*0: dbm */, 0 /*0: alle 1*/);
        LOC_ERROR_CHECK(err_code);
        //set_device_name_and_busy(true); // Device now busy
        break;

    case BLE_GAP_EVT_DISCONNECTED:
#ifdef NRF_LOG
        handle = p_ble_evt->evt.gap_evt.conn_handle;
        NRF_LOG_INFO("Disconnected (%u)\n", handle);
#endif
        m_conn_handle = BLE_CONN_HANDLE_INVALID;
        ble_connected_flag = false;
        nus_stat.comm_started_flag = false;
        pin_ok = false;
        // Div. Flags loeschen!
        f_info.cmd = 0;

        break;

    case BLE_GAP_EVT_RSSI_CHANGED:
        // Statistics of RSSI
        rssi_stat.rssi_sum += (int32_t)p_ble_evt->evt.gap_evt.params.rssi_changed.rssi;
        rssi_stat.anz++;
        //NRF_LOG_DEBUG("RSSI(%d/%d) -> %d\n",rssi_stat.rssi_sum,rssi_stat.anz,rssi_stat.rssi_sum/rssi_stat.anz);
        break;

    case BLE_GAP_EVT_PHY_UPDATE_REQUEST: {
#ifdef NRF_LOG
        NRF_LOG_DEBUG("PHY update request.\n");
#endif
        ble_gap_phys_t const phys = {
            .rx_phys = BLE_GAP_PHY_AUTO,
            .tx_phys = BLE_GAP_PHY_AUTO};
        err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
        LOC_ERROR_CHECK(err_code);
    } break;

    case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
        // Pairing not supported
#ifdef NRF_LOG
        NRF_LOG_DEBUG("(EVT_SPR)\n");
#endif
        err_code = sd_ble_gap_sec_params_reply(m_conn_handle, BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP, NULL, NULL);
        LOC_ERROR_CHECK(err_code);
        break;

    case BLE_GATTS_EVT_SYS_ATTR_MISSING:
        // No system attributes have been stored.
#ifdef NRF_LOG
        NRF_LOG_DEBUG("(EVT_SAM)\n");
#endif
        err_code = sd_ble_gatts_sys_attr_set(m_conn_handle, NULL, 0, 0);
        LOC_ERROR_CHECK(err_code);
        break;

    case BLE_GATTC_EVT_TIMEOUT:
        // Disconnect on GATT Client timeout event.
#ifdef NRF_LOG
        NRF_LOG_DEBUG("TIMEOUT(GATTC)\n");
#endif
        err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
            BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
        LOC_ERROR_CHECK(err_code);
        break;

    case BLE_GATTS_EVT_TIMEOUT:
        // Disconnect on GATT Server timeout event.
#ifdef NRF_LOG
        NRF_LOG_DEBUG("TIMEOUT(GATTS)\n");
#endif
        err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
        LOC_ERROR_CHECK(err_code);
        break;

    case BLE_GAP_EVT_ADV_SET_TERMINATED:
        // Bei Wechel FAST->SLOW, nach einer Verbindung bleibt ADV-Modus FAST, mit diesem EVT
#ifdef NRF_LOG
        NRF_LOG_DEBUG("ADV__TERMINATED\n");
#endif
        fast_advertising_flag = false;
        
        break;

    case BLE_GAP_EVT_SCAN_REQ_REPORT: // We wwere scanned: go faster
        trigger_fast_advertising_mode(); // MAC gibt keine Aussage
#ifdef NRF_LOG
        NRF_LOG_DEBUG("SCAN_REQ:%d\n", p_ble_evt->evt.gap_evt.params.scan_req_report.rssi);
#endif
#ifdef DEBUG
        dbg_scan_cnt++;
#endif
        break;
    case BLE_GAP_EVT_CONN_PARAM_UPDATE: {
        // Abgefahren langer Scheiss
        ble_gap_conn_params_t gap_conn_params; // Spezialparameter teil von ...
        gap_conn_params = p_ble_evt->evt.gap_evt.params.conn_param_update.conn_params;
#ifdef NRF_LOG
        NRF_LOG_INFO("PARAM Update: Mi/Ma:%u/%u\n", gap_conn_params.min_conn_interval, gap_conn_params.max_conn_interval);
#endif
        // Update the actual parameters
        act_conn.min_interval = gap_conn_params.min_conn_interval;
        act_conn.max_interval = gap_conn_params.max_conn_interval;
        act_conn.updated = true;
    } break;

    case BLE_GATTS_EVT_HVN_TX_COMPLETE:
        /*
        NRF_LOG_DEBUG("EVT_NOTIF_TX\n"); // 0x57/87 nur zur info (Notification Trabsmissiont)
        */
        break;
    case BLE_GATTS_EVT_WRITE:
        /*
        NRF_LOG_DEBUG("EVT_WRITE\n"); // 0x50/80 nur zur info (kommt oefters bei Characteristik-Anederungen)
        */
        break;
#ifdef S112
//#warning "S112 no LL_UPDATES"
#else
    case BLE_GAP_EVT_DATA_LENGTH_UPDATE:
#ifdef NRF_LOG
        NRF_LOG_DEBUG("LL_UPDATE\n"); // 36 nur zur info
#endif
        break;
#endif
    case BLE_GATTS_EVT_EXCHANGE_MTU_REQUEST:
#ifdef NRF_LOG
        NRF_LOG_DEBUG("EXC_MTU REQ\n"); // 0x55/85 nur zur info
#endif
        break;
    case BLE_GATTC_EVT_EXCHANGE_MTU_RSP:
        // Hier kommt Response auf MTU: ble_gattc_evt_exchange_mtu_rsp_t. Mich interessiert ja eigtl. aich nur der Server
        // Hier fand ich aber auch schon >500 bei Windows PCs. Also nur wenn kleiner uebernhemen
#ifdef NRF_LOG
        NRF_LOG_DEBUG("EXC_MTU RSP (MTU:%u)\n", p_ble_evt->evt.gattc_evt.params.exchange_mtu_rsp.server_rx_mtu); // 58/0x3A nur zur info
#endif
        break;

    default:
        // No implementation needed.
#ifdef NRF_LOG
        NRF_LOG_DEBUG("ble_evt_handler(): %u\n", p_ble_evt->header.evt_id); // Was ist das??? - Eingebaut JW
#endif
        break;
    }
}

/* Function for handling events from the GATT library. */
void gatt_evt_handler(nrf_ble_gatt_t *p_gatt, nrf_ble_gatt_evt_t const *p_evt) {
    if ((m_conn_handle == p_evt->conn_handle) && (p_evt->evt_id == NRF_BLE_GATT_EVT_ATT_MTU_UPDATED)) {
        // Hier Datenlaenge setzen
        m_ble_nus_max_data_len = p_evt->params.att_mtu_effective - OPCODE_LENGTH - HANDLE_LENGTH;
#ifdef NRF_LOG
        NRF_LOG_INFO("Data len is set to 0x%X(%d)\n", m_ble_nus_max_data_len, m_ble_nus_max_data_len);
#endif
    }
#ifdef NRF_LOG
    NRF_LOG_DEBUG("ATT MTU exchange completed. central 0x%x peripheral 0x%x\n",
        p_gatt->att_mtu_desired_central,
        p_gatt->att_mtu_desired_periph);
    //MTU:-> OK, die beiden uebernehmen wir als Max. 240 als Datenblock ist ein guter Wert
#endif
}

/* Function for initializing the GATT library. */
void gatt_init(void) {
    ret_code_t err_code;

    err_code = nrf_ble_gatt_init(&m_gatt, gatt_evt_handler);
    LOC_ERROR_CHECK(err_code);

    err_code = nrf_ble_gatt_att_mtu_periph_set(&m_gatt, NRF_SDH_BLE_GATT_MAX_MTU_SIZE); // sdk_config.h 247
    LOC_ERROR_CHECK(err_code);
}

// Setup structure members (if not NULL), Structs must be init with 0
static void setup_src_adv_data(ble_advdata_t *p_advdata, ble_advdata_t *p_srdata) {
//#define USE_MINIMUM_ADV // If defined, advertise in 3 Bytes, Data are in Scan Response, FULL_NAME always in SRDATA!
#ifdef USE_MINIMUM_ADV //
#if DEVICE_NAME_MAXLEN > 10
#warning "POSSIBLE Clip for Device Name in SCAN DATA"
#endif

    // SHORT_NAME is FULL_NAME, limited to advdata.short_name_len
    // -- advertise data --
    //p_advdata->name_type          = BLE_ADVDATA_FULL_NAME;
    if (p_advdata != NULL) {
        p_advdata->name_type = BLE_ADVDATA_NO_NAME;
        p_advdata->include_appearance = false;
        p_advdata->flags = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;
    }
    // -- scan response data ---
    if (p_srdata != NULL) {
        // Include all in scan response to save power
        p_srdata->name_type = BLE_ADVDATA_FULL_NAME;
        p_srdata->uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
        p_srdata->uuids_complete.p_uuids = m_adv_uuids;
    }
#else
    // -- advertise data -- // Advertise with UUID for faster Scan
    if (p_advdata != NULL) {
        p_advdata->name_type = BLE_ADVDATA_NO_NAME;
        p_advdata->include_appearance = false;
        p_advdata->flags = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;
        p_advdata->uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
        p_advdata->uuids_complete.p_uuids = m_adv_uuids;
    }
    // -- scan response data ---
    if (p_srdata != NULL) {
        p_srdata->name_type = BLE_ADVDATA_FULL_NAME; // Name still in SCAN
    }
    // Nothing
#endif
}
#endif // BLE_ENABLE

#ifdef ENABLE_BLE
/**@brief Function for initializing the Advertising functionality.
 */
void advertising_init(void) {
    uint32_t err_code;
    ble_advertising_init_t init;

    memset(&init, 0, sizeof(init));
    setup_src_adv_data(&init.advdata, &init.srdata);

    init.config.ble_adv_slow_enabled = true;
    init.config.ble_adv_slow_interval = APP_ADV_DEFAULT_INTERVAL;
    init.config.ble_adv_slow_timeout = APP_ADV_DEFAULT_DURATION; // Ignored in ble_advertising_init(?)
    init.config.ble_adv_fast_enabled = true;
    init.config.ble_adv_fast_interval = APP_ADV_FAST_INTERVAL;
    init.config.ble_adv_fast_timeout = APP_ADV_FAST_DURATION;

    init.evt_handler = NULL; // on_adv_evt; (removed)

    err_code = ble_advertising_init(&m_advertising, &init);
    LOC_ERROR_CHECK(err_code);

    ble_advertising_conn_cfg_tag_set(&m_advertising, APP_BLE_CONN_CFG_TAG);
}

/* Function for starting advertising. */
void advertising_start(void) {
    ret_code_t err_code;

    /* SCAN-Evernt erzeugen: (verlagert nach lib):
    * m_advertising.adv_params.scan_req_notification = true; // - wenn gesetzt: loest Ereignis aus: BLE_GAP_EVT_SCAN_REQ_REPORT
    * Muss in, 'ble_advertising.c' direktgesetzt werden, da dort adv_params genullt werden... */

    // 15 sec. Fast advertising nach RESET
    err_code = ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST /*BLE_ADV_MODE_SLOW */);
    LOC_ERROR_CHECK(err_code);
    fast_advertising_flag = true;

    // NORDIC: SET HIGH TX POWER FOR ADVERTISING
    err_code = sd_ble_gap_tx_power_set(BLE_GAP_TX_POWER_ROLE_ADV, m_advertising.adv_handle, TX_POWER);
    LOC_ERROR_CHECK(err_code);

#ifdef NRF_LOG
    NRF_LOG_INFO("Advertising Start\n");
#endif
}

/* Radio-Handler NOTIFICATION anwerfen Timig Diagramme dazu siehe JW Doku
* Prinzipiell sind BLE-Notioficatios nicht noetig, wenn noch ein Timer dazu laeuft,
* Dann bleibt MAIN ruhig wenn es nix zu tun hat */
void ble_on_radio_active_evt(bool radio_active) {
  // Dumy Fkt. - Nur damit Task aufwacht
}

static void radio_notification_init(void) {
    uint32_t err_code;
    // Aktiviert bereits Notification BOTH und verwendet in toggelndes flag das handler-Argument
    err_code = ble_radio_notification_init(APP_IRQ_PRIORITY_LOW, // Mittlere/3
        NRF_RADIO_NOTIFICATION_DISTANCE_800US,
        ble_on_radio_active_evt);
    LOC_ERROR_CHECK(err_code);

    /* NUR wenn nur EINE Flanke gewuenscht wird, hier zusaetzlich. */
    err_code = sd_radio_notification_cfg_set(NRF_RADIO_NOTIFICATION_TYPE_INT_ON_ACTIVE,
        NRF_RADIO_NOTIFICATION_DISTANCE_800US);
    LOC_ERROR_CHECK(err_code);
}


// Send large Data Block in Small blocks. tx_len can be 0, if <0: strlen(pout)
// Try as long as success or ERROR. Retry if Busy
ret_code_t bl_tx_block_reliable(uint8_t const *pout, int32_t tx_len, uint8_t token) {
    size_t tosend;
    uint32_t err_code = NRF_SUCCESS;

    if (tx_len < 0)
        tx_len = strlen(pout); // Auto Calculate LEN
    for (;;) {
        tosend = tx_len;
        if (tosend > (m_ble_nus_max_data_len - 2))
            tosend = (m_ble_nus_max_data_len - 2);
        tx_wrk_blk[0] = tosend;
        tx_wrk_blk[1] = token;
        if (tosend)
            memcpy(&tx_wrk_blk[2], (uint8_t *)pout, tosend); // DSN
        do {
            uint16_t slen = (uint16_t)(tosend + 2);
            err_code = ble_nus_data_send(&m_nus, tx_wrk_blk, &slen, m_conn_handle);
            //tb_printf("ReliableRes:%d\n",err_code);
            if ((err_code != NRF_ERROR_INVALID_STATE) && // Err 8
                (err_code != NRF_ERROR_RESOURCES) &&     // Err 19 (Normal if TX FULL)
                (err_code != NRF_ERROR_NOT_FOUND)) {     // Err 5

                if (err_code == BLE_ERROR_INVALID_CONN_HANDLE)
                    return err_code; // Err 12290 Stopped TX
                // z.B. ble_printf() auf nicht komplett aufgebaute Connections gibt hier FATAL 0x3401 (evtl. entschaerfen...)
                LOC_ERROR_CHECK(err_code);

            }
            if (err_code == NRF_ERROR_RESOURCES) {
                tb_board_led_off(0);
                tb_delay_ms(1); // Allow Sleep
            } // else tb_board_led_on(0);
            
        } while (err_code == NRF_ERROR_RESOURCES);
        //nus_stat.tx_out_cnt++;

        // tb_printf("Res:%d TX-Out/Cnt:%u/%u\n",err_code,nus_stat.tx_out_cnt,nus_stat.tx_ready_cnt);

        pout += tosend;
        tx_len -= tosend;
        if (!tx_len)
            break;
    }
    return err_code;
}

/* -----------Get RSSI of Connection-------------
* -128 wenn nix bekannt/anz=0
* Am Anfang Summe ueber evtl. sehr viele, daher 1 mal leer lesen */
int32_t get_con_rssi(int32_t *p_anz) {
    int32_t anz;
    int32_t sum;
    for (;;) {
        anz = rssi_stat.anz;
        sum = rssi_stat.rssi_sum;
        if (anz == rssi_stat.anz)
            break; // Wenn changed: Wdh!
    }
    if (p_anz)
        *p_anz = anz; // Anzahl uebergeben
    if (!anz) {
        sum = -999; // -999 dbm als Annahme wenn nix bekannt
    } else {
        sum /= anz;
        for (;;) { // Reset
            rssi_stat.anz = 0;
            rssi_stat.rssi_sum = 0;
            if (!rssi_stat.anz)
                break; // Falls geandert, nochmal
        }
    }
    return sum;
}

#define MAX_PARSE_OBUF 80
static uint8_t parse_obuf[MAX_PARSE_OBUF]; // Achtung: xx Bytes max.

bool ble_reliable_printf=true;    // Flag Extern setzen , je nach Verwendung! Als externes Flag damit komp. zu printf()

// reliable = false: Nur raushauen in einer Zeile
uint32_t ble_printf(char *fmt, ...) {
    ret_code_t ret;
    uint16_t slen;
    va_list argptr;
    if (ble_connected_flag == false) return NRF_ERROR_INVALID_STATE;
    va_start(argptr, fmt);
    vsnprintf((char *)parse_obuf, MAX_PARSE_OBUF, fmt, argptr); // vsn: limit!
    va_end(argptr);
    if (ble_reliable_printf) {
        return bl_tx_block_reliable(parse_obuf, -1, BB_BLE_REPLY_INTERM); // Token for ASCII
    } else {                                                              // Raus in einer Zeile, egal ob geht oder nicht:
        uint16_t slen = strlen(parse_obuf);
        tx_wrk_blk[0] = slen;
        tx_wrk_blk[1] = BB_BLE_REPLY_INTERM;
        memcpy(&tx_wrk_blk[2], (uint8_t *)parse_obuf, slen); // DSN
        slen += 2;                                           // header dazu
        return ble_nus_data_send(&m_nus, tx_wrk_blk, &slen, m_conn_handle);
    }
}


/*** ble_printf darf pares_obuf verwenden ***/
/*** Check auf Dateikommando ***/
static void ble_file_cmd(void) {
    int16_t res, i;
    char *pc;
    uint32_t val;
    int32_t ival;

    pc = &rx_wrk_blk[2];
    val = strtoul(pc + 1, 0, 0); // Leerstring bereits abgefangen
    switch (*pc++) {
    //----------- JesFs-Commands START --------------


    // --- Write To Internal Memory -----------
    case 'K': // Clear Sectors
      strcpy(parse_obuf, "???");
      if (*pc++ == ':') {
            val=strtoul(pc, &pc, 0);
            if((val%CPU_SECTOR_SIZE) || val<IBOOT_FLASH_START || val>=(IBOOT_FLASH_START + IBOOT_FLASH_SIZE)){
                strcpy(parse_obuf, "ERROR: Sector Addr.");
                break;  // Sector E
            }
            ival = atoi(pc);  // LEN in Bytes, ACHTUNG: Loescht IMMER ganze Sektoren
            for(;;){
              if(ival<=0) break;
              nrf_nvmc_page_erase(val);
              //tb_printf("Clear:%x\n",val);
              ival-=CPU_SECTOR_SIZE;
              val+=CPU_SECTOR_SIZE;
            }
            strcpy(parse_obuf, "~K"); // OK
      }
      break;
    case 'I': // Write to Int. Memory
        strcpy(parse_obuf, "???");
        if (*pc++ == ':') {
            if (*pc > ' ') {
                memset(&f_info, 0, sizeof(f_info));
                f_info.mem_addr=strtoul(pc, 0, 0);
                strcpy(parse_obuf, "~I"); // OK
                f_info.cmd = 'I';         // Filesystem blockiert!
            }
        }
        break;
    // Close "File" (only Memory)
    case 'L':                                // Close (nur fuer block empfangen)
        strcpy(parse_obuf, "ERROR: No 'P/I'"); // Nothing open
        if (f_info.cmd == 'I') {
            f_info.fname[0]=0; // No Filename!
            sprintf(parse_obuf, "~L:%u", f_info.file_len);
        }
        f_info.cmd = 0;
        break;
    //-------------
    case 'R': // System Reset. Muss HIER stehen wg. 'val'
        if(val) _tb_novo[0]=0;  // Full Reset
        ble_reliable_printf=true;
        ble_printf( "RESET...");
        if (ble_connected_flag == true) {
            sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            tb_delay_ms(3000); // Migh need some time
        }
        tb_delay_ms(100);
        tb_system_reset();
        break;

    default:
       if(type_cmdline(SRC_BLE, &rx_wrk_blk[2], val)==false){
            strcpy(parse_obuf, "???");
       }else{
          strcpy(parse_obuf, "OK");
       }

    }
}

/* Parse and reply USER_TERM - Kommands nur fuer USER - AKtuell nicht verwendet
static void parse_and_reply_userterm(void) {
    uint32_t err_code;
    // uint8_t cmdlen=rx_wrk_blk[0]; // Erstmal nur informativ (String ist 0-terminiert)
    uint8_t *pc = &rx_wrk_blk[2];

    // An Userprogramm
    sprintf(parse_obuf, "USER-CMD: '%s'", pc);

    err_code = bl_tx_block_reliable(parse_obuf, -1, BB_BLE_REPLY_END); // Token for ASCII
    tb_printf("'%s' -> USER-CMD:RES:%u\n", &rx_wrk_blk[2], err_code);
}
*/

/* Parse and reply BLT_TERM */
static void parse_and_reply_bleterm(void) {
    uint32_t err_code;
    // uint8_t cmdlen=rx_wrk_blk[0]; // Erstmal nur informativ (String ist 0-terminiert)
    uint8_t *pc = &rx_wrk_blk[2];
    uint32_t val;
    int32_t ival;
    uint16_t i;
    uint8_t hf; // helf-Flags
    int16_t res;

#ifdef NRF_LOG
    NRF_LOG_INFO("BLETERM:'%s'\n", pc);
#endif

    switch (*pc++) {
    case 0:
        strcpy(parse_obuf, "OK"); //
        break;
    case '~': // HALLO-Token, evtl. mit Zusatzdaten und 16-ID
        sprintf(parse_obuf, "~B:%u %08X%08X", m_ble_nus_max_data_len, mac_addr_h, mac_addr_l);
        break;

    case '/': // Mehr infos plus CPU, get **Default-PIN**
#if DEVICE_TYP >= 300  // Pin-Check nur fuer echte Anwendungen oder echte Sensoren
        pin_ok = false;
        val = get_pin();
        if(val) { // >0 Bootloader present => PIN found => Check
          if(!strcmp(pc,SECRET_GLOBALPIN)){  // *** SECRET Backdoor Password, defined in ltx_ble.secret ***
            ;;
          }else{
              if(val!=strtoul(pc, 0, 0)){  // Test PIN
              strcpy(parse_obuf, "~E");  // Pin ERROR
              break;
            }
          }
        }
#endif
        pin_ok = true;
      // CPU-Typ
#ifdef NRF52832_XXAA
        sprintf(parse_obuf, "~V:%u %u %u 32",DEVICE_TYP,DEVICE_FW_VERSION, get_firmware_bootloader_cookie());
#endif
#ifdef NRF52840_XXAA
        sprintf(parse_obuf, "~V:%u %u %u 40",DEVICE_TYP,DEVICE_FW_VERSION, get_firmware_bootloader_cookie());
#endif
        break;

    case '#': // 1 oder 0 nur abfragen: UART-Kontrolle
        if(!pin_ok) break;
        if (*pc == '1') {
            tb_init();
            tb_uart_sec_counter = TB_UART_ON_SECS;
        } else if (*pc == '0') {
            tb_uninit();
            tb_uart_sec_counter = 0;
        }
        if (tb_is_uart_init())
            strcpy(parse_obuf, "UART ON");
        else
            strcpy(parse_obuf, "UART OFF");
        break;

    case 'T': // ZEIT holen oder setzen (muss regelmaessig sein)
        if(!pin_ok) break;
        val = strtoul(pc, 0, 0);
        if (val){
            tb_time_set(val);
        }
        val = tb_time_get();
        sprintf(parse_obuf, "~T:%u\n", val); // Interpret to Date
        break;

    case 'C': //  in Fast oder SLOW oder VALUE ((1..5)6..2000) oder nur abfragen
        if (*pc == 'S')
            val = CONN_INTERVAL_STD;
        else if (*pc == 'F')
            val = CONN_INTERVAL_FAST;
        else
            val = strtoul(pc, 0, 0);
        if (val)
            err_code = conn_interval_change(val); // Skalierbar von (1..)6..2000 max.
        else
            err_code = 1; // 1: Kein Fehler: Bed.: keine Werte

        sprintf(parse_obuf, "~C:%u %u", (act_conn.min_interval + act_conn.max_interval) / 2, err_code); // Interpret Reply
        break;


    default:
        // Parse OBUG wird gefuellt (irgendeine EIngabe ist da)
        if(pin_ok) ble_file_cmd();
    }
    err_code = bl_tx_block_reliable(parse_obuf, -1, BB_BLE_REPLY_END); // Token for ASCII

    // tb_printf("'%s' -> BLE_CMD:RES:%u\n", &rx_wrk_blk[2], err_code);
}

/* Wenn BLE Aktiv wird das hier regelmaessig aufgerufen */
void ble_periodic_connected_service(void) {
    size_t ble_anz;
    uint32_t err_code;
    uint8_t blk_hdr;
    int16_t res;

    // Evtl. nus_stat.comm_started_flag pruefen vor dem Senden.
    for (;;) {
        ble_anz = nrf_queue_utilization_get(&m_blrx_queue); // Soviele sind verfuegbar
        if (ble_anz < 2)
            break; // Minimum 2 Chars fuer Minimum CMD-Len

        last_ble_cmd_cnt = 0; // Aktivitaet!
        ble_rssi_report_cnt = 0;

        err_code = nrf_queue_peek(&m_blrx_queue, &blk_hdr);
        LOC_ERROR_CHECK(err_code);
        blk_hdr += 2; // 2 Anfangsbytes zaehlen auch
        if (ble_anz < blk_hdr)
            break; // Block noch nicht ganz da
        if (blk_hdr > NRF_SDH_BLE_GATT_MAX_MTU_SIZE - 3)
            LOC_ERROR_CHECK(OWN_ERR_BUFFERSIZE);
        err_code = nrf_queue_read(&m_blrx_queue, rx_wrk_blk, blk_hdr); // Ganzen Block lesen, aber nicht mehr
        LOC_ERROR_CHECK(err_code);

        rx_wrk_blk[blk_hdr] = 0; // Am Ende 0
        switch (rx_wrk_blk[1]) {
/*** Aktuell nicht verwendet
        case BB_BLE_USERCMD:            // Was fuer User
            parse_and_reply_userterm(); // Terminal-Kommandos auswerten
            break;
***/
        case BB_BLE_CMD:               // BLETERM-CMD
            parse_and_reply_bleterm(); // Terminal-Kommandos auswerten
            break;
        case BB_BLE_PLING: // Nix zu tun
            tb_printf("PLING");
            break;
        case BB_BLE_BINBLK_IN:                        // BinBlock
#if DEBUG
            tb_printf("B[%u]", rx_wrk_blk[0]);        // LEN
#endif
            if(rx_wrk_blk[0]){
                  if(f_info.cmd == 'I'){
                  uint8_t wlen = rx_wrk_blk[0];
/* *todo* HIER NOCH Schreibschutz auf BEREICH und Adresse einbauen!!! ***/
                  if(f_info.mem_addr>=IBOOT_FLASH_START && f_info.mem_addr<(IBOOT_FLASH_START + IBOOT_FLASH_SIZE)){
tb_printf("Mem:%x\n",f_info.mem_addr);
                    nrf_nvmc_write_words(f_info.mem_addr, (uint32_t *)&rx_wrk_blk[2], (wlen+3)/4);
                  }
else tb_printf("MemERROR:%x\n",f_info.mem_addr);
                  f_info.mem_addr += wlen;
                  f_info.file_len += wlen;
                  break;
               }
            } // Else Fall though

        default:
            //tb_printf("??? %u %u '%s'\n", rx_wrk_blk[0], rx_wrk_blk[1], &rx_wrk_blk[2]);
            tb_printf("??? %u %u\n", rx_wrk_blk[0], rx_wrk_blk[1]);
        }
    }

    // --- Optional alle paar Sekunden RSSI schicken. Wenn nicht ankommt: Egal ---
    if (last_ble_cmd_cnt >= BLE_IDLE_CNT && ble_rssi_report_cnt >= BLE_REPORT_LIMIT) {
        ble_rssi_report_cnt = 0;
        // Ohne Wdh., nur einmalig
        tx_wrk_blk[0] = sprintf(&tx_wrk_blk[2], "R:%d", get_con_rssi(NULL));
        tx_wrk_blk[1] = BB_BLE_INFO;
        uint16_t slen = tx_wrk_blk[0] + 2;
        /*err_code */ ble_nus_data_send(&m_nus, tx_wrk_blk, &slen, m_conn_handle);
        // tb_printf("Rep:'%s'",&tx_wrk_blk[2]);
    }
}
#endif

#ifdef ENABLE_BLE
// GLOBAL Possible while advertising, Nename; minimum 3 Chars!
void advertising_change_name(char *newname) {
    uint32_t err_code;
    if (strlen(newname) < 3)
        return;
    // Nix zu tun
    if (!strncmp(ble_device_name, newname, BLE_DEVICE_NAME_MAXLEN))
        return;
    tb_printf("BLE Adv.-Name '%s' -> '%s'\n", ble_device_name, newname);
    strncpy(ble_device_name, newname, BLE_DEVICE_NAME_MAXLEN); // DSN
    set_device_name();
    ble_advdata_t advdata, srdata;
    memset(&srdata, 0, sizeof(srdata));
    memset(&advdata, 0, sizeof(advdata));
    setup_src_adv_data(&advdata, &srdata); // Both required or NULL if omitted
    err_code = ble_advertising_advdata_update(&m_advertising, &advdata, &srdata);
    LOC_ERROR_CHECK(err_code);
}

// Disconnect-Wrapper
uint32_t ble_disconnect(void){
    return sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
}
#endif

void ble_stack_init(void) {
    ret_code_t err_code;

    err_code = nrf_sdh_enable_request();
    LOC_ERROR_CHECK(err_code);

    // Configure the BLE stack using the default settings.
    // Fetch the start address of the application RAM.
    // 1 Connection braucht fast 4K RAM auf dem S113!
    uint32_t ram_start = 0, vorschlag;
    err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
    LOC_ERROR_CHECK(err_code);

    // Enable BLE stack. - Das SD benoetigt eine gewisse Menge RAM, abstimmen mit App
#ifdef NRF_LOG
    vorschlag = ram_start;
#endif
    err_code = nrf_sdh_ble_enable(&ram_start);
#ifdef NRF_LOG
    NRF_LOG_INFO("Stack Frei: %d Bytes\n", (int32_t)(vorschlag - ram_start));
#endif
    LOC_ERROR_CHECK(err_code);

    err_code = sd_power_dcdc_mode_set(NRF_POWER_DCDC_ENABLE);
    LOC_ERROR_CHECK(err_code);
#ifdef ENABLE_BLE
    // Register a handler for BLE events.
    NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
#endif
}

//
