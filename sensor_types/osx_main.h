/****************************************************
* type_main.h - Sensor Cmdline
*
***************************************************/
#define SRC_NONE 0 // No Output
#define SRC_CMDLINE 1 // Debug TB_UART
#define SRC_SDI 2 // SDI12
#define SRC_BLE 3 // BLE

// SDI12 allows max. 9 Chars or 7 Digits!
// E.g use 'snprintf(channel_val[].txt,10,"%+f",...)'
// and check for >< 10 Mio, '+9999999.' is not legal
#define MAX_CHAN 10 // Note: Logger-Driver can acept >10
typedef struct {
  int8_t didx;  // Index for 'D'/'R'-Commands, -1: None
  char txt[11]; // Output in SDI121 Format '+/-dd.ddddd', max. 10 char
  char *punit;  // OPt.
} CHANNEL_VAL;

typedef struct{
  // Input
  char measure_cmd; // 'M', 'R', ..
  uint8_t measure_arg; // 0..9
  // Output
  uint16_t anz_channels;
  uint32_t m_msec;  // Rounded up to sec
  
  CHANNEL_VAL channel_val[MAX_CHAN];
} SDI_VALIO;

extern SDI_VALIO sdi_valio;
extern char sensor_id[8 + 6 + 3 + 13 + 1];

// Internam Parameter IDs:
#define ID_INTMEM_SDIADR 1 // Memory ID fuer Addresse
#define ID_INTMEM_USER0  100 // First ID for Sensor

extern char my_sdi_adr;
#define MAX_OUTBUF SDI_OBUFS
extern char outrs_buf[MAX_OUTBUF];


#define VMON_CHANNEL            NRF_SAADC_INPUT_AIN7
#define HK_VBAT_KOEFF           0.00346 // Div: 4M7-1M6 (0 - 14.2V)

void type_init(void);
bool type_cmdline(uint8_t isrc, uint8_t *pc, uint32_t val);
bool type_service(void); // Ret: false: Kein Periodic

//=== in xxxx_sensor_xxx.c: ===
void sensor_init(void);
bool sensor_valio_input(char cmd, uint8_t carg);
int16_t sensor_valio_measure(uint8_t isrc);
void sensor_valio_xcmd(uint8_t isrc, char *pc);
int16_t sensor_wait_break(uint8_t isrc, int16_t wt);

//***