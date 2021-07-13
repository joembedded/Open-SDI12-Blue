/***********************************************************************
* filepool.h LIGHT - Diverse allg. Fkt. fuer Files
* Minimalvariante fuer content.x
***********************************************************************/

//----------------- GLOBALS ---------------
// Allgemeine Puffer fuer File-Ops
// For common use
// Running FS CMD (verwendet fs_desc)
typedef struct {
    uint8_t cmd; // 0:NIX, "G" oder ..
    uint8_t fname[22];
    uint32_t file_len;
    uint32_t getpos;
    uint32_t getlen;
    // Direkter Speicherzugriff
    uint32_t mem_addr;  
} F_INFO;
extern F_INFO f_info; // Haupts. von BLE-Transfer verw. MAIN


#define UNI_LINE_SIZE 240   //  (macht Sinn das gross zu waehlen um z.B. Files zu verifizieren)
extern uint8_t uni_line[UNI_LINE_SIZE];
extern uint16_t uni_line_cnt;  // Zaehlt die Zeilen

// Helper
#define DBUF_SIZE 24
extern char date_buffer[DBUF_SIZE];
void conv_secs_to_date_buffer(uint32_t secs);

uint32_t fpool_get_hex(char **ppc, uint8_t max);    // Read HEX


// END
