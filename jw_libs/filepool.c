/***********************************************************************
* filepool.c - Diverse allg. Fkt. fuer Files - LIGHT
* Minimalvariante fuer content.x
***********************************************************************/


#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>

#include "device.h"

#include "JesFs.h"  // required for fs_sec1970-conversions!
#include "filepool.h"


//----------------- GLOBALS ---------------

F_INFO f_info; // Haupts. von BLE-Transfer verw. (auch MEMORY)
FS_DATE fs_date; // Structe holding date-time

// Macht Sinn uni_line[] gut gross zu halten, z.B. damit Files schneller verifizierz werden koennen
uint8_t uni_line[UNI_LINE_SIZE]; // *** MULTI USE * UNI_LINE_SIZE 240 by default ***
uint16_t uni_line_cnt;  // Zaehlt die Zeilen

#define UNI_CACHE_SIZE 128    // max 255
static uint8_t _uni_cache[UNI_CACHE_SIZE];
static uint8_t _uni_cache_out; // Ausgangsziger
static uint8_t _uni_cache_len; // Wieviel max. da


char date_buffer[DBUF_SIZE];   // Datebuffer, max. 20 Chars

//=== common helpers === - Noch rausnehmen nach Filepool
// Helper Function for readable timestamps -- Hier optional noch Zeitformate anhand iparam.flags,,,
void conv_secs_to_date_buffer(uint32_t secs) {
    fs_sec1970_to_date(secs, &fs_date);
    if(secs<1000000000){ // max 11k days
      sprintf(date_buffer, "PwrOn+%u.%02u:%02u:%02u", (secs/86400), fs_date.h, fs_date.min, fs_date.sec);
    }else{
      sprintf(date_buffer, "%02u.%02u.%04u %02u:%02u:%02u", fs_date.d, fs_date.m, fs_date.a, fs_date.h, fs_date.min, fs_date.sec);
    }
}

/*****************************************************************************
* fpool_get_hex - get a value in HEX representatin from a pointer in BE Format
* Reads max. max chars, so a Byte 1 is max=2 (and max<=8 of course)
* ppc might be NULL.
* This can be used to store/read binary blocks in Files.
****************************************************************************/
uint32_t fpool_get_hex(char **ppc, uint8_t max){
    char *pc,c,anz=0;
    uint32_t val=0;

    pc=*ppc;
    for(;;){
        c=*pc;
        if(c<32) break;   // End of String detected? STOP
        pc++;
        if((c<'0' || c>'9') && (c<'a' || c>'f') && (c<'A' || c>'F')) {
            if(!anz) continue;  // ignore leading Whitespace or Non-HEX
            else break;         // but exit after the first
        }
        if(c>='a') c-=('a'-10); // kleinbuchstaben hoechste!
        else if(c>='A') c-=('A'-10);
        else c-='0';

        val<<=4;;
        val+=c;
        anz++;
        if(anz==max) break; // limit length
    }
    if(ppc) *ppc=pc;    // Save End-Pointer (if not NULL)
    return val;
}
// END
