/*************************************************
* ONEWIRE.C - For Timelot API
* (C) joembedded.de
* V1.10 / 02.11.2021
*
* Timeslots require 20msec! 
* OW_ERROR_TIMESLOT Error for shorter Connection
* Intervals
*
*
* ACHTUNG: Anscheinend gibt es Clones des DS18B20!
* Zeigt bei SPEE 57788(-xx) an. "Echte" DS18B20 liefern
* offensichtlich immer SPEE = 65535(-1) und die Reset-Temperatur
* ist 85.0 Grad (pee gemapped aufint16)
*
* Man sollte das auch anhand der SNO herausbekommen koennen:
* Sehr interessanter Artikel: 
* https://github.com/cpetrich/counterfeit_DS18B20
**************************************************/

#include "app_error.h"
#include "nrf.h"
#include "nrf_delay.h"
#include "nrf_gpio.h"
#include "tb_tools.h"
#include "nrf_sdh.h"
#include "nrf_sdh_soc.h"
#include <stdbool.h>
#include <stdint.h>

#include "osx_pins.h"

#include "onewire.h"

/********************************************************************
* Globals and Defs
*********************************************************************/
#define GID 6   // Guard-ID fuer dieses Modul
//#define SCP_DEBUG // Scratchpad ausgeben

// --- Timeslot  ---
static nrf_radio_request_t  m_timeslot_request;
#define M_SLOT_LENGTH_US      22000 // SubScan ist das langsamste, OK fuer OW_EXTRA_US 30
#define M_TIMEOUT_US        1000000
// Protokoll is OK fuer OW_EXTRA_US=0, laeuft aber bis OW_EXTRA_US=30 (dann noch 1msec Reserve)
#define OW_EXTRA_US         30

static nrf_radio_signal_callback_return_param_t signal_callback_return_param;

// Parameter-Block Timeslot
#define TEMP_TRIGGER    1
#define TEMP_READ       2
#define OWBUS_SUBSCAN    3
#define SENSOR_WRITE_SPEE 4

static volatile bool ts_closed_flag=true;
static uint8_t ts_param_ts_cmd;
static uint8_t *ts_param_addr;
static int16_t *ts_param_pspee;
static uint16_t ts_param_val;
static int16_t ts_return;        

// --- OneWire ---
OW_ADDR ow_addr[MAX_OW_SENS]; // The 1 Wire IDs 8 Byte per sensor
uint8_t ow_last_error_code; // ERROR 0..x
uint8_t ow_anz;

// --Internas--
//#define USE_CTRL_LED    // For DEBUG 
//#define TRACE_TIME     // Misst die Zeiten und zeigt sie an
//#define USE_PASSIVE     // Dann auf Passiv Pin Pulse ausgeben

#ifdef USE_PASSIVE       // Feinaufloesung
  #define USE_PASSIVE_W // Pulse beim Schreiben
  #define USE_PASSIVE_R // Pulse beim Lesen
  #define USE_PASSIVE_S // Pulse beim SubScan
#endif

#ifdef USE_CTRL_LED
  #define CTRL_LED x14x     // To measure Runtime LED2 on PCA10056 (alt. wenn nicht 3-er Klinke PASSIVE_0)
#endif

#define ONEWIRE_PIN0        IX_SDA   // Kommunikations-Pin Sensor, Kanal 0
#define ONEWIRE_PASSIVE0    IX_SCL // Evt. per 3p-Klinke 2. Pin Kanal 0

static uint8_t io_kanal;
static NRF_GPIO_Type *ponewire_gpio_base; // Which PIN used?
static uint32_t pin_mask; 

#ifdef USE_PASSIVE     
 static NRF_GPIO_Type *ppass_gpio_base; // GPIO for passive pin DEBUG
 static uint32_t pass_mask;
 #define OW_PASSIVE_LOW()  (ppass_gpio_base->OUTCLR=pass_mask)
 #define OW_PASSIVE_HIGH() (ppass_gpio_base->OUTSET=pass_mask)
#endif

//---------------------------------------------------------------
// Mgl. Pin Configs

#define OW_READ()      (!!(ponewire_gpio_base->IN&pin_mask))
#define OW_PULL_LOW()  (ponewire_gpio_base->OUTCLR=pin_mask)
#define OW_PULL_HIGH() (ponewire_gpio_base->OUTSET=pin_mask)

/*****************************************************************************************************
* Helper Functions
* Return CRC8 of a block, See AN27. Attention: CRC of 0-Block is 0
*****************************************************************************************************/
static uint8_t _onewire_check_crc(uint8_t *pu, uint8_t len){
    uint8_t i,h;
    uint8_t crc=0;
    while(len--){
        h=*pu++;
        for(i=0;i<8;i++){
            if((crc^h)&1) {crc>>=1; crc^=0x8C;}
            else crc>>=1;
            h>>=1;
        }
    }
    return crc;
}

/*********************************************************************
* TIMESLOT START - Run in granted defined timeslots
*
* Functions:
* static int16_t _ts_onewire_sub_scan(uint8_t idx);
* static int16_t _ts_onewire_write_spee(uint8_t *adr,int16_t val);
* static int16_t _ts_onewire_temptrigger(void);
* static int16_t _ts_onewire_tempread(uint8_t *addr, int16_t *pspee);
*
* OneWire-Functions:
*********************************************************************/

/* Takes *1* msec  returns OW_ERROR_NONE (==0) if any 1-Wire-devices are found,
* OW_ERROR_NOSENSOR (==1) or OW_ERROR_SHORTCIRCUIT (==2)
* All with _ts_ handled via timeslots */

static int16_t _ts_onewire_reset(void){
    uint8_t pin;
    int16_t result;

#if OW_EXTRA_US
      nrf_delay_us(OW_EXTRA_US);
#endif
    if(!OW_READ()) {
        ow_last_error_code=OW_ERROR_SHORTCIRCUIT; // Short Circuit!
        result=-OW_ERROR_SHORTCIRCUIT;
    }else{
      OW_PULL_LOW(); // Pull LOW active
      nrf_delay_us(480+OW_EXTRA_US*2);
      OW_PULL_HIGH(); // Pull High passive
      nrf_delay_us(70);
      pin=OW_READ();    // Sample PIN
      nrf_delay_us(410+OW_EXTRA_US*2);
      if(pin){
         ow_last_error_code=OW_ERROR_NOSENSOR; // Nothing
         result=-OW_ERROR_NOSENSOR;
      }else{
        ow_last_error_code=OW_ERROR_NONE;    // Assume no error
        result=0;  // = OW_ERROR_NONE
      }
    }
    return result;
}

/* Takes 70+OW_EXTRA_US usec Write Bit0 of bit
* If Bit1-x is set, Fkt. can exit fast for optional BusPower with
* Mosfet.* Which must be switched on at at least 10usec after command
* For CMDs 0x44 and 0x48; Last Bit is 0.
* and keep IRQs disabled!
*/

static void _ts_onewire_writebit(uint8_t bit){
#ifdef USE_PASSIVE_W
 OW_PASSIVE_HIGH();
 nrf_delay_us(1);
 OW_PASSIVE_LOW();
#endif
    if(bit&1){ // Write 1
        OW_PULL_LOW();  // Pull LOW active
        nrf_delay_us(6-1); // 6
        OW_PULL_HIGH(); // Pull High passive
        nrf_delay_us(64+OW_EXTRA_US); // 64
   }else{ // Write 0
       OW_PULL_LOW();  // Pull LOW active
        nrf_delay_us(60-1); // 60
       OW_PULL_HIGH(); // Pull High passive
       if(bit) { // Bit&2 was set
           // In Parasite Power mode Power must be switched on <10usec after command
//#if MOSFET
//  MOSFET=ON
//#endif           
           return; // Fast Exit 
       }
       nrf_delay_us(10+OW_EXTRA_US); // 10
   }
}

// Takes 70+OW_EXTRA_US usec
static uint8_t _ts_onewire_readbit(void){
    uint8_t pin;
#ifdef USE_PASSIVE_R
 OW_PASSIVE_HIGH();
 nrf_delay_us(1);
 OW_PASSIVE_LOW();
#endif
    OW_PULL_LOW();  // Pull LOW active
    nrf_delay_us(6-1); // 6
    OW_PULL_HIGH(); // Pull High passive
    nrf_delay_us(9); // 9
    pin=OW_READ();    // Sample PIN
    nrf_delay_us(55+OW_EXTRA_US); // 55
    return pin;
}


/* Write complete Byte in LSBit first. Option for Fast exit, see writebit
Both commands which require strong Pullup have 0 in the MSB (4x) 
If Bit 8 is set: MOSFET ON!*/
static void _ts_onewire_writebyte(uint16_t bval){
    uint8_t i;
    for(i=0;i<8;i++) {
        if(i==7) _ts_onewire_writebit(bval&3);
        else _ts_onewire_writebit(bval&1);
        bval>>=1;
    }
}

// Read complete Byte in LSBit first
static uint8_t _ts_onewire_readbyte(void){
    uint8_t bval=0; // Problem: If not initialised, wrong shifts
    uint8_t i;
    for(i=0;i<8;i++) {
        bval>>=1;
        if(_ts_onewire_readbit()) {
            bval|=128;
        }
    }
    return bval;
}

// *** Selects ONE sensor (Match ROM), 0: OK, 1: ERROR with ow_last_error_code set
int16_t _ts_onewire_select_adr(uint8_t *pu){
    uint8_t i;
    int16_t result;
    result = _ts_onewire_reset();
    if(result) return result;
    if(pu){
      _ts_onewire_writebyte(0x55);   // Select one sensor MMATCH ROM
      for(i=0;i<8;i++) {
          _ts_onewire_writebyte(*pu++); // Send Addres
      }
    }else{
      _ts_onewire_writebyte(0xCC);   // Skip ROM
    }
    return 0;
}


/* onewire_temptrigger(): Timeslot-Part Start all sensors. After abt. (200 to) 750 msec all
* sensors are ready. During the conversion a switch might supply power in
* 2 lines version (parasitic power)
* Return: 0: OK, else Error
*/
static int16_t _ts_onewire_temptrigger(void){
    int16_t result;
    result = _ts_onewire_reset();
    if(result) return result;   

    _ts_onewire_writebyte(0xCC);     // all Sensors (Skip ROM)

    // Command: Simultaneous conversion (valid for S and B) with opt. Power ON
    _ts_onewire_writebyte(0x44|0x100);
    return 0;
}


/* TIMESLOT Read the temperature from the formerly selected sensor, inkl. CRC
* Return the temp in Steps of 0.01oC or -1000-ERROR on errors
* The fam_id determins the algorithm. 0x10 for S, 0x28 for B
* psp_ee might be NULL if not used.
*
* Wenn nur 1 DS18*B*20 am BUS ist, kann ROM geskippt werden: addr=NULL
* ACHTUNG: Wenn PEE nicht NULL: Integritaet checken, dazu MUSS addr vorhanden sein
*/
static int16_t _ts_onewire_tempread(uint8_t *addr, int16_t *pspee){
    int16_t tval;
    uint8_t i;
    uint8_t ow_res[8];  // 8+1 Bytes of Data
    int16_t result;
    uint8_t h;

    result=_ts_onewire_select_adr(addr);
    if(result){ // Select THIS
        // Result 1..x already in ow_last_error_code
        return -10000+result; // Result is neg.
    }

    _ts_onewire_writebyte(0xBE);     // Command: Read Scratchpad (valid for S and B)
    for(i=0;i<8;i++){
        ow_res[i]=_ts_onewire_readbyte();
    }

    // Very long cables have high capacity, all bits might be 0
    // This is not caught by the CRC8. Electrically: Short circuit
    for(i=0;i<8;i++){
        if(ow_res[i]) break;
        if(i==7) {
            ow_last_error_code=OW_ERROR_SHORTCIRCUIT;
            return -10000-OW_ERROR_SHORTCIRCUIT;
        }
    }

    // Take Value in Advance: If Sensor doesn not reply: will give FFFF and wrong CRC,
    // to distinguish from real CRC-Errors..
    tval=(ow_res[0]|(ow_res[1]<<8));   // Temp. LSB/MSB (LE)
    if(tval==-1){
        ow_last_error_code=OW_ERROR_NOSENSOR;
        return -10000-OW_ERROR_NOSENSOR;
    }

    // last Byte must be equal to calculatet CRC
    i=_ts_onewire_readbyte();

    if(_onewire_check_crc(ow_res,8)!=i){
        ow_last_error_code=OW_ERROR_CRC8;
        return -10000-OW_ERROR_CRC8;
    }

#ifdef SCP_DEBUG
    // Show Scratchpad. Die "guten" Clones haben korrektes Scratchpad, aber nicht die 0 in SNO! 
    // Die "Guten" clones kann man ja im Layer darueber (ueber Adresse) pruefen

    // 18*B*20: SNO 28 xx xx xx xx xx 00 CRC
    // bei power ON: 5005:85Grad 4b46:EEPOM65535 7F:Conv12Bit FF:Fest xx:ignore: 10:Fest
    //            50  05   4b   46   7F   FF   xx   10 
    //
    // 18*S*20: SNO 10 xx xx xx xx xx 00 CRC
    // bei power ON: aa00:85Grad 4b46:EEPOM65535 FF FF:Fest 0c:CntsRem(<=10): 10:Fest
    //            aa  00   4b   46   FF   FF   0c   10
    tb_printf("(%02x %02x %02x %02x %02x %02x %02x %02x)\n",ow_res[0],ow_res[1],ow_res[2],ow_res[3],ow_res[4],ow_res[5],ow_res[6],ow_res[7]);
#endif

    // Entweder PSP erfragen und/oder integritaets-Check
    if(pspee && addr){ // If needed..
        *pspee=((ow_res[3]|(ow_res[2]<<8))^46265);   // SP_EE MSB/LSB (BE) Scratchpad, mapped
        switch(addr[0]){ // family ID
        case 0x10: // *S*
          if((ow_res[4]!=0xFF) || (ow_res[5]!=0xFF) || (ow_res[6]>0x10) || (ow_res[7]!=0x10)){
            ow_last_error_code=OW_ERROR_COEFFERR;
            return -10000-OW_ERROR_COEFFERR;
          }
          break;
        case 0x28: // *B*
          if((ow_res[5]!=0xFF) || (ow_res[7]!=0x10)){
            if((ow_res[4]!=0x7F)){ // 0x3F und 0x5F: Lowres
              ow_last_error_code=OW_ERROR_NOT_HIRES;
              return -10000-OW_ERROR_NOT_HIRES;
            }else{
              ow_last_error_code=OW_ERROR_COEFFERR;
              return -10000-OW_ERROR_COEFFERR;
            }
          }
          break;
        default:
          ow_last_error_code=OW_ERROR_UNKNID;
          return -10000-OW_ERROR_UNKNID;
        }
    }

    if(addr) h=addr[0];
    else h=0x28;  // ASSUME Ds1820
    switch(h){ // family ID
    case 0x10:  // DS18S20 Tricky calculation, see Datasheet
        if(tval==0xAA){ // 85.0 oC: Keine Wandlung(?)
          ow_last_error_code=OW_ERROR_NOCONV;
          return -10000-OW_ERROR_NOCONV;
        }
        tval=tval*50+75-(ow_res[6]*25)/4;
        return tval;
    case 0x28: // DS18B20: 1 Dig. 6.25 *0.01 oC
        if(tval==0x550) { // 85.0 oC: Keine Wandlung(?)
          ow_last_error_code=OW_ERROR_NOCONV;
          return -10000-OW_ERROR_NOCONV;
        }
        return (tval*6)+(tval/4);
    default:
        ow_last_error_code=OW_ERROR_UNKNID;
        return -10000-OW_ERROR_UNKNID;
    }
}


/* Write a 16Bit Word into the sensor
* Not Verified!
* Return 0: OK, >0: ERROR */
static int16_t _ts_onewire_write_spee(uint8_t *adr,int16_t val){
    uint8_t i;
    int16_t result;
    result=_ts_onewire_reset();
    if(result) return result;   // Some Error

    _ts_onewire_writebyte(0x55);         // ONE Sensore select
    for(i=0;i<8;i++) {
         _ts_onewire_writebyte(*adr++); // Send Addres
     }
     adr-=8;

     _ts_onewire_writebyte(0x4E);         // Write Scratchpad
     val^=46265;    // Map Def. value to 65535
     _ts_onewire_writebyte((uint8_t)(val>>8));      // H
     _ts_onewire_writebyte((uint8_t)val);       // L

     if(*adr==0x28){
           _ts_onewire_writebyte(0x7F);       // DS18B20: Write Konfig:0x7F:12Bit 0x1F:9Bit
     }

    result=_ts_onewire_reset();
    if(result) return result;   // Some Error

     _ts_onewire_writebyte(0x55);         // ON Sensore select
     for(i=0;i<8;i++) {
         _ts_onewire_writebyte(*adr++); // Send Addres
     }

     _ts_onewire_writebyte(0x48|0x100);     // Command: Copy Scratchpad with opt. Power ON

     return 0;
}

// Fork-List
static uint8_t forkbits[8]; // 64 Bits
static void onewire_setforkbit(uint8_t idx){
    uint8_t *pfork;
    uint8_t maske;
    maske=(1<<(idx&7));
    pfork=&forkbits[idx>>3];
    *pfork |= maske;
}
static void onewire_clrforkbit(uint8_t idx){
    uint8_t *pfork;
    uint8_t maske;
    maske=(1<<(idx&7));
    pfork=&forkbits[idx>>3];
    *pfork &= ~maske;
}
static uint8_t onewire_getforkbit(uint8_t idx){
    uint8_t *pfork;
    uint8_t maske;
    maske=(1<<(idx&7));
    pfork=&forkbits[idx>>3];
    return (*pfork) & maske;
}


/* onwire_sub_scan() (internal)
* This function does a non-recoursive, partly tree-walk on a 1W-Bus
*
* Start with idx = 0x40
* Return: 0..3F: Branch found
* 0x40: Done
* <0 Error!
*/
static int16_t _ts_onewire_sub_scan(uint8_t idx){
    int16_t result;
    uint8_t i,temp,d;

    if(idx==0x40){  // First clear all bits
        for(i=0;i<64;i++) onewire_clrforkbit(i); // Lower to 0
    }else{
        onewire_setforkbit(idx);
        for(i=idx+1;i<64;i++) onewire_clrforkbit(i); // Lower to 0
    }

    result=_ts_onewire_reset();
    if(result) return result;   // Error: <0

#ifdef USE_PASSIVE_S
 OW_PASSIVE_HIGH();
 nrf_delay_us(1);
 OW_PASSIVE_LOW();
#endif

    _ts_onewire_writebyte(0xF0);         // Search ROM
    idx=0x40;
    for(i=0;i<64;i++){

        temp=_ts_onewire_readbit();
        temp+=(_ts_onewire_readbit()<<1);

        switch(temp){
        case 0:
            // Fork-Point
            if(onewire_getforkbit(i)){
                d=1;    // Mark node as used
            }else{
                d=0;    // Not marked: again
                idx=i;  // Save Index, take 0
            }
            break;
        case 1:
            d=1;
            break;
        case 2:
            d=0;
            break;
        case 3:
            ow_last_error_code=OW_ERROR_BITERROR;
            return -OW_ERROR_BITERROR;
        }

        _ts_onewire_writebit(d&1); // Only lowest

        if(d) onewire_setforkbit(i);
        else  onewire_clrforkbit(i);
    }
    return idx;         // Fork-Index
}

/*********************************************************************
* TIMESLOT API Helpers
*********************************************************************/
static uint32_t request_next_event_earliest(void){

    m_timeslot_request.request_type                = NRF_RADIO_REQ_TYPE_EARLIEST;
    m_timeslot_request.params.earliest.hfclk       = NRF_RADIO_HFCLK_CFG_XTAL_GUARANTEED;
    m_timeslot_request.params.earliest.priority    = NRF_RADIO_PRIORITY_NORMAL;
    m_timeslot_request.params.earliest.length_us   = M_SLOT_LENGTH_US;
    m_timeslot_request.params.earliest.timeout_us  = M_TIMEOUT_US;
    return sd_radio_request(&m_timeslot_request);
}

static void nrf_evt_signal_handler(uint32_t evt_id){
    uint32_t err_code;
    switch (evt_id)
    {
        case NRF_EVT_RADIO_SIGNAL_CALLBACK_INVALID_RETURN:
            break;
        case NRF_EVT_RADIO_SESSION_IDLE:
            sd_radio_session_close(); // Shutdown in 2 Stufen: 1.)
            break;
        case NRF_EVT_RADIO_SESSION_CLOSED:
            ts_closed_flag=true;  // Ready ! 2.) 
            //session ended NOW
            break;
        case NRF_EVT_RADIO_BLOCKED:     //Fall through
        case NRF_EVT_RADIO_CANCELED:
            sd_radio_session_close(); // ERROR - Shutdown in 2 Stufen: 1.)
            break;
        default:
            break;
    }
}

static nrf_radio_signal_callback_return_param_t * radio_callback(uint8_t signal_type){
    int16_t ret;
    switch(signal_type){
        case NRF_RADIO_CALLBACK_SIGNAL_TYPE_START:
            GUARD(GID);
            switch(ts_param_ts_cmd){
            case TEMP_TRIGGER:
              ret= _ts_onewire_temptrigger(); // ohne Ext: ca. 2.5 msec
              break;
            case TEMP_READ:
              ret = _ts_onewire_tempread(ts_param_addr, ts_param_pspee);  // ohne Ext: ca. 12.5 msec
              break;
            case OWBUS_SUBSCAN:
              ret = _ts_onewire_sub_scan((uint8_t)ts_param_val);   //ohne Ext:  ca. 16 msec
              break;
            case SENSOR_WRITE_SPEE:
              ret = _ts_onewire_write_spee(ts_param_addr,ts_param_val); // ohne Ext: ca. 15 msec
              break;
            }
            ts_return=ret;
            
            signal_callback_return_param.params.request.p_next = NULL;
            signal_callback_return_param.callback_action = NRF_RADIO_SIGNAL_CALLBACK_ACTION_END;
            break;

        case NRF_RADIO_CALLBACK_SIGNAL_TYPE_RADIO:
            signal_callback_return_param.params.request.p_next = NULL;
            signal_callback_return_param.callback_action = NRF_RADIO_SIGNAL_CALLBACK_ACTION_NONE;
            break;
        default:
            break;
    }
    return (&signal_callback_return_param);
}

static void time_slot_soc_evt_handler(uint32_t evt_id, void * p_context){
    nrf_evt_signal_handler(evt_id);
}
NRF_SDH_SOC_OBSERVER(m_time_slot_soc_observer, 0, time_slot_soc_evt_handler, NULL);

// trigger timeslot, normally return unused
static int16_t ts_trigger_and_wait(void){
    uint32_t err_code;
    uint16_t wait;
    int16_t res;

    if(ts_closed_flag==false) return -1; // Busy 1
    ts_closed_flag=false;

    err_code = sd_radio_session_open(radio_callback);
    if (err_code != NRF_SUCCESS){
        ts_closed_flag=true;
        return -2;  // Busy 2
    }
    
    err_code = request_next_event_earliest();
    if (err_code != NRF_SUCCESS){
        sd_radio_session_close();
        ts_closed_flag=true;
        return -2;  // Busy 3
    }

    // Wait 50 msec longer than theroretically possible
    wait=(M_SLOT_LENGTH_US + M_TIMEOUT_US)/1000 + 50;
    res=0;
    while(ts_closed_flag==false){
      tb_delay_ms(1);
      if(wait) {
        wait--;
        if(!wait) {
          sd_radio_session_close(); // Implicit resets ts_closed_flag
          res=-3;
        }
      }
    }
    return res;
}

/*********************************************************************
* TIMESLOT END
*********************************************************************/

/*********************************************************************
* Wrapped:
* static uint8_t _ts_onewire_sub_scan(uint8_t idx);
* static uint8_t _ts_onewire_write_spee(uint8_t *adr,int16_t val);
* static uint8_t _ts_onewire_temptrigger(void);
* static int16_t _ts_onewire_tempread(uint8_t *addr, uint16_t *pspee);
*********************************************************************/
static int16_t ts_run(uint8_t ts_cmd, uint8_t* addr, uint16_t *pspee,uint16_t val){
  int16_t ret=-1; 
#ifdef USE_CTRL_LED
    nrf_gpio_pin_clear(CTRL_LED); // ON 
#endif


    ts_param_ts_cmd = ts_cmd;
    ts_param_addr = addr;
    ts_param_pspee = pspee;
    ts_param_val = val;
    ts_return = -OW_ERROR_TIMESLOT; // preset Error if timeslot fails
    ow_last_error_code = OW_ERROR_TIMESLOT;
#ifdef TRACE_TIME
    uint32_t t0=tb_get_ticks();
#endif
    ts_trigger_and_wait();
#ifdef TRACE_TIME
    tb_printf("C:%u ms:%u\n",ts_cmd,tb_deltaticks_to_ms(t0,tb_get_ticks()));
#endif

    ret=ts_return;


#ifdef USE_CTRL_LED
    nrf_gpio_pin_set(CTRL_LED); // OFF
#endif

  return ret;
}

/********************************************************************************
* onewire_temptrigger(): Wrapper Start all sensors. After abt. (200 to) 750 msec all
* sensors are ready. During the conversion a switch might supply power in
* 2 lines version (parasitic power)
* Return: 0: OK, else Error
********************************************************************************/
int16_t onewire_temptrigger(void){
   int16_t res;
   if(!ponewire_gpio_base) return -OW_ERROR_NOT_OPEN;

   //---------- TIMESLOT START --------------
   res=ts_run(TEMP_TRIGGER, NULL, NULL,0);
   //---------- TIMESLOT END --------------

   if(res) return res;

    //------ Not Time Critical-----------
    tb_delay_ms(750);
//#if MOSFET
//  MOSFET=OFF
//  wait(xusec)
//#endif           
    return 0;           // OK

}
/********************************************************************************
* Wrapper for tempread 
********************************************************************************/
int16_t onewire_tempread(uint8_t *addr, int16_t *pspee){
  int16_t res;
  //---------- TIMESLOT START --------------
  res = ts_run(TEMP_READ,addr, pspee, 0);
  //---------- TIMESLOT END --------------
  return res;
}

/********************************************************************************
* Wrapper fuer onewire_write_spee with noncritical timing parts
********************************************************************************/
int16_t onewire_write_spee(uint8_t *adr,int16_t val){
     int16_t res;

     //---------- TIMESLOT START --------------
     res=ts_run(SENSOR_WRITE_SPEE,adr,NULL,val);
     //---------- TIMESLOT END --------------
     
     if(res) return res;
     tb_delay_ms(15);      // Wait ca 10 msec until conversion is finished

//#if MOSFET
//  MOSFET=OFF
//  wait(xusec)
//#endif           
    return 0;
}

/*********************************************************************
* onewire_scan_bus() (external)
* This function does a compelete non-recoursive tree-walk on a 1W-Bus
* and finds all IDs of all connected sensors.
*
* Returns 0 (OK) or index in Error Code (also might set ow_last_error_code)
*********************************************************************/
int16_t onewire_scan_bus(void){
    uint8_t anz,i,h;
    uint8_t *pu;
    int16_t idx;

    ow_anz=0;   // Assume nothing found
    anz=0;
    idx=0x40;
    memset(forkbits,0,sizeof(forkbits));
    for(;;){
        //---------- TIMESLOT START --------------
        idx=ts_run(OWBUS_SUBSCAN,NULL,NULL,idx);
        //---------- TIMESLOT END --------------
        if(idx<0) break; // Error 

        pu=ow_addr[anz].addr;
        for(i=0;i<8;i++){
            h=forkbits[i];
            *pu=h;
            pu++;
        }

        // Passiert manchmal beim Scannen der (billigen China-Kopie) Sensoren?
        if(_onewire_check_crc(pu-8,7)!=h){
            ow_last_error_code=OW_ERROR_COEFFCRC; // Sensor broken?
            break;
        }

        ow_anz=++anz;

        if(idx&0xC0) break; // End (Bit 7 set)(or Error)
        if(anz>=MAX_OW_SENS){ // Maximum Number reached, but more Sensors followinng?
            ow_last_error_code=OW_ERROR_TOOMANY;    // Too many is not really an error...
            break;  // Too much... Stop scanning
        }else if(!anz){ // Normally detected already during reset_puse
            ow_last_error_code=OW_ERROR_NOSENSOR;
        }
    }
    return -ow_last_error_code;
}

/********************************************************************
* open/close 1-Wire Port
* 0: OK <0: ERROR. Port not available?
*********************************************************************/
int16_t onewire_open(uint8_t io_kanal) {
    uint32_t pin = 0; // Wird durch decode veraendert! (SDA via 4kt PU)
    uint32_t passive_pin=0; // 2. Pasiver PIN (SCL via 2.2Mega-Ohm)
    switch (io_kanal) {
    case 0:
        pin = ONEWIRE_PIN0; // Vzw. SDA
        passive_pin=ONEWIRE_PASSIVE0; // Vzw. SCL
        break;
    default: 
        return -OW_ERROR_UNKNID;
    }
#ifdef USE_CTRL_LED
    nrf_gpio_cfg_output(CTRL_LED); // 1 TIEM ON/OFF
    nrf_gpio_pin_set(CTRL_LED); // OFF
#endif
    // Bidirectional 8051-style OpenDrain
    nrf_gpio_cfg(pin, NRF_GPIO_PIN_DIR_OUTPUT, NRF_GPIO_PIN_INPUT_CONNECT,NRF_GPIO_PIN_NOPULL,
        NRF_GPIO_PIN_H0D1,  // High-Drive 0, Disconnect 1
        NRF_GPIO_PIN_NOSENSE);

    nrf_gpio_cfg_default(passive_pin); // Disable Port (evtl. Kurzschluss gg GND bei Klinke 3P)
    nrf_gpio_pin_clear(passive_pin); // Set to 0

#ifdef USE_PASSIVE  // Kurze Pulse
    nrf_gpio_cfg_output(passive_pin); // Passive Pin as Output
    nrf_gpio_pin_clear(passive_pin); // Start with 0
    ppass_gpio_base=nrf_gpio_pin_port_decode(&passive_pin); // maskiert PIN
    pass_mask=(1<<passive_pin);
#endif

    // Die 3P-Klinke zieht diesen Pin(Vzw. SCL) nach GND, doe 4P-Klinke laesst ihn offen (2M2 Pullup)
    ponewire_gpio_base = nrf_gpio_pin_port_decode(&pin); // maskiert PIN
    pin_mask=(1<<pin);
    OW_PULL_HIGH(); // Pull High passive - Default State
    return 0;
}
void onewire_close(void) {
    uint32_t pin=0;
    uint32_t passive_pin=0; // 2. Pasiver PIN (SCL via 2.2Mega-Ohm)
    switch (io_kanal) {
    case 0:
        pin = ONEWIRE_PIN0;
        passive_pin=ONEWIRE_PASSIVE0;
        break;
    default: 
        return; // ???
    }
    nrf_gpio_cfg_default(pin);
    nrf_gpio_cfg_default(passive_pin);
} 

/*********************************************************************
* After successful scan the sensors might be sorted according to
* the EEPROM value. This allows to map sensors to channels and positions,
* Sort increasing: Sensors without spee set (== Factory: 65535/57xxx(?)) 
* at the end. 
* **Attention: spee is only valid after a measure**
*
* Also possible: Use parts (eg. lower few bits) of spee to correct Offset
* for a dedicated Temperature (e.g. 0 deg. Celsius or so)...
*********************************************************************/
void onewire_sort_spee(void){
    OW_ADDR otmp;
    uint8_t h,i;
    h=ow_anz;
    if(h<1) return; // Nothing to do
    while(h--){     // Simple Bubble Sort
        for(i=0;i<h;i++){
            // Swap if
            if(ow_addr[i].spee>ow_addr[i+1].spee){
                otmp=ow_addr[i];    // Structure Copy!
                ow_addr[i]=ow_addr[i+1];
                ow_addr[i+1]=otmp;
            }
        }
    }
}


/*********************************************************************
* TEST-Display open Bus. Nur wandeln wenn meas>0! (Zum Lesen des
* Power On-Wertes, der bei Clones manchmal merkwuerdig sein kann.
* Flag 1: Measure 2: Scan immer: Anzeigen
*********************************************************************/
void test_show_onewire(uint8_t todo){
     int16_t res;
     uint8_t anz,i,j;
     OW_ADDR *plp;

     tb_printf("---TestShow1W:%u---\n",todo);
     if(todo&2){
      if(onewire_scan_bus()){
           tb_printf("ERROR(%u)\n",ow_last_error_code);
           return;
       }
     }
     anz=ow_anz;
     if(todo&1){
      if(onewire_temptrigger()){
           tb_printf("ERROR(%u)\n",ow_last_error_code);
           return;
       }
     }
     tb_printf("---Sensors:%u---\n",anz);
     for(i=0;i<anz;i++){
         plp=&ow_addr[i];
         //if(plp->addr[7]!=0) tb_printf("*Clone!* ");
         tb_printf("#%u: Addr:",i);
         for(j=0;j<8;j++) tb_printf("%02x ",plp->addr[j]);
         tb_printf("-> ");
         
         // Get SPEE with the data (Temp in 0.01 oC)!  Values <-10000 are ERRORS
         res=onewire_tempread(plp->addr, &plp->spee); // 
         if(res<=-10000) tb_printf("ERROR(%u)\n",ow_last_error_code);
         else tb_printf("Temp: %d SPEE:%d\n",res, plp->spee);
     }
}


// END


