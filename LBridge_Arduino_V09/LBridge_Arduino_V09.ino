/*
   LBridge scans the Freestyle Libre Sensor every 5 minutes
   and sends the current BG readings to the xDrip+ Android app.

   This sketch is based on the LimiTTer project from JoernL, the
   sample sketches for the BM019 module from Solutions Cubed and the
   protocol extensions done be savek-cc in the Wixel project.

   This code replaces the LimiTTer code without any changes needed.

   Hardwaresource in xDrip has to be set to "LimiTTer". Please use
   an xDrip+ version >= nightly build 2107/02/09

   The xBridge2 protocol is used to send BG readings to xDrip+. In case of failure it queues up not sended BG readings
   with the correct timestamp and send them within the next BLE connection. There should be no missed BG readings
   when the LimiTTer is worn. Battery performance is improved compared to LimiTTer.

   Wiring for UNO / Pro-Mini:

   Arduino          BM019           BLE-HM11
   IRQ: Pin 9       DIN: pin 2
   SS: pin 10       SS: pin 3
   MOSI: pin 11     MOSI: pin 5
   MISO: pin 12     MISO: pin4
   SCK: pin 13      SCK: pin 6
   I/O: pin 3 (VCC with Android 4)  VCC: pin 9
   I/O: pin 5                       TX:  pin 2
   I/O: pin 6                       RX:  pin 4
*/

/* 
 changes since 170716_1950:
  	- AT+RENEW fail mechanism disabled to ensure usage with modified fake modules
	- display serial status chars while sleeping
	- detect faulty sensorsa during normal lifetime of 14 days
	- removed fault in status display of spike removal
	- added AT command before AT+ commands to break BLE connection if active
	- hard system reset if AT+NOTI1 command cannot be processed
	- slightly modified debug oputput
  V09.01:
  - new version number / minor version number
*/

/* ************************************************ */
/* *** config #DEFINES **************************** */
/* ************************************************ */

#define N_RELEASE_VERSION

#define LB_DATETIME "180207_2208"
#define LB_VERSION "V0.9"        // version number
#define LB_MINOR_VERSION ".01"  // indicates minor version

#ifdef RELEASE_VERSION

#define LNAME    "LimiTTer"   // BLE name, must begin with "LimiTTer" to avoid misfunctions in xDrip+
// important features 
#define N_USE_DEAD_SENSOR     // we can test with a dead sensor
#define REMOVE_SPIKES         // use NFC data smoothing (LimiTTer method)?
#define AUTOCAL_WDT           // code for auto calibrating WDT timing
#define ATFAIL_REBOOT         // reboot system if AT+NOTI goes fail
#define N_ATRESET             // do a AT+RESET after every BLE wakeup?
#define TRANSFER_LIVETIME     // send sensor lifetime
#define N_XBEXT               // xbridge extended code

// less common settings for debug or test
#define N_DB_PKT              // display BG queue, 'X' command
#define N_DB_VOLTAGE            // display battery usage, 'V' command
#define N_SIMU_LIVETIME       // simulate sensor livetime
#define N_HW_BLE              // detect BLE conn status with system LED pin connected to arduino pin 2
#define N_DYNAMIC_TXID        // get TXID automatically
#define N_DB_NFC_DATA         // debug NFC data
#define N_SIMU_BG             // simulate BG readings from dead sensor, ramp curve
#define N_UPDATE_HM1X         // use with extreme care! update HM-11
#define N_DB_PROCESSING       // extended debug output
#define N_INIT_WITH_RENEW     // send AT+RENEW at BLE init - will kill fake modules as setting them to 115200
#define N_SAFETY_RENEW        // send AT+RENEW after 30 min of no BLE

#else /* RELEASE_VERSION */

#define LNAME    "LimiTTerTVW"  // BLE name, must begin with "LimiTTer" to avoid misfunctions in xDrip+
// important features 
#define USE_DEAD_SENSOR       // we can test with a dead sensor
#define REMOVE_SPIKES         // use NFC data smoothing (LimiTTer method)?
#define AUTOCAL_WDT           // code for auto calibrating WDT timing
#define ATFAIL_REBOOT         // reboot system if AT+NOTI goes fail
#define N_ATRESET             // do a AT+RESET after every BLE wakeup?
#define TRANSFER_LIVETIME     // send sensor lifetime
#define N_XBEXT               // xbridge extended code

// less common settings for debug or test
#define N_DB_PKT                // display BG queue, 'X' command
#define N_DB_VOLTAGE            // display battery usage, 'V' command
#define N_SIMU_LIVETIME       // simulate sensor livetime
#define N_HW_BLE              // detect BLE conn status with system LED pin connected to arduino pin 2
#define DYNAMIC_TXID        // get TXID automatically
#define N_DB_NFC_DATA         // debug NFC data
#define N_SIMU_BG               // simulate BG readings from dead sensor, ramp curve
#define N_UPDATE_HM1X         // use with extreme care! update HM-11
#define N_DB_PROCESSING         // extended debug output
#define N_INIT_WITH_RENEW     // send AT+RENEW at BLE init - will kill fake modules as setting them to 115200
#define N_SAFETY_RENEW        // send AT+RENEW after 30 min of no BLE

#endif /* RELEASE_VERSION */

#include <SPI.h>
#include <SoftwareSerial.h>
#include <avr/sleep.h>
#include <avr/power.h>
#include <avr/wdt.h>
#ifdef XBEXT
#include "xbridge_libre.h"
#endif

#define MAX_NFC_READTRIES 3   // amount of tries for every nfc block-scan
#define NFC_READ_RETRY 3      // amount of tries to recall read_memory() in case of NFC read error
#define STOP_SEND_READINGS (((14*24)+12)*60) // 14,5 days
#define SPIKE_RANGE 40        // eliminates spikes which are +- from the last reading

/* ********************************************* */
/* ********* LimiTTer stuff ******************** */
/* ********************************************* */

float calibv = 1.0;   // ratio of real clock with WDT clock

#define MIN_V 3450 // battery empty level
#define MAX_V 4050 // battery full level

#define WDT_8S_MASK 0b00100001
#define WDT_1S_MASK 0b00000110

const int SSPin = 10;   // Slave Select pin
const int IRQPin = 9;   // Sends wake-up pulse for BM019
const int NFCPin1 = 7;  // Power pin BM019
const int NFCPin2 = 8;  // Power pin BM019
const int NFCPin3 = 4;  // Power pin BM019
const int BLEPin = 3;   // BLE power pin.
const int MOSIPin = 11;
const int SCKPin = 13;

#ifdef HW_BLE
const int bleSysLedPin = 2; // connect to Sys LED pin of HM-1x
#endif

byte RXBuffer[24];
byte NFCReady = 0;  // used to track NFC state
byte FirstRun = 1;
byte batteryLow;
int batteryPcnt;
long batteryMv;

int sleepTime = 32;   // sleeptime in multipliers of 8 s

int noDiffCount = 0;
int sensorMinutesElapse;
float lastGlucose;
float trend[16];

SoftwareSerial ble_Serial(5, 6); // RX | TX

/* ************************************************* */
/* *** control stuff ******************************* */
/* ************************************************* */

// global error counters
unsigned long loop_count = 0;          // of main loop
unsigned long normal_loop_count = 0;
unsigned long firstret_loop_count = 0;
unsigned long secondret_loop_count = 0;

unsigned long ble_connect_errors = 0;  // no BLE connect after 40 s wait time
unsigned long nfc_read_errors = 0;     // e. g. no sensor in range
unsigned long nfc_scan_count = 0;      // how many scans?
unsigned long nfc_inv_cmd_count = 0;   // how much SetInventroy commands?

unsigned long nfc_prot_set_cmd_errors = 0;
unsigned long nfc_inv_cmd_errors = 0;
unsigned long nfc_read_mem_errors = 0;
unsigned long resend_pkts_events = 0;
unsigned long queue_events = 0;

unsigned long resend_wakeup_cnt = 0;

static boolean show_ble = 1;        // what is shown in serial monitor
static boolean show_state = 1;      // show print_state() infos, disabled ftm

boolean sensor_oor;                 // sensor out of range
int ble_answer;                     // char from BLE reeived?
boolean ble_lost_processing = 1;    // send AT+RESET after OK+LOST?

// absolute progam run time in s counting also sleep() phase
unsigned long prg_run_time = 0;     // in sec
unsigned long next_normal_wakeup;
int cons_loop_wo_ble = 0;           // consequtive loops w/o BLE connect

unsigned long loop_start_time;      // loop starting time in ms
unsigned long loop_time;            // current loop duration in ms
unsigned long ble_start_time = 0;   // time when BLE starts ms
unsigned long ble_connect_time = 0; // time up to BLE connection in ms

float avg_sleepTime = 0;            // average sleep time
float bleTime = 0;                  // average arduino+BLE time
float bleNFCTime = 0;

unsigned long var_sleepTime;        // howlong do we want to sleep / s
unsigned long queuesendtime;
unsigned long now;
boolean evtflg1;
boolean evtflg2;
int get_packet_result;
unsigned long next_normal_start;  // next normal 5 min start time

#define NORMAL 0
#define ST_RESEND 1
#define ND_RESEND 2

int loop_state;                   // state machine for loop

boolean v_command = 0;              // v command received?

#ifdef DB_VOLTAGE
#define VOLTAGE_INTERVAL  24
#endif

boolean x_command = 0;              // X command received?

int pkt_retries;

unsigned long no_nfc_reading = 0;
unsigned long bg_is_null = 0;

int spikeCount = 0;
int trendSpikeCount = 0;
int averageSpikeCount = 0;
int glucoseWasChanged = 0;

int simuBGindex = 0;

unsigned long arduino_ontime = 0;
unsigned long arduino_start_time = 0;
unsigned long ble_ontime = 0;
//unsigned long ble_activated = 0;
unsigned long nfc_ontime = 0;
unsigned long nfc_start_time = 0;

unsigned long last_arduino_ontime = 0;
unsigned long last_ble_ontime = 0;
unsigned long last_nfc_ontime = 0;

/* ************************************************************* */
/* *** xBridge2 stuff ****************************************** */
/* ************************************************************* */

// defines the xBridge protocol functional level.  Sent in each packet as the last byte.
#define DEXBRIDGE_PROTO_LEVEL (0x01)

//static volatile boolean do_sleep = 0;     // indicates we should go to sleep between packets
static volatile boolean got_ack = 0;      // indicates if we got an ack during the last do_services.
static volatile boolean got_txid = 0;     // indicates if we got a TXID packet during the last do_services.
//int resend_wakeup;                        // prior packet transfer successful?
//int stop_resending = 0;
#ifdef XBEXT
static volatile boolean got_drp;          // DataRequestPacket
#endif
static volatile boolean dex_tx_id_set;    // indicates if the Dexcom Transmitter id (settings.dex_tx_id) has been set.  Set in doServices.
static volatile boolean ble_connected;    // bit indicating the BLE module is connected to the phone.  Prevents us from sending data without this.
#ifdef HW_BLE
static volatile boolean hwble_connected;  // bit indicating the BLE module is connected to the phone.  Prevents us from sending data without this.
#endif

static volatile boolean got_ok = 0;         // flag indicating we got OK from the HM-1x
static unsigned long last_ble_send_time;
static boolean crlf_printed = 0;

//static volatile unsigned long pkt_time = 0;
static volatile unsigned long abs_pkt_time = 0;
static volatile unsigned long last_abs_pkt_time = 0;

//define the maximum command string length for commands.
#define COMMAND_MAXLEN 40

char at_answer[COMMAND_MAXLEN];

//structure of a USB command
typedef struct _command_buff
{
  unsigned char commandBuffer[COMMAND_MAXLEN];
  unsigned char nCurReadPos;
} t_command_buff;

static t_command_buff command_buff;

typedef struct _Dexcom_packet // container for NFC readings on LimiTTer
{
  unsigned long raw;
  unsigned long ms;
#ifdef DB_PKT
  unsigned long sended;
  unsigned char retries;
#endif
} Dexcom_packet;

#ifdef DB_VOLTAGE
typedef struct {
  unsigned char voltage;
  unsigned char a_on;
  unsigned char b_on;
  unsigned char n_on;
} t_voltage_storage;

// store battery levels every 2 h
#define MAX_VOLTAGE 24
#define VOLT_CORR 250  // store range of 470 ... 300 in 8 bit

t_voltage_storage batt_voltage[MAX_VOLTAGE];
unsigned char voltage_wi = 0;
unsigned char voltage_ri = 0;
#endif /* DB_VOLTAGE */

#define DXQUEUESIZE (5*12)      // queue depth, 12 values per hour

typedef struct {
  volatile unsigned char read;
  volatile unsigned char write;
  Dexcom_packet buffer[DXQUEUESIZE];
} Dexcom_fifo;

Dexcom_fifo Pkts;

Dexcom_packet * DexPkt;

// structure of a raw record we will send.
typedef struct _nRawRecord
{
  unsigned char size; //size of the packet.
  unsigned char cmd_code; // code for this data packet.  Always 00 for a Dexcom data packet.
  unsigned long raw;  //"raw" BGL value. ??? use unfiltered NFC readings here?
  unsigned long filtered; //"filtered" BGL value
  unsigned char dex_battery;  //battery value
  unsigned char my_battery; //xBridge battery value
  unsigned long dex_src_id;   //raw TXID of the Dexcom Transmitter
  unsigned long delay;
  unsigned char function; // Byte representing the xBridge code funcitonality.  01 = this level.
} nRawRecord;

// _xBridge_settings - Type definition for storage of xBridge_settings
// used for compatibility
typedef struct _xBridge_settings
{
  unsigned long dex_tx_id;     //4 bytes
  unsigned long uart_baudrate; //4 bytes
} xBridge_settings;     //14 bytes total

xBridge_settings settings;

// array of HM-1x baudrates for rate detection.
unsigned long uart_baudrate[9] = {9600L, 19200L, 38400L, 57600L, 115200L, 4800, 2400, 1200, 230400L};

#ifdef XBEXT
_QuarterPacket1 qpkt1;
_QuarterPacket2 qpkt2;
#endif

/* ********************************************* */
/* *** help functions ************************** */
/* ********************************************* */

// get free mem available
extern unsigned int __bss_end;
extern unsigned int __heap_start;
extern void *__brkval;
int freeMemory() {
  int free_memory;

  if ((int)__brkval == 0)
    free_memory = ((int)&free_memory) - ((int)&__bss_end);
  else
    free_memory = ((int)&free_memory) - ((int)__brkval);

  return free_memory;
}

// millis() since program start, Arduino millis() are not counting when in sleep mode
unsigned long abs_millis(void)
{
  return (prg_run_time * 1000 + (millis() - loop_start_time));
}

/* **************************************************************** */
/* ********* modified LimiTTer code ******************************* */
/* **************************************************************** */

// initialize the hardware
void setup() {
  // NFC part
  pinMode(IRQPin, OUTPUT);
  digitalWrite(IRQPin, HIGH);
  pinMode(SSPin, OUTPUT);
  digitalWrite(SSPin, HIGH);
  pinMode(NFCPin1, OUTPUT);
  digitalWrite(NFCPin1, HIGH);
  pinMode(NFCPin2, OUTPUT);
  digitalWrite(NFCPin2, HIGH);
  pinMode(NFCPin3, OUTPUT);
  digitalWrite(NFCPin3, HIGH);

  // BLE part
  pinMode(BLEPin, OUTPUT); // Disable this for Android 4
  digitalWrite(BLEPin, HIGH); // Disable this for Android 4
  ble_start_time = millis();
  ble_connected = 0;
#ifdef HW_BLE
  hwble_connected = 0;
  pinMode(bleSysLedPin, INPUT);
#endif

  // NFC part
  /* not needed here, is done in SPI.begin(), acoording to Bert Roode, 170302
      pinMode(MOSIPin, OUTPUT);
      pinMode(SCKPin, OUTPUT);
  */
  // try this first
  Serial.begin(9600);

  arduino_start_time = millis();
}

/* *********************************************************** */
/* *** NFC LimiTTer code, only small modifications *********** */
/* *********************************************************** */

void SetProtocol_Command() {
  unsigned long ct;

//  print_state(F("do SetProtocol_Command() ..."));

  digitalWrite(SSPin, LOW);
  SPI.transfer(0x00);  // SPI control byte to send command to CR95HF
  SPI.transfer(0x02);  // Set protocol command
  SPI.transfer(0x02);  // length of data to follow
  SPI.transfer(0x01);  // code for ISO/IEC 15693
  SPI.transfer(0x0D);  // Wait for SOF, 10% modulation, append CRC
  digitalWrite(SSPin, HIGH);
  delay(1);

  ct = millis();
  digitalWrite(SSPin, LOW);
  while ( RXBuffer[0] != 8 )
  {
    RXBuffer[0] = SPI.transfer(0x03);  // Write 3 until
    RXBuffer[0] = RXBuffer[0] & 0x08;  // bit 3 is set
    if ( (millis() - ct) > 1000) {
      print_state(F("poll SetProtocol_Command not successfull"));
      break;
    }

  }
  digitalWrite(SSPin, HIGH);
  delay(1);

  digitalWrite(SSPin, LOW);
  SPI.transfer(0x02);   // SPI control byte for read
  RXBuffer[0] = SPI.transfer(0);  // response code
  RXBuffer[1] = SPI.transfer(0);  // length of data
  digitalWrite(SSPin, HIGH);

  if ((RXBuffer[0] == 0) & (RXBuffer[1] == 0))  // is response code good?
  {
#ifdef DB_PROCESSING
    print_state(F("Protocol Set Command OK"));
#endif
    NFCReady = 1; // NFC is ready
  }
  else
  {
    print_state(F("Protocol Set Command FAIL"));
    NFCReady = 0; // NFC not ready
    nfc_prot_set_cmd_errors++;
  }
//  print_state(F("done SetProtocol_Command() ..."));
}

void Inventory_Command() {
  unsigned long ct;
  int cnt = 0;

  digitalWrite(SSPin, LOW);
  SPI.transfer(0x00);  // SPI control byte to send command to CR95HF
  SPI.transfer(0x04);  // Send Receive CR95HF command
  SPI.transfer(0x03);  // length of data that follows is 0
  SPI.transfer(0x26);  // request Flags byte
  SPI.transfer(0x01);  // Inventory Command for ISO/IEC 15693
  SPI.transfer(0x00);  // mask length for inventory command
  digitalWrite(SSPin, HIGH);
  delay(1);

  ct = millis();
  digitalWrite(SSPin, LOW);
  while (RXBuffer[0] != 8)
  {
    RXBuffer[0] = SPI.transfer(0x03);  // Write 3 until
    RXBuffer[0] = RXBuffer[0] & 0x08;  // bit 3 is set
    if ( (millis() - ct) > 1000) {
      print_state(F("poll Inventory_Command not successfull"));
      break;
    }
  }
  digitalWrite(SSPin, HIGH);
  delay(1);

//  print_state(F("poll Inventory_Command successfull"));

  while ( cnt++ < 10 ) {
    digitalWrite(SSPin, LOW);
    SPI.transfer(0x02);   // SPI control byte for read
    RXBuffer[0] = SPI.transfer(0);  // response code
    RXBuffer[1] = SPI.transfer(0);  // length of data
    for (byte i = 0; i < RXBuffer[1]; i++)
      RXBuffer[i + 2] = SPI.transfer(0); // data
    digitalWrite(SSPin, HIGH);
    delay(1);

    if (RXBuffer[0] == 128)  // is response code good?
    {
  #ifdef DB_PROCESSING
      print_state(F("Sensor in range ... OK"));
  #endif
      NFCReady = 2;
      sensor_oor = 0;
      break;
    }
    else
    {
      print_state(F("Sensor out of range"));
      NFCReady = 1;
      nfc_inv_cmd_errors++;
      sensor_oor = 1;
    }

    delay(100);

  }
}

float Read_Memory(boolean *readDone)
{
  byte oneBlock[8];
  String hexPointer = "";
  String trendValues = "";
  String hexMinutes = "";
  String elapsedMinutes = "";
  float trendOneGlucose;
  float trendTwoGlucose;
  float currentGlucose = 0;  // initialise to avoid compiler warning
  float shownGlucose;
  float averageGlucose = 0;
  int glucosePointer;
  int validTrendCounter = 0;
  float validTrend[16];
  unsigned long ct;
  byte readError = 0;
  int readTry;

//  print_state(F("entering Read_Memory()"));

  // see also http://www.marcelklug.de/2015/04/15/freestyle-libre-ein-offenes-buch/
  // 15 min trend values in 8 byte blocks #3 - #15
  for ( int b = 3; b < 16; b++) {
    readTry = 0;
    do {
      readError = 0;
      // read single block
      digitalWrite(SSPin, LOW);
      SPI.transfer(0x00);  // SPI control byte to send command to CR95HF
      SPI.transfer(0x04);  // Send Receive CR95HF command
      SPI.transfer(0x03);  // length of data that follows
      SPI.transfer(0x02);  // request Flags byte
      SPI.transfer(0x20);  // Read Single Block command for ISO/IEC 15693
      SPI.transfer(b);     // memory block address
      digitalWrite(SSPin, HIGH);
      delay(1);
  
      // wait for BM019 answer, test for possible timeout
      ct = millis();
      digitalWrite(SSPin, LOW);
      while (RXBuffer[0] != 8) {
        RXBuffer[0] = SPI.transfer(0x03);  // Write 3 until
        RXBuffer[0] = RXBuffer[0] & 0x08;  // bit 3 is set
        if ( (millis() - ct) > 1000) {
          print_state(F("poll failure in Read_Memory()"));
          break;
        }
      }
      digitalWrite(SSPin, HIGH);
      delay(1);
  
      // read the received data block
      digitalWrite(SSPin, LOW);
      SPI.transfer(0x02);             // SPI control byte for read
      RXBuffer[0] = SPI.transfer(0);  // response code
      RXBuffer[1] = SPI.transfer(0);  // length of data
      for (byte i = 0; i < RXBuffer[1]; i++)
        RXBuffer[i + 2] = SPI.transfer(0); // data
      if ( RXBuffer[0] != 128 )
        readError = 1;
      digitalWrite(SSPin, HIGH);
      delay(1);
  
      // copy to convert
      for (int i = 0; i < 8; i++)
        oneBlock[i] = RXBuffer[i + 3];
  
      char str[24];
      unsigned char * pin = oneBlock;
      const char * hex = "0123456789ABCDEF";
      char * pout = str;
      for (; pin < oneBlock + 8; pout += 2, pin++) {
        pout[0] = hex[(*pin >> 4) & 0xF];
        pout[1] = hex[ *pin     & 0xF];
      }
      pout[0] = 0;
  
      if ( !readError )
      {
        // trendValues contains all data in a row
        trendValues += str;
        // VW - debug
#ifdef DB_NFC_DATA
        print_state(F("")); Serial.print(str);
#endif
      }
      readTry++;
    } while ( (readError) && (readTry < MAX_NFC_READTRIES) );
  } /* read memory blocks 3 ... 15 */

//  print_state(F("trendValues filled"));

  /* *********************************************************************************** */

  // decoding sensor minutes elapsed from block #39
  readTry = 0;
  do {
    readError = 0;
    digitalWrite(SSPin, LOW);
    SPI.transfer(0x00);                    // SPI control byte to send command to CR95HF
    SPI.transfer(0x04);                    // Send Receive CR95HF command
    SPI.transfer(0x03);                    // length of data that follows
    SPI.transfer(0x02);                    // request Flags byte
    SPI.transfer(0x20);                    // Read Single Block command for ISO/IEC 15693
    SPI.transfer(39);                      // memory block address
    digitalWrite(SSPin, HIGH);
    delay(1);
  
    digitalWrite(SSPin, LOW);
    while (RXBuffer[0] != 8) {
      RXBuffer[0] = SPI.transfer(0x03);   // Write 3 until
      RXBuffer[0] = RXBuffer[0] & 0x08;   // bit 3 is set
    }
    digitalWrite(SSPin, HIGH);
    delay(1);
  
    digitalWrite(SSPin, LOW);
    SPI.transfer(0x02);                   // SPI control byte for read
    RXBuffer[0] = SPI.transfer(0);        // response code
    RXBuffer[1] = SPI.transfer(0);        // length of data
    for (byte i = 0; i < RXBuffer[1]; i++)
      RXBuffer[i + 2] = SPI.transfer(0);  // data
    if ( RXBuffer[0] != 128 )
      readError = 1;
    digitalWrite(SSPin, HIGH);
    delay(1);
  
    for (int i = 0; i < 8; i++)
      oneBlock[i] = RXBuffer[i + 3];
  
    char str[24];
    unsigned char * pin = oneBlock;
    const char * hex = "0123456789ABCDEF";
    char * pout = str;
    for (; pin < oneBlock + 8; pout += 2, pin++) {
      pout[0] = hex[(*pin >> 4) & 0xF];
      pout[1] = hex[ *pin     & 0xF];
    }
    pout[0] = 0;
  
  #ifdef DB_NFC_DATA
    print_state(F("raw data minutes elapsed ")); Serial.println(str);
  #endif
    if ( !readError )  
      elapsedMinutes += str;
    readTry++;
  } while ( (readError) && (readTry < MAX_NFC_READTRIES) );
  
  // is response code good?
  if ( !readError ) {
    hexMinutes = elapsedMinutes.substring(10, 12) + elapsedMinutes.substring(8, 10);
    hexPointer = trendValues.substring(4, 6);
    sensorMinutesElapse = strtoul(hexMinutes.c_str(), NULL, 16);
    glucosePointer = strtoul(hexPointer.c_str(), NULL, 16);

#ifdef DB_NFC_DATA
    print_state(F("min elapsed: ")); Serial.print(sensorMinutesElapse);
    Serial.print(F(", gluPtr: ")); Serial.print(glucosePointer);
#endif
    int ii = 0;
    // read all 6 bytes BG reading blocks, one block is 12 ASCII
    for ( int i = 8 ; i <= 200 ; i += 12 ) {
      if (glucosePointer == ii) {
        if (glucosePointer == 0) {
          String trendNow = trendValues.substring(190, 192) + trendValues.substring(188, 190);
          String trendOne = trendValues.substring(178, 180) + trendValues.substring(176, 178);
          String trendTwo = trendValues.substring(166, 168) + trendValues.substring(164, 166);
          currentGlucose = Glucose_Reading(strtoul(trendNow.c_str(), NULL , 16));
          trendOneGlucose = Glucose_Reading(strtoul(trendOne.c_str(), NULL , 16));
          trendTwoGlucose = Glucose_Reading(strtoul(trendTwo.c_str(), NULL , 16));
          print_state(F("sensor trend = ")); Serial.print(currentGlucose);
#ifdef DB_NFC_DATA1
          print_state(F("glucosepointer = 0: "));
          Serial.print(trendNow); Serial.print(F(" "));
          Serial.print(trendOne); Serial.print(F(" "));
          Serial.print(trendTwo); Serial.print(F(" "));
          Serial.print(currentGlucose); Serial.print(F(" "));
          Serial.print(trendOneGlucose); Serial.print(F(" "));
          Serial.print(trendTwoGlucose); Serial.print(F(" "));
#endif
          if (FirstRun == 1)
            lastGlucose = currentGlucose;
          if (((lastGlucose - currentGlucose) > SPIKE_RANGE) || ((currentGlucose - lastGlucose) > SPIKE_RANGE)) {
              spikeCount++;
            print_state(F("spike detected: CG ")); Serial.print(currentGlucose); 
            Serial.print(F(", LG ")); Serial.print(lastGlucose); 
            if (((lastGlucose - trendOneGlucose) > SPIKE_RANGE) || ((trendOneGlucose - lastGlucose) > SPIKE_RANGE))
              currentGlucose = trendTwoGlucose;
            else
              currentGlucose = trendOneGlucose;
          }
        }
        else if (glucosePointer == 1)
        {
          String trendNow = trendValues.substring(i - 10, i - 8) + trendValues.substring(i - 12, i - 10);
          String trendOne = trendValues.substring(190, 192) + trendValues.substring(188, 190);
          String trendTwo = trendValues.substring(178, 180) + trendValues.substring(176, 178);
          currentGlucose = Glucose_Reading(strtoul(trendNow.c_str(), NULL , 16));
          trendOneGlucose = Glucose_Reading(strtoul(trendOne.c_str(), NULL , 16));
          trendTwoGlucose = Glucose_Reading(strtoul(trendTwo.c_str(), NULL , 16));
          print_state(F("sensor trend = ")); Serial.print(currentGlucose);
#ifdef DB_NFC_DATA1
          print_state(F("glucosepointer = 1: "));
          Serial.print(trendNow); Serial.print(F(" "));
          Serial.print(trendOne); Serial.print(F(" "));
          Serial.print(trendTwo); Serial.print(F(" "));
          Serial.print(currentGlucose); Serial.print(F(" "));
          Serial.print(trendOneGlucose); Serial.print(F(" "));
          Serial.print(trendTwoGlucose); Serial.print(F(" "));
#endif
          if (FirstRun == 1)
            lastGlucose = currentGlucose;
          if (((lastGlucose - currentGlucose) > SPIKE_RANGE) || ((currentGlucose - lastGlucose) > SPIKE_RANGE)) {
              spikeCount++;
            print_state(F("spike detected: CG ")); Serial.print(currentGlucose); 
            Serial.print(F(", LG ")); Serial.print(lastGlucose); 
            if (((lastGlucose - trendOneGlucose) > SPIKE_RANGE) || ((trendOneGlucose - lastGlucose) > SPIKE_RANGE))
              currentGlucose = trendTwoGlucose;
            else
              currentGlucose = trendOneGlucose;
          }
        }
        else
        {
          String trendNow = trendValues.substring(i - 10, i - 8) + trendValues.substring(i - 12, i - 10);
          String trendOne = trendValues.substring(i - 22, i - 20) + trendValues.substring(i - 24, i - 22);
          String trendTwo = trendValues.substring(i - 34, i - 32) + trendValues.substring(i - 36, i - 34);
          currentGlucose = Glucose_Reading(strtoul(trendNow.c_str(), NULL , 16));
          trendOneGlucose = Glucose_Reading(strtoul(trendOne.c_str(), NULL , 16));
          trendTwoGlucose = Glucose_Reading(strtoul(trendTwo.c_str(), NULL , 16));
          print_state(F("sensor trend = ")); Serial.print(currentGlucose);
#ifdef DB_NFC_DATA
          print_state(F("glucosepointer > 1: "));
          Serial.print(trendNow); Serial.print(F(" "));
          Serial.print(trendOne); Serial.print(F(" "));
          Serial.print(trendTwo); Serial.print(F(" "));
          Serial.print(currentGlucose); Serial.print(F(" "));
          Serial.print(trendOneGlucose); Serial.print(F(" "));
          Serial.print(trendTwoGlucose); Serial.print(F(" "));
          int k;
          Serial.print(F("{(i=")); Serial.print(i); Serial.print(F(",ii="));
          Serial.print(ii); Serial.print(F(")"));
          for ( k = i - 12 ; k < i - 8 ; k++ )
            Serial.print(trendValues.substring(k, k + 1));
          Serial.print(F(" - "));
          for ( k = i - 24 ; k < i - 20 ; k++ )
            Serial.print(trendValues.substring(k, k + 1));
          Serial.print(F(" - "));
          for ( k = i - 36 ; k < i - 32 ; k++ )
            Serial.print(trendValues.substring(k, k + 1));
          Serial.print(F("}"));
#endif
          if (FirstRun == 1)
            lastGlucose = currentGlucose;
          if (((lastGlucose - currentGlucose) > SPIKE_RANGE) || ((currentGlucose - lastGlucose) > SPIKE_RANGE)) {
              spikeCount++;
            print_state(F("spike detected: CG ")); Serial.print(currentGlucose); 
            Serial.print(F(", LG ")); Serial.print(lastGlucose); 
            if (((lastGlucose - trendOneGlucose) > SPIKE_RANGE) || ((trendOneGlucose - lastGlucose) > SPIKE_RANGE))
              currentGlucose = trendTwoGlucose;
            else
              currentGlucose = trendOneGlucose;
          }
        }
      } /* if (glucosePointer == ii) */
      ii++;
    } /* for i ... */

    /* ***************************************************************************** */

    // get 15min trend data
#ifdef DB_NFC_DATA
     print_state(F("trend[] : "));
#endif
    for (int i = 8, j = 0; i < 200; i += 12, j++) {
      String t = trendValues.substring(i + 2, i + 4) + trendValues.substring(i, i + 2);
      trend[j] = Glucose_Reading(strtoul(t.c_str(), NULL , 16));
#ifdef DB_NFC_DATA
      Serial.print(j); Serial.print(F(" "));Serial.print(trend[j]);Serial.print(F(" "));
#endif
    }

    // calculate 15min trend average value
    validTrendCounter = 0;
    for (int i = 0; i < 16; i++) {
      if (((lastGlucose - trend[i]) > SPIKE_RANGE) || ((trend[i] - lastGlucose) > SPIKE_RANGE)) { // invalid trend check
        trendSpikeCount++;
        continue;
      }
      else {
        validTrend[validTrendCounter] = trend[i];
        validTrendCounter++;
      }
    }

    if (validTrendCounter > 0)
    {
      // average value
      averageGlucose = 0.0;
      for (int i = 0; i < validTrendCounter; i++)
        averageGlucose += validTrend[i];
      averageGlucose = averageGlucose / validTrendCounter;

      if (((lastGlucose - currentGlucose) > SPIKE_RANGE) || ((currentGlucose - lastGlucose) > SPIKE_RANGE)) {
        averageSpikeCount++;
        shownGlucose = averageGlucose; // If currentGlucose is still invalid take the average value
      }
      else
        shownGlucose = currentGlucose; // All went well. Take and show the current value
    }
    else
      shownGlucose = currentGlucose; // If all is going wrong, nevertheless take and show a value

    if ( lastGlucose == currentGlucose ) // faulty sensor check, can happen before end of lifetime (@NFCole)
      noDiffCount++;

    if (lastGlucose != currentGlucose) // Reset the counter
      noDiffCount = 0;

    if (currentGlucose != 0)
      lastGlucose = currentGlucose;

    /* *********************************************************************** */

    NFCReady = 2;
    FirstRun = 0;

    if ( shownGlucose != currentGlucose ) {
      print_state(F("shownGlucose!=currentGlucose"));
      glucoseWasChanged++;
    }

#ifndef REMOVE_SPIKES
    print_state(F("no LimiTTer spike removing: "));
    Serial.print(shownGlucose);    Serial.print(F(" / ")); Serial.print(currentGlucose);
    shownGlucose = currentGlucose;
#endif

#ifdef SIMU_BG
    shownGlucose += simuBGindex;
    if ( (simuBGindex += 10) > 100 )
      simuBGindex = 0;
#endif

/*
    if ( ((shownGlucose/18.0182) < 2.5) || ((shownGlucose/18.0182) > 20.0) )
      shownGlucose = 0;
*/
    print_state(F("BG reading ["));
    Serial.print(shownGlucose);
    Serial.print(F("], BatLev: "));
    Serial.print(batteryPcnt);
    Serial.print(F("%, "));
    Serial.print(F("BatMv: "));
    Serial.print(batteryMv);
    Serial.print(F("mV, "));
    Serial.print(F("SensLife: "));
    Serial.print(sensorMinutesElapse);
    Serial.print(F(" min elapsed"));

    /*
        print_state(F("15 minutes-trend: "));
        for (int i = 0; i < 16; i++) {
          Serial.print(trend[i]);
          Serial.print(F(" "));
          if ( i == 6 )
            print_state(F(""));
        }
    */

#ifdef XBEXT
    int i;
    qpkt1.size = 19;
    qpkt1.cmd_code = 0x02;
    qpkt1.sub_code = 0x04;
    for ( i = 0 ; i < 8 ; i++ )
      qpkt1.trend[i] = trend[i];
    qpkt2.size = 19;
    qpkt2.cmd_code = 0x02;
    qpkt2.sub_code = 0x05;
    for ( i = 8 ; i < 16 ; i++ )
      qpkt2.trend[i - 8] = trend[i];
#endif /* XBEXT */

#ifdef USE_DEAD_SENSOR
    // we do support tests with expired sensors!
    noDiffCount = 0;
#endif

// print_state(F("mem avail "));
// Serial.print(freeMemory());

    // hard cut - after 14 days and 12 h dont send any value
#ifndef USE_DEAD_SENSOR
    if ( sensorMinutesElapse >= STOP_SEND_READINGS ) {
      print_state(F("max sensor livetime of ")); Serial.print(STOP_SEND_READINGS);
      Serial.print(F(" mins reached: "));   Serial.print(sensorMinutesElapse); 
      Serial.print(F(" elapsed, stop sending BG readings"));
      *readDone = 0;
      return(0);
    }
#endif

    if (noDiffCount > 5) {
      print_state(F("five times identical reading - this sensor is dead"));  
      *readDone = 0;
      return (0);
    }
    else {
      *readDone = 1;
      return (shownGlucose);
    }
  } /* if (RXBuffer[0] == 128) */
  else {
    Serial.print(F(" - ************** Read Memory Block Command FAIL *****************"));
    NFCReady = 0;
    readError = 0;
    nfc_read_mem_errors++;
    *readDone = 0;
    return (0);
  }
  // should never be executed
  *readDone = 0;
  return (0); // return added to avoid compiler warning
}

float Glucose_Reading(unsigned int val) {
  int bitmask = 0x0FFF;
  return ((val & bitmask) / 8.5);
}

/* ******************************************* */
/* *** Vcc and sleep ************************* */
/* ******************************************* */

int readVcc() {
  // Read 1.1V reference against AVcc
  // set the reference to Vcc and the measurement to the internal 1.1V reference
#if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
  ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
#elif defined (__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
  ADMUX = _BV(MUX5) | _BV(MUX0);
#elif defined (__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
  ADMUX = _BV(MUX3) | _BV(MUX2);
#else
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
#endif

  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Start conversion
  while (bit_is_set(ADCSRA, ADSC)); // measuring

  uint8_t low  = ADCL; // must read ADCL first - it then locks ADCH
  uint8_t high = ADCH; // unlocks both

  batteryMv = (high << 8) | low;

  batteryMv = 1125300L / batteryMv; // Calculate Vcc (in mV); 1125300 = 1.1*1023*1000
  int batteryLevel = min(map(batteryMv, MIN_V, MAX_V, 0, 100), 100); // Convert voltage to percentage
  return batteryLevel;
}

void goToSleep(int time) {
  int eights, ones;

  arduino_ontime += (millis() - arduino_start_time);
  
  // say how long we want to sleep in absolute
  //  print_state(F("go to sleep for ")); Serial.print(time); Serial.print(F(" calibrated ticks ("));
  // calculate waiting time for units in 8s and units in 1 s
  eights = time / 8;
  ones = time % 8;
  //  Serial.print(eights); Serial.print(F(" x 8 s, ")); Serial.print(ones); Serial.print(F(" x 1 s) - "));
  Serial.print(F("\r\n"));
  waitDoingServices(300, 1);      // empty serial line, decode OK+LOST including error outout
  // wait in the 8 s cycle
  for (int i = 0; i < eights; i++) {
    Serial.print(F("~"));
    delay(5),
    MCUSR = 0;
    WDTCSR |= 0b00011000;
    WDTCSR =  0b01000000 | WDT_8S_MASK;
    set_sleep_mode (SLEEP_MODE_PWR_DOWN);
    sleep_enable();
    sleep_cpu();
  }
  // wait in the 1 s cycle
  //  Serial.print(F(" - the ones"));
  //  delay(15); // give time to print the serial chars
  for (int i = 0; i < ones; i++) {
   Serial.print(F("."));
    delay(5);
    MCUSR = 0;
    WDTCSR |= 0b00011000;
    WDTCSR =  0b01000000 | WDT_1S_MASK;
    set_sleep_mode (SLEEP_MODE_PWR_DOWN);
    sleep_enable();
    sleep_cpu();
  }
}
#ifndef AUTOCAL_WDT
ISR(WDT_vect)
{
  wdt_disable();
}
#endif

void lowBatterySleep() {

  //  shutNFCDown();
  shutBLEDown(0);

  print_state(F("Battery low! LEVEL: "));
  Serial.print(batteryPcnt);
  Serial.print(F("%"));
  waitDoingServices(100, 1);

  // Switch LED on and then off shortly
  for (int i = 0; i < 10; i++) {
    digitalWrite(SCKPin, HIGH);
    delay(50);
    digitalWrite(SCKPin, LOW);
    delay(100);
  }

  MCUSR = 0;
  WDTCSR |= 0b00011000;
  WDTCSR =  0b01000000 | 0b100001;
  set_sleep_mode (SLEEP_MODE_PWR_DOWN);
  sleep_enable();
  sleep_cpu();
  sleep_disable();
  power_all_enable();
  wdt_reset();
}

void wakeUp(void)
{
  sleep_disable();
  power_all_enable();
  wdt_reset();
}

// LimiTTer code to save battery
void check_battery(void)
{
  // LimiTTer battery OK ?
  batteryPcnt = readVcc();
  if (batteryPcnt < 1)
    batteryLow = 1;
  while (batteryLow == 1)
  {
    lowBatterySleep();
    batteryPcnt = readVcc();
    if (batteryPcnt > 10)
    {
      batteryLow = 0;
      wakeUp();
      //      wakeBLEUp();
      waitDoingServices(250, 1);
    }
  }
}

/* ************************************************ */
/* *** calibration of WDT oscillator *** */
/* *** see http://forum.arduino.cc/index.php/topic,38046.0.html *** */
/* *** and http://forum.arduino.cc/index.php?topic=49549.0 ***/
/* ************************************************ */

#ifdef AUTOCAL_WDT

unsigned long timeSleep = 0;  // total time due to sleep
//float calibv = 0.93; // ratio of real clock with WDT clock
volatile byte isrcalled = 0;  // WDT vector flag

// Internal function: Start watchdog timer
// byte psVal - Prescale mask
void WDT_On (byte psVal)
{
  // prepare timed sequence first
  byte ps = (psVal | (1 << WDIE)) & ~(1 << WDE);
  cli();
  wdt_reset();
  /* Clear WDRF in MCUSR */
  MCUSR &= ~(1 << WDRF);
  // start timed sequence
  WDTCSR |= (1 << WDCE) | (1 << WDE);
  // set new watchdog timeout value
  WDTCSR = ps;
  sei();
}

// Internal function.  Stop watchdog timer
void WDT_Off() {
  cli();
  wdt_reset();
  /* Clear WDRF in MCUSR */
  MCUSR &= ~(1 << WDRF);
  /* Write logical one to WDCE and WDE */
  /* Keep old prescaler setting to prevent unintentional time-out */
  WDTCSR |= (1 << WDCE) | (1 << WDE);
  /* Turn off WDT */
  WDTCSR = 0x00;
  sei();
}

// Calibrate watchdog timer with millis() timer(timer0)
void calibrate(long caltime) {
  // timer0 continues to run in idle sleep mode
  set_sleep_mode(SLEEP_MODE_IDLE);
  unsigned long tt1 = millis();
  doSleep(caltime);
  unsigned long tt2 = millis();
  calibv = ((float)caltime) / (tt2 - tt1);

  calibv -= 0.03;         // 3% downside correction, needed to be accurate
  
  print_state(F("calibv = ")); Serial.print(calibv * 1000);
  Serial.print(F(", tt1 = ")); Serial.print(tt1);
  Serial.print(F(", tt2 = ")); Serial.print(tt2);
}

// Estimated millis is real clock + calibrated sleep time
long estMillis() {
  return millis() + timeSleep;
}

// Delay function
void sleepCPU_delay(unsigned long sleepTime) {
  ADCSRA &= ~(1 << ADEN); // adc off
  PRR = 0xEF; // modules off

  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  int trem = doSleep(sleepTime * calibv);
  timeSleep += (sleepTime - trem);

  PRR = 0x00; //modules on
  ADCSRA |= (1 << ADEN); // adc on
}

// internal function.
int doSleep(long timeRem) {
  byte WDTps = 9;  // WDT Prescaler value, 9 = 8192ms

  isrcalled = 0;
  sleep_enable();
  while (timeRem > 0) {
    //work out next prescale unit to use
    while ((0x10 << WDTps) > timeRem && WDTps > 0) {
      WDTps--;
    }
    // send prescaler mask to WDT_On
    WDT_On((WDTps & 0x08 ? (1 << WDP3) : 0x00) | (WDTps & 0x07));
    isrcalled = 0;
    while (isrcalled == 0) {
      // turn bod off
      MCUCR |= (1 << BODS) | (1 << BODSE);
      MCUCR &= ~(1 << BODSE); // must be done right before sleep
      sleep_cpu();  // sleep here
    }
    // calculate remaining time
    timeRem -= (0x10 << WDTps);
  }
  sleep_disable();
  return timeRem;
}

// wdt int service routine
ISR(WDT_vect) {
  WDT_Off();
  isrcalled = 1;
}

#endif /* AUTOCAL_WDT */

/* ************************************************ */
/* *** BLE handling ******************************* */
/* ************************************************ */

#ifdef ATFAIL_REBOOT
void(* resetFunc) (void) = 0;  // declare reset fuction at address 0
#endif

// init HM-1x module
int setupBLE()
{
  int i;

  //detect HM-1x baudrate, if not currently set
  print_state(F("setup HM-1x, determining baudrate"));

  for ( i = 0 ; i <= 8 ; i++ ) {
    init_command_buff(&command_buff);
    print_state(F("trying ")); Serial.print(uart_baudrate[i]);
    settings.uart_baudrate = uart_baudrate[i];
    ble_Serial.begin(uart_baudrate[i]);
    send_string(F("AT"), 500);
    if ( waitDoingServicesInterruptible(500, &got_ok, 1) )
      break;
  }

  if (!got_ok) {
    print_state(F("Could not detect baudrate of HM-1x, setting 9600"));
    settings.uart_baudrate = 9600;
    resetBLE();
    return (0);
  }
  else
    got_ok = 0;

  print_state(F("baudrate set to "));
  Serial.print(settings.uart_baudrate);
  init_command_buff(&command_buff);
  return (1);
}

// Configure the BlueTooth module with a name.
// from LimiTTer setp() moved to here
void configBLE(boolean do_renew) {
  String name;

  send_string(F("AT"), 500);            // cut BLE connection to do commands

  if ( do_renew ) {
    send_ble_string(F("AT+RENEW"), 0);    // restore factory defaults
    waitDoingServices(3000, 1);           // time to settle
  }

  // notify CONNECT and LOST
  send_string(F("AT"), 500);            // cut BLE connection to do commands
  if ( send_ble_string(F("AT+NOTI1"), 0) ) {
    if ( strstr(at_answer, "+Set:1") != NULL ) {
      Serial.print(F(" AT command successfull"));
    }
    else
#ifdef ATFAIL_REBOOT
    {
      print_state(F("AT+ command not processed, we have to reset the Arduino ..."));
      waitDoingServices(2000, 1);
      resetFunc();
    }
#else
      resetBLE();
#endif
  }
  else {
    print_state(F("command not succesfull - do a reset"));
    resetFunc();
  }
  send_string(F("AT"), 500);            // cut BLE connection to do commands
  name = (String)"AT+NAME";
  name += (String)F(LNAME);
  send_ble_string(name, 0);             // set unique LimiTTer name, max 11 (not 12) chars

  send_string(F("AT"), 500);            // cut BLE connection to do commands
  send_ble_string(F("AT+VERR?"), 0);    // look for correct BLE module - answer schould be "HMSoft V54x"
  // to detect fake modules which starts with 115200
  send_string(F("AT"), 500);            // cut BLE connection to do commands
  send_ble_string(F("AT+ADDR?"), 0);    // get MAC address

  send_string(F("AT"), 500);            // cut BLE connection to do commands
  send_ble_string(F("AT+RESET"), 0);    // reset the module
  waitDoingServices(500, 1);
}

void resetBLE(void)
{
  print_state(F("reset BLE, power down - 1000 ms - power up - 1000 ms"));

  ble_connected = 0;

  digitalWrite(5, LOW); // Disable this for Android 4
  digitalWrite(6, LOW); // Disable this for Android 4
  digitalWrite(BLEPin, LOW); // Disable this for Android 4

  waitDoingServices(1000, 1);

  digitalWrite(BLEPin, HIGH);
  digitalWrite(5, HIGH);
  digitalWrite(6, HIGH);

  waitDoingServices(1000, 1);
  send_string(F("AT"), 500);            // cut BLE connection to do commands
  send_string(F("AT+RESET"), 500);      // try to reset the module
  waitDoingServices(1000, 1);

}

int wakeBLEUp(void)
{
  String name;
  int wait_time;

  if ( cons_loop_wo_ble > 1 && cons_loop_wo_ble <= 18 ) {
//#ifdef DB_PROCESSING
    print_state(F("")); Serial.print(cons_loop_wo_ble); Serial.print(F(" loops without BLE, wait 60 s for connect"));
//#endif
    wait_time = 60;
  }
  else {
    wait_time = 40;
  }

  print_state(F(" ....... wake BLE up - wait ")); Serial.print(wait_time); Serial.print(F(" s for BLE connection - "));

  ble_start_time = millis();
  ble_connected = 0;

  digitalWrite(BLEPin, HIGH);
  digitalWrite(5, HIGH);
  digitalWrite(6, HIGH);

  // wait 1 s for initial connect, time for a fast BLE connect
  waitDoingServicesInterruptible(1000, &ble_connected, 1);

  // if HM17 is puzzled do a factory reset after 30 min without valid BLE connection, 3 loops / 5 min
#ifdef SAFETY_RENEW
  if ( cons_loop_wo_ble == (3*6) ) {
    configBLE(1);
  }
#endif
  
#ifdef ATRESET
  if ( !ble_connected ) {
    send_ble_string(F("AT+RESET"), 0);  // reset the BLE module
    waitDoingServices(500, 1);
  }
#endif

  int i;
  for ( i = 0 ; i < wait_time * 2 ; i++) {
    waitDoingServices(500, 1);
    // typical CONN/LOST time difference is 1 - 2 s, mostly happens at the beginning of loop
    // wait for this 2 s before break
    if ( ble_connected ) {
      print_state(F("connected, wait 1 s to manage CONN+LOST cyle then send packet(s)"));
      // LOST on CONN mostly after 800 - 900 ms
      waitDoingServices(1000, 1);
      break;
    }
  }

  // must be connected here to start communication
  if ( !ble_connected ) {
    ble_connect_errors++;
    return (0);
  }
  else
    return (1);
}

void shutBLEDown(boolean quit_ble)
{
  // cut BLE conection via AT command
  //  if ( quit_ble && ble_connected ) {
  // for connected and lost state - we want to see HM-11 answer
  if ( quit_ble ) {
#ifdef DB_PROCESSING
    print_state(F("disconnect BLE"));
#endif
    ble_lost_processing = 0;
    send_string(F("AT"), 500);
    waitDoingServices(500, 1);
    ble_lost_processing = 1;
  }
  digitalWrite(5, LOW); // Disable this for Android 4
  digitalWrite(6, LOW); // Disable this for Android 4
  digitalWrite(BLEPin, LOW); // Disable this for Android 4

  ble_ontime += (millis() - ble_start_time);    // how long was BLE switched on?

  ble_connected = 0;
#ifdef HW_BLE
  hwble_connected = 0;
#endif
}

/* ************************************************** */
/* *** NFC handling *** */
/* ************************************************** */

// new function to fit Limitter in xBridge concept, called from loop(), formerly in setup()
void configNFC(void)
{
  SPI.begin();

  SPI.setDataMode(SPI_MODE0);
  SPI.setBitOrder(MSBFIRST);
  SPI.setClockDivider(SPI_CLOCK_DIV32);

  delay(10);                      // send a wake up
  digitalWrite(IRQPin, LOW);      // pulse to put the
  delayMicroseconds(100);         // BM019 into SPI
  digitalWrite(IRQPin, HIGH);     // mode
  delay(10);
  digitalWrite(IRQPin, LOW);

  SPI.end();    // according to Bert Roode 170227. Nested SPI.begin() and SPI.end()
}

void wakeNFCUp(void)
{
  // print_state(F("mem avail "));
  // Serial.print(freeMemory());

  digitalWrite(NFCPin1, HIGH);
  digitalWrite(NFCPin2, HIGH);
  digitalWrite(NFCPin3, HIGH);
  digitalWrite(IRQPin, HIGH);

  SPI.begin();

  SPI.setDataMode(SPI_MODE0);
  SPI.setBitOrder(MSBFIRST);
  SPI.setClockDivider(SPI_CLOCK_DIV32);
  delay(10);
  digitalWrite(IRQPin, LOW);
  delayMicroseconds(100);
  digitalWrite(IRQPin, HIGH);
  delay(10);
  digitalWrite(IRQPin, LOW);

  NFCReady = 0;

  nfc_start_time = millis();
}

void shutNFCDown(void)
{
  // print_state(F("mem avail "));
  // Serial.print(freeMemory());

  SPI.end();

  digitalWrite(MOSIPin, LOW);
  digitalWrite(SCKPin, LOW);
  digitalWrite(NFCPin1, LOW); // Turn off all power sources completely
  digitalWrite(NFCPin2, LOW); // for maximum power save on BM019.
  digitalWrite(NFCPin3, LOW);
  digitalWrite(IRQPin, LOW);

  nfc_ontime += (millis() - nfc_start_time);
}

/* ****************************************************************** */
/* port of xBridge2 code */
/* ***************************************************************** */

// send data to BLE
void send_data(unsigned char *msg, unsigned char len)
{
  unsigned char i = 0;

  last_ble_send_time = millis();
  crlf_printed = 0;

  for ( i = 0; i < len; i++ )
    ble_Serial.write(msg[i]);

  // wait up to 40 chars (@9600)
  delay(40);
  print_state(F("sending: <"));
  for ( i = 0 ; i < len ; i++ ) {
    Serial.print(msg[i], HEX);
    Serial.print(F(" "));
  }
  Serial.print(F(">"));
  print_state(F("response: "));
}

// due to Arduino cast problems between String and char * use extra function
int send_ble_string(String cmd, boolean connect_state)
{
  int i, j;
  unsigned long nnow;
  boolean timeout;

  // take care of connect state?
  if ( !connect_state ) {
    for ( i = 0 ; i < 3 ; i++ ) {
      if ( show_ble ) {
        print_state(F(" ->(")); Serial.print(cmd); Serial.print(F(") - "));
      }

      last_ble_send_time = millis();  // beautyfing output
      crlf_printed = 0;               // BLE input in seperate line

      ble_Serial.print(cmd);          // send the command via BLE

      nnow = millis();
      ble_answer = 0;
      timeout = 1;

      // wait max. 2 s to get an answer
      while ( (millis() - nnow) < 2000 ) {
        waitDoingServices(30, 1);
        if ( ble_answer == 2 ) {
          // show the BLE answer received in 2 s after AT command sended
          j = 0;
          Serial.print(F("\r\n("));
          while ( at_answer[j] != 0 )
            Serial.print(at_answer[j++]);
          Serial.print(F(")"));
          ble_answer = 0;
          timeout = 0;
          break;
        }
      }
      // wait additional 500 ms to be sure for next command
      waitDoingServices(500, 1);
      if ( timeout ) {
        resetBLE();             // in case of timeout try to hard reset the HM-1x
      }
      else {
        Serial.print(F(" - "));
        return (1);
      }
    }

    // AT command not successful - do a reset
    //    resetFunc();

    return (0);
  }
  return (0);
}

void send_string(String cmd, int dly)
{
  last_ble_send_time = millis();
  crlf_printed = 0;
  ble_Serial.print(cmd);
  if ( show_ble ) {
    print_state(F(" ->("));
    Serial.print(cmd);
    Serial.print(F(") - "));
  }
  waitDoingServices(dly, 1);
}

// send a beacon with the TXID
void sendBeacon(void)
{
  //char array to store the response in.
  unsigned char cmd_response[7];
  //return if we don't have a connection or if we have already sent a beacon
  //prepare the response
  //responding with number of bytes,
  cmd_response[0] = sizeof(cmd_response);
  //responding to command 01,
  cmd_response[1] = 0xF1;
  //return the encoded TXID
  memcpy(&cmd_response[2], &settings.dex_tx_id, sizeof(settings.dex_tx_id));
  cmd_response[6] = DEXBRIDGE_PROTO_LEVEL;
  send_data(cmd_response, sizeof(cmd_response));
}

int init_command_buff(t_command_buff* pCmd)
{
  if (!pCmd)
    return 0;
  memset(pCmd->commandBuffer, 0, COMMAND_MAXLEN);
  pCmd->nCurReadPos = 0;
  return 0;
}

//decode a command received ??
int commandBuffIs(const char* command)
{
  unsigned char len = strlen(command);
  if (len != command_buff.nCurReadPos)
    return (0);
  return ( memcmp(command, command_buff.commandBuffer, len) == 0 );
}

// decode incoming serial or BLE data commands
int doCommand(boolean is_serial_char)
{
  // TXID packet?
  if (command_buff.commandBuffer[1] == 0x01 && command_buff.commandBuffer[0] == 0x06)
  {
    memcpy(&settings.dex_tx_id, &command_buff.commandBuffer[2], sizeof(settings.dex_tx_id));
    got_txid = 1;
    print_state(F("got a TXID packet, new TXID is - "));
    Serial.print(settings.dex_tx_id);
    // send back the TXID we think we got in response
    return (0);
  }
  // ACK packet?
  if (command_buff.commandBuffer[0] == 0x02 && command_buff.commandBuffer[1] == 0xF0 && !got_ack) {
    got_ack = 1;
    init_command_buff(&command_buff);
    return (0);
  }
#ifdef XBEXT
  // DataRequestPacket?
  if (command_buff.commandBuffer[0] == 0x0C && command_buff.commandBuffer[1] == 0x02 ) {
    print_state(F("DataRequestPacket received"));
    got_drp = 1;
    init_command_buff(&command_buff);
    return (0);
  }
#endif
  if ( is_serial_char && (toupper(command_buff.commandBuffer[0]) == 'R') ) {
    print_state(F("R command - send AT+RENEW to HM-1x then do a reset"));
    send_ble_string(F("AT+RENEW"), 0);    // restore factory defaults
    waitDoingServices(1000, 1);            // time to settle
    print_state(F("done, do a RESET in 3 s"));
    waitDoingServices(3000, 1);            // time to settle
    resetFunc();
  }
#ifdef UPDATE_HM1X
  if ( is_serial_char && (toupper(command_buff.commandBuffer[0]) == 'U') ) {
    print_state(F("U command - bridge serial port to HM-1x to upload firmware"));
    print_state(F("entering bridge mode, please reboot LBridge once update is ready"));
    waitDoingServices(1000, 1);            // time to settle
    while ( 1 ) {
      while ( Serial.available() ) {
        ble_Serial.write(Serial.read());
      }
      while ( ble_Serial.available() ) {
        Serial.write(ble_Serial.read());
      }
    }
  }
#endif
#ifdef DB_PKT
  if ( is_serial_char && (toupper(command_buff.commandBuffer[0]) == 'X') ) {
    print_state(F("X command - show queue before sleep, "));
    if ( x_command == 0 ) {
      Serial.print(F(" toggled ON"));
      x_command = 1;
    }
    else {
      Serial.print(F(" toggled OFF"));
      x_command = 0;
    }
  }
#endif
  if ( is_serial_char && (toupper(command_buff.commandBuffer[0]) == 'V') ) {
#ifdef DB_VOLTAGE
    print_state(F("V command - show voltage queue before sleep, "));
    if ( v_command == 0 ) {
      Serial.print(F(" toggled ON"));
      v_command = 1;
    }
    else {
      Serial.print(F(" toggled OFF"));
      v_command = 0;
    }
#endif /* DB_VOLTAGE */
  }
  if ( is_serial_char && (toupper(command_buff.commandBuffer[0]) == 'B') ) {
    if ( show_ble ) {
      print_state(F("B command - dont show BLE communication"));
      show_ble = 0;
    }
    else {
      print_state(F("B command - show BLE communication"));
      show_ble = 1;
    }
  }
  if ( is_serial_char && (toupper(command_buff.commandBuffer[0]) == 'S') ) {
    if ( show_state ) {
      print_state(F("S command - dont show states"));
      show_state = 0;
    }
    else {
      show_state = 1;
      print_state(F("S command - show states"));
    }
  }  // "OK+..." answer from BLE?
  String cmd2proof = F("OK");
  if ( commandBuffIs(cmd2proof.c_str()) )
  {
    got_ok = 1;
    //    print_state(F("got_ok = 1"));
    return (0);
  }
  // we don't respond to unrecognised commands.
  return (1);
}

typedef struct {
  unsigned char pattern[8];   // pattern string including '\0'
} t_pattern;

t_pattern okstr[3] = {
  { "OK+CONN" },  // HM-1x answer when BLE connection is active
  { "OK+LOST" },  // HM-1x answer when BLE connection is disconnected
  { "OK+ERRS" }   // message sent via BLE to force LBridge to send error counters via BLE
};

#ifdef HW_BLE
// detect BLE connection status via SysLED pin on HM-1x module
void bleConnectMonitor() {
  //to store the time we went high
  static unsigned long timer;
  //to store P1_2 the last time we looked.
  static boolean last_ble_check;
  boolean last_ble;

  last_ble = hwble_connected;
  // if P1_2 is high, ble_connected is low, and the last_ble_check was low, sav the time and set last_ble_check.
  if ( digitalRead(bleSysLedPin) && !hwble_connected && !last_ble_check) {
    timer = millis();
    last_ble_check = 1;
    // otherwise if P1_2 goes low, and ble_connected is high, we cancel everything.
  } else if ( !digitalRead(bleSysLedPin) ) {
    hwble_connected = 0;
    last_ble_check = 0;
    //otherwise, if P1_2 has been high for more than 550ms, we can safely assume we have ble_connected, so say so.
  } else if ( digitalRead(bleSysLedPin) && last_ble_check && ((millis() - timer) > 550)) {
    hwble_connected = 1;
  }

  // show if BLE state has changed here
  if ( last_ble != hwble_connected ) {
    print_state(F("***** HW BLE "));
    if ( hwble_connected )
      Serial.print(F("connected"));
    else
      Serial.print(F("lost"));
  }
}
#endif /* HW_BLE */

// detect BLE connection status via software / AT commands
boolean monitor_ble(unsigned char b)
{
  static int bi = 0;
  int i, c_found = 0;

  // print char in readable form
  if ( show_ble ) {
    if ( b >= ' ' && b < 128 ) {
      Serial.write(b);
    }
    else {
      //      Serial.print(F("0x"));
      Serial.print(b, HEX);
      Serial.print(F(" "));
    }
  }

  // look in all 3 patterns
  for ( i = 0 ; i < 3 ; i++) {
    if ( b == okstr[i].pattern[bi] )
      c_found |= 1;
  }

  if ( c_found )
    bi++;
  else {
    // ??? OKOK direkt nacheinander?
    if ( b == 'O' )   // OKOK... sequence?
      bi = 1;
    else
      bi = 0;
  }

  if ( bi == 7 ) { // one of pattern complete
    bi = 0;
    if ( b == okstr[0].pattern[6] ) {
      ble_connected = 1;
      cons_loop_wo_ble = 0;
      print_state(F("+++++ BLE connected after "));
      ble_connect_time = millis();
      Serial.print( millis() - ble_start_time );
      Serial.print(F(" ms"));
      return (1);
    } else if ( b == okstr[1].pattern[6] ) {
      ble_connected = 0;
      ble_connect_time = 0;
      print_state(F("+++++ BLE lost after "));
      Serial.print( millis() - ble_start_time );
      Serial.print(F(" ms"));

      // ??? test
      if ( ble_lost_processing )
        //          send_string(F("AT+RESET"), 500);

        return (1);
    } else if ( b == okstr[2].pattern[6] ) {
      // future option to send error counters via BLE
    }
  }
  return (0);
}

void print_command_buff(void)
{
  int i;
  print_state(F("command_buff is ["));
  for ( i = 0 ; i < command_buff.nCurReadPos ; i++ )
    Serial.print((char)command_buff.commandBuffer[i]);
  Serial.print(F("]"));
}

// Process any commands from BLE
int controlProtocolService()
{
  static unsigned long cmd_to;
  // ok this is where we check if there's anything happening incoming on the USB COM port or UART 0 port.
  int nRet = 1;
  int i, j;
  unsigned char b;
  boolean serial_char = 0;

  j = 0;

  //if we have timed out waiting for a command, clear the command buffer and return.
  if (command_buff.nCurReadPos > 0 && (millis() - cmd_to) > 2000)
  {
    print_state(F("got <-["));
    for ( i = 0 ; i  < command_buff.nCurReadPos ; i++ )
      if ( command_buff.commandBuffer[0] >= ' ') {
        Serial.write(command_buff.commandBuffer[i]);
      }
      else {
        Serial.print(F("0x"));
        Serial.print(command_buff.commandBuffer[i], HEX);
        Serial.print(F(" "));
      }
    Serial.print(F("]"));
    // clear command buffer if there was anything
    init_command_buff(&command_buff);
    return (nRet);
  }
  //while we have something in either buffer,
  while ( (Serial.available() || ble_Serial.available()) && command_buff.nCurReadPos < COMMAND_MAXLEN) {

    if ( ble_answer == 0 ) {
      //      Serial.print(F("-1-"));
      ble_answer = 1;
    }

    // beatifying output - wait for ACK, print CR LF after 4 s of no answer
    if ( ((millis() - last_ble_send_time) > 4000) && !crlf_printed ) {
      //        print_state(F("CRLF printed"));
      Serial.print(F("\r\n"));
      crlf_printed = 1;
    }

    if ( ble_Serial.available() ) {
      monitor_ble( b = ble_Serial.read() );
      serial_char = 0;
    }
    else {
      b = Serial.read();
      print_state(F("serial char received: <"));
      Serial.write(b);
      Serial.print(F(">"));
      serial_char = 1;
    }

    // get all char for 1 sec after AT command from BLE
    at_answer[j++] = b;

    command_buff.commandBuffer[command_buff.nCurReadPos] = b;
    command_buff.nCurReadPos++;

    cmd_to = millis();
    // if it is the end for the byte string, we need to process the command
    // valid data packet or "OK" received?
    if (      command_buff.nCurReadPos == command_buff.commandBuffer[0] \
              || (command_buff.commandBuffer[0] == 'O' && command_buff.commandBuffer[1] == 'K' ) \
              || (command_buff.nCurReadPos == 1 && toupper(command_buff.commandBuffer[0]) == 'B') \
              || (command_buff.nCurReadPos == 1 && toupper(command_buff.commandBuffer[0]) == 'R') \
              || (command_buff.nCurReadPos == 1 && toupper(command_buff.commandBuffer[0]) == 'U') \
              || (command_buff.nCurReadPos == 1 && toupper(command_buff.commandBuffer[0]) == 'V') \
              || (command_buff.nCurReadPos == 1 && toupper(command_buff.commandBuffer[0]) == 'X') \
              || (command_buff.nCurReadPos == 1 && toupper(command_buff.commandBuffer[0]) == 'S') )
    {
      // ok we got the end of a command;
      if (command_buff.nCurReadPos) {
        // do the command
        nRet = doCommand(serial_char);
        serial_char = 0;
        //re-initialise the command buffer for the next one.
        init_command_buff(&command_buff);
        // break out if we got a breaking command
        if (!nRet)
          return (nRet);
      }
    }
    serial_char = 0;

    if ( crlf_printed ) {
      crlf_printed = 0;
      last_ble_send_time = millis();
    }

    // otherwise, if the command is not up to the maximum length, add the character to the buffer.
  }

  if ( ble_answer == 1 ) {
    //    Serial.print(F("-2-"));
    ble_answer = 2;

    // copy the BLE answer into extra string
    at_answer[j++] = 0;
    /*
        j = 0;
        Serial.print("\r\n<");
        while ( at_answer[j] != 0 ) {
          Serial.print(at_answer[j++]);
        }
        Serial.print(">");
    */
  }

  if ( command_buff.nCurReadPos ) {
    //re-initialise the command buffer for the next one.
    init_command_buff(&command_buff);
  }
  return (nRet);
}

// process each of the services we need to be on top of.
// if bWithProtocol is true, also check for commands on both USB and UART
int doServices(unsigned char bWithProtocol)
{
  dex_tx_id_set = (settings.dex_tx_id != 0);
#ifdef HW_BLE
  bleConnectMonitor();
#endif
  if (bWithProtocol)
    return (controlProtocolService());
  return (1);
}

#ifdef DYNAMIC_TXID
//format an array to decode the dexcom transmitter name from a Dexcom packet source address.
char SrcNameTable[32] = { '0', '1', '2', '3', '4', '5', '6', '7',
                          '8', '9', 'A', 'B', 'C', 'D', 'E', 'F',
                          'G', 'H', 'J', 'K', 'L', 'M', 'N', 'P',
                          'Q', 'R', 'S', 'T', 'U', 'W', 'X', 'Y'
                        };

char dex_addr[6];

// convert the passed uint32 Dexcom source address into an ascii string in the passed char addr[6] array.
char *dexcom_src_to_ascii(unsigned long src)
{
  //each src value is 5 bits long, and is converted in this way.
  dex_addr[0] = SrcNameTable[(src >> 20) & 0x1F];   //the last character is the src, shifted right 20 places, ANDED with 0x1F
  dex_addr[1] = SrcNameTable[(src >> 15) & 0x1F];   //etc
  dex_addr[2] = SrcNameTable[(src >> 10) & 0x1F];   //etc
  dex_addr[3] = SrcNameTable[(src >> 5) & 0x1F];    //etc
  dex_addr[4] = SrcNameTable[(src >> 0) & 0x1F];    //etc
  dex_addr[5] = 0;  //end the string with a null character.
  return (char *)dex_addr;
}

unsigned long asciiToDexcomSrc(char addr[6])
{
  // prepare a uint32 variable for our return value
  unsigned long src = 0;
  // look up the first character, and shift it 20 bits left.
  src |= (getSrcValue(addr[0]) << 20);
  // look up the second character, and shift it 15 bits left.
  src |= (getSrcValue(addr[1]) << 15);
  // look up the third character, and shift it 10 bits left.
  src |= (getSrcValue(addr[2]) << 10);
  // look up the fourth character, and shift it 50 bits left.
  src |= (getSrcValue(addr[3]) << 5);
  // look up the fifth character
  src |= getSrcValue(addr[4]);
  //printf("asciiToDexcomSrc: val=%u, src=%u\r\n", val, src);
  return src;
}

/* getSrcValue - function to determine the encoding value of a character in a Dexcom Transmitter ID.
  Parameters:
  srcVal - The character to determine the value of
  Returns:
  uint32 - The encoding value of the character.
*/
unsigned long getSrcValue(char srcVal)
{
  unsigned char i = 0;

  for (i = 0; i < 32; i++)
  {
    if (SrcNameTable[i] == srcVal) break;
  }
  //printf("getSrcVal: %c %u\r\n",srcVal, i);
  return i & 0xFF;
}
#endif /* DYNAMIC_TXID */

// use function instead of macro, use pointer to flag, volatile boolen <var> dont work on Arduino!!!
int waitDoingServicesInterruptible(unsigned long wait_time, volatile boolean *break_flag, unsigned char bProtocolServices)
{
  unsigned long start_wait = millis();
  while ( (millis() - start_wait ) < wait_time ) {
    doServices(bProtocolServices);
    if ( *break_flag ) {
      return (1);
    }
    delay(20);
  }
  return (0);
}

void waitDoingServices(unsigned long wait_time, unsigned char bProtocolServices)
{
  unsigned long start_wait = millis();
  while ( (millis() - start_wait ) < wait_time )
  {
    doServices(bProtocolServices);
    delay(20);
  }
}

// help function to read the Freestyle Libre sensor
// in LimiTter part of loop(), moved to reuse xBridge code
int get_nfc_reading(float *ptr)
{
  boolean readSuccessful;
//  int k;

  // store the current wixel milliseconds so we know how long we are waiting.
  unsigned long start = millis();
  unsigned long ms = 3000;  // try for 3 seconds, then leave
  // set the return code to timeout indication, as it is the most likely outcome.
  float act_glucose;

  print_state(F("read the sensor"));

  // while we haven't reached the delay......
  while ( (millis() - start) < ms ) {
    if (NFCReady == 0) {
      SetProtocol_Command(); // ISO 15693 settings
      waitDoingServices(100, 1);
      continue;
    }
    else if (NFCReady == 1) {
      for (int i = 0; i < 3; i++) {
        Inventory_Command(); // sensor in range?
        nfc_inv_cmd_count++;
        if (NFCReady == 2) {
          Serial.print(F(" - sensor found"));
          break;
        }
        waitDoingServices(1000, 1);
      }
      if (NFCReady == 1) {
        // count missed readings
        nfc_read_errors++;
        print_state(F("NFC timed out"));
        return (0);
      }
    }
    else if ( NFCReady == 2 ) {
      act_glucose = Read_Memory(&readSuccessful);
/*
      for ( k = 0 ; k < NFC_READ_RETRY ; k++ ) {
        act_glucose = Read_Memory(&readSuccessful);
        if ( readSuccessful )
          break;
        print_state(F("redo Read_memory() ..."));
      }
      if ( k == NFC_READ_RETRY ) {
        print_state(F("")); Serial.print(NFC_READ_RETRY); Serial.print(F(" times missed reading in Read_Memory"));
        // we want a 1 sending in send_packet, avoid return here
        return(0);
      }
*/
      if ( readSuccessful ) {
        nfc_scan_count++;
        // only for showing package content
        *ptr = act_glucose;
        return (1);
      }
      else {
        *ptr = 0;
        return(0);  
      }
    }
  } 
  // timeout
  return (0);
}

// wait for a NFC reading and put it into Dexcom_packet
int get_packet(Dexcom_packet* pPkt)
{
  // set the return code to timeout indication, as it is the most likely outcome.
  float glucose;

  if ( get_nfc_reading(&glucose) ) {
    pPkt->raw = glucose * 1000;  // use C casting for conversion of float to unsigned long
    if ( pPkt->raw == 0 ) {
      print_state(F("successful BG readings of 0 - internal error"));
      bg_is_null++;
    }
    pPkt->ms = abs_millis();
#ifdef DB_PKT
    pPkt->sended = 0;     // indcate that a new entry is in the internal queue which was not sended yet
#endif
/*
    print_state(F("packet read at "));
    Serial.print(pPkt->ms / 1000);
    Serial.print(F(" s"));
*/
    if ( abs_pkt_time != 0 )
      last_abs_pkt_time = abs_pkt_time;
    abs_pkt_time = pPkt->ms;
    /*
        print_state(F("last packet read "));
        Serial.print((abs_millis() - last_abs_pkt_time)/1000);
        Serial.print(F(" s before"));
    */
#ifdef DB_PKT
    pPkt->retries = 0;
#endif

    return (1);
  }
  else {
    no_nfc_reading++;
    print_state(F("NFC reading not successful"));
    //    pkt_time = millis();    // to avoid endless waiting if no packet can be read
  }
  return (0);
}

//function to format and send the passed Dexom_packet.
int print_packet(Dexcom_packet* pPkt)
{
  nRawRecord msg;

  //prepare the message
  msg.size = sizeof(msg);
  msg.cmd_code = 0x00;
  msg.raw = pPkt->raw;
  msg.filtered = pPkt->raw;
  msg.dex_battery = 214;  // simulate good dexcom transmitter battery
  msg.my_battery = batteryPcnt;
  msg.dex_src_id = settings.dex_tx_id;
  msg.delay = abs_millis() - pPkt->ms;
  msg.function = DEXBRIDGE_PROTO_LEVEL; // basic functionality, data packet (with ack), TXID packet, beacon packet (also TXID ack).

#ifdef DB_PKT
  pPkt->sended = abs_millis();    // when do we send the packet really?
  print_state(F("pkt sended at ")); Serial.print((pPkt->sended+500)/1000);
  Serial.print(F(" s with a delay of ")); Serial.print(((abs_millis() - pPkt->ms)+500)/1000);
  Serial.print(F(" s"));
#endif

  if ( !ble_connected )
    return (0);
  /*
    print_state(F("sending packet with a delay of "));
    Serial.print(msg.delay/1000);
    Serial.print(F(" s"));
  */
  send_data( (unsigned char *)&msg, msg.size);

  return (1);
}

// print timestamp and current status/action
void print_state(String str)
{
  unsigned long ms = abs_millis();
  unsigned long val;

  
  Serial.print(F("\r\n["));
  Serial.print(ms / 60000);
  Serial.print(F("]["));
  if ( (val = ((ms / 1000) % 60)) < 10 )
    Serial.print(F("0"));
  Serial.print(val);
  Serial.print(F("]["));
  if ( (val = (ms % 1000)) < 10 )
    Serial.print(F("00"));
  else if ( val < 100 )
    Serial.print(F("0"));
  Serial.print(val);
  Serial.print(F("]("));
  Serial.print(freeMemory());
  Serial.print(F(") "));
  /*
    Serial.println(F(""));
    Serial.print(millis());
  */
  Serial.print(str);
}

/* ******************************************** */
/* *** statistics output ********************** */
/* ******************************************** */

void print_start_message(void)
{
  print_state(F(" ------------------------------------------------------------------"));
  print_state(F(" --- LBridge starting ---"));
#ifdef RELEASE_VERSION
  print_state(F(" --- RELEASE version settings ---"));
#endif
  print_state(F(" --- BLE Name: ")); Serial.print(F(LNAME)); Serial.print(F(" ---"));
  print_state(F(" --- Version: ")); Serial.print(F(LB_VERSION)); Serial.print(F(LB_MINOR_VERSION)); Serial.print(F(" from "));
    Serial.print(F(LB_DATETIME)); Serial.print(F(" ---"));
  print_state(F(" --- avail. mem: ")); Serial.print(freeMemory()); Serial.print(F(" ---"));
  print_state(F(" --- queue size: ")); Serial.print(DXQUEUESIZE/12); Serial.print(F(" h"));
#ifdef REMOVE_SPIKES
  print_state(F(" --- spike removal option enabled"));
#endif
#ifdef USE_DEAD_SENSOR
  print_state(F(" --- dead sensors support option enabled - caution when used in APS/loop applications!"));
#endif
#ifdef SIMU_BG
  print_state(F(" --- debug option enabled: add ramp function to BG readings"));
#endif
#ifdef HW_BLE
  print_state(F(" --- HW BLE detection"));
#endif
  print_state(F(" ------------------------------------------------------------------"));
}

void print_statistics1(void)
{
  Serial.print(F("\r\n *** loop time is ")); Serial.print(loop_time / 1000); Serial.print(F(" s, sleep for "));
  Serial.print(var_sleepTime); Serial.print(F(" s"));
  Serial.print(F(", all complete ")); Serial.print((loop_time + (var_sleepTime * 1000)) / 1000);
  Serial.print(F(" s")); Serial.print(F(", cons loops w/o BLE ")); Serial.print(cons_loop_wo_ble);
}

void print_statistics2()
{
  Serial.print(F("\r\n"));
  switch ( loop_state ) {
    case NORMAL:
      print_state(F(" === loop ")); Serial.print(++loop_count); Serial.print(F(" === "));
      Serial.print(F("NORMAL"));
      normal_loop_count++;
      break;
    case ST_RESEND:
      print_state(F(" ### loop ")); Serial.print(++loop_count); Serial.print(F(" === "));
      Serial.print(F("1ST_RESEND"));
      firstret_loop_count++;
      break;
    case ND_RESEND:
      print_state(F(" ### loop ")); Serial.print(++loop_count); Serial.print(F(" === "));
      Serial.print(F("2ND_RESEND"));
      secondret_loop_count++;
      break;
    default:
      print_state(F(" !!! unknown state "));
      Serial.print(loop_state);
      Serial.print(F(", set to NORMAL"));
      loop_state = NORMAL;
      break;
  }

/*  
  Serial.print(F(" === "));
  Serial.print(F(" run time ")); Serial.print(prg_run_time); Serial.print(F(" s === "));
  Serial.print(F(" next normal wakeup ")); Serial.print(next_normal_wakeup); Serial.print(F(" s === Bat: "));
  Serial.print(batteryPcnt); Serial.print(F("% ===================="));

  Serial.print(F("\r\n *** ")); Serial.print(F(LNAME));
  Serial.print(F(" - ")); Serial.print(F(LVERSION));
  Serial.print(F(", avail. mem: ")); Serial.print(freeMemory());

  Serial.print(F("\r\n *** InvCmdErr ")); Serial.print(nfc_inv_cmd_errors);
  Serial.print(F(" ProtCmdErr ")); Serial.print(nfc_prot_set_cmd_errors);
  Serial.print(F(" ReadMem Err ")); Serial.print(nfc_read_mem_errors);
  Serial.print(F(" NFC scans ")); Serial.print(nfc_scan_count);
  Serial.print(F(" BLE err ")); Serial.print(ble_connect_errors);
  Serial.print(F(" RsndPkt ")); Serial.print(resend_pkts_events);
  Serial.print(F(" QueueEvt ")); Serial.print(queue_events);
  Serial.print(F(" RsndWkUp ")); Serial.print(resend_wakeup_cnt);

  Serial.print(F("\r\n *** spkCnt ")); Serial.print(spikeCount);
  Serial.print(F(" tSpkCnt ")); Serial.print(trendSpikeCount);
  Serial.print(F(" aSpkCnt ")); Serial.print(averageSpikeCount);
  Serial.print(F(" gWsChgd ")); Serial.print(glucoseWasChanged);

  if ( loop_count >= 2 ) {
    Serial.print(F("\r\n *** avg. sleep time: "));  Serial.print(avg_sleepTime / (loop_count-1));
    Serial.print(F(" s ProMini+BLE On: "));  Serial.print((bleTime / (loop_count-1)) / 1000);
    Serial.print(F(" s no NFC readed: "));  Serial.print(no_nfc_reading);
    Serial.print(F(" BG is NULL: "));  Serial.print(bg_is_null);
  }
*/

/*
  Serial.print(F("\r\n *** run time: ")); Serial.print(prg_run_time*1000);
  Serial.print(F(", arduino_ontime: ")); Serial.print(arduino_ontime);
  Serial.print(F(", ble_ontime: ")); Serial.print(ble_ontime);
  Serial.print(F(", nfc_ontime: ")); Serial.print(nfc_ontime);
*/
  Serial.print(F("\r\n *** run time: "));  Serial.print(prg_run_time);
  Serial.print(F(" s, Arduino On: "));  Serial.print(((float)arduino_ontime/(prg_run_time*1000))*100);
  Serial.print(F("%, BLE ON: "));  Serial.print(((float)ble_ontime/(prg_run_time*1000))*100);
  Serial.print(F("%, NFC: "));  Serial.print(((float)nfc_ontime/(prg_run_time*1000))*100);
  Serial.print(F("%"));

  Serial.print(F("\r\n *** total loops: "));  Serial.print(loop_count);
  Serial.print(F(", normal: "));  Serial.print(normal_loop_count);
  Serial.print(F(", 1st resend: "));  Serial.print(firstret_loop_count);
  Serial.print(F(", 2nd resend: "));  Serial.print(secondret_loop_count);
  Serial.print(F(", battery voltage: ")); Serial.print(batteryMv); Serial.print(F(" mV"));
/*
  int bmv;
  bmv = (batt_voltage[voltage_ri].voltage+200)*10;
  if ( ++voltage_ri >= MAX_VOLTAGE )
    voltage_ri = 0;
  Serial.print(F("\r\n *** battery voltage: "));  Serial.print(bmv);
*/
}

// main processing loop
void loop(void)
{
  int i, j;
  boolean resend_pkt;
#ifdef TRANSFER_LIVETIME
  unsigned long last_raw = 0;
#endif
  loop_start_time = millis();             // log the current time
  init_command_buff(&command_buff);       //initialise the command buffer

#ifdef SIMU_LIVETIME
  unsigned long simu_lifetime = 1000;
#endif

  waitDoingServices(2000, 1);

  print_start_message();
  /*
    #ifdef AUTOCAL_WDT
    print_state(F("calibrate PWR DOWN sleep timer (2048 ms) ..."));
    calibrate(2048);   // test for 16384 ms accuracy of PWR DWN timer
    #endif
  */
  /*
    var_sleepTime = 60;
    while ( 1 ) {
      print_state(F("wait for [")); Serial.print(var_sleepTime);
      Serial.print(F("] s, calibrate call for goToSleep() to ")); Serial.print(calibv);
      Serial.print(F(", ")); Serial.print(((var_sleepTime * calibv)+1)/1);
      goToSleep(((var_sleepTime * calibv)+1)/1);
    }
  */
  for ( i = 0 ; i < 3 ; i++ ) {
    if ( setupBLE() )                 // Open the UART and initialize HM-1x module
      break;
  }

  waitDoingServices(2000, 1); // wait for possible OK+LOST
#ifdef INIT_WITH_RENEW
  configBLE(1);                // configure the BLE module
#else
  configBLE(0);                // configure the BLE module without AT+RENEW
#endif
  configNFC();                // initialize and configure the NFC module

  // wait for a BLE connection to xDrip to get a TXID if needed
  print_state(F(" ..... initial wake up, waiting 40s fixed for BLE, time for a xDrip+ BT scan if needed"));
  waitDoingServices(40000, 1);

#ifdef DYNAMIC_TXID
  // if BLE connected wait some time to get an answer for TXID
  if ( ble_connected )
    waitDoingServices(3000, 1);

  // if dex_tx_id is zero, we do not have an ID to filter on.  So, we keep sending a beacon every 5 seconds until it is set.
  print_state(F("testing TXID"));
  settings.dex_tx_id = 0xFFFFFFFF;
  // should we ask for a TXID?
  if (settings.dex_tx_id >= 0xFFFFFFFF)
    settings.dex_tx_id = 0;
  // instead of save current TXID to EEPROM get it every time from xDrip using the Beacon mechanism
  while (settings.dex_tx_id == 0) {
    print_state(F("no TXID. Sending beacon"));
    // wait until we have a BLE connection
    //    while(!ble_connected) doServices(1);
    if ( !waitDoingServicesInterruptible(40000, &ble_connected, 1) && (settings.dex_tx_id != 0) )
      break;
    waitDoingServices(1000, 1);
    //send a beacon packet
    sendBeacon();
    // set flag dex_tx_id_set
    doServices(0);
    //wait 5 seconds, beatify output
    waitDoingServicesInterruptible(5000, &dex_tx_id_set, 1);
  }

  print_state(F("new TXID is ")); Serial.print(settings.dex_tx_id, HEX); Serial.print(F(" ("));
  // and show first 5 char from the TXID setting in xDrips menue
  Serial.print(dexcom_src_to_ascii(settings.dex_tx_id)); Serial.print(F(")"));

#else
  settings.dex_tx_id = 0xA5B1AE;    // -> "ABCDE" xBridge Wixel default
#endif /* DYNAMIC_TXID */

  // initialize to empty queue
  Pkts.read = 0;
  Pkts.write = 0;
#ifdef XBEXT
  got_drp = 0;
#endif
  //  resend_wakeup = 0;
  //  stop_resending = 0;

  // check for overflow / wear????
  queuesendtime = 2500L;            // calculate max send time if queue is completely full
  queuesendtime *= (DXQUEUESIZE + 1); // assuming one entry will cost 1 s

  loop_state = NORMAL;
  next_normal_wakeup = 300;

  got_txid = 0;
  resend_pkt = 0;

  print_state(F("entering main loop"));

  while (1)
  {
    check_battery();    // check for low battery and go to sleep here if wrong

#ifdef DB_VOLTAGE
    // store the current voltage of the battery
    if ( normal_loop_count >= 1 ) {                // skip initial loop
//      print_state(F("test loop_count ")); Serial.print(normal_loop_count);
      if ( (((normal_loop_count-1) % VOLTAGE_INTERVAL) == 0 ) && loop_state == NORMAL ) {
        
        if ( voltage_wi >= MAX_VOLTAGE )
          voltage_wi = 0;
        
        batt_voltage[voltage_wi].voltage = (batteryMv/10)-VOLT_CORR;
  
        // calculate for 1 h interval (12 readings)
/*
        batt_voltage[voltage_wi].a_on = (((float)(arduino_ontime-last_arduino_ontime) / (prg_run_time*1000))*100)+0.5;
        batt_voltage[voltage_wi].b_on = (((float)(ble_ontime-last_ble_ontime)     / (prg_run_time*1000))*100)+0.5;
        batt_voltage[voltage_wi].n_on = (((float)(nfc_ontime-last_nfc_ontime)     / (prg_run_time*1000))*100)+0.5;
*/
        batt_voltage[voltage_wi].a_on = (((float)(arduino_ontime-last_arduino_ontime) / (VOLTAGE_INTERVAL*300000.0))*100.0)+0.5;
        batt_voltage[voltage_wi].b_on = (((float)(ble_ontime-last_ble_ontime)         / (VOLTAGE_INTERVAL*300000.0))*100.0)+0.5;
        batt_voltage[voltage_wi].n_on = (((float)(nfc_ontime-last_nfc_ontime)         / (VOLTAGE_INTERVAL*300000.0))*100.0)+0.5;
        
        print_state(F("store voltage statistics: ")); Serial.print(batteryMv); Serial.print(F(" mV"));
/*
        Serial.print(F("mV, ")); Serial.print(arduino_ontime); Serial.print(F(" ms, ")); 
        Serial.print(last_arduino_ontime); Serial.print(F(" ms - "));
*/
        last_arduino_ontime = arduino_ontime;
        last_ble_ontime = ble_ontime;
        last_nfc_ontime = nfc_ontime;

        // overall runtime statistics
/*
        Serial.print((((float)(arduino_ontime) / (float)(prg_run_time*1000))*100.0)+0.5); Serial.print(F("%, "));
        Serial.print((((float)(ble_ontime)     / (float)(prg_run_time*1000))*100.0)+0.5); Serial.print(F("%, "));
        Serial.print((((float)(nfc_ontime)     / (float)(prg_run_time*1000))*100.0)+0.5); Serial.print(F("%"));
*/  
        voltage_wi++;
      }
    }
#endif /* DB_VOLTAGE */

#ifdef AUTOCAL_WDT
    // recalibrate PWR DWN timer every 30 m, loop duration is 5 m
    if ( (normal_loop_count % 6) == 0 ) {  // every 5 loops
      print_state(F("calibrate PWR DOWN sleep timer (2048 ms) ..."));
      calibrate(2048);
    }
#endif

    // *********** NFC ***************************************

    // get BG reading, not in case of last tranfer failed
    int zz;
    if ( loop_state == NORMAL ) {
        for ( zz = 0 ; zz < NFC_READ_RETRY ; zz++ ) {
          wakeNFCUp();
          get_packet_result = get_packet(&Pkts.buffer[Pkts.write]);
          shutNFCDown();
          if ( get_packet_result )
            break;
          if ( sensor_oor )
            break;
          print_state(F("NFC read error, wait 3 s"));
          waitDoingServices(3000, 1);
        }

      // was sensor out of range?
      if ( sensor_oor )
        print_state(F("sensor out of range"));

      if ( get_packet_result ) {
        print_state(F("got packet at ")); Serial.print(Pkts.buffer[Pkts.write].ms/1000);
        Serial.print(F(" s, stored at position ")); Serial.print(Pkts.write);
        // so increment write position for next round...
        if ( ++Pkts.write >= DXQUEUESIZE )
          Pkts.write = 0;
        Serial.print(F(", incrementing write to ")); Serial.print(Pkts.write);
        if (Pkts.read == Pkts.write) {
          print_state(F("queue overflow, incrementing read overwriting oldest entry"));
          if ( ++Pkts.read >= DXQUEUESIZE ) //overflow in ringbuffer, overwriting oldest entry, thus move read one up
            Pkts.read = 0;
        }
      }
      else {
        print_state(F("did not receive a pkt with ")); Serial.print(Pkts.write - Pkts.read); Serial.print(F(" pkts in queue"));
        //        if ( ble_connected )
        //          sendBeacon();
      }
    }

    // ****************** BLE *******************************

    // wake up BLE module, but only if sensor in range
    int ble_found_during_wakeBLEUp = 0;
    if ( (loop_count > 0) && !sensor_oor ) {
      ble_found_during_wakeBLEUp = wakeBLEUp();  // switch on BLE, wait 40 s for connect
    }

    // ******************* handling **************************

    // queue not empty, BLE was connected/disconnected after wakeup, then wait additional 20 s for BLE reconnect
    now = millis();
    if ( Pkts.read != Pkts.write ) { // if we have a packet

#ifdef DB_PROCESSING
      Pkts.buffer[Pkts.read].retries++;
      print_state(F("we have packet(s) to deliver, try #")); Serial.print(Pkts.buffer[Pkts.read].retries);
#endif
      if ( ble_found_during_wakeBLEUp ) {
        while (!ble_connected && ((millis() - now) < 20000) ) {
          print_state(F("packet waiting for ble connect"));
          if ( waitDoingServicesInterruptible(10000, &ble_connected, 1) ) {
            print_state(F("connected, wait 2 s before sending the packet"));
            waitDoingServices(2000, 1);
          }
        }
      }
#ifdef XBEXT
      if ( got_drp ) {
        got_drp = 0;
        send_data((unsigned char *)&qpkt1, (unsigned char)qpkt1.size);
        send_data((unsigned char *)&qpkt2, (unsigned char)qpkt2.size);
      }
#endif
      evtflg1 = 0;
      evtflg2 = 0;

      // we got a connection, so send pending packets now - at most for <time> after the last packet was received
      // wait enough time to empty the complete queue!

      now = millis();
      while ((Pkts.read != Pkts.write) && ble_connected && ((millis() - now) < queuesendtime )) {
        got_ack = 0;
        got_txid = 0;
        print_state(F("send packet at position ")); Serial.print(Pkts.read);
        print_packet(&Pkts.buffer[Pkts.read]);
#ifdef TRANSFER_LIVETIME
        last_raw = Pkts.buffer[Pkts.read].raw;
#endif
        //        print_state(F("wait for ACK"));
        // wait 10 s for ack
        for ( j = 0 ; j < 10 ; j++ ) {
          waitDoingServicesInterruptible(1000, &got_ack, 1);
          if ( !ble_connected ) {
            print_state(F("connection lost during wait for ack, go to sleep"));
            break;
          }
        }

        if ( got_ack ) {
          print_state(F("got ack"));
          Serial.print(F(" for read position ")); Serial.print(Pkts.read); Serial.print(F(" while write is "));
          if ( ++Pkts.read >= DXQUEUESIZE )
            Pkts.read = 0;     //increment read position since we got an ack for the last package
          Serial.print(Pkts.write); Serial.print(F(", incrementing read to ")); Serial.print(Pkts.read);
          if ( !evtflg1 && (Pkts.read != Pkts.write) ) {  // when still available we have a queue event
            evtflg1 = 1;
            queue_events++;
          }
          resend_pkt = 0;
        }
        else if  ( got_txid ) {
          // workaround for Sony Smartwatch 3 running wear collection service
          // there is an alternating TXID setting in both device (nightly 170512)
          // when switching simply resend packet with new TXID
          print_state(F("got new TXID, packet resend with new TXID"));
          got_txid = 0;
        }
        else {
          if ( !resend_pkt ) {
            print_state(F("no ack received, try again"));
            if ( !evtflg2 ) {
              evtflg2 = 1;
              resend_pkts_events++;   // we got no ACK, pkts to be resended
            }
            resend_pkt = 1;         // try to resend packet one time
#ifdef DB_PKT
            Pkts.buffer[Pkts.read].retries++;
            print_state(F("packets to deliver, resend packet #")); Serial.print(Pkts.buffer[Pkts.read].retries);
#endif
          }
          else {
            print_state(F("pkt resent, no ack received, try again next wakeup"));
            resend_pkt = 0;
            break;
          }
        }
      } /* while (send all packets avaibale ) */
#ifdef TRANSFER_LIVETIME
        // transfer the sensor livetime with the last successful tranmitted BG reading
      if ( ((normal_loop_count % 12) == 0) && (last_raw != 0) )
      {
        if ( got_ack ) {
          String packet = "";
          // build a LimiTTer string
          packet = String(last_raw); packet += ' '; packet += "216"; packet += ' ';
          packet += String(batteryPcnt); packet += ' '; 
#ifdef SIMU_LIVETIME
          packet += String(simu_lifetime);
          simu_lifetime += 500;
#else
          packet += String(sensorMinutesElapse);
#endif
          print_state(F("send sensor lifetime in LimiTTer ASCII format: ["));
          Serial.print(packet); Serial.print(F("]")); 
          send_string(packet, 500);
          waitDoingServices(1000, 1);
          last_raw = 0;
        }
      }
#endif

      // did we send all queued readings?
      if ( loop_count > 0 ) {
        if ( Pkts.read != Pkts.write ) {
          switch ( loop_state ) {
            case NORMAL:
              loop_state = ST_RESEND;
              break;
            case ST_RESEND:
              loop_state = ND_RESEND;
              break;
            case ND_RESEND:
              loop_state = NORMAL;
              break;
            default:
              loop_state = NORMAL;
              break;
          }
#ifdef DB_PROCESSING
          print_state(F("queue not empty, loop state changed to ")); Serial.print(loop_state);
#endif
        }
        else {
          // packet queue enpty, done
          loop_state = NORMAL;
          print_state(F("queue empty, change loop state to ")); Serial.print(loop_state);
        }
      }
    } /* if ( queue not empty */

    // empty serial out buffer and open tasks
    //    waitDoingServices(500, 1);

    // in case of dead BLE service on phone dont try longer than 30 mins
    // resending (3 * (30/5))
    if ( cons_loop_wo_ble > 18 ) {
      loop_state = NORMAL;
    }
    cons_loop_wo_ble++;

    // calculate sleep time
    // ensure to use 300 s for a full cycle of loop() and sleeping
    loop_time = millis() - loop_start_time;

    switch ( loop_state ) {
      case NORMAL:
        var_sleepTime = (((next_normal_wakeup * 1000) - abs_millis()) / 1000);
        //        print_state(F(" ===== loop_state NORMAL: packet queue empty , sync to 5 min spacing ===== "));
        break;
      case ST_RESEND:
        resend_wakeup_cnt++;
        var_sleepTime = 30;
        //        print_state(F(" ===== loop_state 1ST_RESEND: resend loop #1, doing short sleep"));
        break;
      case ND_RESEND:
        var_sleepTime = 30;
        resend_wakeup_cnt++;
        //        print_state(F(" ===== loop_state 2ND_RESEND: resend loop #2, doing short sleep"));
        break;
      default:
        print_state(F("loop_state error, setting to NORMAL (was ")); Serial.print(loop_state); Serial.print(F(")"));
        loop_state = NORMAL;
        break;

    }
    Serial.print(F("\r\n *** next_normal_wakeup ")); Serial.print(next_normal_wakeup); Serial.print(F(" s, "));
    Serial.print(F(" current time ")); Serial.print(abs_millis() / 1000); Serial.print(F(" s"));
    Serial.print(F(", var_sleepTime set to ")); Serial.print(var_sleepTime);

    print_statistics1();

    shutBLEDown(1);          // cut BLE connection and switch HM-1x OFF

    if ( var_sleepTime > 320 ) // test possible overflow
      var_sleepTime = 256;

#ifdef DB_PKT
//    int zz = 0;
    if ( x_command ) {
      //    if ( 1 ) {
//      x_command = 0;
      print_state(F("BG queue - "));
      Serial.print(F("write: ")); Serial.print(Pkts.write);
      Serial.print(F(", read: ")); Serial.print(Pkts.read);
      int i;
      zz = 0;
      Serial.print(F("\r\n"));
      for ( i = 0 ; i < DXQUEUESIZE ; i++ ) {
        Serial.print(F("#")); Serial.print(i);
        Serial.print(F(": BG ")); Serial.print(Pkts.buffer[i].raw);
        Serial.print(F(", ")); Serial.print((Pkts.buffer[i].ms + 500) / 1000);
        Serial.print(F(" s, ")); Serial.print((Pkts.buffer[i].sended + 500) / 1000);
        Serial.print(F(" s, ")); Serial.print(Pkts.buffer[i].retries);
        if ( Pkts.buffer[i].sended != 0 ) { 
          Serial.print(F(", ")); Serial.print((Pkts.buffer[i].sended - Pkts.buffer[i].ms + 500) / 1000);
          Serial.print(F(" s"));
        }
        else {
          Serial.print(F(", not sent"));
        }
        if ( (++zz % 2) == 1 )
          Serial.print(F(" "));
        else
          Serial.print(F("\r\n"));
          
      }
    }
#endif

#ifdef DB_VOLTAGE
    if ( v_command ) {
      waitDoingServices(100, 1);
      print_state(F("voltage in ")); Serial.print(VOLTAGE_INTERVAL/12); Serial.print(F(" h spacing - "));
      int i;
      for ( i = 0 ; i < MAX_VOLTAGE; i++ ) {
        if ( ( i % 4 ) == 0 ) {
          Serial.print(F("\r\n"));
          waitDoingServices(100, 1);
        } 
        if ( batt_voltage[i].voltage != 0 ) {
          Serial.print((batt_voltage[i].voltage+VOLT_CORR)*10); Serial.print(F(" mV "));
/*
          Serial.print(batt_voltage[i].a_on); Serial.print(F("% "));
          Serial.print(batt_voltage[i].b_on); Serial.print(F("% "));
          Serial.print(batt_voltage[i].n_on); Serial.print(F("%"));
*/
          Serial.print((float)batt_voltage[i].a_on); Serial.print(F("% "));
          Serial.print((float)batt_voltage[i].b_on); Serial.print(F("% "));
          Serial.print((float)batt_voltage[i].n_on); Serial.print(F("%, "));
        }
        else {
//        Serial.print(F(" - "));
          break;
        }
      }
    }
#endif /* DB_VOLTAGE */

//#ifdef AUTOCAL_WDT
    print_state(F(" === goToSleep for [")); Serial.print(var_sleepTime);
    Serial.print(F("], calibrate call para of goToSleep() with ["));
    Serial.print(calibv); Serial.print(F("] to ")); Serial.print(((var_sleepTime * calibv) + 1) / 1);
    goToSleep(((var_sleepTime * calibv) + 1) / 1);
/*
#else
    goToSleep(var_sleepTime);
#endif
*/
    // ============================ program restart after sleep ========================

    arduino_start_time = millis();

    // waking up, power on BLE, initialze NFC
    // count programm run time up
    prg_run_time += (loop_time + (var_sleepTime * 1000)) / 1000;
    loop_start_time = millis();       // set start point for loop

    // calc time for next wake up, adjust to 300 framing
    long pgrt,  pgrtfrac;
    if ( loop_state == NORMAL ) {
      pgrt = prg_run_time + 300;
      pgrtfrac = (prg_run_time + 300) % 300;
      if ( pgrtfrac > 150 ) {
        pgrtfrac -= 300;
      }
      next_normal_wakeup = pgrt - pgrtfrac;
    }

    init_command_buff(&command_buff); // no BLE connection
    // statistics for ON time and BLE ON time
    if ( loop_count >= 1 ) {
      avg_sleepTime += var_sleepTime;
      bleTime += loop_time;
    }
    else {
      avg_sleepTime = 0;
      bleTime = 0;
    }
    print_statistics2();
    // wake up Arduino
    wakeUp();
    waitDoingServices(250, 1);
  }
}


