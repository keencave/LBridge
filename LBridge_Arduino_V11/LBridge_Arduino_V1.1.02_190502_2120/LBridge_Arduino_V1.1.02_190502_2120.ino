/*
   LBridge scans the Freestyle Libre Sensor every 5 minutes
   and sends the current BG readings to the xDrip+ Android app.

   Supported Libre sensors:

    Freestyle Libre 
    Freestyle Libre2
    Freestyle Libre Pro / H

   Automatic detection of sensor type, on-the-fly change wihtout need to restart

   This sketch is based on the LimiTTer project from JoernL, the sample sketches for the BM019 module 
   from Solutions Cubed and the protocol extensions done by savek-cc in the xbridge Wixel project.

   This code replaces the original LimiTTer by @JoernL code without any changes needed.

   Hardwaresource in xDrip has to be set to "Libre". Please use one of the latest xDrip+ versions.

   The xBridge2 protocol is used to send BG readings to xDrip+. In case of failure it queues up not sended BG readings
   with the correct timestamp and send them within the next BLE connection. There should be no missed BG readings
   when the LimiTTer is worn. Battery performance is improved compared to LimiTTer.

   For usage with Spike app / iOS the LimiTTer protocol has to be used instead (uncomment the USE_LIMITTER_PROTOCOL line)

   Wiring for UNO / Pro-Mini ( https://github.com/JoernL/LimiTTer/blob/master/LimiTTer.pdf )

   Arduino          BM019           BLE-HM11
   IRQ: Pin 9       DIN: pin 2
   SS: pin 10       SS: pin 3
   MOSI: pin 11     MOSI: pin 5
   MISO: pin 12     MISO: pin4
   SCK: pin 13      SCK: pin 6
   I/O: pin 3                       VCC: pin 9
   I/O: pin 5                       TX:  pin 2
   I/O: pin 6                       RX:  pin 4
   I/O: pin 2                       Sys LED: pin 15
*/

/* 
 changes since V0.9 170716_1950:
 	- AT+RENEW fail mechanism disabled to ensure usage with modified fake modules
	- display serial status chars while sleeping
	- detect expired/faulty sensors during normal lifetime of 14 days
	- added AT command before AT+ commands to break BLE connection if active
	- hard system reset if AT+NOTI1 command cannot be processed
  V1.0.00:
  - support for L2 added
  V1.0.01:
  - NFC part reworked
  V1.0.03
  - automatic detection of L1 or L2 sensor
  V1.0.05
  - changed NFC commands to original reader sequence
  - detect 4 types of libre sensors
  V1.0.07
  - use precompiled lib for decoding L2
  V1.0.08
  - L2 with moving average filter
  V1.1.00
  - support for Libre Pro/H added
  - #define for simple LimiTTer protocol for Spike
  V1.0.01
  - CRC calc for Libre Pro added
*/

/* ************************************************ */
/* *** config #DEFINES **************************** */
/* ************************************************ */

#define LB_DATETIME       "190502_2120"
#define LB_VERSION        "V1.1"        // version number
#define LB_MINOR_VERSION  ".02"         // indicates minor version
#define LNAME             "LimiTTerLx"  // BLE name, must begin with "LimiTTer" to avoid misfunctions in xDrip+

//#define REMOVE_SPIKES       // uncomment to remove spikes from raw data
//#define USE_LIMITTER_PROTOCOL // uncomment to use LimiTTer protocol for Spike app / iOS
//#define USE_DEAD_SENSOR       // uncomment to test with a dead sensor
#define SHOW_BLE              // uncomment log BLE communication
//#define SHOW_NFC_ERROR        // uncomment to log NFC communication
//#define SHOW_NFC_CMD          // uncomment to log NFC errors
#define TRANSFER_LIVETIME     // display sensor livetime when using XBRIDGE2 protocol, xDrip accepts LimiTTer messages also
#define USE_BLE 	            // comment that to test withput BLE communication
//#define ATRESET             // uncomment to do a AT+RESET after every BLE wakeup?
//#define HW_BLE              // uncomment to detect BLE conn status with system LED pin connected to arduino pin 2
//#define SHORT_START         // uncomment for testing with fast start procedure
#define CYCLE_SEC 300         // cycle duration, dafault 300 s = 5 minutes
//#define CYCLE_SEC 60

#include <SPI.h>
#include <SoftwareSerial.h>
#include <avr/sleep.h>
#include <avr/power.h>
#include <avr/wdt.h>

#include <L2read.h>

/*
to do for L2:
moving average from 5 elements -> little faster response to changes is "modified moving average 4":
current BG = average(N, Nlim, N-1, N-2)
N - actual BG,
N-1, N-2 - BG from past 1 and 2 minutes
Nlim - actual BG limited value depending on current "trend arrow"
if N != N-1 by more than maxDifference => Nlim = (N-1) +/- maxDifference
maxDifference strictly depends on actual trend speed change
in efect - this is moving average with added "box" filter
*/

/* ********************************************* */
/* ********* global variables ****************** */
/* ********************************************* */

#define RAW2DEX (8.5f)    // scale factor to transfer raw libre readings to xDrip+/Dexcom scale

#define NFC_READ_RETRY 3  // amount of tries to recall read_memory() in case of NFC read error
#define ATLNAME "AT+NAME" LNAME

#define MIN_V 3450        // battery empty level
#define MAX_V 4050        // battery full level

#define MIN_VALID_BG 40   // sanity check for L2 as we dont have a CRC yet
#define MAX_VALID_BG 500

byte batteryLow;
int  batteryPcnt;
long batteryMv;

#define WDT_8S_MASK 0b00100001  // watch dog timer mask for 8 s
#define WDT_1S_MASK 0b00000110  // watchdog timer mask for 1 s

float calibv = 1.0;       // internal time calibration - ratio of real clock with WDT clock
int sleepTime = 32;       // sleeptime in multipliers of 8 s

#define SSPin 10          // Slave Select pin
#define IRQPin 9          // Sends wake-up pulse for BM019
#define NFCPin1 7         // Power pin BM019
#define NFCPin2 8         // Power pin BM019
#define NFCPin3 4         // Power pin BM019
#define BLEPin 3          // BLE power pin.
#define MOSIPin 11
#define SCKPin 13

#ifdef HW_BLE
const int bleSysLedPin = 2; // connect to Sys LED pin of HM-1x
#endif

static byte RXBuffer[50]; // receiver buffer for NFC bytes from BM019 module

int sensorMinutesElapse = 0;  // internal sensor counter for minutes since activation

SoftwareSerial ble_Serial(5, 6); // RX | TX, // software COM device for accessing the HM1x module

unsigned long uartBaudrate = 9600;  
// array of HM-1x baudrates for rate detection.
unsigned long uartBaudrateTable[] = {9600L, 19200L, 38400L, 57600L, 4800, 2400, 1200};

#define RAWBITMASK 0x1FFF   // use 13 bit mask reading a raw BG values from the FRAM

// types of libre sensor supported
#define LIBRE1    1 
#define LIBRE2    2
#define LIBRE_US  3
#define LIBRE_PRO 4
#define UNKNOWN   5

#define LIBREPRO_HEADER1 3
#define LIBREPRO_HEADER2 4
#define LIBREPRO_DATA    5

#define LPRO_STATUSBYTE 4         
#define LPRO_SN 24             
#define LPRO_SN_LENGTH 13
#define LPRO_SENSORMINUTES 74
#define LPRO_TRENDPOINTER 76
#define LPRO_TRENDOFFSET 80

uint8_t sensorVersion = LIBRE1;

#define LxMAXBLOCK 43               // size of libre sensor FRAM = 344 bytes in 8 bytes blocks
#define FRAMSIZELx (LxMAXBLOCK*8)
static byte FRAM[FRAMSIZELx];

byte ble_answer;                    // char from BLE reeived?

uint16_t crcL2Fram;                // check for dead L2 sensor, FRAM changes every minute
uint16_t lastCrCL2Fram = 0x00;     

unsigned long loop_count = 0;        // number of main loops processed
unsigned long normal_loop_count = 0;
unsigned long firstret_loop_count = 0;
unsigned long secondret_loop_count = 0;

// absolute progam run time in s counting also sleep() phase
unsigned long prg_run_time = 0;     // in sec
unsigned long next_normal_wakeup;
int cons_loop_wo_ble = 0;           // consequtive loops w/o BLE connect

unsigned long loop_start_time;      // loop starting time in ms
unsigned long loop_time;            // current loop duration in ms

unsigned long var_sleepTime;        // how long do we want to sleep in s
unsigned long queuesendtime;

unsigned long arduino_ontime = 0;
unsigned long ble_ontime = 0;
unsigned long nfc_ontime = 0;

unsigned long arduino_start_time = 0;
unsigned long ble_start_time = 0;   // time when BLE starts ms
unsigned long nfc_start_time = 0;

#define NORMAL 0
#define ST_RESEND 1
#define ND_RESEND 2

static int loop_state;              // state machine for loop

static int NFCReadErrCnt = 0;       // count NFC read errors

boolean deadLxSensor = false; 

/* ************************************************************* */
/* *** xBridge2 stuff ****************************************** */
/* ************************************************************* */

// defines the xBridge protocol functional level.  Sent in each packet as the last byte.
#define DEXBRIDGE_PROTO_LEVEL (0x01)

static volatile boolean got_ack = 0;        // indicates if we got an ack during the last do_services.
static volatile boolean ble_connected = 0;  // bit indicating the BLE module is connected to the phone
#ifdef HW_BLE
static volatile boolean hwble_connected = 0;// bit indicating the BLE module is connected to the phone
#endif

static volatile boolean got_ok = 0;         // flag indicating we got OK from the HM-1x
static unsigned long last_ble_send_time;    // for beuatifying BLE debug output
static boolean crlf_printed = 0;

#define COMMAND_MAXLEN 40
static char at_answer[COMMAND_MAXLEN];             // buffer for AT answers from HM1x module

static uint8_t uid[8];                      // sensor UID
static char *ptrSensorSN;                   // pointer to decoded sensor number
#define SENSORIDLEN     15                  // length of decoded sensor ID string
static char decodedSensorSN[SENSORIDLEN];   // decoced sensor ID

byte sensorStatus;  	                      // indicates sensor working status: starting, working, ...

typedef struct _command_buff {              //structure of a USB command
  unsigned char commandBuffer[COMMAND_MAXLEN];
  unsigned char nCurReadPos;
} t_command_buff;

static t_command_buff command_buff;         // buffer for incoming BLE messages

typedef struct __attribute__((packed)) _Dexcom_packet {             // container for NFC readings on LimiTTer
  uint16_t raw;
  unsigned long ms;
} Dexcom_packet;

#define DXQUEUESIZE (4*12)                  // queue depth, 12 values per hour

typedef struct  __attribute__((packed)) {
  volatile unsigned char read;
  volatile unsigned char write;
  Dexcom_packet buffer[DXQUEUESIZE];
} Dexcom_fifo;

Dexcom_fifo Pkts;                           // queue containing raw BG readings

typedef struct _nRawRecord {                // structure of a Dexcom raw record we will send.
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

unsigned long dex_tx_id = 0xA5B1AE; // correspond to "ABCDE" xBridge Wixel default

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

// initialize the hardware
void setup() 
{
  pinMode(IRQPin, OUTPUT);      // NFC pins
  pinMode(SSPin, OUTPUT);  
  pinMode(NFCPin1, OUTPUT);
  pinMode(NFCPin2, OUTPUT);
  pinMode(NFCPin3, OUTPUT);
  pinMode(MOSIPin, OUTPUT);
  pinMode(SCKPin, OUTPUT);

  pinMode(BLEPin, OUTPUT);      // BLE pins
  digitalWrite(BLEPin, LOW);    // reset of HM1x in case of cold reboot
  delay(1000);
  digitalWrite(BLEPin, HIGH);   // switch HM1x module ON
  ble_start_time = millis();
  ble_connected = 0;
#ifdef HW_BLE
  hwble_connected = 0;
  pinMode(bleSysLedPin, INPUT);
#endif

  Serial.begin(9600);

  arduino_start_time = millis();

  for ( unsigned int i = 0 ; i < sizeof(RXBuffer) ; i++ )
    RXBuffer[i] = 0x00;
}

/* *********************************************************** */
/* *** NFC *************************************************** */
/* *********************************************************** */

#define NFC_PROTOCOL_SELECT         { 0x02, 0x02, 0x01, 0x0D }

#define SINGLE_BLOCK_RETURN         { 0x04, 0x03, 0x02, 0x20, 0x00 }
#define MULTI_BLOCK_RETURN          { 0x04, 0x04, 0x02, 0x23, 0x00, 0x02 }

#define GET_NFC_INVENTORY           { 0x04, 0x03, 0x26, 0x01, 0x00 }
#define READ_SYS_INFO               { 0x04, 0x02, 0x02, 0x2B }

#define NFC_GET_UID                 { 0x04, 0x03, 0x26, 0x01, 0x00 }
#define NFC_GET_LTYPE               { 0x04, 0x03, 0x02, 0xA1, 0x07 }

#define NFC_SENSOR_STATUS           { 0x04, 0x04, 0x02, 0xA0, 0x07, 0x00 }

void sendNFCCommand(byte *commandArray, int length, boolean dbg)
{
  if ( dbg ) {
#ifdef SHOW_NFC_CMD
    print_state(F(" >>>"));
    for ( int i = 0 ; i < length ; i++ ) {
      if ( commandArray[i] > 0x0F )   Serial.print(F(" 0x"));
      else                            Serial.print(F(" 0x0"));
      Serial.print(commandArray[i], HEX);
    }
#endif
  }

  for ( unsigned int i = 0 ; i < sizeof(RXBuffer) ; i++ )
    RXBuffer[i] = 0x00;

  digitalWrite(SSPin, LOW);
  SPI.transfer(0x00);
  for (int i = 0; i < length; i++) {
    SPI.transfer(commandArray[i]);
  }
  digitalWrite(SSPin, HIGH);
  delay(1);
}

void pollNFCUntilResponsIsReady(void)
{
  unsigned long start = millis();

  digitalWrite(SSPin , LOW);
  while ( RXBuffer[0] != 8 ) {
    RXBuffer[0] = SPI.transfer(0x03);
    RXBuffer[0] = RXBuffer[0] & 0x08;

    if ( (millis() - start) > 2000 )
      break;

  }
  digitalWrite(SSPin, HIGH);
  delay(1);
}

byte receiveNFCResponse(boolean dbg)
{
  digitalWrite(SSPin, LOW);
  SPI.transfer(0x02);
  RXBuffer[0] = SPI.transfer(0);
  RXBuffer[1] = SPI.transfer(0);

  if ( RXBuffer[1] > sizeof(RXBuffer) ) {
    print_state(F(" *** RXBuffer too small, skip"));
    for ( unsigned int i = 0 ; i < sizeof(RXBuffer) ; i++ ) {
      Serial.print(RXBuffer[i], HEX); 
      Serial.print(F(" "));
    }
    return(0);
  }

  for (byte i = 0; i < RXBuffer[1]; i++) 
    RXBuffer[i + 2] = SPI.transfer(0);
  digitalWrite(SSPin, HIGH);
  delay(1);

  if ( dbg ) {
#ifdef SHOW_NFC_CMD
    Serial.print(F(" <<< "));
    Serial.print(RXBuffer[0], HEX);
    Serial.print(F(" "));
    Serial.print(RXBuffer[1], HEX);
    for ( int i = 0 ; i < RXBuffer[1] ; i++ ) {
      if ( RXBuffer[i+2] > 0x0F )   Serial.print(F(" "));
      else                          Serial.print(F(" 0"));
      Serial.print(RXBuffer[i+2], HEX);
    }
#endif
  }

  return(RXBuffer[0]);
//  return( RXBuffer[0] == 0x80 );
}

void showNFCReturnCode(byte resp)
{
#ifndef SHOW_NFC_ERROR
  return;
#endif
  if ( resp != 0x80 ) {
    Serial.print(F("\r\n"));
    switch (resp ) {
      case 0x82:  Serial.print(F("***invalid command length")); break;
      case 0x83:  Serial.print(F("***invalid protocol")); break;
      case 0x86:  Serial.print(F("***communication error")); break;
      case 0x87:  Serial.print(F("***frame wait timeout or no tag")); break;
      case 0x88:  Serial.print(F("***invalid SOF")); break;
      case 0x89:  Serial.print(F("***receive buffer overflow")); break;
      case 0x8A:  Serial.print(F("***framing error")); break;
      case 0x8B:  Serial.print(F("***EGT timeout")); break;
      case 0x8C:  Serial.print(F("***invalid length")); break;
      case 0x8D:  Serial.print(F("***CRC error")); break;
      case 0x8E:  Serial.print(F("***reception lost without EOF")); break;
      default:    Serial.print(F("***unknow error: ")); Serial.print(resp); break;
    }
  }
}

/* 
 * manage BM019 module
 */
void configNFC(void)
{
  // power on BM019 here, moved initialisation code (@JoernL)
  digitalWrite(IRQPin, HIGH);
  digitalWrite(SSPin, HIGH);
  digitalWrite(NFCPin1, HIGH);
  digitalWrite(NFCPin2, HIGH);
  digitalWrite(NFCPin3, HIGH);

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

  nfc_start_time = millis();

  NFCProtocolSelect(); // ISO 15693 settings
  waitDoingServices(100, 1);
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

/* ********************************************************************** */

/*
 * select ISO 15693 protocol to start communication
 */ 
boolean NFCProtocolSelect() 
{
  byte command[] = NFC_PROTOCOL_SELECT;

  sendNFCCommand(command, sizeof(command)/sizeof(command[0]), 1);
  pollNFCUntilResponsIsReady();
  receiveNFCResponse(1);
  if ( (RXBuffer[0] == 0) & (RXBuffer[1] == 0) ) {
    return(1);
  }
  else {
    print_state(F(" *** NFCProtocolSelect error"));
    return(0);
  }
}

byte ReadSingleBlockReturn(int blockNum)
{
//  byte command[5] = { 0x04, 0x03, 0x02, 0x20, 0x00 };
  byte command[] = SINGLE_BLOCK_RETURN;
  byte retry = 0;
  byte rc = 1;

  command[4] = blockNum;
  do {
//    print_state(F("single block: ")); Serial.print(sizeof(command)/sizeof(command[0]));
    sendNFCCommand(command, sizeof(command)/sizeof(command[0]), 1);
    pollNFCUntilResponsIsReady();
    rc = receiveNFCResponse(1);
    if ( rc != 0x80 ) {
      showNFCReturnCode(rc);
      NFCReadErrCnt++;
    }
  } while ( !rc && (retry++ < NFC_READ_RETRY) );
  return(rc);
}

byte ReadMultiBlockReturn(int blockNum)
{
  byte command[] = MULTI_BLOCK_RETURN;
  byte retry = 0;
  byte rc = 1;

  command[4] = blockNum;
  do {
//    print_state(F("single block: ")); Serial.print(sizeof(command)/sizeof(command[0]));
    sendNFCCommand(command, sizeof(command)/sizeof(command[0]), 1);
    pollNFCUntilResponsIsReady();
    rc = receiveNFCResponse(1);
    if ( rc != 0x80 ) {
      showNFCReturnCode(rc);
      NFCReadErrCnt++;
    }
  } while ( !rc && (retry++ < NFC_READ_RETRY) );
  return(rc);
}

/*
 * read the sensor UID to detect if a sensor is in range
 */
boolean NFCGetUid(byte maxTrials)
{
  byte command[] = NFC_GET_UID;
  int count = 0;
  byte rc = 1;

  delay(10);
  do
  {
    sendNFCCommand(command, sizeof(command)/sizeof(command[0]), 1);
    pollNFCUntilResponsIsReady();
    rc = receiveNFCResponse(1);
    // good result code and no error flag set?
    if ( (RXBuffer[0] == 0x80) && ((RXBuffer[2] & 0x01) == 0) ) {
      for (int i = 0; i < 8; i++) 
        uid[i] = RXBuffer[11 - i];
      ptrSensorSN = decodeSN(uid);
      return(1);
    }
    else  {
        showNFCReturnCode(rc);
    }
  } while ( (rc != 0x80) && (count++ < maxTrials));
  print_state(F(" *** failed getting UID, clearing UID"));
  memset(uid, 0, sizeof(uid));
  // avoid system crash displaying SN number empty
  strcpy(decodedSensorSN, "-");
  ptrSensorSN = &decodedSensorSN[0];
  return(0);
}

/*
 * determine libre sensor type using custom abbott command
 */
boolean NFCGetLtype(byte maxTrials)
{
  byte command[] = NFC_GET_LTYPE;
  int count = 0;
  byte rc = 1;

  delay(10);
  do
  {
    sendNFCCommand(command, sizeof(command)/sizeof(command[0]), 1);
    pollNFCUntilResponsIsReady();
    rc = receiveNFCResponse(1);
    // good result code and no error flag set?
    if ( (RXBuffer[0] == 0x80) && ((RXBuffer[2] & 0x01) == 0) ) {
        // Germany is DF 00 00 01, Canada is DF 00 00 04
        if ( (RXBuffer[3]==0xDF) && (RXBuffer[4]==0x00) && (RXBuffer[5]==0x00) /*&& (RXBuffer[6]==0x01)*/ ) {
          sensorVersion = LIBRE1;
        } 
        else if ( (RXBuffer[3]==0x9D) && (RXBuffer[4]==0x08) && (RXBuffer[5]==0x30) && (RXBuffer[6]==0x01) ) {
          sensorVersion = LIBRE2;
        }
        else if ( (RXBuffer[3]==0xE5) && (RXBuffer[4]==0x00) && (RXBuffer[5]==0x03) && (RXBuffer[6]==0x02) ) {
          sensorVersion = LIBRE_US;
        }
        else if ( (RXBuffer[3]==0x70) && (RXBuffer[4]==0x00) && (RXBuffer[5]==0x10) && (RXBuffer[6]==0x00) ) {
          sensorVersion = LIBRE_PRO;
        }
        else {
          sensorVersion = UNKNOWN;
        }
        print_state(F("Sensor Version: "));
        if      ( sensorVersion == LIBRE1 )     Serial.print(F("LIBRE1"));
        else if ( sensorVersion == LIBRE2 )     Serial.print(F("LIBRE2"));
        else if ( sensorVersion == LIBRE_US )   Serial.print(F("LIBRE US"));
        else if ( sensorVersion == LIBRE_PRO )  Serial.print(F("LIBRE PRO"));
        else {
          Serial.print(F("UNKNOWN"));
          print_state(F("Sensor type: ")); 
          Serial.print(RXBuffer[3], HEX); Serial.print(F(" ")); Serial.print(RXBuffer[4], HEX); Serial.print(F(" "));
          Serial.print(RXBuffer[5], HEX); Serial.print(F(" ")); Serial.print(RXBuffer[6], HEX); Serial.print(F(" "));
          // assume that unkwnown sensors are a Libre1 type as default
          sensorVersion = LIBRE1;
        }
        Serial.print(F(", Sensor SN: ")); Serial.print(ptrSensorSN);
        return(1);
    }
    else {
      showNFCReturnCode(rc);
    }
  } while ( (rc != 0x80) && (count++ < maxTrials));
  print_state(F(" *** failed getting Libre type"));
  return(0);
}

/*
 * get sensor status using the Abbott command
 */
boolean NFCSensorStatus(void)
{
  byte command[] = NFC_SENSOR_STATUS;

  delay(10);
  sendNFCCommand(command, sizeof(command)/sizeof(command[0]), 1);
  pollNFCUntilResponsIsReady();
  receiveNFCResponse(1);

  if ( RXBuffer[0] == 0x80 ) {
    // Response error flag 0x01 set?
    if ( (RXBuffer[2] & 0x01) == 0) {
      return(1);
    }
    else  {
      print_state(F(" *** unknown sensor status"));
    }
  }
  else {
    print_state(F(" *** error reading sensor status ")); Serial.print(RXBuffer[0], HEX);
    showNFCReturnCode(RXBuffer[0]);
  }

  print_state(F(" *** failed getting sensor status"));
  return(0);
}

/* 
 * some sanity checks and data smoothing
 */
#ifdef REMOVE_SPIKES

#define SPIKE_HEIGHT 30
boolean spikeFilterFirstRun = true;
static uint16_t lastRawGlucose;

uint16_t apply_spike_filter(uint16_t rawBG)
{
  char outString[40];

  if ( spikeFilterFirstRun ) {
    spikeFilterFirstRun = 0;
    lastRawGlucose = rawBG;
  }
  print_state(F("apply spike filter: ")); 
  sprintf(outString, "lG %d, cG: %d, delta: %d (max: %d)", lastRawGlucose, rawBG, rawBG-lastRawGlucose, (int)(SPIKE_HEIGHT*RAW2DEX));
  Serial.print(outString);
  
  if ( ((lastRawGlucose-rawBG) > ((int)(SPIKE_HEIGHT*RAW2DEX))) || ((rawBG-lastRawGlucose) > ((int)(SPIKE_HEIGHT*RAW2DEX))) ) {
    print_state(F("*** spike detected:")); 
    sprintf(outString, " cG %d, lG %d", rawBG, lastRawGlucose);
    Serial.print(outString);
    return(lastRawGlucose);
  }
  
  lastRawGlucose = rawBG;
  return(rawBG);
}

#endif /* REMOVE_SPIKES */

uint16_t bgA0 = 0;                       // calculate moving average
uint16_t bgA1 = 0;
uint16_t bgA2 = 0;
uint16_t bgA3 = 0;
uint16_t averageStep=0;

uint16_t averageBG(uint16_t current)
{
  uint16_t av;
  if (averageStep==0) {
    averageStep++;
    bgA0 = current;
    av=bgA0;
  }
  else if (averageStep==1)  {
    averageStep++;
    bgA1 = bgA0;
    bgA0 = current;
    av = (bgA0+bgA1)/2;
  }
  else if (averageStep==2)  {
    averageStep++;
    bgA2=bgA1;
    bgA1=bgA0;
    bgA0=current;
    av = (bgA2+bgA1+bgA0)/3;  
  }
  else if (averageStep==3)  {
    averageStep++;    
    bgA3=bgA2;
    bgA2=bgA1;
    bgA1=bgA0;
    bgA0=current;
    av = (bgA3+bgA2+bgA1+bgA0)/4; 
  }
  else {
    bgA3=bgA2;
    bgA2=bgA1;
    bgA1=bgA0;
    bgA0=current;
    av = (bgA3+bgA2+bgA1+bgA0+bgA0)/5;       
  }
  return av;
}

/*
 * read BG from Libre2 sensor
 */

void decodeL2FRAM(byte *RXBuf, byte *FRAM);

uint16_t readSensorFRAM_L2(boolean *readDone)
{
  uint16_t movingAverageTrend;
  uint16_t rawBG;

  NFCReadErrCnt = 0;
//  print_state(F("read L2 sensor"));
  for (int block = 0; block < 0x27 ; block++ ) {
    ReadSingleBlockReturn(block);
    for (int j = 3; j < RXBuffer[1] + 3 - 4; j++)   
      FRAM[block * 8 + j - 3] = RXBuffer[j];
  } 
  decodeL2FRAM(&RXBuffer[0], &FRAM[0]);

  Serial.print(F(" - done (")); Serial.print(NFCReadErrCnt); Serial.print(F(")"));

  crcL2Fram = calcCRC16(FRAM, 1);
  print_state(F("crc FRAM: 0x")); Serial.print(crcL2Fram, HEX);
  if ( crcL2Fram == lastCrCL2Fram ) {
    Serial.print(F(" *** dead L2 sensor"));
#ifndef USE_DEAD_SENSOR
    Serial.print(F(", skip"));
    deadLxSensor = true;
    *readDone = 0;
    return(0);
#endif
  }
  lastCrCL2Fram = crcL2Fram;

  // get current trend raw BG
  byte addrT2[16] = { 170, 80, 86, 92, 98, 104, 110, 116, 122, 128, 134, 140, 146, 152, 158, 164 };
  int sensorT = FRAM[3*8+2];
  int sensorH = FRAM[3*8+3]; 

  print_state(F("")); Serial.print("trend "); Serial.print(sensorT, HEX); Serial.print(", history "); Serial.print(sensorH, HEX);

  if ( (sensorT >= 0) && sensorT < 16 ) {
    int b0 = FRAM[addrT2[sensorT]];
    int b1 = FRAM[addrT2[sensorT]+1];

    Serial.print(F(" [")); Serial.print(b0, HEX); Serial.print(F("/")); Serial.print(b1, HEX); Serial.print(F("]"));

    rawBG = ((b1  << 8)  + (b0 & 0xFF)) & RAWBITMASK;

    Serial.print(", raw BG 0x"); Serial.print(rawBG, HEX); Serial.print("("); Serial.print(rawBG); 
    Serial.print(") -> "); Serial.print(convertRawBG2WixelScale(rawBG)/1000);
    
    Serial.print(", counter76: "); Serial.print(FRAM[76], HEX); Serial.print(" ("); Serial.print(FRAM[76]); Serial.print(")");
    Serial.print(", timer316/317: "); Serial.print(FRAM[316], HEX); Serial.print(FRAM[317], HEX);
    Serial.print("-> ");
    int t3 = (FRAM[317]<<8) | FRAM[316];
    Serial.print(t3); Serial.print(F(" (")); Serial.print(t3, HEX); Serial.print(F(")"));
  }
  else {
    print_state(F("*** error in trend pointer: ")); 
    Serial.print(sensorT, HEX);
    *readDone = 0;
    return(0);
  }

  // print trend data
  int tP = sensorT;
  int tBG;
  print_state(F("trend: "));
  for ( int k = 0 ; k < 16 ; k++ ) {
    if ( tP-- == 0 )
      tP = 15;
    int b0 = FRAM[addrT2[tP]];
    int b1 = FRAM[addrT2[tP]+1];
    tBG = ((b1  << 8)  + (b0 & 0xFF)) & RAWBITMASK;
    Serial.print(tBG); Serial.print(F(" "));
  }

#ifdef REMOVE_SPIKES
  rawBG = apply_spike_filter(rawBG);
#endif /* REMOVE_SPIKES */

//  *readDone = 1;
//  return (rawBG);

  uint16_t testBG = convertRawBG2WixelScale(rawBG)/1000;
  // valid BG?
  if ( testBG >= MIN_VALID_BG && testBG < MAX_VALID_BG ) {
    movingAverageTrend = averageBG(rawBG);
  }
  else {
    *readDone = 0;
    return(0);
  }

  print_state(F("apply moving average to ")); 
  Serial.print(rawBG); Serial.print(F(" -> ")); Serial.print(movingAverageTrend); 
  Serial.print(F(" ")); Serial.print(bgA0); Serial.print(F("/")); Serial.print(bgA1); Serial.print(F("/")); Serial.print(bgA2); 
  Serial.print(F("/")); Serial.print(bgA3);

  *readDone = 1;
  return (movingAverageTrend);
}

/*
 * scale the raw BG readings to be compatible with xDrip+ / Dexcom format
 */
unsigned long convertRawBG2WixelScale(unsigned long glucose)
{
//  print_state(F("converting raw ")); Serial.print(glucose); Serial.print(F(" to ")); Serial.print((glucose/RAW2DEX)*1000.0f); 
  return((glucose/RAW2DEX)*1000.0f);
}

/*
 * calculate CRC for data sanity check
 */
const uint16_t crc16table[256] PROGMEM =
{ 0, 4489, 8978, 12955, 17956, 22445, 25910, 29887, 35912, 40385,
  44890, 48851, 51820, 56293, 59774, 63735, 4225, 264, 13203, 8730,
  22181, 18220, 30135, 25662, 40137, 36160, 49115, 44626, 56045, 52068,
  63999, 59510, 8450, 12427, 528, 5017, 26406, 30383, 17460, 21949,
  44362, 48323, 36440, 40913, 60270, 64231, 51324, 55797, 12675, 8202,
  4753, 792, 30631, 26158, 21685, 17724, 48587, 44098, 40665, 36688,
  64495, 60006, 55549, 51572, 16900, 21389, 24854, 28831, 1056, 5545,
  10034, 14011, 52812, 57285, 60766, 64727, 34920, 39393, 43898, 47859,
  21125, 17164, 29079, 24606, 5281, 1320, 14259, 9786, 57037, 53060,
  64991, 60502, 39145, 35168, 48123, 43634, 25350, 29327, 16404, 20893,
  9506, 13483, 1584, 6073, 61262, 65223, 52316, 56789, 43370, 47331,
  35448, 39921, 29575, 25102, 20629, 16668, 13731, 9258, 5809, 1848,
  65487, 60998, 56541, 52564, 47595, 43106, 39673, 35696, 33800, 38273,
  42778, 46739, 49708, 54181, 57662, 61623, 2112, 6601, 11090, 15067,
  20068, 24557, 28022, 31999, 38025, 34048, 47003, 42514, 53933, 49956,
  61887, 57398, 6337, 2376, 15315, 10842, 24293, 20332, 32247, 27774,
  42250, 46211, 34328, 38801, 58158, 62119, 49212, 53685, 10562, 14539,
  2640, 7129, 28518, 32495, 19572, 24061, 46475, 41986, 38553, 34576,
  62383, 57894, 53437, 49460, 14787, 10314, 6865, 2904, 32743, 28270,
  23797, 19836, 50700, 55173, 58654, 62615, 32808, 37281, 41786, 45747,
  19012, 23501, 26966, 30943, 3168, 7657, 12146, 16123, 54925, 50948,
  62879, 58390, 37033, 33056, 46011, 41522, 23237, 19276, 31191, 26718,
  7393, 3432, 16371, 11898, 59150, 63111, 50204, 54677, 41258, 45219,
  33336, 37809, 27462, 31439, 18516, 23005, 11618, 15595, 3696, 8185,
  63375, 58886, 54429, 50452, 45483, 40994, 37561, 33584, 31687, 27214,
  22741, 18780, 15843, 11370, 7921, 3960
};

bool checkCRC16(void *bytes, byte type)
{
  int offset = 0;
  byte *data = (byte*) bytes;

  if      (type == 0)   offset = 0;
  else if (type == 1)   offset = 24;
  else if (type == 2)   offset = 320;
  else if (type == LIBREPRO_HEADER1)  offset = 0;
  else if (type == LIBREPRO_HEADER2)  offset = 40;
  else if (type == LIBREPRO_DATA)     offset = 80;
  uint16_t x = data[0 + offset] | (data[1 + offset] << 8);
  uint16_t crc16calculated = computeCRC16(bytes, type);
  if (crc16calculated == x) return true;
  else                      return false;
}

uint16_t calcCRC16(void *bytes, byte type)
{
  uint16_t crc16calculated = computeCRC16(bytes, type);
  return(crc16calculated );
}

uint16_t computeCRC16(void *bytes, byte type)
{
  int number_of_bytes_to_read = 0;
  int offset = 0;

  if (type == 0) {
    number_of_bytes_to_read = 24;
    offset = 0;
  }
  else if (type == 1) {
    number_of_bytes_to_read = 296;
    offset = 24;
  }
  else if (type == 2) {
    number_of_bytes_to_read = 24;
    offset = 320;
  }
  // Libre Pro header 1
  else if (type == LIBREPRO_HEADER1 ) {
    number_of_bytes_to_read = 40;
    offset = 0;
  }
  // Libre Pro header 2
  else if (type ==  LIBREPRO_HEADER2 ) {
    number_of_bytes_to_read = 32;
    offset = 40;
  }
  // unknown size for Libre Pro data segment
  else if (type == LIBREPRO_DATA ) {
    number_of_bytes_to_read = 16*8;
    offset = 72;
  }
  byte *data = (byte*) bytes;
  uint16_t crc = 0xffff;
  uint16_t crc16Table_P;
  for (int i = offset + 2; i < number_of_bytes_to_read + offset; ++i)           // first two bytes = crc16 included in data
  {
    crc16Table_P = pgm_read_word(&crc16table[(crc ^ data[i]) & 0xff]);
    crc = (uint16_t)((crc >> 8) ^ crc16Table_P);
//    crc = (uint16_t)((crc >> 8) ^ crc16table[(crc ^ data[i]) & 0xff]);
  }
  uint16_t reverseCrc = 0;
  for (int i = 0; i < 16; i++)
  {
    reverseCrc = (uint16_t)((uint16_t)(reverseCrc << 1) | (uint16_t)(crc & 1));
    crc >>= 1;
  }
  return reverseCrc;
}

/*
 * read L1 sensor
*/
uint16_t readSensorFRAM_L1(boolean *readDone)
{
  uint16_t rawGlucose = 0;  // initialise to avoid compiler warning
  byte trendPointer;
  int trendPos;

  NFCReadErrCnt = 0;
  for ( int block = 0 ; block < 0x27 ; block+=3 ) {
    if ( !ReadMultiBlockReturn(block) ) {
      *readDone = 0;
      return(0);
    }
//    Serial.print(F(" - "));
    for (int j = 3; j < RXBuffer[1] + 3 - 4; j++ ) {
      FRAM[block * 8 + j - 3] = RXBuffer[j];
//      Serial.print(F(" ")); Serial.print(FRAM[block*8+j-3], HEX);
    }
  }
  int block = 0x27;
  ReadSingleBlockReturn(block);
  for (int j = 3; j < RXBuffer[1] + 3 - 4; j++ ) {
    FRAM[block * 8 + j - 3] = RXBuffer[j];
//    Serial.print(F(" ")); Serial.print(FRAM[block*8+j-3], HEX);
  }

  bool resultH = checkCRC16(FRAM, 0);
  bool resultB = checkCRC16(FRAM, 1);
  bool resultF = checkCRC16(FRAM, 2);

  if ( resultH && resultB /* && resultF */ ) {
//    print_state(F("CRC OK, sensor data valid"));
  }
  else {
    print_state(F(" *** CRC error, sensor data invalid "));
    Serial.print(resultH); Serial.print(F(" "));
    Serial.print(resultB); Serial.print(F(" "));
    Serial.print(resultF); Serial.print(F(" "));
    *readDone = 0;
    return(0);
  }

  sensorStatus = FRAM[4];
  print_state(F("sensorStatus = ")); Serial.print(sensorStatus);

#ifndef USE_DEAD_SENSOR
  if ( (sensorStatus != 3) && (sensorStatus != 4) ) {
    print_state(F(" *** sensor expired: ")); Serial.print(sensorStatus, HEX);
    *readDone = 0;
    return(0);
  }
#endif

  trendPointer = FRAM[3*8+2];
  sensorMinutesElapse = FRAM[39*8+5]<<8 | FRAM[39*8+4];
  print_state(F("NFCRdErr/trendPointer/minutes elapse: "));
  Serial.print(NFCReadErrCnt); Serial.print(F(" ")); Serial.print(trendPointer); Serial.print(F(" ")); Serial.print(sensorMinutesElapse);

  if ( trendPointer == 0 )
    trendPointer = 0x0F;
  else
    trendPointer -= 1;
  trendPos = (3*8+4)+trendPointer*6;
  rawGlucose = FRAM[trendPos+1]<<8 | FRAM[trendPos];
  rawGlucose &= RAWBITMASK;

  Serial.print(trendPos); Serial.print(F(" ")); Serial.print(FRAM[trendPos+1]<<8, HEX); Serial.print(F(" "));
  Serial.print(FRAM[trendPos], HEX); Serial.print(F(" ")); Serial.print(rawGlucose);

  print_state(F("raw BG reading ["));
  Serial.print(rawGlucose); Serial.print(F("/")); Serial.print(convertRawBG2WixelScale(rawGlucose));
  Serial.print(F("], BatLev: ")); Serial.print(batteryPcnt); Serial.print(F("%, "));
  Serial.print(F("BatMv: ")); Serial.print(batteryMv); Serial.print(F("mV, "));
  Serial.print(F("SensLife: ")); Serial.print(sensorMinutesElapse); Serial.print(F(" min elapsed"));

  *readDone = 1;
  return (rawGlucose);
}

/*
 * read L Pro/H sensor
*/
uint16_t readSensorFRAM_LPro(boolean *readDone)
{
  uint16_t rawGlucose = 0;  // initialise to avoid compiler warning
  byte trendPointer;
  int trendPos;

  NFCReadErrCnt = 0;
  // read the first 26 blocks containing the SN and 15 min trend values
  for ( int block = 0 ; block < 27 ; block+=3 ) {
    if ( !ReadMultiBlockReturn(block) ) {
      *readDone = 0;
      return(0);
    }
//    Serial.print(F(" - "));
    for (int j = 3; j < RXBuffer[1] + 3 - 4; j++ ) {
      FRAM[block * 8 + j - 3] = RXBuffer[j];
//      Serial.print(F(" ")); Serial.print(FRAM[block*8+j-3], HEX);
    }
  }

  // for Libre Pro check only header1 and header2 as data segement is too large to read in short time
  bool resultH = checkCRC16(FRAM, LIBREPRO_HEADER1);
  bool resultB = checkCRC16(FRAM, LIBREPRO_HEADER2);

  if ( resultH && resultB ) {
    print_state(F("CRC OK, sensor data valid"));
  }
  else {
    print_state(F(" *** CRC error, Libre Pro sensor data invalid "));
    Serial.print(resultH); Serial.print(F(" "));
    Serial.print(resultB); Serial.print(F(" "));
    *readDone = 0;
    return(0);
  }

  sensorStatus = FRAM[LPRO_STATUSBYTE];
//  print_state(F("sensorStatus = ")); Serial.print(sensorStatus);
/*
  print_state(F("SN = "));
  for ( int i = 0 ; i < LPRO_SN_LENGTH ; i++ ) {
    Serial.print((char)FRAM[LPRO_SN+i]);
  }
*/
  sensorMinutesElapse = (FRAM[LPRO_SENSORMINUTES+1]<<8) | FRAM[LPRO_SENSORMINUTES];
//  print_state(F("sensorMinutes = 0x")); Serial.print(sensorMinutesElapse, HEX);
//  Serial.print(F(" = ")); Serial.print(sensorMinutesElapse);

  trendPointer = FRAM[LPRO_TRENDPOINTER];
//  print_state(F("trendPointer = 0x")); Serial.print(trendPointer, HEX);

#ifndef USE_DEAD_SENSOR
  if ( (sensorStatus != 3) && (sensorStatus != 4) ) {
    print_state(F(" *** sensor expired: 0x")); Serial.print(sensorStatus, HEX);
    deadLxSensor = true;
    *readDone = 0;
    return(0);
  }
#endif

  if ( trendPointer == 0 )
    trendPointer = 0x0F;
  else
    trendPointer -= 1;
  trendPos = LPRO_TRENDOFFSET+trendPointer*6;

  rawGlucose = (FRAM[trendPos+1]<<8 | FRAM[trendPos]) & RAWBITMASK;
/*
  print_state(F("trendPos = "));
  Serial.print(trendPos); Serial.print(F(" 0x")); Serial.print(FRAM[trendPos+1], HEX); Serial.print(F(" 0x"));
  Serial.print(FRAM[trendPos], HEX); Serial.print(F(" -> ")); Serial.print(rawGlucose, HEX);
*/
  print_state(F("raw BG reading ["));
  Serial.print(rawGlucose); Serial.print(F("/")); Serial.print(convertRawBG2WixelScale(rawGlucose));
  Serial.print(F("], BatLev: ")); Serial.print(batteryPcnt); Serial.print(F("%, "));
  Serial.print(F("BatMv: ")); Serial.print(batteryMv); Serial.print(F("mV, "));
  Serial.print(F("SensLife: ")); Serial.print(sensorMinutesElapse); Serial.print(F(" min elapsed"));

//  showLProTrend(FRAM[LPRO_TRENDPOINTER]);

  *readDone = 1;
  return (rawGlucose);
}

void showLProTrend(byte currentTrend)
{
  byte tP = currentTrend;
  int tPos;
  unsigned long cG;
  print_state(F("trend Buffer: "));
  for ( int i = 0 ; i < 16 ; i++ ) {
    if ( tP == 0 )  tP = 0x0F;
    else            tP--;
    tPos = LPRO_TRENDOFFSET+tP*6;
    cG = (FRAM[tPos+1]<<8|FRAM[tPos])&RAWBITMASK;
    Serial.print(F("\r\n 0x")); Serial.print(tP, HEX);
    Serial.print(F(" raw = ")); Serial.print(cG); Serial.print(F("/0x")); Serial.print(cG, HEX);
    Serial.print(F("/")); Serial.print(convertRawBG2WixelScale(cG));
  }
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

  print_state(F("battery level : "));
  Serial.print(batteryMv);

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

/* ************************************************ */
/* *** BLE handling ******************************* */
/* ************************************************ */

void(* resetFunc) (void) = 0;  // declare reset fuction at address 0

// init HM-1x module
int setupBLE()
{
  uint16_t i;

  //detect HM-1x baudrate, if not currently set
  print_state(F("setup HM-1x, determining baudrate"));

  for ( i = 0 ; i < (sizeof(uartBaudrateTable)/sizeof(uartBaudrateTable[0])) ; i++ ) {
    init_command_buff(&command_buff);
    print_state(F("trying ")); Serial.print(uartBaudrateTable[i]);
    uartBaudrate = uartBaudrateTable[i];
    ble_Serial.begin(uartBaudrateTable[i]);
    send_string_P(F("AT"), 500);
    if ( waitDoingServicesInterruptible(500, &got_ok, 1) ) {
      print_state(F("got OK"));
      break;
    }
  }

  if (!got_ok) {
    print_state(F("Could not detect baudrate of HM-1x, setting 9600"));
    uartBaudrate = 9600;
    resetBLE();
    return (0);
  }
  else
    got_ok = 0;

  print_state(F("baudrate set to "));
  Serial.print(uartBaudrate);
  init_command_buff(&command_buff);
  return (1);
}

// Configure the BlueTooth module with a name.
// from LimiTTer setp() moved to here
void configBLE() 
{
  send_string_P(F("AT"), 500);            // cut BLE connection to do commands

  // notify CONNECT and LOST
  send_string_P(F("AT"), 500);            // cut BLE connection to do commands
  if ( send_ble_string_P(F("AT+NOTI1")) ) {
    if ( strstr(at_answer, "+Set:1") == NULL ) {
      print_state(F("no answer, reset LimiTTer ..."));
      delay(1000);
//      waitDoingServices(2000, 1);
      resetFunc();
    }
  }
  else {
    print_state(F("command not succesfull - do a reset"));
//    waitDoingServices(2000, 1);
    delay(1000);
    resetFunc();
  }
  send_string_P(F("AT"), 500);            // cut BLE connection to do commands
  send_ble_string_P(F(ATLNAME));             // set unique LimiTTer name, max 11 (not 12) chars

  send_string_P(F("AT"), 500);            // cut BLE connection to do commands
  send_ble_string_P(F("AT+VERR?"));    // look for correct BLE module - answer schould be "HMSoft V54x"
  // to detect fake modules which starts with 115200
  send_string_P(F("AT"), 500);            // cut BLE connection to do commands
  send_ble_string_P(F("AT+ADDR?"));    // get MAC address

  send_string_P(F("AT"), 500);            // cut BLE connection to do commands
  send_ble_string_P(F("AT+RESET"));    // reset the module
  waitDoingServices(500, 1);
}

void resetBLE(void)
{
  print_state(F("HM1x power cycle, AT+RESET"));

  ble_connected = 0;

  digitalWrite(5, LOW); // Disable this for Android 4
  digitalWrite(6, LOW); // Disable this for Android 4
  digitalWrite(BLEPin, LOW); // Disable this for Android 4

  waitDoingServices(1000, 1);

  digitalWrite(BLEPin, HIGH);
  digitalWrite(5, HIGH);
  digitalWrite(6, HIGH);

  waitDoingServices(1000, 1);
  send_string_P(F("AT"), 500);            // cut BLE connection to do commands
  send_string_P(F("AT+RESET"), 500);      // try to reset the module
  waitDoingServices(1000, 1);

}

int wakeBLEUp(void)
{
  int wait_time;

  if ( cons_loop_wo_ble > 1 && cons_loop_wo_ble <= 18 ) {
    print_state(F("")); Serial.print(cons_loop_wo_ble); Serial.print(F(" loops without BLE, wait 60 s for connect"));
    wait_time = 60;
  }
  else {
    wait_time = 40;
  }

  print_state(F(" ....... wake BLE up - wait ")); Serial.print(wait_time); Serial.print(F(" s for BLE connection - "));

  ble_start_time = millis();
  ble_connected = 0;
#ifdef HW_BLE
  hwble_connected = 0;
#endif

  digitalWrite(BLEPin, HIGH);
  digitalWrite(5, HIGH);
  digitalWrite(6, HIGH);

  // wait 1 s for initial connect, time for a fast BLE connect
  waitDoingServicesInterruptible(1000, &ble_connected, 1);
  if ( ble_connected )
    print_state(F("wakeBLEUp - ble_connected set"));

#ifdef ATRESET
  if ( !ble_connected ) {
    send_ble_string_P(F("AT+RESET"));  // reset the BLE module
    waitDoingServices(500, 1);
  }
#endif

  int i;
  for ( i = 0 ; i < wait_time * 2 ; i++) {
    waitDoingServices(500, 1);
    Serial.print(F("."));
    // typical CONN/LOST time difference is 1 - 2 s, mostly happens at the beginning of loop
    // wait for this 2 s before break
    if ( ble_connected ) {
      print_state(F("connected, wait 1 s to manage CONN+LOST cyle then send packet(s)"));
      // LOST on CONN mostly after 800 - 900 ms
      waitDoingServices(1000, 1);
      break;
    }
  }

  return(ble_connected);
}

void shutBLEDown(boolean quit_ble)
{
  // cut BLE conection via AT command
  //  if ( quit_ble && ble_connected ) {
  // for connected and lost state - we want to see HM-11 answer
  if ( quit_ble ) {
    send_string_P(F("AT"), 500);
    waitDoingServices(500, 1);
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

int send_ble_string_P(const __FlashStringHelper * cmd)
//int send_ble_string(String cmd, boolean connect_state)
{
  int i, j;
  unsigned long nnow;
  boolean timeout;

  // take care of connect state?
  for ( i = 0 ; i < 3 ; i++ ) {
#ifdef SHOW_BLE
    print_state(F(" ->(")); Serial.print(cmd); Serial.print(F(") <-> "));
#endif
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
        Serial.print(F(" <-("));
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
  return (0);
}

void send_string(String cmd, int dly)
{
  last_ble_send_time = millis();
  crlf_printed = 0;
  ble_Serial.print(cmd);
#ifdef SHOW_BLE
  print_state(F(" ->(")); Serial.print(cmd); Serial.print(F(") <-> "));
#endif
  waitDoingServices(dly, 1);
}

void send_string_P(const __FlashStringHelper *cmd, int dly)
{
  last_ble_send_time = millis();
  crlf_printed = 0;
  ble_Serial.print(cmd);
#ifdef SHOW_BLE
  print_state(F(" ->(")); Serial.print(cmd); Serial.print(F(") <-> "));
#endif
  waitDoingServices(dly, 1);
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
int commandBuffIs(const __FlashStringHelper *command)
{
  unsigned char len = strlen_P((PGM_P)command);
  if ( len != command_buff.nCurReadPos )
    return (0);
  return ( memcmp_P(command_buff.commandBuffer, (PGM_P)command, len) == 0 );
}

// decode incoming serial or BLE data commands
int doCommand()
{
  // ACK packet?
  if (command_buff.commandBuffer[0] == 0x02 && command_buff.commandBuffer[1] == 0xF0 && !got_ack) {
    got_ack = 1;
    init_command_buff(&command_buff);
    return (0);
  }
  // "OK+..." answer from BLE?
  if ( commandBuffIs(F("OK")) ) {
    got_ok = 1;
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
    if ( hwble_connected ) {
      Serial.print(F("connected"));
      if ( !ble_connected ) {
        print_state(F("BLE connected, flag set by HW detection "));
        ble_connected = 1;
      }
    }
    else {
      Serial.print(F("lost"));
      if ( ble_connected ) {
        print_state(F("BLE disconnected, flag cleared by HW detection "));
        ble_connected = 0;
      }
    }
  }
}
#endif /* HW_BLE */

// detect BLE connection status via software / AT commands
boolean monitor_ble(unsigned char b)
{
  static int bi = 0;
  int i, c_found = 0;

  // print char in readable form
#ifdef SHOW_BLE
    if ( b >= ' ' && b < 128 ) {
      Serial.write(b);
    }
    else {
      Serial.print(F("0x"));
      Serial.print(b, HEX);
      Serial.print(F(" "));
    }
#endif

  // look in all 3 patterns
  for ( i = 0 ; i < 3 ; i++ ) {
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
      Serial.print( millis() - ble_start_time );
      Serial.print(F(" ms"));
      return (1);
    } else if ( b == okstr[1].pattern[6] ) {
      ble_connected = 0;
      print_state(F("+++++ BLE lost after "));
      Serial.print( millis() - ble_start_time );
      Serial.print(F(" ms"));

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
  unsigned char b = '\0';

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
  while ( ble_Serial.available() && command_buff.nCurReadPos < COMMAND_MAXLEN) {

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
      b = ble_Serial.read();
      // get all char for 1 sec after AT command from BLE
      if ( j+1 >= (int)(sizeof(at_answer)) ) {
        print_state(F(" *** BLE buffer overflow"));
        --j;
      }

      at_answer[j++] = b;
      command_buff.commandBuffer[command_buff.nCurReadPos] = b;
      command_buff.nCurReadPos++;
      monitor_ble(b);
    }

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
        nRet = doCommand();
        //re-initialise the command buffer for the next one.
        init_command_buff(&command_buff);
        // break out if we got a breaking command
        if (!nRet)
          return (nRet);
      }
    }
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
    if ( j+1 >= (int)(sizeof(at_answer)) ) {
      print_state(F(" *** BLE buffer overflow"));
      --j;
    }
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
#ifdef HW_BLE
  bleConnectMonitor();
#endif
  if (bWithProtocol)
    return (controlProtocolService());
  return (1);
}

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

char *decodeSN(byte *data)
{
  char lookupTable[32] =
  {
    '0', '1', '2', '3', '4', '5', '6', '7', '8', '9',
    'A', 'C', 'D', 'E', 'F', 'G', 'H', 'J', 'K', 'L',
    'M', 'N', 'P', 'Q', 'R', 'T', 'U', 'V', 'W', 'X',
    'Y', 'Z'
  };
  byte uuidShort[8];    // UUID from the sensor FRAM
  char binary[70];      // contains at most 8 x 8 digits in ASCII = 64
  char binS[10];        // binary byte in ASCII, 8 chars with leading ZEROs
  binS[0] = '\0';
  char bbyte[10];       // direct conversiopn result with itoa(), between 1 to 8 bytes
  int i;

//  print_state("tag info: ");
  for (int i = 2; i < 8; i++) { 
    uuidShort[i - 2] = data[i];
//    Serial.printf(" %x", uuidShort[i-2]);
  }
  uuidShort[6] = 0x00;
  uuidShort[7] = 0x00;

  // convert the raw ID to ASCII binary
  binary[0] = '\0';
  for (int i = 0; i < 8; i++)
  {
    itoa(uuidShort[i], bbyte, 2);
    int l = strlen(bbyte);
    if      (l == 1) sprintf(binS, "%s%s", "0000000", bbyte);
    else if (l == 2) sprintf(binS, "%s%s", "000000", bbyte);
    else if (l == 3) sprintf(binS, "%s%s", "00000", bbyte);
    else if (l == 4) sprintf(binS, "%s%s", "0000", bbyte);
    else if (l == 5) sprintf(binS, "%s%s", "000", bbyte);
    else if (l == 6) sprintf(binS, "%s%s", "00", bbyte);
    else if (l == 7) sprintf(binS, "%s%s", "0", bbyte);
    else if (l == 8) strcpy(binS, bbyte);
    strcat(binary, binS);
  }
  strcpy(decodedSensorSN, "0");
  char pozS[5];
  for ( i = 0; i < 10; i++)
  {
    for (int k = 0; k < 5; k++)
      pozS[k] = binary[(5 * i) + k];
    int value = (pozS[0] - '0') * 16 + (pozS[1] - '0') * 8 + (pozS[2] - '0') * 4 + (pozS[3] - '0') * 2 + (pozS[4] - '0') * 1;
    decodedSensorSN[i+1] = lookupTable[value];
  }
  decodedSensorSN[i+1] = '\0';        // append delemite
  return(&decodedSensorSN[0]);
}


boolean getNFCReading(uint16_t *ptr)
{
  boolean readSuccessful;

  // store the current wixel milliseconds so we know how long we are waiting.
  unsigned long start = millis();
  // set the return code to timeout indication, as it is the most likely outcome.
  uint16_t rawGlucose;

  memset(FRAM, 0, sizeof(FRAM));
  *ptr = 0;

  // detect if sensor is in range
  if ( !NFCGetUid(10) ) {
    print_state(F("*** non sensor"));
    *ptr = 0;
    return(0);
  }

  // determine libre sensor type
  if ( !NFCGetLtype(3) ) {
    print_state(F("*** unknown libre sensor type"));
    *ptr = 0;
    return(0);
  }
//  NFCSensorStatus();

  // while we haven't reached the delay......
  while ( (millis() - start) < 3000 ) {
    int kk = 0;
    while ( kk++ < NFC_READ_RETRY ) {
      readSuccessful = 0;
      if ( sensorVersion == LIBRE1 ) {
        rawGlucose = readSensorFRAM_L1(&readSuccessful);
      }
      else if ( sensorVersion == LIBRE2 ) {
        rawGlucose = readSensorFRAM_L2(&readSuccessful);
        if ( deadLxSensor ) {
          *ptr = 0;
          return(0);
        }
      }
      else if ( sensorVersion == LIBRE_US ) {
        print_state(F(" *** LIBRE US not supported yet, leaving"));
        *ptr = 0;
        return(0);
      }
      else if ( sensorVersion == LIBRE_PRO ) {
//        print_state(F(" *** LIBRE PRO not supported yet, leaving"));
//        *ptr = 0;
//        return(0);
        rawGlucose = readSensorFRAM_LPro(&readSuccessful);
      }
      else
        print_state(F(" *** unknown sensor type"));

      if ( readSuccessful ) {
        *ptr = rawGlucose;
        return(1);
      }
      if ( deadLxSensor == true ) break;
      print_state(F("redo readSensorFRAM_x() ..."));
      waitDoingServices(1000, 1);
    }
    *ptr = 0;
    return(0);  
  } 
  *ptr = 0;
  return (0);
}

// wait for a NFC reading and put it into Dexcom_packet
bool getBGReading(Dexcom_packet* pPkt)
{
  // set the return code to timeout indication, as it is the most likely outcome.
  uint16_t rawGlucose;

  if ( getNFCReading(&rawGlucose) ) {
    pPkt->raw = rawGlucose; 
    pPkt->ms = abs_millis();
    return (1);
  }
//  print_state(F("NFC reading not successful"));
  pPkt->raw = 0;
  pPkt->ms = 0;
  return (0);
}

//function to format and send the passed Dexom_packet.
int print_packet(Dexcom_packet* pPkt)
{
  nRawRecord msg;

  //prepare the message
  msg.size = sizeof(msg);
  msg.cmd_code = 0x00;
  msg.raw = convertRawBG2WixelScale(pPkt->raw);
  msg.filtered = msg.raw;
  msg.dex_battery = 214;  // simulate good dexcom transmitter battery
  msg.my_battery = batteryPcnt;
  msg.dex_src_id = dex_tx_id;
  msg.delay = abs_millis() - pPkt->ms;
  msg.function = DEXBRIDGE_PROTO_LEVEL; // basic functionality, data packet (with ack), TXID packet, beacon packet (also TXID ack).

  print_state(F("sending BG ")); Serial.print(msg.raw/1000); Serial.print(F(", -")); Serial.print(msg.delay/1000); Serial.print(F(" s"));

  if ( !ble_connected )
    return (0);

  send_data( (unsigned char *)&msg, msg.size);

  return (1);
}

// print timestamp and current status/action
void print_state(const __FlashStringHelper *str)
{
  unsigned long ms = abs_millis();
  unsigned long val;
  
  Serial.print(F("\r\n["));
  if ( ble_connected )  Serial.print(F("B]["));
  else                  Serial.print(F(".]["));
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
      Serial.print(F("NORMAL "));
      normal_loop_count++;
      break;
    case ST_RESEND:
      print_state(F(" ### loop ")); Serial.print(++loop_count); Serial.print(F(" === "));
      Serial.print(F("1ST_RESEND "));
      firstret_loop_count++;
      break;
    case ND_RESEND:
      print_state(F(" ### loop ")); Serial.print(++loop_count); Serial.print(F(" === "));
      Serial.print(F("2ND_RESEND "));
      secondret_loop_count++;
      break;
    default:
      print_state(F(" !!! unknown state "));
      Serial.print(loop_state);
      Serial.print(F(", set to NORMAL "));
      loop_state = NORMAL;
      break;
  }
  Serial.print(F(LB_VERSION)); Serial.print(F(LB_MINOR_VERSION)); Serial.print(F(" ")); Serial.print(F(LB_DATETIME));

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
}

// main processing loop
void loop(void)
{
  int i;
  int j;
  boolean resendPktFlag;
#ifdef TRANSFER_LIVETIME
  unsigned long last_raw = 0;
#endif

//  if ( precompTestFunc() )
//    print_state(F(" *** precomp called!"));

  loop_start_time = millis();         // log the current time
  init_command_buff(&command_buff);   //initialise the command buffer
  waitDoingServices(2000, 1);         // safety wait
  print_start_message();

#ifdef USE_BLE
  i = 0;
  // Open the UART and initialize HM-1x module
  while ( !setupBLE() && i++ < 3 );
  waitDoingServices(2000, 1); // wait for possible OK+LOST
#endif

#ifdef USE_BLE
  configBLE();                // configure the BLE module
#endif
  configNFC();                // initialize and configure the NFC module

#ifdef USE_BLE
  // wait for a BLE connection to xDrip 
  print_state(F(" ..... initial wake up, waiting 40s fixed for BLE, time for a xDrip+ BT scan if needed"));
#ifdef SHORT_START
  waitDoingServicesInterruptible(40000, &ble_connected, 1);
#else
  waitDoingServices(40000, 1);
#endif /* SHORT_START */
#endif /* USE_BLE */

  Pkts.read = 0;                      // initialize empty queue
  Pkts.write = 0;
  queuesendtime = 2500L;              // calculate max send time if queue is completely full
  queuesendtime *= (DXQUEUESIZE + 1); // assuming one entry will cost 1 s
  loop_state = NORMAL;
  next_normal_wakeup = CYCLE_SEC;
  resendPktFlag = 0;
  print_state(F("entering main loop next_normal_wakeup = ")); Serial.print(next_normal_wakeup);

  while ( 1 )
  {
    // check for low battery and go to sleep here if wrong, only come back when voltage rises
    check_battery();    
    // recalibrate PWR DWN timer every 30 m, loop duration is 5 m
    if ( (normal_loop_count % 6) == 0 ) {  // every 5 loops
      print_state(F("calibrate PWR DOWN sleep timer (2048 ms) ..."));
      calibrate(2048);
    }

    // *********** NFC ***************************************
    // get BG reading, not in case of last tranfer failed
    byte zz = 0;
    boolean readResult = 0;
    if ( loop_state == NORMAL ) {
      for ( zz = 0 ; zz < NFC_READ_RETRY ; zz++ ) {
        wakeNFCUp();
        deadLxSensor = false;
        readResult = getBGReading(&Pkts.buffer[Pkts.write]);
        shutNFCDown();
        if ( readResult || deadLxSensor ) {
//          print_state(F("readresult = ")); Serial.print(readResult); Serial.print(F(", dead sensor = ")); Serial.print(deadLxSensor);
          break;
        }
        print_state(F("NFC read error, wait 3 s"));
        waitDoingServices(3000, 1);
      } 

      if ( readResult && !deadLxSensor ) {
        print_state(F("got packet with raw BG ")); Serial.print(Pkts.buffer[Pkts.write].raw); 
        Serial.print(F(" at ")); Serial.print(Pkts.buffer[Pkts.write].ms/1000);
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
      }
    }

    // ****************** BLE *******************************

    // wake up BLE module, but only if sensor in range
    int ble_found_during_wakeBLEUp = 0;
    if ( (loop_count > 0) && readResult ) {
      ble_found_during_wakeBLEUp = wakeBLEUp();  // switch on BLE, wait 40 s for connect
    }

    // ******************* handling **************************

    // queue not empty, BLE was connected/disconnected after wakeup, then wait additional 20 s for BLE reconnect
    unsigned long nnow = millis();
    if ( Pkts.read != Pkts.write ) { // if we have a packet to send
      if ( ble_found_during_wakeBLEUp ) {
        while (!ble_connected && ((millis() - nnow) < 20000) ) {
          print_state(F("packet waiting for ble connect"));
          if ( waitDoingServicesInterruptible(10000, &ble_connected, 1) ) {
            print_state(F("connected, wait 2 s before sending the packet"));
            waitDoingServices(2000, 1);
          }
        }
      }
      // we got a connection, so send pending packets now - at most for <time> after the last packet was received
      // wait enough time to empty the complete queue!
      nnow = millis();
      while ((Pkts.read != Pkts.write) && ble_connected && ((millis() - nnow) < queuesendtime )) {
        got_ack = 0;

#ifndef USE_LIMITTER_PROTOCOL
        print_state(F("send packet at position ")); Serial.print(Pkts.read);
        print_packet(&Pkts.buffer[Pkts.read]);
#endif /* USE_LIMITTER_PROTOCOL */

#ifdef TRANSFER_LIVETIME
        last_raw = Pkts.buffer[Pkts.read].raw;
#endif

#ifndef USE_LIMITTER_PROTOCOL
        // wait 10 s for ack
        for ( j = 0 ; j < 10 ; j++ ) {
          waitDoingServicesInterruptible(1000, &got_ack, 1);
          if ( !ble_connected ) {
            print_state(F("connection lost during wait for ack, go to sleep"));
            break;
          }
        }
#endif /* USE_LIMITTER_PROTOCOL */

        if ( got_ack ) {
          print_state(F("got ack"));
          Serial.print(F(" for read position ")); Serial.print(Pkts.read); Serial.print(F(" while write is "));
          if ( ++Pkts.read >= DXQUEUESIZE )
            Pkts.read = 0;     //increment read position since we got an ack for the last package
          Serial.print(Pkts.write); Serial.print(F(", incrementing read to ")); Serial.print(Pkts.read);
          resendPktFlag = 0;
        }
        else {
          if ( !resendPktFlag ) {
            print_state(F("no ack received, try again"));
            resendPktFlag = 1;         // try to resend packet one time
          }
          else {
            print_state(F("pkt resent, no ack received, try again next wakeup"));
            resendPktFlag = 0;
            break;
          }
        }
      } /* while (send all packets avaibale ) */
#ifdef TRANSFER_LIVETIME
        // transfer the sensor livetime very hour with the last successful tranmitted BG reading
#ifndef USE_LIMITTER_PROTOCOL
      if ( ((normal_loop_count % 12) == 0) && (last_raw != 0) )
      {
        if ( got_ack ) {
#endif /* USE_LIMITTER_PROTOCOL */
          String packet = "";
          // build a LimiTTer string
          packet = String(convertRawBG2WixelScale(last_raw)); packet += ' '; packet += "216"; packet += ' ';
          packet += String(batteryPcnt); packet += ' '; 
          packet += String(sensorMinutesElapse);
          print_state(F("send sensor lifetime in LimiTTer ASCII format: ["));
          Serial.print(packet); Serial.print(F("]")); 
          send_string(packet, 500);
          waitDoingServices(1000, 1);
          last_raw = 0;
#ifndef USE_LIMITTER_PROTOCOL
        }
      }
#endif /* USE_LIMITTER_PROTOCOL */
#endif

      // did we send all queued readings?
      if ( loop_count > 0 ) {
        if ( Pkts.read != Pkts.write ) {
          switch ( loop_state ) {
            case NORMAL:    loop_state = ST_RESEND; break;
            case ST_RESEND: loop_state = ND_RESEND; break;
            case ND_RESEND: loop_state = NORMAL;    break;
            default:        loop_state = NORMAL;    break;
          }
        }
        else {
          // packet queue enpty, done
          loop_state = NORMAL;
          print_state(F("queue empty, change loop state to ")); Serial.print(loop_state);
        }
      }
#ifdef USE_LIMITTER_PROTOCOL
      loop_state = NORMAL;
#endif
    } /* if ( queue not empty */
    else {
      print_state(F(" *** no data to send"));
    }

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
      case NORMAL:    var_sleepTime = (((next_normal_wakeup * 1000) - abs_millis()) / 1000);  break;
      case ST_RESEND: var_sleepTime = 30;                                                     break;
      case ND_RESEND: var_sleepTime = 30;                                                     break;
      default:
        print_state(F("loop_state error, setting to NORMAL (was ")); Serial.print(loop_state); Serial.print(F(")"));
        loop_state = NORMAL;
        break;

    }

    if ( var_sleepTime > CYCLE_SEC )  {
      var_sleepTime = CYCLE_SEC;
    }

    Serial.print(F("\r\n *** next_normal_wakeup ")); Serial.print(next_normal_wakeup); Serial.print(F(" s, "));
    Serial.print(F(" current time ")); Serial.print(abs_millis() / 1000); Serial.print(F(" s"));
    Serial.print(F(", var_sleepTime set to ")); Serial.print(var_sleepTime);

    print_statistics1();

    shutBLEDown(1);          // cut BLE connection and switch HM-1x OFF

    if ( var_sleepTime > 320 ) // test possible overflow
      var_sleepTime = 256;

    print_state(F(" === goToSleep for [")); Serial.print(var_sleepTime);
    Serial.print(F("], calibrate call para of goToSleep() with ["));
    Serial.print(calibv); Serial.print(F("] to ")); Serial.print(((var_sleepTime * calibv) + 1) / 1);
    goToSleep(((var_sleepTime * calibv) + 1) / 1);

    // ============================ program restart after sleep ========================

    arduino_start_time = millis();

    // waking up, power on BLE, initialze NFC
    // count programm run time up
    prg_run_time += (loop_time + (var_sleepTime * 1000)) / 1000;
    loop_start_time = millis();       // set start point for loop

    // calc time for next wake up, adjust to 300 framing
    long pgrt,  pgrtfrac;
    if ( loop_state == NORMAL ) {
      pgrt = prg_run_time + CYCLE_SEC;
      pgrtfrac = (prg_run_time + CYCLE_SEC) % CYCLE_SEC;
      if ( pgrtfrac > CYCLE_SEC/2 ) {
        pgrtfrac -= CYCLE_SEC;
      }
      next_normal_wakeup = pgrt - pgrtfrac;
    }

    init_command_buff(&command_buff); // no BLE connection
    print_statistics2();
    // wake up Arduino
    wakeUp();
    waitDoingServices(250, 1);
  }
}


