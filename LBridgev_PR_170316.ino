/* 
   It scans the Freestyle Libre Sensor every 5 minutes
   and sends the data to the xDrip Android app. You can
   see the data in the serial monitor of Arduino IDE, too.
   If you want another scan interval, simply change the
   sleepTime value. To work with Android 4 you have to disable
   all lines containing a "for Android 4" comment and set the
   sleep time to 36.
     
   This sketch is based on a sample sketch for the BM019 module
   from Solutions Cubed.

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
 * VWI, V 0.1, 03/2017
 * LBridge, a LimiTTer with xBridge Protocol extension and power saving mods. This sketch is based on LimiTTer code (JoernL) 
 * for NFC reading and sleep mode handling and a port of the xBridge2 protocol with queueing from savek-cc.
 * Thanks to JoernL and savek-cc!
 * 
 * Hardwaresource in xDrip has to be set to "LimiTTer". Please use an xDrip+ version >= nightly build 2107/02/09
 * 
 * The xBridge2 protocol is used to send BG readings to xDrip+. In case of failure it queues up not sended BG readings 
 * with the correct timestamp and send them within the next BLE connection. There should be no missed BG readings
 * when LimiTTer is worn.
 */

#include <SPI.h>
#include <SoftwareSerial.h>
#include <avr/sleep.h> 
#include <avr/power.h>
#include <avr/wdt.h>
//#include "xbridge_libre.h"

#define LNAME F("AT+NAMELBri0315")

/* ************************************************ */
/* *** config #DEFINES *** */
/* ************************************************ */

#define N_SHOW_LIMITTER       // show original Limitter output
#define USE_DEAD_SENSOR       // we can test with a dead sensor
#define N_SEND_ERROR_COUNTERS // send error counter values via BLE
#define N_HW_BLE              // detect BLE conn status with system LED pin connected to arduino pin 2
#define N_DYNAMIC_TXID        // get TXID automatically
#define N_AUTOCAL_WDT         // code for auto calibrating WDT timing
#define N_XB_NEW              // send 15 min trend

/* ********************************************* */
/* ********* LimiTTer stuff ******************** */
/* ********************************************* */

#define MIN_V 3450 // battery empty level
#define MAX_V 4050 // battery full level

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

// sleeptime in multipliers of 8 s
int sleepTime = 32;

int noDiffCount = 0;
int sensorMinutesElapse;
float lastGlucose;
float trend[16];

SoftwareSerial ble_Serial(5, 6); // RX | TX

/* ************************************************* */
/* *** VWI control stuff *** */
/* ************************************************ */

// global error counters
unsigned long loop_count = 0;          // of main loop
unsigned long ble_connect_errors = 0;  // no BLE connect after 40 s wait time
unsigned long nfc_read_errors = 0;     // e. g. no sensor in range 
unsigned long nfc_scan_count = 0;      // how many scans?
unsigned long nfc_inv_cmd_count = 0;   // how much SetInventroy commands?

unsigned long nfc_prot_set_cmd_errors = 0;
unsigned long nfc_inv_cmd_errors = 0;
unsigned long nfc_read_mem_errors = 0;
unsigned long resend_pkts_events = 0;
unsigned long queue_events = 0;

#ifdef SEND_ERROR_COUNTERS
static boolean send_error_counters_via_ble = 0;
#endif

static boolean show_ble = 1;        // what is shown in serial monitor
static boolean show_state = 1;      // show print_state() infos, disabled ftm  

boolean sensor_oor;                 // sensor out of range
int ble_answer;                     // char from BLE reeived?
boolean ble_lost_processing = 1;    // send AT+RESET after OK+LOST?

// absolute progam run time in s counting also sleep() phase
unsigned long prg_run_time = 0;     // in sec

unsigned long loop_start_time;      // loop starting time in ms
unsigned long loop_time;            // curretn loop duration in ms
unsigned long ble_start_time = 0;   // time when BLE starts ms
unsigned long ble_connect_time = 0; // time up to BLE connection in ms

float avg_sleepTime = 0;            // average sleep time
float bleTime = 0;                  // average arduino+BLE time
float bleNFCTime = 0;

/* ************************************************************* */
/* *** code ported form xBridge2.c *** */
/* ************************************************************* */

// defines the xBridge protocol functional level.  Sent in each packet as the last byte.
#define DEXBRIDGE_PROTO_LEVEL (0x01)

static volatile boolean do_sleep = 0;     // indicates we should go to sleep between packets
static volatile boolean got_ack = 0;      // indicates if we got an ack during the last do_services.
static volatile boolean dex_tx_id_set;    // indicates if the Dexcom Transmitter id (settings.dex_tx_id) has been set.  Set in doServices.
static volatile boolean ble_connected;    // bit indicating the BLE module is connected to the phone.  Prevents us from sending data without this.
#ifdef HW_BLE
static volatile boolean hwble_connected;  // bit indicating the BLE module is connected to the phone.  Prevents us from sending data without this.
#endif

static volatile boolean got_ok;           // flag indicating we got OK from the HM-1x

static volatile unsigned long pkt_time = 0;
static volatile unsigned long abs_pkt_time = 0;
static volatile unsigned long last_abs_pkt_time = 0;

//define the maximum command string length for USB commands.
#define COMMAND_MAXLEN 40

//structure of a USB command
typedef struct _command_buff
{
  unsigned char commandBuffer[COMMAND_MAXLEN];
  unsigned char nCurReadPos;
} t_command_buff;

static t_command_buff command_buff;

typedef struct _Dexcom_packet
{
  unsigned long raw;
  unsigned long ms;
} Dexcom_packet;

#define DXQUEUESIZE 36 // 3 h of queue

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
unsigned long uart_baudrate[9] = {9600L,19200L,38400L,57600L,115200L,4800,2400,1200,230400L};

#ifdef XB_NEW
_QuarterPacket1 qpkt1;
_QuarterPacket2 qpkt2;
#endif

/* ********************************************* */
/*  help functions */
/* *********************************************** */

// get free mem available
extern unsigned int __bss_end;
extern unsigned int __heap_start;
extern void *__brkval;
int freeMemory() {
  int free_memory;
  
  if((int)__brkval == 0)
    free_memory = ((int)&free_memory) - ((int)&__bss_end);
  else
    free_memory = ((int)&free_memory) - ((int)__brkval);
  
  return free_memory;
}

// millis() since program start, Arduino millis() are not counting when in sleep mode
unsigned long abs_millis(void)
{
  return(prg_run_time*1000 + (millis() - loop_start_time));
}

/* **************************************************************** */
/* modified LimiTTer code */
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
}

/* *********************************************************** */
/* LimiTTer code, only small modifications */
/* *********************************************************** */

void SetProtocol_Command() {
unsigned long ct;

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
  while( RXBuffer[0] != 8 )
    {
    RXBuffer[0] = SPI.transfer(0x03);  // Write 3 until
    RXBuffer[0] = RXBuffer[0] & 0x08;  // bit 3 is set
    if ( (millis() - ct) > 1000) {
      print_state(F(" - poll SetProtocol_Command not successfull"));
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
    print_state(F(" - Protocol Set Command OK"));
    NFCReady = 1; // NFC is ready
    }
  else
    {
    print_state(F(" - Protocol Set Command FAIL"));
    NFCReady = 0; // NFC not ready
    nfc_prot_set_cmd_errors++;
    }
}

void Inventory_Command() {
unsigned long ct;
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
  while(RXBuffer[0] != 8)
    {
    RXBuffer[0] = SPI.transfer(0x03);  // Write 3 until
    RXBuffer[0] = RXBuffer[0] & 0x08;  // bit 3 is set
    if ( (millis() - ct) > 1000) {
      print_state(F(" - poll Inventory_Command not successfull"));
      break;
    }
    }
  digitalWrite(SSPin, HIGH);
  delay(1);

  digitalWrite(SSPin, LOW);
  SPI.transfer(0x02);   // SPI control byte for read         
  RXBuffer[0] = SPI.transfer(0);  // response code
  RXBuffer[1] = SPI.transfer(0);  // length of data
  for (byte i=0;i<RXBuffer[1];i++)      
      RXBuffer[i+2]=SPI.transfer(0);  // data
  digitalWrite(SSPin, HIGH);
  delay(1);

  if (RXBuffer[0] == 128)  // is response code good?
    {
    print_state(F(" - Sensor in range ... OK"));
    NFCReady = 2;
    sensor_oor = 0;
    }
  else
    {
    print_state(F(" - Sensor out of range"));
    NFCReady = 1;
    nfc_inv_cmd_errors++;
    sensor_oor = 1; 
    }
 }
 
float Read_Memory() {
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
  
 for ( int b = 3; b < 16; b++) {
 
  digitalWrite(SSPin, LOW);
  SPI.transfer(0x00);  // SPI control byte to send command to CR95HF
  SPI.transfer(0x04);  // Send Receive CR95HF command
  SPI.transfer(0x03);  // length of data that follows
  SPI.transfer(0x02);  // request Flags byte
  SPI.transfer(0x20);  // Read Single Block command for ISO/IEC 15693
  SPI.transfer(b);  // memory block address
  digitalWrite(SSPin, HIGH);
  delay(1);

  ct = millis();
  digitalWrite(SSPin, LOW);
  while(RXBuffer[0] != 8)
    {
    RXBuffer[0] = SPI.transfer(0x03);  // Write 3 until
    RXBuffer[0] = RXBuffer[0] & 0x08;  // bit 3 is set
    if ( (millis() - ct) > 1000) {
      print_state(F(" - poll Read_Memory"));
      break;
    }
    }
  digitalWrite(SSPin, HIGH);
  delay(1);

  digitalWrite(SSPin, LOW);
  SPI.transfer(0x02);   // SPI control byte for read         
  RXBuffer[0] = SPI.transfer(0);  // response code
  RXBuffer[1] = SPI.transfer(0);  // length of data
 for (byte i=0;i<RXBuffer[1];i++)
   RXBuffer[i+2]=SPI.transfer(0);  // data
   
  digitalWrite(SSPin, HIGH);
  delay(1);
  
 for (int i = 0; i < 8; i++)
   oneBlock[i] = RXBuffer[i+3];

  char str[24];
  unsigned char * pin = oneBlock;
  const char * hex = "0123456789ABCDEF";
  char * pout = str;
  for(; pin < oneBlock+8; pout+=2, pin++) {
      pout[0] = hex[(*pin>>4) & 0xF];
      pout[1] = hex[ *pin     & 0xF];
  }
  pout[0] = 0;
#ifdef SHOW_LIMITTER
  Serial.println(str);
#endif
  trendValues += str;
 }

 digitalWrite(SSPin, LOW);
  SPI.transfer(0x00);  // SPI control byte to send command to CR95HF
  SPI.transfer(0x04);  // Send Receive CR95HF command
  SPI.transfer(0x03);  // length of data that follows
  SPI.transfer(0x02);  // request Flags byte
  SPI.transfer(0x20);  // Read Single Block command for ISO/IEC 15693
  SPI.transfer(39);  // memory block address
  digitalWrite(SSPin, HIGH);
  delay(1);
 
  digitalWrite(SSPin, LOW);
  while(RXBuffer[0] != 8)
    {
    RXBuffer[0] = SPI.transfer(0x03);  // Write 3 until
    RXBuffer[0] = RXBuffer[0] & 0x08;  // bit 3 is set
    }
  digitalWrite(SSPin, HIGH);
  delay(1);

  digitalWrite(SSPin, LOW);
  SPI.transfer(0x02);   // SPI control byte for read         
  RXBuffer[0] = SPI.transfer(0);  // response code
  RXBuffer[1] = SPI.transfer(0);  // length of data
 for (byte i=0;i<RXBuffer[1];i++)
   RXBuffer[i+2]=SPI.transfer(0);  // data
   
  digitalWrite(SSPin, HIGH);
  delay(1);
  
 for (int i = 0; i < 8; i++)
   oneBlock[i] = RXBuffer[i+3];
    
  char str[24];
  unsigned char * pin = oneBlock;
  const char * hex = "0123456789ABCDEF";
  char * pout = str;
  for(; pin < oneBlock+8; pout+=2, pin++) {
      pout[0] = hex[(*pin>>4) & 0xF];
      pout[1] = hex[ *pin     & 0xF];
  }
  pout[0] = 0;

#ifdef SHOW_LIMITTER
  Serial.println(str);
#endif

  elapsedMinutes += str;
    
  if (RXBuffer[0] == 128) // is response code good?
    {
      hexMinutes = elapsedMinutes.substring(10,12) + elapsedMinutes.substring(8,10);
      hexPointer = trendValues.substring(4,6);
      sensorMinutesElapse = strtoul(hexMinutes.c_str(), NULL, 16);
      glucosePointer = strtoul(hexPointer.c_str(), NULL, 16);

#ifdef SHOW_LIMITTER             
      Serial.println(F(""));
      Serial.print(F("Glucose pointer: "));
      Serial.print(glucosePointer);
      Serial.println(F(""));
#endif      
      int ii = 0;
      for (int i=8; i<=200; i+=12) {
        if (glucosePointer == ii)
        {
          if (glucosePointer == 0)
          {
            String trendNow = trendValues.substring(190,192) + trendValues.substring(188,190);
            String trendOne = trendValues.substring(178,180) + trendValues.substring(176,178);
            String trendTwo = trendValues.substring(166,168) + trendValues.substring(164,166);
            currentGlucose = Glucose_Reading(strtoul(trendNow.c_str(), NULL ,16));
            trendOneGlucose = Glucose_Reading(strtoul(trendOne.c_str(), NULL ,16));
            trendTwoGlucose = Glucose_Reading(strtoul(trendTwo.c_str(), NULL ,16));

            if (FirstRun == 1)
               lastGlucose = currentGlucose;
       
            if (((lastGlucose - currentGlucose) > 50) || ((currentGlucose - lastGlucose) > 50))
            {
               if (((lastGlucose - trendOneGlucose) > 50) || ((trendOneGlucose - lastGlucose) > 50))
                  currentGlucose = trendTwoGlucose;
               else
                  currentGlucose = trendOneGlucose;
            }
          }
          else if (glucosePointer == 1)
          {
            String trendNow = trendValues.substring(i-10,i-8) + trendValues.substring(i-12,i-10);
            String trendOne = trendValues.substring(190,192) + trendValues.substring(188,190);
            String trendTwo = trendValues.substring(178,180) + trendValues.substring(176,178);
            currentGlucose = Glucose_Reading(strtoul(trendNow.c_str(), NULL ,16));
            trendOneGlucose = Glucose_Reading(strtoul(trendOne.c_str(), NULL ,16));
            trendTwoGlucose = Glucose_Reading(strtoul(trendTwo.c_str(), NULL ,16));

            if (FirstRun == 1)
               lastGlucose = currentGlucose;
               
            if (((lastGlucose - currentGlucose) > 50) || ((currentGlucose - lastGlucose) > 50))
            {
               if (((lastGlucose - trendOneGlucose) > 50) || ((trendOneGlucose - lastGlucose) > 50))
                  currentGlucose = trendTwoGlucose;
               else
                  currentGlucose = trendOneGlucose;
            }
          }
          else
          {
            String trendNow = trendValues.substring(i-10,i-8) + trendValues.substring(i-12,i-10);
            String trendOne = trendValues.substring(i-22,i-20) + trendValues.substring(i-24,i-22);
            String trendTwo = trendValues.substring(i-34,i-32) + trendValues.substring(i-36,i-34);
            currentGlucose = Glucose_Reading(strtoul(trendNow.c_str(), NULL ,16));
            trendOneGlucose = Glucose_Reading(strtoul(trendOne.c_str(), NULL ,16));
            trendTwoGlucose = Glucose_Reading(strtoul(trendTwo.c_str(), NULL ,16));
            

            if (FirstRun == 1)
               lastGlucose = currentGlucose;
               
            if (((lastGlucose - currentGlucose) > 50) || ((currentGlucose - lastGlucose) > 50))
            {
               if (((lastGlucose - trendOneGlucose) > 50) || ((trendOneGlucose - lastGlucose) > 50))
                  currentGlucose = trendTwoGlucose;
               else
                  currentGlucose = trendOneGlucose;
            }
          }
        }  

        ii++;
      }
     
     for (int i=8, j=0; i<200; i+=12,j++) {
          String t = trendValues.substring(i+2,i+4) + trendValues.substring(i,i+2);
          trend[j] = Glucose_Reading(strtoul(t.c_str(), NULL ,16));
       }

    for (int i=0; i<16; i++)
    {
      if (((lastGlucose - trend[i]) > 50) || ((trend[i] - lastGlucose) > 50)) // invalid trend check
         continue;
      else
      {
         validTrend[validTrendCounter] = trend[i];
         validTrendCounter++;
      }
    }

    if (validTrendCounter > 0)
    { 
      for (int i=0; i < validTrendCounter; i++)
         averageGlucose += validTrend[i];
         
      averageGlucose = averageGlucose / validTrendCounter;
      
      if (((lastGlucose - currentGlucose) > 50) || ((currentGlucose - lastGlucose) > 50))
         shownGlucose = averageGlucose; // If currentGlucose is still invalid take the average value
      else
         shownGlucose = currentGlucose; // All went well. Take and show the current value
    }
    else
      shownGlucose = currentGlucose; // If all is going wrong, nevertheless take and show a value 

    if ((lastGlucose == currentGlucose) && (sensorMinutesElapse > 21000)) // Expired sensor check
      noDiffCount++;

    if (lastGlucose != currentGlucose) // Reset the counter
      noDiffCount = 0;

    if (currentGlucose != 0)
      lastGlucose = currentGlucose; 

    NFCReady = 2;
    FirstRun = 0;

    print_state(F(" - bg reading "));
    Serial.print(shownGlucose);
    Serial.print(F(", BatLev: "));
    Serial.print(batteryPcnt);
    Serial.print(F("%, "));
    Serial.print(F("BatMv: "));
    Serial.print(batteryMv);
    Serial.print(F("mV, "));
    Serial.print(F("SensLife: "));
    Serial.print(sensorMinutesElapse);
    Serial.print(F(" min elapsed"));

    print_state(F(" - 15 minutes-trend: "));
    for (int i=0; i<16; i++) {
      Serial.print(trend[i]);
      Serial.print(F(" "));
      if ( i == 8 )
        Serial.println(F(""));
    }

#ifdef XB_NEW
    int i;
    qpkt1.size = 19;
    qpkt1.cmd_code = 0x02;
    qpkt1.sub_code = 0x04;
    for ( i = 0 ; i < 8 ; i++ )
      qpkt1.trend[i] = trend[i];
    qpkt2.size = 19;
    qpkt2.cmd_code = 0x02;
    qpkt2.sub_code = 0x05;
    for ( i = 8 ; i < 16 ; i++ )qpkt
    
      qpkt2.trend[i-8] = trend[i];
#endif /* XB_NEW */

#ifdef USE_DEAD_SENSOR
    // we do support tests with expired sensors!
    noDiffCount = 0;
#endif

    if (noDiffCount > 5)
      return 0;
    else  
      return shownGlucose;
    }
  else
    {
    Serial.print(F("Read Memory Block Command FAIL"));
    NFCReady = 0;
    nfc_read_mem_errors++;
    }
    return(0);  // return added to avoid compiler warning
 }

float Glucose_Reading(unsigned int val) {
        int bitmask = 0x0FFF;
        return ((val & bitmask) / 8.5);
}

/* ******************************************* */
/* *** Vcc and sleep *** */
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
  while (bit_is_set(ADCSRA,ADSC)); // measuring
 
  uint8_t low  = ADCL; // must read ADCL first - it then locks ADCH  
  uint8_t high = ADCH; // unlocks both
 
  batteryMv = (high<<8) | low;
 
  batteryMv = 1125300L / batteryMv; // Calculate Vcc (in mV); 1125300 = 1.1*1023*1000
  int batteryLevel = min(map(batteryMv, MIN_V, MAX_V, 0, 100), 100); // Convert voltage to percentage
  return batteryLevel;
}

void goToSleep(const byte interval, int time) {
  // say how long we want to sleep in absolute
  print_state(F(" - go to sleep for "));
  Serial.print((time)*8);
  Serial.print(F(" s - "));
  waitDoingServices(100, 1);
  for (int i=0; i<time; i++) {
    MCUSR = 0;                         
    WDTCSR |= 0b00011000;           
    WDTCSR =  0b01000000 | interval; 
    set_sleep_mode (SLEEP_MODE_PWR_DOWN);
    sleep_enable();
    sleep_cpu();           
  } 
}
ISR(WDT_vect) 
 {
 wdt_disable(); 
 }

void lowBatterySleep() {

//  shutNFCDown();
  shutBLEDown(0);

  print_state(F("Battery low! LEVEL: "));
  Serial.print(batteryPcnt);
  Serial.print(F("%"));
  waitDoingServices(100, 1);

 // Switch LED on and then off shortly
  for (int i=0; i<10; i++) {
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
/* *** and http://forum.arduino.cc/index.php?topic=49549.0 *** /
/* ************************************************ */

#ifdef AUTOCAL_WDT

long timeSleep = 0;  // total time due to sleep
float calibv = 0.93; // ratio of real clock with WDT clock
volatile byte isrcalled = 0;  // WDT vector flag

// Internal function: Start watchdog timer
// byte psVal - Prescale mask
void WDT_On (byte psVal)
{
 // prepare timed sequence first
 byte ps = (psVal | (1<<WDIE)) & ~(1<<WDE);
 cli();
 wdt_reset();
 /* Clear WDRF in MCUSR */
 MCUSR &= ~(1<<WDRF);
 // start timed sequence
 WDTCSR |= (1<<WDCE) | (1<<WDE);
 // set new watchdog timeout value
 WDTCSR = ps;
 sei();
}

// Internal function.  Stop watchdog timer
void WDT_Off() {
 cli();
 wdt_reset();
 /* Clear WDRF in MCUSR */
 MCUSR &= ~(1<<WDRF);
 /* Write logical one to WDCE and WDE */
 /* Keep old prescaler setting to prevent unintentional time-out */
 WDTCSR |= (1<<WDCE) | (1<<WDE);
 /* Turn off WDT */
 WDTCSR = 0x00;
 sei();
}

// Calibrate watchdog timer with millis() timer(timer0)
void calibrate() {
 // timer0 continues to run in idle sleep mode
 set_sleep_mode(SLEEP_MODE_IDLE); 
 long tt1=millis();
 doSleep(256);
 long tt2=millis();
 calibv = 256.0/(tt2-tt1);
}

// Estimated millis is real clock + calibrated sleep time
long estMillis() {
 return millis()+timeSleep;
}

// Delay function
void sleepCPU_delay(long sleepTime) {
 ADCSRA &= ~(1<<ADEN);  // adc off
 PRR = 0xEF; // modules off

 set_sleep_mode(SLEEP_MODE_PWR_DOWN);
 int trem = doSleep(sleepTime*calibv);
 timeSleep += (sleepTime-trem);

 PRR = 0x00; //modules on
 ADCSRA |= (1<<ADEN);  // adc on
}

// internal function.  
int doSleep(long timeRem) {
 byte WDTps = 9;  // WDT Prescaler value, 9 = 8192ms

 isrcalled = 0;
 sleep_enable();
 while(timeRem > 0) {
   //work out next prescale unit to use
   while ((0x10<<WDTps) > timeRem && WDTps > 0) {
     WDTps--;
   }
   // send prescaler mask to WDT_On
   WDT_On((WDTps & 0x08 ? (1<<WDP3) : 0x00) | (WDTps & 0x07));
   isrcalled=0;
   while (isrcalled==0) {
     // turn bod off
     MCUCR |= (1<<BODS) | (1<<BODSE);
     MCUCR &= ~(1<<BODSE);  // must be done right before sleep
     sleep_cpu();  // sleep here
   }
   // calculate remaining time
   timeRem -= (0x10<<WDTps);
 }
 sleep_disable();
 return timeRem;
}

// wdt int service routine
ISR(WDT_vect) {
 WDT_Off();
 isrcalled=1;
}

#endif /* AUTOCAL_WDT */

/* ************************************************ */
/* *** BLE handling *** */
/* ************************************************ */

// init HM-1x module
int setupBLE()
{
  int i;

  //detect HM-1x baudrate, if not currently set
  print_state(F(" - determining HM-1x baudrate"));
  
  for( i = 0 ; i <= 8 ; i++ ) {
    init_command_buff(&command_buff);
    print_state(F(" - trying ")); Serial.print(uart_baudrate[i]);
    settings.uart_baudrate = uart_baudrate[i];
    ble_Serial.begin(uart_baudrate[i]);
    send_string(F("AT"), 100);
    if ( waitDoingServicesInterruptible(500,&got_ok,1) )
      break;
  }
  
  if(!got_ok){
    print_state(F("Could not detect baudrate of HM-1x, setting 9600"));
    settings.uart_baudrate=9600;
    resetBLE();
    return(0);
  }
  
  print_state(F(" - baudrate set to "));
  Serial.print(settings.uart_baudrate);
  init_command_buff(&command_buff);
  return(1);
}

// Configure the BlueTooth module with a name.
// from LimiTTer setp() moved to here
void configBLE() {
  send_string(F("AT"), 100);        // cut BLE connection to do commands
  send_ble_string(F("AT+RENEW"));   // restore factory defaults
  send_ble_string(F("AT+IMME1"));   // be sure not to be connected before the AT commands are done
  send_ble_string(LNAME);           // set unique LimiTTer name, max 11 (not 12) chars
  send_ble_string(F("AT+VERR?"));   // look for correct BLE module - answer schould be "HMSoft V54x"
                                    // to detect fake modules which starts with 115200
  send_ble_string(F("AT+ADDR?"));   // get MAC address
  send_ble_string(F("AT+NOTI1"));   // notify CONNECT and LOST
  send_ble_string(F("AT+RESET"));   // reset the module
  waitDoingServices(500, 1);
  if ( !ble_connected )
    send_string(F("AT+START"), 200);  // all AT commands done, now we can connect
}

void resetBLE(void)
{
  print_state(F(" - reset BLE, power down - 500 ms - power up - 500 ms"));
  
  digitalWrite(5, LOW); // Disable this for Android 4
  digitalWrite(6, LOW); // Disable this for Android 4
  digitalWrite(BLEPin, LOW); // Disable this for Android 4

  waitDoingServices(500, 1);

  digitalWrite(BLEPin, HIGH);
  digitalWrite(5, HIGH);
  digitalWrite(6, HIGH);

  waitDoingServices(500, 1);

}
  
void wakeBLEUp(void) 
{
  digitalWrite(BLEPin, HIGH);
  digitalWrite(5, HIGH);
  digitalWrite(6, HIGH);

  ble_start_time = millis();
  ble_connected = 0;
#ifdef HW_BLE
  hwble_connected = 0;
#endif
  
  // wait 1 s for initial connect, time for a fast BLE connect
  waitDoingServicesInterruptible(1000, &ble_connected, 1); 

//  send_string(F("AT"), 100);        // cut BLE connection to do commands
  send_ble_string(F("AT+RENEW"));   // restore factory defaults
  send_ble_string(F("AT+IMME1"));   // first do all AT commands
  send_ble_string(LNAME);           // set unique LimiTTer name, max 11 (not 12) chars
  send_ble_string(F("AT+NOTI1"));   // notify CONNECT and LOST
  send_ble_string(F("AT+RESET"));   // reset the BLE module
  waitDoingServices(500, 1);
  if ( !ble_connected )
    send_string(F("AT+START"), 200);  // all AT commands done, now we can connect

  print_state(F(" - wake up - wait 40 s for BLE\r\n"));
  int i;
  for ( i = 0 ; i < 40 ; i++) {
    waitDoingServices(1000, 1);
    // typical CONN/LOST time difference is 1 - 2 s, mostly happens at the beginning of loop
    // wait for this 2 s before break
    if ( ble_connected ) {
      print_state(F(" - connected, wait to manage CONN+LOST cyle then send packet(s)"));
      // LOST on CONN mostly after 800 - 900 ms
      waitDoingServices(2000, 1);
      break;
    }
  }

  // must be connected here to start communication
  if ( !ble_connected )
    ble_connect_errors++;

  // send error counters to xDrip+ for future use
#ifdef SEND_ERROR_COUNTERS
  if ( ble_connected && send_error_counters_via_ble ) {
    ble_Serial.print(F("E"));
    ble_Serial.print(loop_count);
    ble_Serial.print(F(" "));
    ble_Serial.print(ble_connect_errors);
    ble_Serial.print(F(" "));
    ble_Serial.print(nfc_prot_set_cmd_errors);
    ble_Serial.print(F(" "));
    ble_Serial.print(nfc_inv_cmd_errors);
    ble_Serial.print(F(" "));
    ble_Serial.print(nfc_read_mem_errors);
    ble_Serial.print(F(" "));
    ble_Serial.print(resend_pkts_events);
    ble_Serial.print(F(" "));
    ble_Serial.print(queue_events);
    send_error_counters_via_ble = 0;
  }
#endif
}

void shutBLEDown(boolean quit_ble)
{
  // cut BLE conection via AT command
  if ( quit_ble && ble_connected ) {
    print_state(F(" - disconnect BLE, wait 2 s then sleep"));
    ble_lost_processing = 0;
    send_string(F("AT"), 100);
    waitDoingServices(1500, 1);
    ble_lost_processing = 0;
  }
 
  digitalWrite(5, LOW); // Disable this for Android 4
  digitalWrite(6, LOW); // Disable this for Android 4
  digitalWrite(BLEPin, LOW); // Disable this for Android 4

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
// print_state(F(" - mem avail "));
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
}

void shutNFCDown(void)
{
// print_state(F(" - mem avail "));
// Serial.print(freeMemory());
 
  SPI.end();

  digitalWrite(MOSIPin, LOW);
  digitalWrite(SCKPin, LOW);
  digitalWrite(NFCPin1, LOW); // Turn off all power sources completely
  digitalWrite(NFCPin2, LOW); // for maximum power save on BM019.
  digitalWrite(NFCPin3, LOW);
  digitalWrite(IRQPin, LOW);
}

/* ****************************************************************** */
/* port of xBridge2 code */
/* ***************************************************************** */

// send data to BLE
void send_data(unsigned char *msg, unsigned char len)
{
  unsigned char i = 0;

  for( i = 0; i < len; i++ )
    ble_Serial.write(msg[i]);

  // wait up to 40 chars (@9600)
  delay(40);
  print_state(F(" - sending: <"));
  for ( i = 0 ; i < len ; i++ ) {
    Serial.print(msg[i], HEX);
    Serial.print(F(" "));
  }
  Serial.print(F(">"));
  print_state(F(" - response: "));
}

// due to Arduino cast problems between String and char * use extra function
int send_ble_string(String cmd)
{
  int i;
  unsigned long now;
  boolean timeout;
  
  if ( !ble_connected ) {
    for ( i = 0 ; i < 3 ; i++ ) {
      if ( show_ble ) {
        print_state(F(" ->("));
        Serial.print(cmd);
        Serial.print(F(") - "));
      }
  
      ble_Serial.print(cmd);
  
      now = millis();
      ble_answer = 0;
      timeout = 1;
      
      while ( (millis() - now) < 2000 ) {
        waitDoingServices(30, 1);
        if ( ble_answer == 2 ) {
          ble_answer = 0;
          timeout = 0;
          break;
        }
      }
      waitDoingServices(100, 1);
      
      if ( timeout ) {
        resetBLE();
      }
      else
        return(1);
    }
    return(0);
  }
  return(0);
}

void send_string(String cmd, int dly)
{
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
  if(!pCmd)
    return 0;
  memset(pCmd->commandBuffer, 0, COMMAND_MAXLEN);
  pCmd->nCurReadPos = 0;
  return 0;
}

//decode a command received ??
int commandBuffIs(char* command)
{
  unsigned char len = strlen(command);
  if(len != command_buff.nCurReadPos)
    return(0);
  return( memcmp(command, command_buff.commandBuffer, len)==0 );
}

// decode incoming serial/USB or BLE data commands
int doCommand(void)
{
  // TXID packet?
  if(command_buff.commandBuffer[1] == 0x01 && command_buff.commandBuffer[0] == 0x06)
  {
    memcpy(&settings.dex_tx_id, &command_buff.commandBuffer[2],sizeof(settings.dex_tx_id));
    // send back the TXID we think we got in response
    return(0);
  }
  // ACK packet?
  if(command_buff.commandBuffer[0] == 0x02 && command_buff.commandBuffer[1] == 0xF0 && !got_ack) {
    got_ack = 1;
    init_command_buff(&command_buff);
    return(0);
  }
  if( toupper(command_buff.commandBuffer[0]) == 'B' ) {
    if ( show_ble ) {
      print_state(F(" - B command - dont show BLE communication"));
      show_ble = 0;
    }
    else {
      print_state(F(" - B command - show BLE communication"));
      show_ble = 1;
    }
  }
  if( toupper(command_buff.commandBuffer[0]) == 'S' ) {
    if ( show_state ) {
      print_state(F(" - S command - dont show states"));
      show_state = 0;
    }
    else {
      show_state = 1;
      print_state(F(" - S command - show states"));
    }
  }  // "OK+..." answer from BLE?
  if( commandBuffIs("OK") ) 
  {
    got_ok = 1;;
    return(0);
  }
  // we don't respond to unrecognised commands.
  return(1);
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
    hwble_connected =0;
    last_ble_check = 0;
  //otherwise, if P1_2 has been high for more than 550ms, we can safely assume we have ble_connected, so say so.
  } else if ( digitalRead(bleSysLedPin) && last_ble_check && ((millis() - timer)>550)) {
    hwble_connected = 1;
  }

  // show if BLE state has changed here
  if ( last_ble != hwble_connected ) {
    print_state(F(" - ***** HW BLE "));
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

  if ( show_ble ) {
    if ( b >= ' ' && b < 128 ) {
      Serial.write(b);
    }
    else { 
      Serial.print(b, HEX); 
      Serial.print(F(" "));
    }
  }

  for ( i = 0 ; i < 3 ; i++) {
    if ( b == okstr[i].pattern[bi] )
      c_found |= 1;
  }
  
  if ( c_found )
    bi++;
  else
    bi = 0;
    
  if ( bi == 7 ) { // one of pattern complete
    bi = 0;
    if ( b == okstr[0].pattern[6] ) { 
        ble_connected = 1;
        print_state(F(" - +++++ BLE connected after "));
        ble_connect_time = millis();
        Serial.print( millis() - ble_start_time );
        Serial.print(F(" ms"));
        return(1);
    } else if ( b == okstr[1].pattern[6] ) {
        ble_connected = 0;
        ble_connect_time = 0;
        print_state(F(" - +++++ BLE lost after "));
        Serial.print( millis() - ble_start_time );
        Serial.print(F(" ms"));
        
        // ??? test
        if ( ble_lost_processing )
          send_string(F("AT+RESET"), 500);

        return(1);
    } else if ( b == okstr[2].pattern[6] ) {
#ifdef SEND_ERROR_COUNTERS
        send_error_counters_via_ble = 1;
        print_state(F(" - send error counters after next wakeup"));
        return(1);
#endif
    }
  }
  return(0);
}

// Process any commands from BLE
int controlProtocolService()
{
  static unsigned long cmd_to;
  // ok this is where we check if there's anything happening incoming on the USB COM port or UART 0 port.
  int nRet = 1;
  int i;
  unsigned char b;

  //if we have timed out waiting for a command, clear the command buffer and return.
  if(command_buff.nCurReadPos > 0 && (millis() - cmd_to) > 2000) 
  {
    print_state(F(" - got <-[")); 
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
    return(nRet);
  } 
  //while we have something in either buffer,
  while( (Serial.available() || ble_Serial.available()) && command_buff.nCurReadPos < COMMAND_MAXLEN) {

    if ( ble_answer == 0 ) {
//      Serial.print("-1-");      
      ble_answer = 1;
    }

    if ( ble_Serial.available() )
      monitor_ble( b = ble_Serial.read() );
    else
      b = Serial.read();

    command_buff.commandBuffer[command_buff.nCurReadPos] = b;
    command_buff.nCurReadPos++;
    cmd_to = millis();
    // if it is the end for the byte string, we need to process the command
    // valid data packet or "OK" received?
    if(      command_buff.nCurReadPos == command_buff.commandBuffer[0] \
         || (command_buff.commandBuffer[0] == 'O' && command_buff.commandBuffer[1] == 'K' ) \
         || (command_buff.nCurReadPos == 1 && toupper(command_buff.commandBuffer[0]) == 'B') \
         || (command_buff.nCurReadPos == 1 && toupper(command_buff.commandBuffer[0]) == 'S') )
     {
      // ok we got the end of a command;
      if(command_buff.nCurReadPos) {
        // do the command
        nRet = doCommand();
        //re-initialise the command buffer for the next one.
        init_command_buff(&command_buff);
        // break out if we got a breaking command
        if(!nRet)
          return(nRet);
      }
    }
    // otherwise, if the command is not up to the maximum length, add the character to the buffer.
  }

  if ( ble_answer == 1 ) {
//    Serial.print("-2-");
    ble_answer = 2;
  }
  
  if ( command_buff.nCurReadPos ){
    //re-initialise the command buffer for the next one.
    init_command_buff(&command_buff);
  }
  return(nRet);
}

// process each of the services we need to be on top of.
// if bWithProtocol is true, also check for commands on both USB and UART
int doServices(unsigned char bWithProtocol)
{
  dex_tx_id_set = (settings.dex_tx_id != 0);
#ifdef HW_BLE
  bleConnectMonitor();
#endif
  if(bWithProtocol)
    return(controlProtocolService());
  return(1);
}

#ifdef DYNAMIC_TXID
//format an array to decode the dexcom transmitter name from a Dexcom packet source address.
char SrcNameTable[32] = { '0', '1', '2', '3', '4', '5', '6', '7',
              '8', '9', 'A', 'B', 'C', 'D', 'E', 'F',
              'G', 'H', 'J', 'K', 'L', 'M', 'N', 'P',
              'Q', 'R', 'S', 'T', 'U', 'W', 'X', 'Y' };

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

  for(i = 0; i < 32; i++)
  {
    if (SrcNameTable[i]==srcVal) break;
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
      return(1);      
    }
    delay(20);
  }
  return(0);
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
  // store the current wixel milliseconds so we know how long we are waiting.
  unsigned long start = millis();
  unsigned long ms = 3000;  // try for 3 seconds, then leave 
  // set the return code to timeout indication, as it is the most likely outcome.
  float act_glucose;

  // while we haven't reached the delay......
  while ( (millis() - start) < ms ) {
    if (NFCReady == 0) {
      SetProtocol_Command(); // ISO 15693 settings
      waitDoingServices(100, 1);
      continue;
    }
    else if (NFCReady == 1) {
      for (int i=0; i<3; i++) {
        Inventory_Command(); // sensor in range?
        nfc_inv_cmd_count++;
        if (NFCReady == 2)
          break;
        waitDoingServices(1000, 1);
      }
      if (NFCReady == 1) {
        // count missed readings
        nfc_read_errors++;
        print_state(F(" - NFC timed out"));
        return(0);
      }
    }
    else if ( NFCReady == 2 ) {
      act_glucose = Read_Memory();
      nfc_scan_count++;
#ifdef SHOW_LIMITTER
      Serial.print(F("\r\nGlucose level: "));
      Serial.println(act_glucose);
      Serial.println(F("15 minutes-trend: "));
      for (int i=0; i<16; i++)
        Serial.println(trend[i]);
      Serial.print(F("Battery level: "));
      Serial.print(batteryPcnt);
      Serial.println(F("%"));
      Serial.print(F("Battery mVolts: "));
      Serial.print(batteryMv);
      Serial.println(F("mV"));
      Serial.print(F("Sensor lifetime: "));
      Serial.print(sensorMinutesElapse);
      Serial.print(F(" minutes elapsed"));
#endif /* SHOW_LIMITTER */
      // only for showing package content
      *ptr = act_glucose;
      return(1);
    }
  }
  // timeout
  return(0);
}

// wait for a NFC reading and put it into Dexcom_packet
int get_packet(Dexcom_packet* pPkt)
{
  // store the current wixel milliseconds so we know how long we are waiting.
  unsigned long start = millis();
  // set the return code to timeout indication, as it is the most likely outcome.
  float glucose;

  if ( get_nfc_reading(&glucose) ) {
    pPkt->raw = glucose*1000;    // use C casting for conversion of float to unsigned long
    pkt_time = millis();
    pPkt->ms = abs_millis();
/*
    print_state(F(" - packet read at "));
    Serial.print(pPkt->ms/1000);
    Serial.print(F(" s"));
*/
    if ( abs_pkt_time != 0 )
      last_abs_pkt_time = abs_pkt_time;
    abs_pkt_time = pPkt->ms;
/*
    print_state(F(" - last packet read "));
    Serial.print((abs_millis() - last_abs_pkt_time)/1000);
    Serial.print(F(" s before"));
*/    
    return(1);
  }
  else
    pkt_time = millis();    // to avoid endless waiting if no packet can be read
  return(0);
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

  if ( !ble_connected )
    return(0);
/*
  print_state(F(" - sending packet with a delay of "));
  Serial.print(msg.delay/1000);
  Serial.print(F(" s"));
*/
  send_data( (unsigned char *)&msg, msg.size);

  return(1);
}

// print timestamp and current status/action
void print_state(String str)
{
  Serial.println(F(""));
  Serial.print(millis());
  Serial.print(str);
}

// main processing loop
void loop(void)
{   
  int i;
  unsigned long var_sleepTime;
  boolean evtflg1;
  boolean evtflg2;
  int get_packet_result;

  waitDoingServices(2000, 1);

  print_state(F(" - ************************"));
  print_state(F(" - *** LBridge starting ***"));
//  print_state(F(" - *** Version )); Serial.print(LNAME); Serial.print(F(" ***"));
  print_state(F(" - *** mem: "));
  Serial.print(freeMemory()); Serial.print(F(" ***"));
  print_state(F(" - ************************"));

  loop_start_time = millis();             // log the current time
  init_command_buff(&command_buff);       //initialise the command buffer
  
  for ( i = 0 ; i < 3 ; i++ ) {
    if ( setupBLE() )                 // Open the UART and initialize HM-1x module
      break;
  }
  
  waitDoingServices(2000, 1); // wait for possible OK+LOST
  configBLE();                // configure the BLE module
  configNFC();                // initialize and configure the NFC module

  // wait for a BLE connection to xDrip to get a TXID if needed
  print_state(F(" - initial wake up, waiting 40s fixed for BLE / time for a xDrip+ BT scan if needed\r\n"));
  waitDoingServices(40000, 1);

  // if BLE connected wait some time to get an answer for TXID
  if ( ble_connected )
    waitDoingServices(3000, 1);

#ifdef DYNAMIC_TXID
  // if dex_tx_id is zero, we do not have an ID to filter on.  So, we keep sending a beacon every 5 seconds until it is set.
  print_state(F(" - testing TXID"));
  settings.dex_tx_id = 0xFFFFFFFF;
  // should we ask for a TXID?
  if(settings.dex_tx_id >= 0xFFFFFFFF) 
    settings.dex_tx_id = 0;
  // instead of save current TXID to EEPROM get it every time from xDrip using the Beacon mechanism
  while(settings.dex_tx_id == 0) {
    print_state(F(" - no TXID. Sending beacon"));
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

  print_state(F(" - new TXID is ")); Serial.print(settings.dex_tx_id, HEX); Serial.print(F(" ("));
  // and show first 5 char from the TXID setting in xDrips menue
  Serial.print(dexcom_src_to_ascii(settings.dex_tx_id)); Serial.print(F(")"));

#else
  settings.dex_tx_id = 0xA5B1AE;    // -> "ABCDE" xBridge Wixel default
#endif /* DYNAMIC_TXID */

  // initialize to empty queue
  Pkts.read = 0;
  Pkts.write = 0;

  print_state(F(" - entering main loop"));

  while (1)
  {
    check_battery();    // check for low battery and go to sleep here if wrong
//    print_state(F(" - wake up NFC"));
    wakeNFCUp();
    get_packet_result = get_packet(&Pkts.buffer[Pkts.write]);
//    print_state(F(" - shut NFC down"));
    shutNFCDown();

    if ( sensor_oor )
      print_state(F(" - sensor out of range"));

    if( get_packet_result ) {
      print_state(F(" - got packet, stored at position "));
      Serial.print(Pkts.write);
      Serial.print(F(", incrementing write to "));
      // so increment write position for next round...
      if ( ++Pkts.write >= DXQUEUESIZE )
        Pkts.write = 0;
      Serial.print(Pkts.write);
      if (Pkts.read == Pkts.write) {
        print_state(F(" - queue overflow, incrementing read overwriting oldest entry"));
        if ( ++Pkts.read >= DXQUEUESIZE ) //overflow in ringbuffer, overwriting oldest entry, thus move read one up
          Pkts.read = 0;
      }

      do_sleep = 1; // we got a packet, so we are aligned with the 5 minute interval - so go to sleep after sending out packets
    } 
    else {
      print_state(F(" - did not receive a pkt with "));
      Serial.print(Pkts.write-Pkts.read);
      Serial.print(F(" pkts in queue"));
//      if ( ble_connected ) 
//        sendBeacon();
      do_sleep = 1; // no NFC reading, go to sleep to save battery
    }

    if ( (loop_count > 0) && !sensor_oor ) {
//      print_state(F(" - wake up BLE"));
      wakeBLEUp();
    }

    if ( (Pkts.read != Pkts.write) && !sensor_oor ) { // if we have a packet
      // we wait up to 40 s for BLE connect
      while (!ble_connected && ((millis() - pkt_time) < 40000 )) {
        print_state(F(" - packet waiting for ble connect"));
        if ( waitDoingServicesInterruptible(10000, &ble_connected, 1) ) {
          print_state(F(" - connected, wait before sending the packet"));
          waitDoingServices(2000, 1);
        }
      }

      evtflg1 = 0;
      evtflg2 = 0;
      // we got a connection, so send pending packets now - at most for two minutes after the last packet was received
      // wait enough time to empty the complete queue!
      while ((Pkts.read != Pkts.write) && ble_connected && ((millis() - pkt_time) < ((DXQUEUESIZE+1)*1000) )) {
        got_ack = 0;

        print_packet(&Pkts.buffer[Pkts.read]);

        // wait 10 s for ack
        int j;
        for ( j = 0 ; j < 10 ; j++ ) {
          waitDoingServicesInterruptible(1000, &got_ack, 1);
          if ( !ble_connected ) {
            print_state(F(" - connection lost during wait for ack, go to sleep"));
            break;
          }
        }
        
        if (got_ack) {
          print_state(F(" - got ack for read position "));
          Serial.print(Pkts.read); Serial.print(F(" while write is "));
          Serial.print(Pkts.write); Serial.print(F(", incrementing read to "));
          if ( ++Pkts.read >= DXQUEUESIZE )
            Pkts.read = 0;     //increment read position since we got an ack for the last package
          Serial.print(Pkts.read);
          if ( !evtflg1 && (Pkts.read != Pkts.write) ) {  // when still available we have a queue event
            evtflg1 = 1;
            queue_events++;
          }
#ifdef XB_NEW
          send_data((unsigned char *)&qpkt1, (unsigned char)qpkt1.size);
          send_data((unsigned char *)&qpkt2, (unsigned char)qpkt2.size);
#endif
        }
        else {
          print_state(F(" - no ack received, try again next wakeup"));
          if ( !evtflg2 ) {
            evtflg2 = 1;
            resend_pkts_events++;   // we got no ACK, pkts to be resended
          }
          break;
        }
      }
    }

    // can't safely sleep if we didn't get an ACK, or if we are already sleeping!
    if ( do_sleep )
    {
      // empty serial out buffer and open tasks
      waitDoingServices(500, 1);

      // ensure to use 300 s for a full cycle of loop() and sleeping
      loop_time = millis() - loop_start_time; 
      if ( loop_time < 290000 )
        var_sleepTime = (((300000 - loop_time)/1000)+4) / 8;
      else
        var_sleepTime = 32;

      print_state(F(" - loop time ")); Serial.print(loop_time); Serial.print(F(" ms, sleep for ")); 
      Serial.print(var_sleepTime*8); Serial.print(F("s"));
      Serial.print(F(", complete loop is ")); Serial.print((loop_time+var_sleepTime*8000)/1000);
      Serial.print(F(" s"));

      shutBLEDown(1);          // cut BLE connection and switch HM-1x OFF

      goToSleep(0b100001, var_sleepTime);    // mask for 8s
      
      // waking up, power on BLE, initialze NFC
      // count programm run time up
      prg_run_time += (loop_time + var_sleepTime*8000)/1000;
      loop_start_time = millis();       // set start point for loop 
      do_sleep = 0;                     // clear do_sleep, cause we have just woken up.
      init_command_buff(&command_buff); // no BLE connection

      print_state(F(" - ******************************************************************"));
      print_state(F(" - loop ")); Serial.print(++loop_count);
      Serial.print(F(" BLE conn errors ")); Serial.print(ble_connect_errors);
      Serial.print(F(" run time ")); Serial.print(prg_run_time); Serial.print(F(" s"));
      print_state(F(" - InvCmdErr ")); Serial.print(nfc_inv_cmd_errors);
      Serial.print(F(" ProtCmdErr ")); Serial.print(nfc_prot_set_cmd_errors);
      Serial.print(F(" ReadMem Err ")); Serial.print(nfc_read_mem_errors);
      Serial.print(F(" NFC scans ")); Serial.print(nfc_scan_count);
      print_state(F(" - ResndPktEvt ")); Serial.print(resend_pkts_events);
      Serial.print(F(" QueueEvt ")); Serial.print(queue_events);

      if ( loop_count > 1 ) {
        avg_sleepTime += var_sleepTime*8;
        bleTime += loop_time;
      }
      else {
        avg_sleepTime = 0;
        bleTime = 0;
      }
      print_state(F(" - avg. sleep time: "));  Serial.print(avg_sleepTime/loop_count);  
      Serial.print(F(" s, "));
      Serial.print(F("ProMini+BLE On: "));  Serial.print((bleTime/loop_count)/1000);  
      Serial.print(F(" s"));

      wakeUp();
      waitDoingServices(250,1);
    }
  }
}


