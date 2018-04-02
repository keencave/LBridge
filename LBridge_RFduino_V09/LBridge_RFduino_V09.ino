/*
 * LBridge for RFduino / Simblee
 * 
 * BLE Transmitter for read blood glucose from the Freestyle Libre Sensor via NFC and transfer to 
 * xDrip+ via BLE using the xbridge2 protocol (xbridge wixel)
 
 * Supports backfilling of older BG readings which were not transfered at the time of reading.
 * 
 * Code based on on the work of @UPetersen, @MarekMacner, @jstevensog, @savek-cc and @bertrooode
 * 
 * keencave, 06/2017
 * 
 * ToDo: 
 *  - optimize BLE reconnect
 *  - only read changed NFC blocks from sensor for better battery performance
 *  - enable "hidden" power modes on BM019
 */

/*
 *  29.7.2017:
 *  - added backfilling from sensor FRAM
 *  - added simple spike detection and removal
 *  30.1.2018:
 *  - code clean up
 *  - 8 h backfilling
 * 3.2.2018:
 *  - code cleanup
 *  - check sensorStatusByte for sensor dead detection
 *  - switch off serial interface during ULP for better power processing (@chaosbiber)
 *  - 8 h backfilling - send per default last 8 h, xDrip+ will only accept and display values which are missing
 *  - shadow FRAM added (@UPetersen)
 *  - system, start with faster cycle time @FPV-UAV
 * V0.9.01
 *  - modifications due to @UPetersen s code review
 *  - bit mask for raw BG data 13 bits = 0x1FFF
 * V09.02
 *  - minor Simblee code changes (@clvsjr9)
 * V09.03
 *  - added #define SHORTSTARTCYCLE to activate 10 short start cycles for staring new sensor faster
 *  V0.9.04
 *  - minimum voltage set to 2300 mV to ensure proper BLE operation (@bertrooode)
 *  - debug oputput with no sensor available cleanded
 *  V0.9.05
 *  - possible overflow error in sleep time fixed (@bertrooode)
 *  V0.9.06
 *  - queue length adjusted to 8h
 *  - added debug output for decodeSN
 *  V0.9.07
 *  - serial.begin wait now for port online (delay code by @bertrooode)
 *  - error fixed in decodeSN (@bertrooode)
 *  - reworked program config section (#define section)
 *  - no serial output in BLE interrupt routine (@chaosbiber)
 * V0.9.08
 *  - Serial.flush after heavy serial output (@bertrooode)
 *  - additional reset of CR95HF after ULP wakeup and Serial.start (@bertrooode)
 *  - own printf() code to avoid fragmented Serial.print() calls / reduce system load? to be verified
 *  - set different minimum voltage level for Simblee (@FPV-UAV)
 * V0.9.10
 *  - getSocData() with new readVDD function for Simblee (@clvsjr9)
 *  - adjusted MIN_VOLTAGE/MAX_VOLTAGE for Simblee
 *  - adjusted flash page for Simblee usage (@FPV-UAV)
 *  - bugfix for backfilling (@libxmike)
 *  - bugfix to enable live sensor change (@bertrooode, @chaosbiber)
 *  - minor bugfixes
 * V0.9.11
 *  - fixed code errors in LimiTTer mode, no tested before (uncomment #define USE_XBRIDGE2 to activate)
 * V0.9.12
 *  - removed useless while (!Serial), replaced by a fixed delay, RFduino Serial implementation is interrupt driven, only work on Arduino
 *  - removed own printf code to avoid stack overflow, serial output only done with Serial.printf() now
 *  - added while (RDduinoBLE.radioActive); after ...BLE.send / to be tested
 * V0.9.13
 *  - this version number was intentional skipped
 * V0.9.14
 *  - replaced all String class depend code to char[] type to reduce heavy memory usage of the String class
 *  - added stack_check() code / no action taken on underflow / displays function call history after remaining memory is too low
 *  - track and display absolute minimum of available memory
 *  - removed radioActive check before/after BLE.send routines
 *  - reduced BLE packet length to 20 bytes fixed to avoid sending 2 packets / xDrip accepts shorter packets
 *    if the length is coded correctly at start of packet
 * V0.9.15
 *  - restart of BLE stack when not connected at end of loop() to heal slowly packet transmission after reconnect (experimental)
 *  - restart BLE stack if BLE lost during sending the current queue (40 s waiting time)
 *  - testing function result of BLE.send to check proper BLE stack queue filling (currently RFduino only)
 *  - added watchdog again to overcome possible RFduino freezing due to BLE stack overload
 */

 /*
  * to do:
  * - clear up SHADOWFRAM code, implemented with #defined parts and bool flag
  * - adopt Simblee FLASH handling
  * - reintegrate a classic watchdog timer to overcome RFduino freezing
  */

/* ********************* program configuration options **********************/

#define RFD                     // uncomment for RFduino platform, otherwise Simblee platform will be used
#define USE_DEAD_SENSOR         // uncomment to work with live sensors
#define USE_SPIKE_FILTER        // uncomment to use a spike filter on raw data
#define USE_XBRIDGE2            // uncomment to use two way xbridge2 protocol with backfilling instead of LimiTTer
                                // ASCII protocol (one way communication)
#define USE_SHADOWFRAM          // uncomment to work with a shadow FRAM copy of the Libre sensor FRAM
#define SERIALOFF               // switch off Serial interface during ULP for power saving
//#define SHOW_BLESTATECHANGE     // uncomment to have serial BLE connect/disconnect messages (see also #define SERIALOFF)

//#define SHORTSTARTCYCLE         // uncomment to have 10 inital 1 min cycles to have faster sensor start (@FPV_UAV)

// debug and other less important options, uncomment to disbale the option
//#define SIM_MINUTES_UPCOUNT     // for testing backfilling with dead sensor, disable for active sensors!

//#define DEBUG                   // uncomment to have verbose debug output
//#define FRAM_DEBUG              // uncomment to have verbose debug output for FRAM mechanism

//#define BLE_REAL_LEN            // use real length of BLE packets to send or limit to 20 chars?
//#define SEND_LIMITTER_SA      // send sensor minutes via LimiTTer string
#define USE_WDT               // use watchdog with 11 min duration

/* ******************** BLE and central seetings ***************************** */

#define UARTDELAY 2000          // delay after switching serial interface on/off

#define DXQUEUESIZE (9*12)     // 12 h of queue (remember and send the last x hours when reconnected via BLE)

#define LB_NAME   "xbridgeR"    // dont change "xbridge", space for 1 char left to indicate different versions
#define LB_ADVERT "rfduino"     // dont change "rfduino"
                                // length of device name and advertisement <=15!!!
#define LB_VERSION "V0.9"       // program version
#define LB_MINOR_VERSION ".15"  // indicates minor version
#define LB_DATETIME "180326_0915" // date_time
#define SPIKE_HEIGHT 40         // minimum delta to be a spike

#ifdef RFD
#define MAX_VOLTAGE 3600        // adjust voltage measurement to have a wider rrange
#define MIN_VOLTAGE 2300        // minimum voltage where BLE will work properly (@bertrooode)
#else RFD
#define MAX_VOLTAGE 3000        // adjust voltage measurement to have a wider rrange
#define MIN_VOLTAGE 2200        // minimum voltage where BLE will work properly (@FPV-UAV)
#endif RFD

/* ****************************** includes ********************************** */

#ifdef RFD
#include <RFduinoBLE.h>
#else RFD
#include <SimbleeBLE.h>
#include <ota_bootloader.h>
#endif RFD

#include <SPI.h>
#include <Stream.h>
#include <Memory.h>
#include <itoa.h>

// by @clvsjr9 to Arduino 1.6.5 compile
//#include <data_types.h>

/* ************************** NFC defines ************************************** */

#define PIN_SPI_SCK   4
#define PIN_SPI_MOSI  5
#define PIN_SPI_MISO  3
#define PIN_SPI_SS    6
#define PIN_IRQ       2

#define ALL_BYTES 0x1007
#define IDN_DATA 0x2001
#define SYSTEM_INFORMATION_DATA 0x2002
#define BATTERY_DATA 0x2005

/* CR95HF Commands */
#define IDN               0x01  // identification number of CR95HF
#define SELECT_PROTOCOL   0x02  // select protocol
#define POLL              0x03  // poll
#define SENDRECEIVE       0x04  // send and receive data (most commonly used)
#define READ              0x05  // read values from registers internal to CR95HF
#define WRITE             0x06  // write values to registers internal to CR95HF
#define ECHO              0x55

// send receive commands for ISO/IEC 15693 protocol
#define INVENTORY               0x01  // receives information about tags in range
#define STAY_QUIET              0x02  // selected unit will not send back a response
#define READ_BLOCK              0x20  // read single block of memory from RF tag
#define WRITE_BLOCK             0x21  // write single block to memory of RF tag
#define LOCK_BLOCK              0x22  // permanently locks a block of memory on RF tag
#define READ_BLOCKS             0x23  // reads multiple blocks of memory from RF tag
#define WRITE_BLOCKS            0x24  // writes multiple blocks of memory to RF tag
#define SELECT                  0x25  // used to select a specific tag for communication via the uid
#define RESET_TO_READY          0x26  // resets RF tag to ready state
#define WRITE_AFI               0x27  // writes application family identifier to RF tag
#define LOCK_AFI                0x28  // permanently locks application family identifier
#define WRITE_DSFID             0x29  // writes data storage format identifier to RF tag
#define LOCK_DSFID              0x2A  // permanentlylocks data storage format identifier
#define GET_SYSTEM_INFORMATION  0x2B  // gets information from RF tag that includes memory
// block size in bytes and number of memory blocks
#define GET_BLOCKS_SECURITY_STATUS  0x2C

const int SS_PIN = SS;   // Slave Select pin, uses standard wiring from variant.h (see comment section above)
//const int IRQ_PIN = 2;  // IRQ/DIN pin used for wake-up pulse

/* ******************************** accessing the FLASH üages ******************** */

#ifdef RFD
#define MY_FLASH_PAGE  251
#else
#define MY_FLASH_PAGE  237  // to enable OTA programming, 230/235/237 usable accorind @FPV-UAV
#endif

#define  str(x)   xstr(x) // double level of indirection required to get gcc
#define  xstr(x)  #x      // to apply the stringizing operator correctly

/* ******************************** program variables ******************************** */

#ifdef USE_SHADOWFRAM               // use shadow FRAM mechanism?
bool use_shadowfram = 1;
#else
bool use_shadowfram = 0;
#endif

#ifdef SEND_LIMITTER_SA
unsigned long last_raw = 0;         // last sended RAW value
#endif
int batteryPcnt = 0;                // battery capacity in %

unsigned long loop_cnt = 0;         // count the 5 mins loops
int ble_answer;                     // state counter, char from BLE reeived?

#define BLEBUFLEN 80                // BLE input buffer
unsigned char bleBuf[BLEBUFLEN];
int bleBufRi = 0;                   // BLE read and write index
int bleBufWi = 0;
static boolean BTconnected = false;

unsigned long startMinutesSinceStart = 0;
unsigned long startMillisSinceStart = 0;
unsigned long minutesSinceProgramStart = 0;

uint16_t lastGlucose = 0;
uint16_t currentGlucose = 0;
boolean firstRun = 1;               // flag for spike filter operation

byte protocolType = 1;              // 1 - LimiTTer
byte runPeriod = 1;                 // czas w minutach - 0 = tylko na żądanie

unsigned long time_loop_started = 0;

bool BatteryOK = false;

char TxBuffer[30];

byte NFCReady = 0;            // 0 - not initialized, 1 - initialized, no data, 2 - initialized, data OK

/* ****************************** xbridge2 protocol releated ******************************** */

// defines the xBridge protocol functional level.  Sent in each packet as the last byte.
#define DEXBRIDGE_PROTO_LEVEL (0x01)

static boolean show_ble = 0;
static boolean got_ack = 0;               // indicates if we got an ack during the last do_services.
static unsigned long pkt_time = 0;        // time of last valid received glucose value
static unsigned long last_ble_send_time;  // beautifying BLE debug output
static boolean crlf_printed = 0;          // beautifying BLE debug output

//define the maximum command string length for USB commands.
#define COMMAND_MAXLEN 80

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

typedef struct {
  volatile unsigned char read;
  volatile unsigned char write;
  Dexcom_packet buffer[DXQUEUESIZE];
} Dexcom_fifo;

Dexcom_fifo Pkts;

Dexcom_packet * DexPkt;

// structure of a raw record we will send in a xbridge2 packet
typedef struct _nRawRecord
{
  unsigned char size;                   //size of the packet.
  unsigned char cmd_code;               // code for this data packet.  Always 00 for a Dexcom data packet.
  unsigned long raw;                    //"raw" BGL value. ??? use unfiltered NFC readings here?
  unsigned long filtered;               //"filtered" BGL value
  unsigned char dex_battery;            //battery value
  unsigned char my_battery;             //xBridge battery value
  unsigned long dex_src_id;             //raw TXID of the Dexcom Transmitter
  unsigned long delay;                  // delay of raw reading to the current runtime
  unsigned char function;               // Byte representing the xBridge code funcitonality.  01 = this level.
} nRawRecord;

unsigned long dex_tx_id = 0xA5B1AE;     // TXID for xbridge2 protocol packets = "ABCDE"

/* *************************** NFC related ******************************* */

#define SNLEN 20            // length of decoded sensor number
#define SSBYTE              // length of sensorStatusByte message

typedef struct  __attribute__((packed))
{
  uint8_t resultCode;
  uint8_t deviceID[13];
  uint8_t romCRC[2];
} IDNDataType;

typedef struct  __attribute__((packed))
{
  uint8_t uid[8];
  uint8_t resultCode;
  uint8_t responseFlags;
  uint8_t infoFlags;
  uint8_t errorCode;
  char *sensorSN;
} SystemInformationDataType;

typedef struct  __attribute__((packed))
{
  bool sensorDataOK;
  char *sensorSN;
  byte   sensorStatusByte;
  char sensorStatusString[SSBYTE];
  byte  nextTrend;
  byte  nextHistory;
  uint16_t minutesSinceStart;
  uint16_t minutesHistoryOffset;
  uint16_t trend[16];
  float trendTemperature[16];
  uint16_t history[32];
  float historyTemperature[32];
} SensorDataDataType;

IDNDataType idnData;
SystemInformationDataType systemInformationData;
SensorDataDataType sensorData;

/*
 * to be optimised - data are stored in 3 different arrays
 */

byte sensorDataHeader[24];
byte sensorDataBody[296];
byte sensorDataFooter[24];

byte resultBuffer[40];
byte RXBuffer[400];           // receive buffer for shadow FRAM data
byte dataBuffer[400];         // receive buffer for non shadow FRAM processing

/* ************************* shadow FRAM ************************************* */

#ifdef USE_SHADOWFRAM
/// State of nfc reading
/// @Contains current and last values needed to track what blocks of date have to be read from the FreeStyle Libre via nfc
/// Nomenclature:
///    block ... block of 8 bytes of data of sensor FRAM. The smallest unit of data that can be read from the sensor
///    data  ... consecutive buffer of 6 bytes in sensor FRAM that comprises the FreeStyle Libre data for on point in time. It contains blood sugar data and temperature data and maybe some more information on sensor state.
///    index ... index (number of) the data
typedef struct  __attribute__((packed)) {
    uint8_t uid[8];                  // FreeStyle Libre serial number
    uint8_t oldUid[8];
    uint8_t nextTrend;               // number of next trend block to read
    uint8_t oldNextTrend;
    uint8_t nextHistory;             // number of next history block to read
    uint8_t oldNextHistory;
    uint16_t minutesSinceStart;      // minutes since start of sensor
    uint16_t oldMinutesSinceStart;
    byte fram[344];                  // buffer for Freestyle Libre FRAM data
    byte resultCodes[43];            // result codes from nfc read single block command
} Sensor;

Sensor sensor;
#endif

/* **************************** processor internals *************************************** */

typedef struct  __attribute__((packed))
{
  long voltage;
  int voltagePercent;
  double temperatureC;
  double temperatureF;
  double rssi;
} SoCDataType;

SoCDataType SoCData;            // store processor variables

/* ************************** FLASH variables ******************************************** */

typedef struct dataConfig
{
  byte marker;
  byte protocolType;                  // 1 - LimiTTer
  byte runPeriod;                     // 0-9 main loop period in miutes, 0=on demenad
  byte firmware;                      // firmware version starting 0x02
};

struct dataConfig valueSetup;   
dataConfig *p;

/* ************************* program section **************************** */

void setup()
{
  // set FALSH page adress where programm settings are stored
  p = (dataConfig*)ADDRESS_OF_PAGE(MY_FLASH_PAGE);
  // init and open serial line
  Serial.begin(9600);
  // time to settle, avoid serial ghost characters
  delay(UARTDELAY); 

  Serial.printf("\r\n============================================================================");

#ifdef RFD
  print_state("LBridge starting (RFduino)");
#else
  print_state("LBridge starting (Simblee)");
#endif RFD
  Serial.print("\r\n\tBLE name: "); Serial.print(LB_NAME); 
  Serial.print(", "); Serial.print(LB_VERSION); Serial.print(LB_MINOR_VERSION);
  Serial.print(" from "); Serial.print(LB_DATETIME);
  Serial.printf("\r\n\tRAM used: %d, Flash used: %d", ramUsed(), flashUsed());
  Serial.print("\r\n\tQueue size: "); Serial.print(DXQUEUESIZE);
  Serial.print(" ("); Serial.print(DXQUEUESIZE/12); Serial.print(" h)");
  Serial.print("\r\n\tOptions:");
#ifdef SIM_MINUTES_UPCOUNT
  Serial.print("-simSensorMin");
#endif
#ifdef USE_DEAD_SENSOR
  Serial.print("-useDeadSensor");
#endif
#ifdef USE_SPIKE_FILTER
  Serial.printf("-spikeFilter%d", SPIKE_HEIGHT);
#endif
#ifdef USE_XBRIDGE2
  Serial.print("-xbridge2");
#endif
#ifdef USE_SHADOWFRAM
  Serial.print("-shadowFRAM");
#endif
#ifdef SHORTSTARTCYCLE
  Serial.print("-10cycles1min");
#endif
#ifdef SERIALOFF
  Serial.print("-serialOff");
#endif
#ifdef USE_WDT
  Serial.print("-WDT");
#endif

  getSoCData();
  printSoCData();

  setupInitData();
  protocolType = p->protocolType;
#ifdef SHORTSTARTCYCLE
  // @FPV-UAV: do the first 10 rounds with 1 min cycle time
  runPeriod = 1;          // loop time is 1 min, only for system start
  print_state("system start with "); Serial.print(runPeriod); Serial.print(" min cycyle time");
#else
  runPeriod = p->runPeriod;
#endif

  loop_cnt = 0;

  nfcInit();
  digitalWrite(PIN_SPI_SS, LOW);
  SPI.transfer(0x01);
  digitalWrite(PIN_SPI_SS, HIGH);
  delay(1);

#ifdef USE_SHADOWFRAM
  initSensor(&sensor);      // initialize the sensor data structure for FRAM shadowing
#endif

#ifdef USE_WDT
  configWDT();              // wind up watch dog timer in case of RFduino freezing do a reset
#endif

  setupBluetoothConnection();

  // init xbridge queue
  Pkts.read = 0;
  Pkts.write = 0;
#ifdef USE_XBRIDGE2
  // set timestamps in queue for inital backfilling
  initialFillupBackfillTimestamps();
#endif
  print_state(""); 
  Serial.printf("NFCReady = %d, BatOK = %d", NFCReady, BatteryOK);

  check_stack("#setup#");
  memAndStack("setup()");
}


/* ****************************************************************************** */

// reference: http://forum.arduino.cc/index.php?topic=226880.0
// topic: Stack address > 2k for ATMEGA328p?
// https://learn.adafruit.com/memories-of-an-arduino/measuring-free-memory

#define MAXSTACK 0x20003FFF       // stack begin at RFduino
#define STACKWARNLEVEL 300        // maximum stack size (1k on RFduino)
#define FKTSTACK 20               // entries for calling history
#define FNMLEN 20

boolean sreached = 0;

int minimumMemory = 8192;

char fktstack[FKTSTACK][FNMLEN];
unsigned int fktFree[FKTSTACK];
unsigned char fwind = 0;
unsigned char frind = 0;

void memAndStack(char *func)
{
  print_state("");
  Serial.printf("(%s) - free RAM = %d, min. RAM avail log: %d", func, freeMemory(), minimumMemory);
  int v;
  Serial.print(", Stack Size = ");
  // stack memory of RFduino begins at 0x20003FFF
  Serial.print((MAXSTACK-(int)&v));
}

/*
 * stack size check for ARM Cortec M0 processors (RFduino)
 * according to the Memory.h file from the RFduino library the RAM layout is like this
 * ram:
 * 20000000 - 20001FFF -> stack RAM storage
 * 20002000 - 20003FFF -> application RAM storage
 * where we use the application RAM storage
 */
void check_stack(char *fkt)
{
  unsigned int fm = freeMemory();

  if ( fm < minimumMemory )
    minimumMemory = fm;

  // check if enough memory is left for proper operation between heap top and stack bottom
  if ( fm < STACKWARNLEVEL ) {
    if ( !sreached ) {
      sreached = 1;
      Serial.printf("\r\n *** %s max stack size reached: %d left", fkt, fm);
      // print fnction call history
      while ( frind != fwind ) {
        Serial.printf("\r\n->%s (%d)", fktstack[frind], fktFree[frind]);
        if ( ++frind >= FKTSTACK )
          frind = 0;
      }
      Serial.printf("\r\n *** ->%s max stack size reached: %d left", fkt, fm);
    }
  }
  // store function call history
  if ( strlen(fkt) < FNMLEN )
    strcpy(&fktstack[fwind][0], fkt);
  else {
    memcpy(&fktstack[fwind][0], fkt, FNMLEN-1);
    fktstack[fwind][FNMLEN-1] = '\0';
  }
  fktFree[fwind] = fm;
  if ( ++fwind >= FKTSTACK )
    fwind = 0;
  // queue overflow, put read index one ahead
  if ( fwind == frind ) {
    if ( ++frind >= FKTSTACK )
      frind = 0;
  }
}

/* 
 * from: https://learn.adafruit.com/memories-of-an-arduino/measuring-free-memory
 */

#ifdef __arm__
// should use uinstd.h to define sbrk but Due causes a conflict
extern "C" char* sbrk(int incr);
#else  // __ARM__
extern char *__brkval;
#endif  // __arm__
  
int freeMemory() {
  char top;
#ifdef __arm__
  return &top - reinterpret_cast<char*>(sbrk(0));
#elif defined(CORE_TEENSY) || (ARDUINO > 103 && ARDUINO != 151)
  return &top - __brkval;
#else  // __arm__
  return __brkval ? &top - __brkval : &top - __malloc_heap_start;
#endif  // __arm__
}

/* ****************************************************************************** */

void loop()
{
  time_loop_started = mymillis();

  check_stack("#loop#");

  Serial.printf("\r\n%s loop #%u--RSSI:%f--%umV/%d%%--SocTemp:%fC--(%s,%s%s/%s,", \
    BTconnected ? "=== BLE" : "=== ...", loop_cnt, SoCData.rssi, SoCData.voltage, SoCData.voltagePercent, SoCData.temperatureC, \
    LB_NAME, LB_VERSION, LB_MINOR_VERSION, LB_DATETIME);
#ifdef RFD
  Serial.print("RFduino) ===");
#else
  Serial.print("Simblee) ===");
#endif

#ifdef USE_WDT
  restartWDT();
#endif

  // show memory for debug
  memAndStack("loop()");
  // display current data from RFduino
  getSoCData();
//  printSoCData();

  loop_cnt++;

#ifdef SHORTSTARTCYCLE
  // @FPV-UAV: system start with faster cycles, switch back after 10 rounds
  if ( (loop_cnt > 10) && (runPeriod != p->runPeriod) ) {
    runPeriod = p->runPeriod;
    print_state("switch back to normal cycle time "); Serial.print(runPeriod); Serial.print(" min");
  }
#endif

  // @bertroode: wait for serial output in process
  Serial.flush();
  delay(100);

  if ( BatteryOK ) {
    readAllData();
    if (NFCReady == 2) {
#ifdef USE_XBRIDGE2
      fillupMissingReadings(0);
#endif
      dataTransferBLE();
    }
    else {  print_state("no sensor data to transmit");  }
  }
  else {  print_state("low Battery - go sleep");  }
  // calculate sleep time ajusted to runPeriod cycle
  unsigned long sleep_time = (60000 * runPeriod) - (mymillis() - time_loop_started)%60000;

  // sanity check
  if ( sleep_time > 60000*10 ) {
    // in case of error set to 5 mins fix
    print_state(" *** error in sleep time calculationm (sleep "); Serial.print(sleep_time);
    Serial.print(" ms) set to 5 min fix");
    sleep_time = 5*60000L;
  }

  restartBtStack(0);

  print_state("loop #"); Serial.print(loop_cnt-1); Serial.print(" - end - NFCReady = ");
  Serial.print(NFCReady); Serial.print(", sleep for "); Serial.print(sleep_time/1000); Serial.print(" s");

  // @bertroode: wait for serial output in process
  Serial.flush();
  delay(100);

#ifdef SERIALOFF
  Serial.end();         // reduce power consumption
  delay(UARTDELAY);
#endif SERIALOFF

#ifdef RFD
  RFduino_ULPDelay(sleep_time);
#else
  Simblee_ULPDelay(sleep_time);
#endif

#ifdef SERIALOFF
  Serial.begin(9600);
  delay(UARTDELAY);
#endif SERIALOFF

  // @bertroode: reset CR95HF to be sure processing is stable /
  digitalWrite(PIN_SPI_SS, LOW);
  SPI.transfer(0x01);
  digitalWrite(PIN_SPI_SS, HIGH);
  delay(1);
}

#ifdef USE_XBRIDGE2
/*
 * search BG queue for missing entries (0), if within the last 8 h fill the values from the Libre sensor FRAM
 */
void fillupMissingReadings(boolean debug)
{
  int i, j;
  int nulls = 0;
  int difftime;
  unsigned long mytime;

  check_stack("#fmr#");

  // ??
  minutesSinceProgramStart = (mymillis() / (60000));
  mytime = minutesSinceProgramStart;
#ifdef DEBUG
  print_state("fillupMissingReadings() - current time: ");
  Serial.printf("%d (%d/%d), sensorData.minutesHistoryOffset: %d", mytime, mytime/15, mytime%15, sensorData.minutesHistoryOffset);
#endif DEBUG
  // check complete queue for missing BG readings (0 entries)
  for ( int i = 0 ; i < DXQUEUESIZE ; i++ ) {
    // get time of queue entry
    unsigned long timeOfMissingBG_sec = (Pkts.buffer[i].ms+500)/1000;
    // if there is a missing BG reading and a valid time
    if ( (Pkts.buffer[i].raw == 0) && (Pkts.buffer[i].ms != 0) ) {
      if ( debug ) {
        print_state("");
        Serial.printf("%d - timeOfMissingBG %d/%d ", i, timeOfMissingBG_sec/60, mytime);
      }
      nulls++;
      // if the missing value is not more than 15 min old search in trend array
      if ( (mymillis() - Pkts.buffer[i].ms)/1000 <= (15*60) ) {
        if ( debug )
          Serial.print(" - trend array");
        for ( j = 0 ; j < 16 ; j++ ) {
          unsigned long timeGoBackInTrend_sec = (minutesSinceProgramStart-j)*60;
          // trend array begins at current time - counting in minutes
          difftime = timeGoBackInTrend_sec - timeOfMissingBG_sec;
          difftime = abs(difftime);
          // find nearest entry
          if ( difftime < 31 ) {
            Pkts.buffer[i].raw = scale_bg(sensorData.trend[j]);
            if ( debug )
              Serial.printf(" - %d delta %d s, replacing with %d", i, difftime, (Pkts.buffer[i].raw+500)/1000);
            break;
          }
        }
      }
      else {
        if ( debug )
          Serial.print(" - history array");
        for ( j = 0 ; j < 32 ; j++ ) {
          unsigned long timeToHistoryEntry_sec = (minutesSinceProgramStart-(sensorData.minutesHistoryOffset+15*j))*60;
          difftime = timeToHistoryEntry_sec - timeOfMissingBG_sec; 
          difftime = abs(difftime);
          if ( difftime < 451 ) {
            Pkts.buffer[i].raw = scale_bg(sensorData.history[j]);
            if ( debug )
              Serial.printf("(%d/%d min) - delta %d s, set BG %d", i, timeToHistoryEntry_sec/60, difftime, (Pkts.buffer[i].raw+500)/1000);
            break;
          }
        }
        // not found or out of range (diiftime = 537 ...)
        if ( Pkts.buffer[i].raw == 0 ) {
          if ( debug )
            print_state(" *** not found, set to histroy index 31");
          Pkts.buffer[i].raw = scale_bg(sensorData.history[31]);
        }
      }
    }
  }
  if ( nulls > 0 ) {
    print_state("");
    Serial.printf("found %d reading(s) for backfilling, stored in queue ...", nulls);
  }
}
#endif USE_XBRIDGE2

/* ********************************** dump stuff ************************ */

void show_bg_queue(void)
{
  int i;
  int nulls = 0;

  check_stack("#sbgq#");

  print_state("BG Queue, read index: "); Serial.print(Pkts.read); Serial.print("\r\n");
  for ( i = 0 ; i < DXQUEUESIZE ; i++ ) {
    Serial.printf("%d %d s, ", Pkts.buffer[i].raw, (Pkts.buffer[i].ms+500)/1000);
    if ( (Pkts.buffer[i].raw == 0) && (Pkts.buffer[i].ms != 0) )
      nulls++;
    if ( i > 0 && !(i % 6))
      Serial.println("");
  }
  print_state("nulls detected: "); Serial.print(nulls);
}

void show_trend_history(void)
{
//  sensorData.minutesSinceStart;
  check_stack("#sth#");

  print_state("--->BG trend:\r\n");
  for (int i = 0; i < 16; i++) {
    if ( i > 0 && !(i%4) )
      Serial.println("");
    // 1 min distance
    Serial.printf(" %d (-%d min)", (scale_bg(sensorData.trend[i])+500)/1000, i);
  }
#if 0
  if ( 0 ) {
    long average;
    Serial.println("");
    for ( int i = 0 ; i < 16-5 ; i++ ) {
      average = 0;
      for ( int j = 0 ; j < 5 ; j++ ) {
          average += scale_bg(sensorData.trend[i+j]);
      }
      average /= 5;
      Serial.printf("%d avg: %d, ", i, (average+500)/1000);
    }
  }
#endif 0
  print_state("--->BG history:\r\n");
  // 15 min distance, starts at 18 min from now
  for (int i = 0; i < 32; i++) {
    if ( i > 0 && !(i%4) )
      Serial.println("");
    Serial.printf(" %d (-%d min)", (scale_bg(sensorData.history[i])+500)/1000, ((sensorData.minutesHistoryOffset+15*i)));
  }
}

/* ****************************** init section ********************************* */

#ifdef USE_SHADOWFRAM
/* 
 * shadow FRAM processing - init routine for selective nfc reading
 */
void initSensor(Sensor *sensor){

  check_stack("#initsenor#");

    (*sensor).nextTrend = 0;
    (*sensor).oldNextTrend = 0;
    (*sensor).nextHistory = 0;
    (*sensor).oldNextHistory = 0;
    (*sensor).minutesSinceStart = 0;
    (*sensor).oldMinutesSinceStart = 0;
    for (uint i = 0; i < sizeof((*sensor).uid); i++) {
        (*sensor).uid[i] = 0;
        (*sensor).oldUid[i] = 0;
    }
    for (uint i = 0; i < sizeof((*sensor).fram); i++) {
        (*sensor).fram[i] = 0;
    }
    for (uint i = 0; i < sizeof((*sensor).resultCodes); i++) {
        (*sensor).resultCodes[i] = 0;
    }
}
#endif

/*
 * initialize the send queue for backfilling
 * put in a timestamp in the past and a 0 entry
 * every 0 entry will be filled before sending with teh historical entries from the
 * Libre sensor FRAM
 */

#define FILL8H (8*12)     // fillup 8 h - the length of the 8 hour Libre sensor FRAM history over all

#ifdef USE_XBRIDGE2
void initialFillupBackfillTimestamps(void)
{
  unsigned long sim_time_ms;
  unsigned char store;

  check_stack("#ifubft#");

  // fillup queue to transfer the last 8 for test direct after system start
  print_state("prepare 8 h backfilling - current runtime = ");
  Serial.printf("%d min", mymillis()/60000L);
  store = sensorData.sensorStatusByte;
  sensorData.sensorStatusByte = 0x03; // needed for using the put_reading2queue() function without valid sensor data
  for ( int i = 0 ; i < FILL8H ; i++ ) {
    // create timestamps 0, 5, 10, 15, ... min relative to current time
    sim_time_ms = mymillis()-(long)(FILL8H-i)*300000L;
    put_reading2queue(sim_time_ms, 0, 0);
  }
  sensorData.sensorStatusByte = store;
  // set to end of initial queue
  minutesSinceProgramStart = (mymillis() / (60000));
}
#endif USE_XBRIDGE2

/* ************************* NFC/SPI handling **************************** */

#ifdef USE_WDT
void configWDT()
{
  print_state("configure watchdog timer");
  NRF_WDT->CONFIG = (WDT_CONFIG_SLEEP_Run << WDT_CONFIG_SLEEP_Pos) | (WDT_CONFIG_HALT_Pause << WDT_CONFIG_HALT_Pos); 
  NRF_WDT->CRV = 32768 * 60 * 11;         // 11 minut
  NRF_WDT->RREN |= WDT_RREN_RR0_Msk;                      
  NRF_WDT->TASKS_START = 1; 
}

void restartWDT()
{
//  print_state("restart watchdog timer");
  NRF_WDT->RR[0] = WDT_RR_RR_Reload;
}
#endif

/* ************************* NFC/SPI handling **************************** */

/*
 * send wake up pulse to CR95HF and configure to use SPI
 */
void NFC_wakeUP(int show)
{
  check_stack("#nfcWU#");
  if ( show )
    print_state("NFC_wakeUp()");
  digitalWrite(PIN_IRQ, HIGH);
  delay(10);
  digitalWrite(PIN_IRQ, LOW);
  delayMicroseconds(100);
  digitalWrite(PIN_IRQ, HIGH);
  delay(10);
  if ( show )
    Serial.print(" - done");
}

void send_NFC_PollReceive(byte *command, int commandLength)
{
//  print_state("send_NFC_PollReceive() - ");
  check_stack("#sndNFCPllRcv#");
  send_NFC_Command(command, commandLength);
  poll_NFC_UntilResponsIsReady();
  receive_NFC_Response();
}

#define NFCTIMEOUT 500
void new_send_NFC_PollReceive(byte *command, int commandLength)
{
//  print_state("send_NFC_PollReceive() - ");
  check_stack("#nwSndNFCPllRec#");
  digitalWrite(PIN_SPI_SS, LOW);
  SPI.transfer(0x00);
  for (int i = 0; i < commandLength; i++)
  {
    SPI.transfer(command[i]);
  }
  digitalWrite(PIN_SPI_SS, HIGH);
  delay(1);

  unsigned long ms = mymillis();
  byte rb;
  digitalWrite(PIN_SPI_SS , LOW);
  while ( (resultBuffer[0] != 8) && ((mymillis() - ms) < NFCTIMEOUT) )
  {
    rb = resultBuffer[0] = SPI.transfer(0x03);
    resultBuffer[0] = resultBuffer[0] & 0x08;
  }
  digitalWrite(PIN_SPI_SS, HIGH);
  delay(1);
  if ( mymillis() - ms > NFCTIMEOUT ) {
    Serial.print("\r\n *** poll timeout *** -> response ");
    Serial.print(rb);
  }
  digitalWrite(PIN_SPI_SS, LOW);
  SPI.transfer(0x02);
  resultBuffer[0] = SPI.transfer(0);
  resultBuffer[1] = SPI.transfer(0);
  for (byte i = 0; i < resultBuffer[1]; i++) resultBuffer[i + 2] = SPI.transfer(0);
  digitalWrite(PIN_SPI_SS, HIGH);
  delay(1);
  Serial.print("done");
}

void send_NFC_Command(byte *commandArray, int length)
{
//  check_stack("#nfcCmd#");
  digitalWrite(PIN_SPI_SS, LOW);
  SPI.transfer(0x00);
  for (int i = 0; i < length; i++) {
    SPI.transfer(commandArray[i]);
  }
  digitalWrite(PIN_SPI_SS, HIGH);
  delay(1);
}

void poll_NFC_UntilResponsIsReady()
{
  unsigned long ms = mymillis();
  byte rb;

//  check_stack("#pllNFCURisR#");

  digitalWrite(PIN_SPI_SS , LOW);
  while ( (resultBuffer[0] != 8) && ((mymillis() - ms) < NFCTIMEOUT) )
  {
    rb = resultBuffer[0] = SPI.transfer(0x03);
    resultBuffer[0] = resultBuffer[0] & 0x08;
  }
  digitalWrite(PIN_SPI_SS, HIGH);
  delay(1);
  if ( mymillis() - ms > NFCTIMEOUT ) {
    Serial.print("\r\n *** poll timeout *** -> response ");
    Serial.print(rb);
  }
}
void receive_NFC_Response()
{
//  check_stack("#recNFCRsp#");
  digitalWrite(PIN_SPI_SS, LOW);
  SPI.transfer(0x02);
  resultBuffer[0] = SPI.transfer(0);
  resultBuffer[1] = SPI.transfer(0);
  for (byte i = 0; i < resultBuffer[1]; i++) resultBuffer[i + 2] = SPI.transfer(0);
  digitalWrite(PIN_SPI_SS, HIGH);
  delay(1);
}

void print_NFC_WakeUpRegisterResponse()
{
  check_stack("#pnwrr#");
  print_state("print_NFC_WakeUpRegisterResponse() - ");
  Serial.printf("Result code (byte 0): %x", resultBuffer[0]);
  Serial.printf(", Length of data (byte 1): %d", resultBuffer[1]);
  if (resultBuffer[1] > 0)
  {
    for (int i = 2; i < 2 + resultBuffer[1]; i++)
    {
      Serial.printf("   Data byte %d: %x", i, resultBuffer[i]);
    }
  }
}

void SetNFCprotocolCommand(int show)
{
  check_stack("#stNFCPtCmd#");
  for (int t = 0; t < 9; t++)
  {
    if ( show )
      print_state("SetNFCprotocolCommand()");
    int length = 4;
    byte command[length];
    command[ 0] = 0x02;
    command[ 1] = 0x02;
    command[ 2] = 0x01;
    command[ 3] = 0x0F;
    send_NFC_PollReceive(command, sizeof(command));
    if ((resultBuffer[0] == 0) & (resultBuffer[1] == 0)) {
      if ( t > 0 ) {
        Serial.print(", Try = ");
        Serial.print(t);
      }
//      Serial.print(" - OK");
      NFCReady = 1;
      break;
    }
    else {
      Serial.print(" - error, resultBuffer: ");
      for (byte i = 0; i < 2; i++) {
        Serial.print(resultBuffer[i], HEX);
        Serial.print(" ");
      }
      Serial.print(", Try #");
      Serial.print(t);
      Serial.print(" - BAD RESPONSE TO SET PROTOCOL");
      NFCReady = 0; // NFC not ready
    }
  }
  if ( show )
    Serial.print(" - done");
}

/*
 * requests information about the CR95HF and its revision
 * command returns the device ID and the ROM CRC
 */
void runIDNCommand(int maxTrials)
{
  check_stack("#ridnc#");
  byte command[2];
  print_state("runIDNCommand() - ");
  command[0] = 0x01;
  command[1] = 0x00;
  delay(10);
//  Serial.printf("maxTrials: %d, RXBuffer[0]: %x", maxTrials, resultBuffer[0]);
  runIDNCommandUntilNoError(command, sizeof(command), maxTrials);
  Serial.print("done");
}

void runIDNCommandUntilNoError(byte *command, int length, int maxTrials)
{
  int count = 0;
  bool success;

  check_stack("#ridncune#");

  do
  {
    count++;
    memset(resultBuffer, 0, sizeof(resultBuffer));
    send_NFC_PollReceive(command, sizeof(command));
    success = idnResponseHasNoError();
  } while ( !success && (count < maxTrials));
  delay(10);
}

bool idnResponseHasNoError()
{
  check_stack("#idnrhne#");

  if (resultBuffer[0] == 0x00)
  {
    return true;
  }
  return false;
}

IDNDataType idnDataFromIDNResponse()
{
  check_stack("#idndfir#");

  idnData.resultCode = resultBuffer[0];
  for (int i = 0; i < 13; i++)
  {
    idnData.deviceID[i] = resultBuffer[i + 2];
  }
  idnData.romCRC[0] = resultBuffer[13 + 2];
  idnData.romCRC[1] = resultBuffer[14 + 2];
  return idnData;
}

#define NFCDEVICEID 20      // length of NFC device ID (12)

void printIDNData(IDNDataType idnData)
{
   char nfc[NFCDEVICEID];
   int i;

  check_stack("#prtIDNDta#");

  Serial.printf("\r\n\tNFC Device ID: %x", idnData.deviceID[0]);
  nfc[0] = (char) idnData.deviceID[0];
  for ( i = 1; i < 12; i++)
  {
    Serial.printf(":%x", idnData.deviceID[i]);
    nfc[i] = (char) idnData.deviceID[i];
  }
  nfc[i] = '\0';
  Serial.printf(" (%s)", nfc);
  Serial.printf("\r\n\tNFC Device CRC  %x:%x", idnData.romCRC[0], idnData.romCRC[1] );
}

/*
 * do a system information command
 * mainly to get the sensors serial number
 */

void runSystemInformationCommandUntilNoError(int maxTrials)
{
  check_stack("#rSysInfCmd#");

  memset(resultBuffer, 0, sizeof(resultBuffer));
  byte command[4];
  command[0] = 0x04;
  command[1] = 0x02;
  command[2] = 0x03;
  command[3] = 0x2B;
  delay(10);
  runNFCcommandUntilNoError(command, sizeof(command), maxTrials);
}

void runNFCcommandUntilNoError(byte *command, int length, int maxTrials)
{
  check_stack("#rNFCCmdUNErr#");

  int count = 0;
  bool success;
  do
  {
    delay(1);
    count++;
    send_NFC_PollReceive(command, sizeof(command));
    success = responseHasNoError();
  } while ( !success && (count < maxTrials));
  delay(1);
}

bool responseHasNoError()
{
//  check_stack("#rspHsNoErr#");

  if (resultBuffer[0] == 0x80)
  {
    if ((resultBuffer[2] & 0x01) == 0)
    {
      return true;
    }
  }
  return false;
}

SystemInformationDataType systemInformationDataFromGetSystemInformationResponse()
{
  check_stack("#sysInfDta#");

  SystemInformationDataType systemInformationData;
  systemInformationData.resultCode = resultBuffer[0];
  systemInformationData.responseFlags = resultBuffer[2];
  if (systemInformationData.resultCode == 0x80)
  {
    if ((systemInformationData.responseFlags & 0x01) == 0)
    {
      systemInformationData.infoFlags = resultBuffer[3];
      for (int i = 0; i < 8; i++)
      {
        systemInformationData.uid[i] = resultBuffer[11 - i];
      }
      systemInformationData.errorCode = resultBuffer[resultBuffer[1] + 2 - 1];
    }
    else
    {
      systemInformationData.errorCode = resultBuffer[3];
    }
    systemInformationData.sensorSN = decodeSN(systemInformationData.uid);
    sensorData.sensorSN = systemInformationData.sensorSN;
  }
  else
  {
    print_state(" *** error reading patch info, clearing UID");
    clearBuffer(systemInformationData.uid);
    systemInformationData.errorCode = resultBuffer[3];
  }
  return systemInformationData;
}

void printSystemInformationData(SystemInformationDataType systemInformationData)
{
  check_stack("#psid#");

  print_state("");
  Serial.printf("Sensor SN: %s", systemInformationData.sensorSN);
}

void clearBuffer(byte *tmpBuffer)
{
  check_stack("#clrb#");

  memset(tmpBuffer, 0, sizeof(tmpBuffer));
}

bool readSensorData()
{
  byte resultCode = 0;
  int trials = 0;
  int maxTrials = 10;
  bool crcResult = false;

  check_stack("#rsdta#");

  if ( !use_shadowfram ) {
    clearBuffer(dataBuffer);
    // @UPetersen: Footer sollte nur einmal gelesen werden, da er nicht verwendet wird
    for (int i = 0; i < 43; i++) {
      resultCode = ReadSingleBlockReturn(i);
      if (resultCode != 0x80 && trials < maxTrials) {
        i--;        // repeat same block if error occured, but
        trials++;   // not more than maxTrials times per block
      }
      else if (trials >= maxTrials) {
        break;
      }
      else {
        trials = 0;
        for (int j = 3; j < resultBuffer[1] + 3 - 4; j++) {
          dataBuffer[i * 8 + j - 3] = resultBuffer[j];
        }
      }
    }
    bool resultH = checkCRC16(dataBuffer, 0);
    bool resultB = checkCRC16(dataBuffer, 1);
    bool resultF = checkCRC16(dataBuffer, 2);

    if (resultH && resultB && resultF) 
      crcResult = true;
    else { 
      crcResult = false;
      Serial.printf(" - failed, CRC check (Header-Body-Footer) = %d %d %d", resultH, resultB, resultF);
    }
    if (crcResult) NFCReady = 2;
    else NFCReady = 1;
  }

#ifdef USE_SHADOWFRAM

  if ( use_shadowfram ) {
    #ifdef DEBUG
    print_state("read Libre sensor using shadow FRAM mechanism");
    #endif DEBUG

//    memcpy(sensor.uid, systemInformationData.uid, sizeof(sensor.uid));

    readHeader(RXBuffer, &sensor, SS_PIN);
    if (!checkCRC16(sensor.fram, (byte) 0)) 
      readHeader(RXBuffer, &sensor, SS_PIN); // one more time

    readBody(RXBuffer, &sensor, SS_PIN);
    if (!checkCRC16(sensor.fram, (byte) 1)) 
      readBody(RXBuffer, &sensor, SS_PIN); // one more time

    // @UPetersen: footer will not be used, shall only be read once at program start
    readFooter(RXBuffer, &sensor, SS_PIN);
    if (!checkCRC16(sensor.fram, (byte) 2)) 
      readFooter(RXBuffer, &sensor, SS_PIN); // one more time

    // // try again for blocks that had read errors
    reReadBlocksWhereResultCodeStillHasEnError(RXBuffer, &sensor, SS_PIN);

    // to be optimized - copy FRAM sensor data to thr original used array
    clearBuffer(dataBuffer);
    for (int i = 0; i < 43*8; i++) {
      dataBuffer[i] = sensor.fram[i];
    }
    bool resultH = checkCRC16(dataBuffer, 0);
    bool resultB = checkCRC16(dataBuffer, 1);
    bool resultF = checkCRC16(dataBuffer, 2);

    if (resultH && resultB && resultF) 
      crcResult = true;
    else { 
      crcResult = false;
      Serial.printf(" - failed, CRC check (Header-Body-Footer) = %d %d %d", resultH, resultB, resultF);
    }
   
    if (crcResult) NFCReady = 2;
    else NFCReady = 1;
  }

#endif /* USE_SHADOWFRAM */

  return crcResult;
}

/*
 * read a single block via NFC
 */
byte ReadSingleBlockReturn(int blockNum)
{
  int length = 5;
  byte command[length];

  check_stack("#rsbr#");

  command[0] = 0x04;
  command[1] = 0x03;
  command[2] = 0x03;
  command[3] = 0x20;
  command[4] = blockNum;
  send_NFC_Command(command, 5);
  poll_NFC_UntilResponsIsReady();
  receive_NFC_Response();
  delay(1);
  return resultBuffer[0];
}

/*
 * initialize the CR95HF chip / BM019 module
 */
void nfcInit()
{
  check_stack("#nfci#");

  configSPI();
  NFC_wakeUP(1);
  NFCReady = 0;
  SetNFCprotocolCommand(1);
  runIDNCommand(10);
  idnData = idnDataFromIDNResponse();
  printIDNData(idnData);
}

/*
 * send the CR95 to deep sleep mode
 */
void sendNFC_ToHibernate()
{
//  print_state("sendNFC-ToHibernate()");
  int length = 16;
  byte command[length];

  check_stack("#snfcthib#");

  command[ 0] = 0x07;
  command[ 1] = 0x0E;
  command[ 2] = 0x08;
  command[ 3] = 0x04;
  command[ 4] = 0x00;
  command[ 5] = 0x04;
  command[ 6] = 0x00;
  command[ 7] = 0x18;
  command[ 8] = 0x00;
  command[ 9] = 0x00;
  command[10] = 0x00;
  command[11] = 0x00;
  command[12] = 0x00;
  command[13] = 0x00;
  command[14] = 0x00;
  command[15] = 0x00;
  send_NFC_Command(command, sizeof(command));
}

/*
 * set SPI pinMode and paras
 */
void configSPI()
{
  check_stack("#cspi#");

  print_state("configSPI()");
  pinMode(PIN_SPI_SS, OUTPUT);
  pinMode(PIN_IRQ, OUTPUT);
  SPI.begin();
  SPI.setDataMode(SPI_MODE0);
  SPI.setBitOrder(MSBFIRST);
  SPI.setFrequency(1000);
  Serial.print(" - done");
}

/* ****************************** shadow FRAM code ****************************************** */

void sendSPICommand(int ssPin, byte *commandArray, int length) {
  check_stack("#sspic#");

    digitalWrite(ssPin, LOW);
    
    SPI.transfer(0x00);  // SPI control byte to send command to CR95HF
    
    for (int i=0; i<length; i++) {
        SPI.transfer(commandArray[i]);
    }
    digitalWrite(ssPin, HIGH);
    delay(1);
}

void pollSPIUntilResponsIsReady(int ssPin, byte *RXBuffer) {
  check_stack("#pspiurir#");
    
    digitalWrite(ssPin , LOW);
    
    while(RXBuffer[0] != 8) {
        RXBuffer[0] = SPI.transfer(0x03);  // Write 3 until
        RXBuffer[0] = RXBuffer[0] & 0x08;  // bit 3 is set in response byte
    }
    digitalWrite(ssPin, HIGH);
    delay(1);
}

void receiveSPIResponse(int ssPin, byte *RXBuffer) {
  check_stack("#rspir#");
    
    digitalWrite(ssPin, LOW);
    SPI.transfer(0x02);   // SPI control byte for read
    
    RXBuffer[0] = SPI.transfer(0);  // response code
    RXBuffer[1] = SPI.transfer(0);  // length of data
    
    for (byte i=0; i<RXBuffer[1]; i++)
        RXBuffer[i+2] = SPI.transfer(0);  // data
    
    digitalWrite(ssPin, HIGH);
    delay(1);
}


void sendPollReceiveSPINew(int ssPIN, byte *command, int commandLength, byte *RXBuffer) {
  check_stack("#sprspinw#");
    
    // step 1 send the command, which is stored in RXBuffer
    sendSPICommand(ssPIN, command, commandLength);
    
    // step 2, poll for data ready
    pollSPIUntilResponsIsReady(ssPIN, RXBuffer);
    
    // step 3, read the data into RXBuffer
    receiveSPIResponse(ssPIN, RXBuffer);
}


// Reads a (single) block of 8 bytes at the position blockNum from the FreeStyle Libre FRAM and copies
// the content to the corresponding position of the dataBuffer.
// If an error occurs (result code != 128), the reading process is repeated up to maxTrials times.
// The final result code is returned.
// blockNum    number of the block of the FreeStyle Libre FRAM to be read, possible values are 0 to 243.
// maxTrials   maximum number of reading attempts.
// RXBuffer    buffer used for sending and receiving data.
// fram        buffer of bytes the final block of data is copied to at the position corresponding to blockNum.
//             Example 1: If blockNumber is 0, the 8 bytes of data are copied into fram[0] to fram[7] (i.e. 0*8 to 0*8+7)
//             Example 2: If blockNumber is 17, the 8 bytes of data are copied into fram[136] to fram[143] (i.e. 17*8 = 136.. 17*8+7 = 143)
byte readSingleBlock(byte blockNum, int maxTrials, byte *RXBuffer, byte *fram, int ssPIN) {

    byte command[5];

  check_stack("#rsb#");

    command[ 0] = SENDRECEIVE;   // command code for send receive CR95HF command
    command[ 1] = 0x03;          // length of data that follows (3 bytes)
    // command[ 2] = 0x02;          // request Flags byte, single carrier, high data rate
    command[ 2] = 0x03;          // request Flags byte dual carrier, high data rate
    command[ 3] = 0x20;          // read single block Command for ISO/IEC 15693
    command[ 4] = blockNum;      // Block number

    int trials = 0;
    do {
        sendPollReceiveSPINew(ssPIN, command, sizeof(command), RXBuffer);
        trials++;
        delay(1);

        #ifdef DEBUG
        // Print to Serial for debugging
        if (RXBuffer[0] == 0x80)  {
            print_state("");
            Serial.printf("readSingleBlock: block #%d:", blockNum);
#ifdef FRAM_DEBUG
            for (byte i = 3; i < RXBuffer[1] + 3 - 4; i++) {
                Serial.print(RXBuffer[i], HEX);
                Serial.print(" ");
            }
#endif
        } else {
            print_statef("");
            Serial.printf("readSingleBlock: block #%d not available, response code is: ", blockNum);
            Serial.println(RXBuffer[0], HEX);
        }
//        Serial.println(" ");
        #endif

    } while (RXBuffer[0] != 0x80 && trials <= maxTrials); // repeat until result code == 0x80 (no error), but only maxTrials times

    if (RXBuffer[0] == 0x80) {
        for (byte i = 0; i < 8; i++) {
            fram[blockNum*8+i] = RXBuffer[i+3];  // succes case: copy received data to fram
        }
    } else {
        for (byte i = 0; i < 8; i++) {
            fram[blockNum*8+i] = 0;  // error case: fill fram with zeros
        }
    }
    return RXBuffer[0];  // result code of command response
}

#ifdef USE_SHADOWFRAM
void readHeader(byte *RXBuffer,  Sensor *sensor, int SS_Pin) {

    int maxTrials = 10; // (Re)read a block max four times

  check_stack("#rh#");
#ifdef DEBUG
    print_state("read header");
#endif
    // read first three blocks (0 , 1, 2)
    for (byte block = 0; block < 3; block++) {
        (*sensor).resultCodes[block] =  readSingleBlock((byte) block, maxTrials, RXBuffer, (*sensor).fram, SS_PIN);
        #ifdef DEBUG
        Serial.printf("resultCode 0x%x", (*sensor).resultCodes[block]);
        #endif
    }

    #ifdef DEBUG
    // Check crc16 for header
    if (!checkCRC16((*sensor).fram, (byte) 0)) {
        print_state(" ****************** CRC16 check for header failed");
    } else {
        print_state(" CRC16 check for header succeded");
    }
    #endif
}

// read complete body data
void readBody(byte *RXBuffer,  Sensor *sensor, int SS_Pin) {
  check_stack("#rb#");

    int maxTrials = 10; // (Re)read a block max four times
#ifdef DEBUG
    print_state("read body");
#endif
    // read block 0x27 (i.e. 39) with minute counter and block 3 with indices on trend and history data
    (*sensor).resultCodes[39] =  readSingleBlock((byte) 39, maxTrials, RXBuffer, &(sensor)->fram[0], SS_PIN);
    #ifdef FRAM_DEBUG
    Serial.printf("resultCode 0x%x", (*sensor).resultCodes[39]);
    #endif
    (*sensor).resultCodes[3] =  readSingleBlock((byte) 3, maxTrials, RXBuffer, (*sensor).fram, SS_PIN);
    #ifdef FRAM_DEBUG
    Serial.printf("resultCode 0x%x", (*sensor).resultCodes[39]);
    #endif

    // if no read erros continue with checking for new data
    if ((*sensor).resultCodes[3] == 0x80 && (*sensor).resultCodes[39] == 0x80) {

        (*sensor).oldNextTrend = (*sensor).nextTrend;
        (*sensor).nextTrend = (*sensor).fram[26];
        (*sensor).oldNextHistory = (*sensor).nextHistory;
        (*sensor).nextHistory = (*sensor).fram[27];

        #ifdef FRAM_DEBUG
        Serial.printf("resultCode Block 3: 0x%x\n\r", (*sensor).resultCodes[3]);
        Serial.printf("oldNextTrend:       0x%x\n\r", (*sensor).oldNextTrend);
        Serial.printf("nextTrend:          0x%x\n\r", (*sensor).nextTrend);
        Serial.printf("oldNextHistory:     0x%x\n\r", (*sensor).oldNextHistory);
        Serial.printf("nextHistory:        0x%x\n\r", (*sensor).nextHistory);
        #endif

        (*sensor).oldMinutesSinceStart = (*sensor).minutesSinceStart;
        (*sensor).minutesSinceStart = ((uint16_t) (*sensor).fram[317] << 8) + (uint16_t) (*sensor).fram[316]; // bytes swapped
        uint16_t minutesSinceLastReading = (*sensor).minutesSinceStart - (*sensor).oldMinutesSinceStart;

        #ifdef FRAM_DEBUG
        Serial.printf("resultCode Block 39:     0x%x\n\r", (*sensor).resultCodes[39]);
        Serial.printf("oldMinutesSinceStart:    0x%x\n\r", (*sensor).oldMinutesSinceStart);
        Serial.printf("minutesSinceStart:       0x%x\n\r", (*sensor).minutesSinceStart);
        Serial.printf("minutesSinceLastReading: 0x%x\n\r", minutesSinceLastReading);
        #endif

        // amount of new trend data
        byte trendDelta = (*sensor).nextTrend - (*sensor).oldNextTrend;
        byte trendsSinceLastReading = (trendDelta >= 0) ? trendDelta : trendDelta + 16; // compensate ring buffer
        if (minutesSinceLastReading > 15) trendsSinceLastReading = 16;  // Read all 16 trend data if more than 15 minute have passed

        #ifdef FRAM_DEBUG
        Serial.printf("trendDelta:             0x%x\n\r", trendDelta);
        Serial.printf("trendsSinceLastReading: 0x%x\n\r", trendsSinceLastReading);
        #endif

        // if there is new trend data, read the new trend data, along the ring buffer from old to new
        if (trendsSinceLastReading > 0) {
            int firstTrend = (*sensor).oldNextTrend;
            int firstTrendByte = 28  +  (6 * firstTrend);
            int lastTrendByte = firstTrendByte + (6 * trendsSinceLastReading) -1; // Does not consider the ring buffer (this is considered later in the reading loop)

            byte firstTrendBlock = firstTrendByte / 8; // integer division without remainder
            byte lastTrendBlock = lastTrendByte / 8;   // integer division without remainder

            #ifdef FRAM_DEBUG
            Serial.printf("firstTrend:      0x%x\n\r", firstTrend);
            Serial.printf("firstTrendByte:  0x%x\n\r", firstTrendByte);
            Serial.printf("lastTrendByte:   0x%x\n\r", lastTrendByte);
            Serial.printf("firstTrendBlock: 0x%x\n\r", firstTrendBlock);
            Serial.printf("lastTrendBlock:  0x%x\n\r", lastTrendBlock);
            #endif

            // read the trend blocks that are new since last reading, but no more than 100 times to avoid endless loop in error case
            byte trendBlock = firstTrendBlock;
            byte count = 0;
            while (count <= 100) {
                if (trendBlock != 3) { // block 3 was read already and can be skipped
                    (*sensor).resultCodes[trendBlock] =  readSingleBlock(trendBlock, maxTrials, RXBuffer, (*sensor).fram, SS_PIN);
                    #ifdef FRAM_DEBUG
                    Serial.printf("resultCode Block 0x%x: 0x%x", trendBlock, (*sensor).resultCodes[trendBlock]);
                    #endif
                }
                if (trendBlock == lastTrendBlock) break;
                trendBlock++;
                if (trendBlock > 15) trendBlock = trendBlock -13; // block 16 -> start over at block 3
                count++;
            }
        }

        // amount of new history data
        byte historyDelta = (*sensor).nextHistory - (*sensor).oldNextHistory;
        byte historiesSinceLastReading = (historyDelta >= 0) ? historyDelta : historyDelta + 32; // compensate ring buffer

        // new: ensure to read all history blocks after program start
        if ( minutesSinceLastReading > 31*15 )
          historiesSinceLastReading = 32;

        #ifdef FRAM_DEBUG
        Serial.printf("\r\nhistoryDelta:              0x%x\n\r", historyDelta);
        Serial.printf("historiesSinceLastReading: 0x%x\n\r", historiesSinceLastReading);
        #endif

        // if there is new trend data, read the new trend data
        if (historiesSinceLastReading > 0) {
            int firstHistory = (*sensor).oldNextHistory;
            int firstHistoryByte = 124  +  (6 * firstHistory);
            int lastHistoryByte = firstHistoryByte + (6 * historiesSinceLastReading) -1; // Does not consider the ring buffer (this is considered later in the reading loop)

            byte firstHistoryBlock = firstHistoryByte / 8; // integer division without remainder
            byte lastHistoryBlock = lastHistoryByte / 8;   // integer division without remainder

            #ifdef FRAM_DEBUG
            Serial.printf("firstHistory:       0x%x\n\r", firstHistory);
            Serial.printf("firstHistoryByte:   0x%x\n\r", firstHistoryByte);
            Serial.printf("lastHistoryByte:    0x%x\n\r", lastHistoryByte);
            Serial.printf("firstHistoryBlock:  0x%x\n\r", firstHistoryBlock);
            Serial.printf("lastHistoryBlock:   0x%x\n\r", lastHistoryBlock);
            #endif

            // read the history blocks that are new since last reading, but no more than 100 times
            byte historyBlock = firstHistoryBlock;
            byte count = 0;
            while (count <= 100) {
                if (historyBlock != 39) { // block 39 was read already and can be skipped
                    (*sensor).resultCodes[historyBlock] =  readSingleBlock(historyBlock, maxTrials, RXBuffer, (*sensor).fram, SS_PIN);
                }
                if (historyBlock == lastHistoryBlock) break;
                historyBlock++;
                if (historyBlock > 39) historyBlock = historyBlock -25; // block 40 -> start over at block 15
                count++;
            }
        }

        #ifdef DEBUG
        // Check crc16
        if (!checkCRC16((*sensor).fram, (byte) 1)) {
            print_state(" ********************** CRC16 check for body failed");
        } else {
            print_state(" CRC16 check for body succeded");
        }
        #endif
    }
}

void readFooter(byte *RXBuffer,  Sensor *sensor, int SS_Pin) {
  check_stack("#rf#");

    int maxTrials = 10; // (Re)read a block max four times
#ifdef DEBUG
    print_state("read footer");
#endif
    // read three blocks of footer (40, 41, 42)
    for (byte block = 40; block < 43; block++) {
        (*sensor).resultCodes[block] =  readSingleBlock((byte) block, maxTrials, RXBuffer, (*sensor).fram, SS_PIN);
        #ifdef DEBUG
        Serial.printf("resultCode 0x%x", (*sensor).resultCodes[block]);
        #endif
    }

    #ifdef DEBUG
    // Check crc16 for header
    if (!checkCRC16((*sensor).fram, (byte) 2)) {
        print_state(" ************************* CRC16 check for footer failed");
    } else {
        print_state(" CRC16 check for footer succeded");
    }
    #endif
}

void reReadBlocksWhereResultCodeStillHasEnError(byte *RXBuffer,  Sensor *sensor, int SS_Pin) {
    int maxTrials = 3;
  check_stack("#rrbwrcshee#");

    for (byte block = 0; block < 40; block++) {
        if ((*sensor).resultCodes[block] != 0x80) {
            (*sensor).resultCodes[block] =  readSingleBlock(block, maxTrials, RXBuffer, (*sensor).fram, SS_PIN);
#ifdef DEBUG
           Serial.printf("Reread block #0x%x with result code 0x%x", block, (*sensor).resultCodes[block]);
#endif
        }
    }
}
#endif /* USE_SHADOWFRAM */

/* ***************************** end of shawdow FRAM code ************************************** */

/* ******************** get and display the System on Chip internal data: temp, Vcc, ... ******* */

// simple low-level subroutine to get the VDD supply voltage
uint16_t readVDD() {
  uint16_t value;

  check_stack("#rvdd#");

  // Compare VDD against VBG(1.2v reference) at 10 bit resolution
  NRF_ADC->CONFIG = 0x0000001A;
  NRF_ADC->ENABLE = 0x00000001; // Enable the ADC
  NRF_ADC->TASKS_START = 1; // Start a conversion
  while(NRF_ADC->BUSY == 1){} // Busy Wait for completion
  delay(1); // needed if used on RFduino for some reason.
  value = (uint16_t)NRF_ADC->RESULT; // Read the ADC Result
  NRF_ADC->TASKS_STOP = 1; // Turn off ADC
  NRF_ADC->ENABLE = 0; // Disable the ADC
  return(value);
}

void getSoCData()
{
  check_stack("#gsocd#");

  analogReference(VBG);
  analogSelection(VDD_1_3_PS);
#ifdef RFD
  uint16_t sensorValue1 = readVDD();
//  print_state(""); Serial.printf("readVDD() = %d, ", sensorValue1);
  int sensorValue = analogRead(1);
//  Serial.printf("analogRead(1) = %d, ", sensorValue);
  SoCData.voltage = sensorValue * (360 / 1023.0) * 10;
  SoCData.temperatureC = RFduino_temperature(CELSIUS);
  SoCData.temperatureF = RFduino_temperature(FAHRENHEIT);
#else
  uint16_t sensorValue = readVDD();
  SoCData.voltage = sensorValue * (380 / 1023.0) * 10;
  SoCData.temperatureC = Simblee_temperature(CELSIUS);
  SoCData.temperatureF = Simblee_temperature(FAHRENHEIT);
#endif
//  Serial.printf(", TempC = %f", SoCData.temperatureC);
  SoCData.voltagePercent = map(SoCData.voltage, MIN_VOLTAGE, MAX_VOLTAGE, 0, 100);
//  Serial.printf(", Batt = %d", SoCData.voltagePercent);
  if (SoCData.voltage < MIN_VOLTAGE) 
    BatteryOK = false;
  else 
    BatteryOK = true;
}

void printSoCData(void)
{
  check_stack("#prtSoCDt#");

  print_state("SoC Data:");
  Serial.printf("\r\n\tU[mv]: %d (%d%%)", SoCData.voltage, SoCData.voltagePercent);
  Serial.printf("\r\n\tSoC Temp[C]: %f Temp[F]: %f", SoCData.temperatureC, SoCData.temperatureF);
  Serial.printf("\r\n\tRSSI: %f", SoCData.rssi);
}

int scale_bg(int glucose)
{
  check_stack("#sbg#");
  return((glucose/8.5f)*1000.0f);
}

void apply_spike_filter(void)
{
  check_stack("#asf#");

  if ( firstRun == 1  ) {
    firstRun = 0;
    lastGlucose = sensorData.trend[0];
  }
  else {
    lastGlucose = currentGlucose;
  }
  currentGlucose = sensorData.trend[0];

  if ( ((lastGlucose-currentGlucose) > SPIKE_HEIGHT) || ((currentGlucose-lastGlucose) > SPIKE_HEIGHT) ) {
    print_state("spike detected: cG "); Serial.print(currentGlucose); Serial.print(", lG ");
    Serial.print(lastGlucose);
    currentGlucose = lastGlucose;
  }
}

/* *************************** process and decode the sensor data informations ********************** */

void readAllData()
{
  check_stack("#rdAllDta#");

  NFC_wakeUP(0);
  NFCReady = 0;
  SetNFCprotocolCommand(0);

//  print_statef("read patch info");
  runSystemInformationCommandUntilNoError(10);
  systemInformationData = systemInformationDataFromGetSystemInformationResponse();
  printSystemInformationData(systemInformationData);

#ifdef USE_SHADOWFRAM
  // get current sensor SN
  memcpy(sensor.uid, systemInformationData.uid, sizeof(sensor.uid));
  // check if sensor SN has changed
  bool identical = 1;
  for ( int i = 0 ; i < 8 ; i++ ) {
    if ( sensor.uid[i] != sensor.oldUid[i] ) {
      print_state("sensor UID missing or changed, ");
      identical = 0;
      break;
    }
  }
  // if we have a new or changed sensor UID initialize
  if ( !identical ) {
    Serial.print("reinitialize sensor data structure");
    initSensor(&sensor);
  }
  // after reinitialization store the current UID for the next loop 
  memcpy(sensor.oldUid, systemInformationData.uid, sizeof(sensor.uid));
#endif USE_SHADOWFRAM

  sensorData.sensorDataOK = readSensorData();

  if ( sensorData.sensorDataOK ) {
    decodeSensor();
#ifdef USE_SPIKE_FILTER
    apply_spike_filter();
#endif USE_SPIKE_FILTER 
    put_reading2queue(mymillis(), sensorData.trend[0], 1);
  }
  else {
    print_state("no sensor data received, put 0 to queue");
    put_reading2queue(mymillis(), 0, 1);
  }
  sendNFC_ToHibernate();
}

/*
 * print some sensor data
 */
void displaySensorData()
{
  check_stack("#dsd#");

  if (!sensorData.sensorDataOK) 
    print_state("Sensor data error");
  else {
//    print_state("");
//    Serial.printf("BG raw reading: %d, therm. temp est.: %f", sensorData.trend[0], sensorData.trendTemperature[0]);
  }
}

/*
 * calculate the estimated sensor temperature using Ettores formula derived from the TI data sheet
 * has to be proofed, experimental!!!
 */
float decode_temperature(byte *buf)
{
  float currentTemp;
  float NTC;
  float Temp;

  check_stack("#dtemp#");

  currentTemp = ((buf[1]*256)+buf[0]) & 0x3FFF;
  NTC = currentTemp * 11.44;
  Temp = 22.124 * log(309443.81/NTC);

  return(Temp);
}

/*
 * decode the sensors serial number (according to @UPetersen's reference)
 */

static char decodedSensorSN[15];

char *decodeSN(byte *data)
{
  byte uuid[8];
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

  check_stack("#decSN#");

  for (int i = 2; i < 8; i++)  
    uuidShort[i - 2] = data[i];
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

/*
 * put the current BG to the send queue
 * glucose of 0 values indicates a missed reading which has to be filled from the trend/history buffer
 * of the Libre sensor before sending (backfilling)
 */
int put_reading2queue(unsigned long current_time_ms, int glucose, int debug)
{
  check_stack("#prtoq#");

  Pkts.buffer[Pkts.write].raw = scale_bg(glucose);
  Pkts.buffer[Pkts.write].ms = current_time_ms;
  pkt_time = current_time_ms;

#ifndef USE_DEAD_SENSOR
  // test for dead sensor, check the sensor status byte
  if ( sensorData.sensorStatusByte != 0x03 ) {
    print_state(" *** sensor not ready: ");
    Serial.printf("%d", sensorData.sensorStatusByte);
    Pkts.buffer[Pkts.write].raw = 0; 
    return(0);
  }
#endif

  if ( debug ) {
    print_state("");
    Serial.printf("got glucose(%f) at %d min, stored at %d, inc. write to ", \
      scale_bg(glucose)/1000.0, current_time_ms/60000, Pkts.write); 
  }
  // so increment write position for next round...
  if ( ++Pkts.write >= DXQUEUESIZE )
    Pkts.write = 0;
  if ( debug ) {
    Serial.print(Pkts.write); Serial.print(" - ");
  }
  if (Pkts.read == Pkts.write) {
    if ( debug )
      print_state("queue overflow, incrementing read overwriting oldest entry");
    if ( ++Pkts.read >= DXQUEUESIZE ) //overflow in ringbuffer, overwriting oldest entry, thus move read one up
      Pkts.read = 0;
  }
}

/*
 * check the sensor status byte which indicates if the sensor is working or dead
 */
void decodeSensorHeader()
{
  check_stack("#dsh#");

  sensorData.sensorStatusByte = sensorDataHeader[4];
//  print_state("decodeSensorHeader()"); 
  switch (sensorData.sensorStatusByte)
  {
    case 0x01:
      strcpy(sensorData.sensorStatusString, "not yet started");
      break;
    case 0x02:
      strcpy(sensorData.sensorStatusString, "starting");
      break;
    case 0x03:      // normal operation, 14 days and 12 h
      strcpy(sensorData.sensorStatusString, "READY");
      break;
    case 0x04:      // sensor operation is terminated, this status remains the next 12 h
      strcpy(sensorData.sensorStatusString, "EXPIRED");
      break;
    case 0x05:      // sensor operation is terminated, end status
      strcpy(sensorData.sensorStatusString, "SHUTDOWN");
      break;
    case 0x06:
      strcpy(sensorData.sensorStatusString, "failure");
      break;
    default:
      strcpy(sensorData.sensorStatusString, "unknown state");
      break;
  }
  Serial.printf(" - Sensor status: %s", sensorData.sensorStatusString);
}

/*
 * work on the Libre sensor body data
 * extract current BG value
 */
void decodeSensorBody()
{
  int j;
//  print_state("decodeSensorBody() - ");
  byte pomiar[6];

  check_stack("#dsb#");

  // calculate block indices for trend/history
  sensorData.nextTrend = sensorDataBody[2];
  sensorData.nextHistory = sensorDataBody[3];

  // calculate sensor minutes eplapsed
  byte minut[2];
  minut[0] = sensorDataBody[293];
  minut[1] = sensorDataBody[292];
  sensorData.minutesSinceStart = minut[0] * 256 + minut[1];
  
  // store sensor livetime at program start
  if ( startMinutesSinceStart == 0 ) {
    startMinutesSinceStart = sensorData.minutesSinceStart;
    startMillisSinceStart = mymillis();
    print_state(" *** set initial minutesSinceStart to "); 
    Serial.printf("%d min, %d ms", startMinutesSinceStart, startMillisSinceStart);    
  }
  // count the minutes from program start using sensor minutes counter
#ifdef SIM_MINUTES_UPCOUNT
  // simulate upcounting minutes using a dead sensor
  sensorData.minutesSinceStart += (loop_cnt-1)*runPeriod;
#endif
  minutesSinceProgramStart = sensorData.minutesSinceStart - startMinutesSinceStart;
  // calculate minute distance to most recent history entry
  sensorData.minutesHistoryOffset = (sensorData.minutesSinceStart - 3) % 15 + 3 ;
  // copy 16 trend data
  int index = 0;
  for (int i = 0; i < 16; i++)                              
  {
    index = 4 + (sensorData.nextTrend - 1 - i) * 6;
    if (index < 4) index = index + 96;
    for (int k = index; k < index + 6; k++) 
      pomiar[k - index] = sensorDataBody[k];
    // 13 Bits for raw BG reading are valid with the Libre sensor
    // meanwhile also hardcoded in xDrip+ (manual scan via NFC and blucon)
    sensorData.trend[i] = ((pomiar[1] << 8) & 0x1F00) + pomiar[0];
  	sensorData.trendTemperature[i] = decode_temperature(&pomiar[3]);

    // print rawdata
    if ( false ) {
      Serial.printf("\r\nb #%d -> trend:[%x](%d)\t", i, sensorData.trend[i], sensorData.trend[i]); 
      Serial.print("[");
      for ( j = 0 ; j < 6 ; j++) {
        if ( pomiar[j] <= 0x0f )
          Serial.print("0");
        Serial.print(pomiar[j], HEX);
        Serial.print(" ");
      }
      Serial.print("]");
      Serial.printf(" flg1: %b, temp: %f, flg: %b", pomiar[2], decode_temperature(&pomiar[3]), pomiar[5]);
    }
  }

  // copy 32 history data
  index = 0;
  for (int i = 0; i < 32; i++)
  {
    index = 100 + (sensorData.nextHistory - 1 - i) * 6;
    if (index < 100) index = index + 192;
    byte pomiar[6];
    for (int k = index; k < index + 6; k++) pomiar[k - index] = sensorDataBody[k];
    sensorData.history[i] = ((pomiar[1] << 8) & 0x0F00) + pomiar[0];
  }
}

void decodeSensorFooter()
{
  check_stack("#dsf#");
}

/*
 * process the sensor data
 */
void decodeSensor()
{
  check_stack("#ds#");

  for (int i = 0; i < 24; i++) sensorDataHeader[i] = dataBuffer[i];
  for (int i = 24; i < 320; i++) sensorDataBody[i - 24] = dataBuffer[i];
  for (int i = 320; i < 344; i++) sensorDataFooter[i - 320] = dataBuffer[i];
  decodeSensorHeader();
  decodeSensorBody();
  decodeSensorFooter();
  displaySensorData();
}

/* ************************** CRC check ************************************ */

uint16_t crc16table[256] =
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

uint16_t computeCRC16(void *bytes, byte type)
{
  int number_of_bytes_to_read = 0;
  int offset = 0;

  check_stack("#ccrc16#");

  if (type == 0)
  {
    number_of_bytes_to_read = 24;
    offset = 0;
  }
  else if (type == 1)
  {
    number_of_bytes_to_read = 296;
    offset = 24;
  }
  else if (type == 2)
  {
    number_of_bytes_to_read = 24;
    offset = 320;
  }
  byte *data = (byte*) bytes;
  uint16_t crc = 0xffff;
  for (int i = offset + 2; i < number_of_bytes_to_read + offset; ++i)           // first two bytes = crc16 included in data
  {
    crc = (uint16_t)((crc >> 8) ^ crc16table[(crc ^ data[i]) & 0xff]);
  }
  uint16_t reverseCrc = 0;
  for (int i = 0; i < 16; i++)
  {
    reverseCrc = (uint16_t)((uint16_t)(reverseCrc << 1) | (uint16_t)(crc & 1));
    crc >>= 1;
  }
  return reverseCrc;
}

bool checkCRC16(void *bytes, byte type)
{
  int number_of_bytes_to_read = 0;
  int offset = 0;

  check_stack("#ccrc16#");

  if (type == 0)
  {
    number_of_bytes_to_read = 24;
    offset = 0;
  }
  else if (type == 1)
  {
    number_of_bytes_to_read = 296;
    offset = 24;
  }
  else if (type == 2)
  {
    number_of_bytes_to_read = 24;
    offset = 320;
  }

  byte *data = (byte*) bytes;
  uint16_t x = data[0 + offset] | (data[1 + offset] << 8);
  uint16_t crc16calculated = computeCRC16(bytes, type);
  if (crc16calculated == x) return true;
  else return false;
}

/* ********************************* BLE handling, send and receive ********************** */

void setupBluetoothConnection()
{
  check_stack("#sblec#");

  print_state("configure and start BLE stack");
#ifdef RFD
  if (protocolType == 1) RFduinoBLE.deviceName = LB_NAME;
#ifdef USE_XBRIDGE2
  Serial.print(" - xbridge/wixel device");
  RFduinoBLE.advertisementData = LB_ADVERT;
  RFduinoBLE.customUUID = "0000ffe0-0000-1000-8000-00805f9b34fb";
#else /* USE_XBRIDGE2 */
  Serial.print(" - LimiTTer device");
  RFduinoBLE.deviceName = "LimiTTer";
  RFduinoBLE.advertisementData = "data";
  RFduinoBLE.customUUID = "c97433f0-be8f-4dc8-b6f0-5343e6100eb4";
#endif /* USE_XBRIDGE2 */
  RFduinoBLE.advertisementInterval = MILLISECONDS(500);
  RFduinoBLE.txPowerLevel = 4;
  RFduinoBLE.begin();
#else /* RFD */
  if (protocolType == 1) SimbleeBLE.deviceName = LB_NAME;
#ifdef USE_XBRIDGE2
  Serial.print(" - setting xbridge/wixel device");
  SimbleeBLE.advertisementData = LB_ADVERT;
  SimbleeBLE.customUUID = "0000ffe0-0000-1000-8000-00805f9b34fb";
#else /* USE_XBRIDGE2 */
  Serial.print(" - setting LimiTTer device");
  SimbleeBLE.deviceName = "LimiTTer";
  SimbleeBLE.advertisementData = "data";
  SimbleeBLE.customUUID = "c97433f0-be8f-4dc8-b6f0-5343e6100eb4";
#endif /* USE_XBRIDGE2 */
  SimbleeBLE.advertisementInterval = MILLISECONDS(500);
  SimbleeBLE.txPowerLevel = 4;
  SimbleeBLE.begin();
#endif /* RFD */
  Serial.print(" - done");
}

/*
 * check if we are connected to BLE and send then the current queue entries
 */
void dataTransferBLE()
{
  check_stack("#dtble#");

  if ( !BTconnected ) {
    print_state("");
    Serial.printf("dataTransferBLE(), wait 100 s for BLE connect (protocolType = %d) ...", protocolType);
  }
  for (int i = 0; i < 100; i++) {
    if (BTconnected) {
      if (protocolType == 1) {
        sendToXdripViaBLE();
      }
      else {
        print_state("");
        Serial.printf(" *** wrong protocol type %d", protocolType);
      }
      break;
    }
    else {
      waitDoingServices(1000, 1);
    }
  }
  NFCReady = 1;
}

void restartBtStack(boolean waitForBT)
{
  // if we loosed the BLE connection during transfer or whatever reason restart the BLE stack here
  if (!BTconnected)
  {
    print_state("restarting BT stack, stop BLE");
    // check for endless loop
    while (!RFduinoBLE.radioActive) {};
    while (RFduinoBLE.radioActive) {};
    RFduinoBLE.end();
    setupBluetoothConnection();
    // wait for max 40 s for BLE reconnect
    if ( waitForBT ) {
      Serial.printf(" - waiting up to 50 s for reconnect");
      waitDoingServicesInterruptible(50000, &BTconnected, 1);
    }
  }
}

/*
 * send the BG reading to xDrip+
 */
void sendToXdripViaBLE(void)
{
  check_stack("#stxvble#");

#ifndef USE_XBRIDGE2
  TxBuffer[0] = '\0';
  sprintf(TxBuffer, "%d %d %d %d", \ 
    sensorData:TREND[0] * 100, SoCData.voltage, SoCData.voltagePercent, sensorData.minutesSinceProgramStart/10);

  print_state("");
  Serial.printf("sendToXdripViaBLE: %s", TxBuffer);
#ifdef RFD
  // send is queued (the ble stack delays send to start of next tx window)
  while ( !RFduinoBLE.send(TxBuffer, strlen(TxBuffer)) ) {
    // check for endless loop needed!
    ;
  }
  // wait for end of BLE stack time slot (roughly 6 ms)
  while ( RFduinoBLE.radioActive);
#else
  SimbleeBLE.send(TxBuffer, strlen(TxBuffer));
  while ( SimbleeBLE.radioActive);
#endif
#else /* USE_XBRIDGE2 */
  boolean resend_pkt = 0;
  int more_than_one = 0;
  // if we are connected via BLE, the queue contains BG readings to be send and we have enough time left
  while ((Pkts.read != Pkts.write) && BTconnected && ((mymillis() - pkt_time) < ((DXQUEUESIZE + 1) * 5000) )) {
    got_ack = 0;
    unsigned long got_ack_time;

    // dont send a 0 entry
    if ( Pkts.buffer[Pkts.read].raw ) {

      // waiting time before next packet will be sended
      if ( more_than_one++ )
        waitDoingServices(100, 1);
      got_ack_time = mymillis();
      print_state("");
      Serial.printf("try sending [%d BG / -%d min / %d] - ", Pkts.buffer[Pkts.read].raw/1000, (mymillis() - Pkts.buffer[Pkts.read].ms)/60000, Pkts.read);
//      Serial.printf("[%d/-%d]", Pkts.buffer[Pkts.read].raw/1000, (mymillis() - Pkts.buffer[Pkts.read].ms)/60000);

      sendBgViaBLE(&Pkts.buffer[Pkts.read]);

#ifdef SEND_LIMITTER_SA
      last_raw = Pkts.buffer[Pkts.read].raw;
#endif
      // wait 10 s for ack
      int j;
      for ( j = 0 ; j < 10 ; j++ ) {
        waitDoingServicesInterruptible(1000, &got_ack, 1);
        if ( !BTconnected ) {
          print_state("BLE connection lost during 10 s wait for ack, go to sleep");
          break;
        }
      }
    }
    // when it was 0 increment read counter
    else {
      print_state("do not send a raw value containing 0, increment read pointer instead, simulate ACK");
      got_ack = 1;
    }

    if (got_ack) {
      Serial.printf("ACK / %d ms", mymillis()-got_ack_time);
//      Serial.printf(" (read %d, write %d), inc read to ", Pkts.read, Pkts.write);
      if ( ++Pkts.read >= DXQUEUESIZE )
        Pkts.read = 0;     //increment read position since we got an ack for the last package
//      Serial.printf("%d - ", Pkts.read);
      resend_pkt = 0;
    }
    else {
      if ( !resend_pkt ) {
        print_state("no ACK received, try again");
        resend_pkt = 1;
        // restart BT stack in case of BT transfer breaks after some blocks sended
        restartBtStack(1);
        // wait additional 5 s for DexCollectionService to sttle
        if ( BTconnected )
          delay(5000);
      }
      else {
        print_state("no ACK received, try again next wakeup");
        resend_pkt = 0;
        break;
      }
    }
  } /* while (send all packets available) */
//  print_state("end of while schleife");
//  waitDoingServices(100, 1);

#ifdef SEND_LIMITTER_SA
  // transfer the sensor livetime with the last successful tranmitted BG reading
  // currently not supported as an wixel device
  if ( ((loop_cnt % 12) == 0) && (last_raw != 0) ) {
    print_state("send LimiTTer type sensor age ...");
//    waitDoingServices(100, 1);
    if ( got_ack ) {
      char packet[30];
      // build a LimiTTer string
      packet[0] = '\0';
      batteryPcnt = min(map(SoCData.voltage, MIN_VOLTAGE, MAX_VOLTAGE, 0, 100), 100); // Convert voltage to percentage
      sprintf(packet, "%d 216 %d %d", last_raw, batteryPcnt, sensorData.minutesSinceStart);
      print_state("");
      Serial.printf("send sensor lifetime in LimiTTer ASCII format: [%s]", packet);
      send_data((unsigned char*)packet, strlen(packet));
      waitDoingServices(1000, 1);
      last_raw = 0;
    }
  }
#endif SEND_LIMITTER_SA

#endif /* USE_XBRIDGE2 */
}

// BLE char buffer status
boolean bleCharAvailable(void)
{
  check_stack("#bca#");

  if ( bleBufRi == bleBufWi )
    return (0);
  else
    return (1);
}

// read one char form BLE incoming buffer
unsigned char bleBufRead(void)
{
  unsigned char ret = 0xFF;

  check_stack("#bbr#");

  if ( bleBufRi != bleBufWi ) {
    ret = bleBuf[bleBufRi++];
    if ( show_ble) {
      Serial.print(ret, HEX); Serial.print(" ");
    }
    if ( bleBufRi >= BLEBUFLEN )
      bleBufRi = 0;
  }
  return (ret);
}

// write char into BEL char buffer, called from IRQ
void bleBufWrite(unsigned char c)
{
  check_stack("#bbw#");

  bleBuf[bleBufWi++] = c;
  if ( bleBufWi >= BLEBUFLEN )
    bleBufWi = 0;
//  Serial.print(c, HEX);
}

#ifdef RFD
void RFduinoBLE_onReceive(char *data, int len)
{
  int i;

  check_stack("#onr#");

  for ( i = 0 ; i < len ; i++ )
    bleBufWrite(data[i]);
}

void RFduinoBLE_onDisconnect()
{
  check_stack("#odis#");

  BTconnected = false;
#ifdef SHOW_BLESTATECHANGE
  print_state("+++++++++++++ BLE lost");
#endif
}

void RFduinoBLE_onConnect()
{
  check_stack("#onc#");

  BTconnected = true;
#ifdef SHOW_BLESTATECHANGE
  print_state("+++++++++++++ BLE conn");
#endif
}

void RFduinoBLE_onRSSI(int rssi)
{
  check_stack("#onRSSI#");
  SoCData.rssi = rssi;
}

void RFduinoBLE_onAdvertisement(bool start) {
}

bool BLEconnected()
{
}
#else /* RFD */
void SimbleeBLE_onReceive(char *data, int len)
{
  int i;
  check_stack("#orc#");
  for ( i = 0 ; i < len ; i++ )
    bleBufWrite(data[i]);
}

void SimbleeBLE_onDisconnect()
{
  check_stack("#odis#");
  BTconnected = false;
#ifdef SHOW_BLESTATECHANGE
  print_state("+++++++++++++ BLE lost");
#endif
}

void SimbleeBLE_onConnect()
{
  check_stack("#ocon#");
  BTconnected = true;
#ifdef SHOW_BLESTATECHANGE
  print_state("+++++++++++++ BLE conn");
#endif
}

void SimbleeBLE_onRSSI(int rssi)
{
  check_stack("#orssi#");
  SoCData.rssi = rssi;
}

void SimbleeBLE_onAdvertisement(bool start) {
}

bool BLEconnected()
{
}
#endif /* RFD */

/*
 * send the passed (emulated) Dexom packet to xDrip+ via the xbridge2 protocol
 */
int sendBgViaBLE(Dexcom_packet* pPkt)
{
  nRawRecord msg;
  unsigned char msgbuf[80];
  check_stack("#sbgvb#");
/*
  print_state("");
  Serial.printf("send[%d]@-%dmin", pPkt->raw/1000, (mymillis() - pPkt->ms)/60000);
*/
  if ( !BTconnected ) {
    print_state("no BT connection, skip");
    return (0);
  }

  // prepare the message
  // calculate struct size by hand, structs seems to be long aligned (adding 0x00 0x20 to gaps)
  // better change to ___attribute____ packed
  msg.size = sizeof(msg.size) + sizeof(msg.cmd_code) + sizeof(msg.raw) + sizeof(msg.filtered)
             + sizeof(msg.dex_battery) + sizeof(msg.my_battery) + sizeof(msg.dex_src_id)
             + sizeof(msg.delay) + sizeof(msg.function);
  msg.cmd_code = 0x00;
  msg.raw = pPkt->raw;
#ifdef USE_DEAD_SENSOR
  // xDrip+ filters out identical values too short after each other, simulate small jitter
  // value range is 100000++
  if ( runPeriod < 5 )
    pPkt->raw += loop_cnt % 2;
#endif USE_DEAD_SENSOR
  msg.filtered = pPkt->raw;
  msg.dex_battery = 214;  // simulate good dexcom transmitter battery
  msg.my_battery = SoCData.voltagePercent;
  msg.dex_src_id = dex_tx_id;
  msg.delay = mymillis() - pPkt->ms;
  msg.function = DEXBRIDGE_PROTO_LEVEL; // basic functionality, data packet (with ack), TXID packet, beacon packet (also TXID ack).
  *(unsigned char *)&msgbuf[0] = msg.size;
  *(unsigned char *)&msgbuf[1] = msg.cmd_code;
  *(unsigned long *)&msgbuf[2] = msg.raw;
  *(unsigned long *)&msgbuf[6] = msg.filtered;
  *(unsigned char *)&msgbuf[10] = msg.dex_battery;
  *(unsigned char *)&msgbuf[11] = msg.my_battery;
  *(unsigned long *)&msgbuf[12] = msg.dex_src_id;
  *(unsigned long *)&msgbuf[16] = msg.delay;
  *(unsigned char *)&msgbuf[20] = msg.function;

  // send the message via BLE
#ifdef BLE_REAL_LEN
  send_data((unsigned char *)&msgbuf, msg.size);
  // limit length to 20 for one BLE packet
  // last element .function will be ignored by xDrip+ anyway
#else
  send_data((unsigned char *)&msgbuf, 20);
#endif

  return (1);
}

/*
 * send a raw data packet via BLE
 */

#define MAXBLELEN 20

void send_data(unsigned char *msg, unsigned char len)
{
  unsigned char i = 0;
  check_stack("#sd#");

  last_ble_send_time = mymillis();
  crlf_printed = 0;
#ifdef RFD
// @bertroode
/*
  if (!BTconnected)
  {
    while (!RFduinoBLE.radioActive) {};
    while (RFduinoBLE.radioActive) {};
    RFduinoBLE.end();
    setupBluetoothConnection();
  }
*/
  if ( len > MAXBLELEN ) {
  RFduinoBLE.send((char *)msg, len);
  RFduinoBLE.send((char *)&msg[MAXBLELEN], len-MAXBLELEN);
#else RFD
  SimbleeBLE.send((char *)msg, len);
  SimbleeBLE.send((char *)&msg[MAXBLELEN], len-MAXBLELEN);
#endif RFD
  }
  else {
#ifdef RFD
  RFduinoBLE.send((char *)msg, len);
#else RFD
  SimbleeBLE.send((char *)msg, len);
#endif RFD
  }
  delay(100);

  if ( show_ble ) {
    print_state("sending: <");
    for ( i = 0 ; i < len ; i++ )
      Serial.printf("%x ", msg[i]);
    Serial.print(">");
    print_state("response: ");
  }
}

/* ********************************* xbridge2 and serial char handling ********************** */

int init_command_buff(t_command_buff* pCmd)
{
  check_stack("#icb#");
  if (!pCmd)
    return 0;
  memset(pCmd->commandBuffer, 0, COMMAND_MAXLEN);
  pCmd->nCurReadPos = 0;
  return 0;
}

/*
 * decode incoming serial/USB or BLE data commands
 * commands are: B, S, V, M, R
 */
int doCommand(void)
{
  check_stack("#doCmd#");
  // TXID packet?
  if (command_buff.commandBuffer[1] == 0x01 && command_buff.commandBuffer[0] == 0x06)
  {
    memcpy(&dex_tx_id, &command_buff.commandBuffer[2], sizeof(dex_tx_id));
    // send back the TXID we think we got in response
    return (0);
  }
  // ACK packet?
  if (command_buff.commandBuffer[0] == 0x02 && command_buff.commandBuffer[1] == 0xF0 && !got_ack) {
    got_ack = 1;
    init_command_buff(&command_buff);
    return (0);
  }
  if ( toupper(command_buff.commandBuffer[0]) == 'B' ) {
    if ( show_ble ) {
      print_state("B command - dont show BLE communication");
      show_ble = 0;
    }
    else {
      print_state("B command - show BLE communication");
      show_ble = 1;
    }
  }
  if ( toupper(command_buff.commandBuffer[0]) == 'S' ) {
    // empty command
  }
  if ( toupper(command_buff.commandBuffer[0]) == 'V' ) {
    print_state("V command - show FLASH settings");
    char v[20];
    sprintf(v, "v %d %d %x v", p->protocolType, p->runPeriod, p->firmware);
    #ifdef DEBUG
      Serial.println("V-command received.");
      Serial.println(v);
    #endif   
    #ifdef RFD
      RFduinoBLE.send(v, strlen(v));
//      while ( RFduinoBLE.radioActive);
    #else
      SimbleeBLE.send(v, strlen(v));
//      while ( SimbleeBLE.radioActive);
    #endif
    displayData();
  }
  if ( toupper(command_buff.commandBuffer[0]) == 'M' ) {
    print_state("M command - set FLASH settings");
    valueSetup.protocolType = (byte) (command_buff.commandBuffer[1]- '0');  
    valueSetup.runPeriod = (byte) (command_buff.commandBuffer[2]- '0');
    valueSetup.firmware = p->firmware; 
    #ifdef DEBUG
      Serial.println("M-command received.");
    #endif    
    #ifdef RFD
      while ( RFduinoBLE.radioActive );
    #else
      while ( SimbleeBLE.radioActive );
    #endif
    delay(6);
    eraseData();
    writeData();
    displayData();  
    protocolType = p->protocolType;
    runPeriod = p->runPeriod;
  }
  if ( toupper(command_buff.commandBuffer[0]) == 'R' ) {
    print_state("R command - dop an extra read");
    #ifdef DEBUG
      Serial.println("R-command received.");
    #endif  
    readAllData();
#ifdef USE_XBRIDGE2
    fillupMissingReadings(0);
#endif
    dataTransferBLE();
  }  
  // we don't respond to unrecognised commands.
  return (1);
}

/*
 * process incoming BLE packets or USB/seriall chars
 */
int controlProtocolService()
{
  static unsigned long cmd_to;
  // ok this is where we check if there's anything happening incoming on the USB COM port or UART 0 port.
  int nRet = 1;
  int i;
  unsigned char b;

  check_stack("#cps#");

  //if we have timed out waiting for a command, clear the command buffer and return.
  if (command_buff.nCurReadPos > 0 && (mymillis() - cmd_to) > 2000)
  {
    print_state("got <-[");
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
  while ( (Serial.available() || bleCharAvailable()) && command_buff.nCurReadPos < COMMAND_MAXLEN) {

    if ( ble_answer == 0 ) {
      //      Serial.print("-1-");
      ble_answer = 1;
    }

    // BLE receive which is not 3 s after last sending?
    if ( ((mymillis() - last_ble_send_time) > 3000) && !crlf_printed ) {
      //        print_state(F(" - CRLF printed"));
      if ( show_ble ) {
        Serial.print(F("\r\n"));
      }
      crlf_printed = 1;
    }

    if ( bleCharAvailable() )
      b = bleBufRead();

    command_buff.commandBuffer[command_buff.nCurReadPos] = b;
    command_buff.nCurReadPos++;
    cmd_to = mymillis();
    // if it is the end for the byte string, we need to process the command
    // valid data packet or "OK" received?
    if (      command_buff.nCurReadPos == command_buff.commandBuffer[0] \
              || (command_buff.commandBuffer[0] == 'O' && command_buff.commandBuffer[1] == 'K' ) \
              || (command_buff.nCurReadPos == 1 && toupper(command_buff.commandBuffer[0]) == 'B') \
              || (command_buff.nCurReadPos == 1 && toupper(command_buff.commandBuffer[0]) == 'S') \
              || (command_buff.nCurReadPos == 1 && toupper(command_buff.commandBuffer[0]) == 'V') \
              || (command_buff.nCurReadPos == 3 && toupper(command_buff.commandBuffer[0]) == 'M') \
              || (command_buff.nCurReadPos == 1 && toupper(command_buff.commandBuffer[0]) == 'R') )
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
      last_ble_send_time = mymillis();
    }
    // otherwise, if the command is not up to the maximum length, add the character to the buffer.
  }

  if ( ble_answer == 1 ) {
    //    Serial.print("-2-");
    ble_answer = 2;
  }

  if ( command_buff.nCurReadPos ) {
    //re-initialise the command buffer for the next one.
    init_command_buff(&command_buff);
  }
  return (nRet);
}

/*
 * poll for an flag change or incoming chars to be processed during wait times
 */
int waitDoingServicesInterruptible(unsigned long wait_time, volatile boolean *break_flag, unsigned char bProtocolServices)
{
  unsigned long start_wait = mymillis();
  check_stack("#wdsi#");
  while ( (mymillis() - start_wait ) < wait_time ) {
    if ( bProtocolServices )
      controlProtocolService();
    if ( *break_flag ) {
      return (1);
    }
    delay(20);
  }
  return (0);
}

/*
 * during any wait action look for incoming chars to be processed
 */
void waitDoingServices(unsigned long wait_time, unsigned char bProtocolServices)
{
  unsigned long start_wait = mymillis();
  check_stack("#wds#");
  while ( (mymillis() - start_wait ) < wait_time )
  {
    if ( bProtocolServices )
      controlProtocolService();
    delay(20);
  }
}

/* **************************** FLASH data handling ********************************** */

void setupInitData()
{
  check_stack("#sid#");
  print_state("setupInitData()");

  if (p->marker == 'T')
  // for initial config of FLASH
//  if (p->marker == '1')
  {
    protocolType = p->protocolType;
    runPeriod = p->runPeriod;
    Serial.printf(" - data from FLASH page #%d", MY_FLASH_PAGE);
    Serial.printf("\r\n\tprotocolType = %d", p->protocolType);
    Serial.printf("\r\n\trunPeriod = %d minutes", p->runPeriod);
    Serial.printf("\r\n\tfirmware = %d", p->firmware);
  }
  else
  {
    eraseData();
    valueSetup.marker = 'T';
    valueSetup.protocolType = 1;                  // 1 - LimiTTer
    valueSetup.runPeriod = 5;
    valueSetup.firmware = 0x02;
    writeData();
    protocolType = p->protocolType;
    runPeriod = p->runPeriod;
    Serial.print("\r\n\tnew init data stored at page ");
    Serial.print(MY_FLASH_PAGE);
    Serial.print(", protocolType = ");
    Serial.print(p->protocolType);
    Serial.print(", runPeriod = ");
    Serial.print(p->runPeriod, HEX);
    Serial.print(", firmware = ");
    Serial.print(p->firmware, HEX);
  }
}

void eraseData()
{
  check_stack("#ed#");
  print_state("eraseData() - ");
  int rc;
  Serial.print("Attempting to erase flash page ");
  Serial.print(MY_FLASH_PAGE);
  rc = flashPageErase(PAGE_FROM_ADDRESS(p));
  if (rc == 0)
    Serial.print(" -> Success");
  else if (rc == 1)
    Serial.print(" -> Error - the flash page is reserved");
  else if (rc == 2)
    Serial.print(" -> Error - the flash page is used by the sketch");
}

void writeData()
{
  int rc;
  check_stack("#wd#");

  print_state("writeData() - ");

  Serial.print("Attempting to write data to flash page ");
  Serial.print(MY_FLASH_PAGE);
  valueSetup.marker = 'T';
  rc = flashWriteBlock(p, &valueSetup, sizeof(valueSetup));
  if (rc == 0)
    Serial.print(" -> Success");
  else if (rc == 1)
    Serial.print(" -> Error - the flash page is reserved");
  else if (rc == 2)
    Serial.print(F(" -> Error - the flash page is used by the sketch"));
}

void displayData()
{
  check_stack("#dd#");
  Serial.print(F("The data stored in flash page "));
  Serial.print(MY_FLASH_PAGE);
  Serial.println(F(" contains: "));
  Serial.print("  protocolType = ");
  Serial.println(p->protocolType);
  Serial.print("  runPeriod = ");
  Serial.println(p->runPeriod);
  Serial.print("  firmware = ");
  Serial.println(p->firmware, HEX);
}

/* ****************** help functions *********************************************** */

/*
 * program runtime starts virtual 8 h in the past for backfilling
 */

#define TIMEOF8HINMS ((long)(8L*12L*5L*60L*1000L))

unsigned long mymillis(void)
{
  return(millis()+TIMEOF8HINMS);    // next round value above 8 h buffer depth in ms
}

void print_state(char *str)				// print timestamp and description of current status/action
{
	Serial.print(timeStamp());
	Serial.print(str);
}

char timestring[20];

char *timeStamp()
{
	unsigned long ms = mymillis();
	unsigned long val;
	int i;

  check_stack("#timStmp#");

  timestring[0] = '\0';
  sprintf(timestring, "\r\n[%03d][%03d][%03d] ", ms / 60000, (ms / 1000) % 60, ms % 1000);
	return timestring;
}

/* ************************ end of LBridge code **************** */

