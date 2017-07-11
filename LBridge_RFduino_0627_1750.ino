/*
 * LBridge for RFduino / Simblee
 * 
 * LBridge running on RFduino or Simblee platform (aka Transmiter2 from @MarekMacner)
 * 
 * Reads every 5 min a Libre Freestyle sensor via NFC and transmit the BG readings to xDrip+ via BLE.
 * Supports backfilling of older BG readings which were not transfered at the time of reading.
 * 
 * Code used from the work of @UPetersen, @MarekMacner, @jstevensog, @savek-cc and @bertrooode
 * 
 * keencave, 20.6.2017
 * 
 * ToDo: 
 *  - optimize BLE reconnect
 *  - only read changed NFC blocks from sensor for better battery performance
 *  - enable "hidden" power modes on BM019
 */

/* 
 * program configuration 
 */

#define RFD                     // uncomment for RFduino platform?
//#define N_RFD                   // uncomment for Simblee platform

#define N_USE_DEAD_SENSOR       // we can test with a dead sensor?

#define N_SHOW_LIMITTER         // show original Limitter output
#define N_DYNAMIC_TXID          // get TXID automatically
#define N_XBEXT                 // xbridge+ code
#define N_T2                    // T2 Transmiter from Marek?

// length of device name and advertisement <=15!!!
#define LB_NAME   "xbridge0"    // dont change "xbridge", space for 1 char left to indicate different versions
#define LB_ADVERT "rfduino"     // dont change "rfduino"
#define LB_VERSION "0627_1750"

#ifdef RFD
#include <RFduinoBLE.h>
#else
#include <SimbleeBLE.h>
#endif

#include <SPI.h>
#include <Stream.h>
#include <Memory.h>
#ifdef XBEXT
#include "xbridge_libre.h"
#endif

#define PIN_SPI_SCK   4
#define PIN_SPI_MOSI  5
#define PIN_SPI_MISO  3
#define PIN_SPI_SS    6
#define PIN_IRQ       2

#define ALL_BYTES 0x1007
#define IDN_DATA 0x2001
#define SYSTEM_INFORMATION_DATA 0x2002
#define BATTERY_DATA 0x2005

/* ------------------ program variables ----------- */

#define MIN_VOLTAGE 2100
//#define MAX_VOLTAGE 3200
#define MAX_VOLTAGE 3600            // adjust voltage measurement to have a wider rrange

unsigned long loop_cnt = 0;         // count the 5 mins loops
static boolean show_ble = 1;        // what is shown in serial monitor
static boolean show_state = 1;      // show print_state() infos, disabled ftm
int ble_answer;                     // state counter, char from BLE reeived?

#define BLEBUFLEN 80                // BLE input buffer
unsigned char bleBuf[BLEBUFLEN];
int bleBufRi = 0;                   // BLE read and write index
int bleBufWi = 0;

// defines the xBridge protocol functional level.  Sent in each packet as the last byte.
#define DEXBRIDGE_PROTO_LEVEL (0x01)

static volatile boolean do_sleep = 0;     // indicates we should go to sleep between packets
static volatile boolean got_ack = 0;      // indicates if we got an ack during the last do_services.
#ifdef XBEXT
static volatile boolean got_drp;          // DataRequestPacket
#endif
static volatile boolean dex_tx_id_set;    // indicates if the Dexcom Transmitter id (settings.dex_tx_id) has been set.  Set in doServices.
static volatile boolean ble_connected;    // bit indicating the BLE module is connected to the phone.  Prevents us from sending data without this.

static volatile boolean got_ok = 0;       // flag indicating we got OK from the HM-1x
static unsigned long last_ble_send_time;
static boolean crlf_printed = 0;

static volatile unsigned long pkt_time = 0;

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

#define DXQUEUESIZE (8*12) // 8 h of queue

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
  unsigned long dex_tx_id = 0xA5B1AE;     //4 bytes
  unsigned long uart_baudrate; //4 bytes
} xBridge_settings;     //14 bytes total

xBridge_settings settings;

// array of HM-1x baudrates for rate detection.
unsigned long uart_baudrate[9] = {9600L, 19200L, 38400L, 57600L, 115200L, 4800, 2400, 1200, 230400L};

#ifdef XBEXT
_QuarterPacket1 qpkt1;
_QuarterPacket2 qpkt2;
#endif

/* ------------- endof program veriables ---------- */

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
  String sensorSN;
} SystemInformationDataType;

typedef struct  __attribute__((packed))
{
  bool sensorDataOK;
  String sensorSN;
  byte   sensorStatusByte;
  String sensorStatusString;
  byte  nextTrend;
  byte  nextHistory;
  uint16_t minutesSinceStart;
  uint16_t minutesHistoryOffset;
  uint16_t trend[16];
  uint16_t history[32];
} SensorDataDataType;

typedef struct  __attribute__((packed))
{
  long voltage;
  int voltagePercent;
  double temperatureC;
  double temperatureF;
  double rssi;
} SoCDataType;

typedef struct  __attribute__((packed))
{
  uint8_t allBytes[345];
} AllBytesDataType;

typedef struct  __attribute__((packed))
{
  float voltage;
  float temperature;
} BatteryDataType;

typedef struct dataConfig
{
  byte marker;
  byte protocolType;                  // 1 - LimiTTer, 2 - Transmiter, 3 - LibreCGM, 4 - Transmiter II
  byte runPeriod;                     // 0-9 main loop period in miutes, 0=on demenad
  byte firmware;                      // firmware version starting 0x02
};

typedef enum {
  UBP_TxFlagNone = 0 << 0,
  UBP_TxFlagIsRPC = 1 << 0,
  UBP_TxFlagRequiresACK = 1 << 1

} UBP_TxFlags;

bool UBP_isTxPending = false;
bool hostIsConnected = false;
extern void UBP_incomingChecksumFailed() __attribute__((weak));
extern void UBP_receivedPacket(unsigned short packetIdentifier, UBP_TxFlags txFlags, void *packetBuffer) __attribute__((weak));
extern void UBP_didAdvertise(bool start) __attribute__((weak));
extern void UBP_didConnect() __attribute__((weak));
extern void UBP_didDisconnect() __attribute__((weak));

byte resultBuffer[40];
byte dataBuffer[400];
byte NFCReady = 0;            // 0 - not initialized, 1 - initialized, no data, 2 - initialized, data OK
bool sensorDataOK = false;

IDNDataType idnData;
SystemInformationDataType systemInformationData;
SensorDataDataType sensorData;

SoCDataType SoCData;        // store processor variables

struct dataConfig valueSetup;
dataConfig *p;
AllBytesDataType allBytes;
BatteryDataType batteryData;

byte sensorDataHeader[24];
byte sensorDataBody[296];
byte sensorDataFooter[24];

byte noOfBuffersToTransmit = 1;
String TxBuffer[10];
String TxBuffer1 = "";

byte protocolType = 1;    // 1 - LimiTTer, 2 - Transmiter, 3 - Transmiter II
byte runPeriod = 1;       // czas w minutach - 0 = tylko na żądanie

// ====== changed by Bert Roode
//byte  MY_FLASH_PAGE =  251;  definition is not as it should be
#define MY_FLASH_PAGE  251
#define  str(x)   xstr(x) // double level of indirection required to get gcc
#define  xstr(x)  #x      // to apply the stringizing operator correctly

unsigned int time_loop_started = 0;
unsigned int time_elapsed = 0;
// ====== end Bert Roode

bool BTconnected = false;
bool BatteryOK = false;

void setup()
{
  p = (dataConfig*)ADDRESS_OF_PAGE(MY_FLASH_PAGE);
  Serial.begin(9600);
  delay(2000);    // give serial interface time to settle

  Serial.print("\r\n=== loop #"); Serial.print(loop_cnt);
  Serial.print(" =====================================================================================================");
  print_state("setup()");
  print_state(" ------------------------------------------------------------------");
  print_state(" --- LBridge starting ---");
  print_state(" --- BLE Name: "); Serial.print(LB_NAME); Serial.print(" ---");
  print_state(" --- Version: "); Serial.print(LB_VERSION); Serial.print(" ---");
  print_state(" --- RAM used: "); Serial.print(ramUsed()); Serial.print(", Flash used: "); Serial.print(flashUsed()); Serial.print(" ---");
  print_state(" --- Queue size: "); Serial.print(DXQUEUESIZE);
  print_state(" ------------------------------------------------------------------");

  print_state("setup - start - ");

  SoC_Data();
  setupInitData();
  protocolType = p->protocolType;
  runPeriod = p->runPeriod;

//  runPeriod = 1;          // loop time is 1 min, only for test

  setupBluetoothConnection();
  nfcInit();
  // ====== changed by Bert Roode
  //  configWDT();

  // reset the CR95HF because of possible instability
  digitalWrite(PIN_SPI_SS, LOW);
  SPI.transfer(0x01);
  digitalWrite(PIN_SPI_SS, HIGH);
  delay(1);
  // ====== changed by Bert Roode

  print_state("NFCReady = "); Serial.print(NFCReady);
  Serial.print(", BatOK = "); Serial.print(BatteryOK);
  print_state("setup - end - ");

  // init xbridge queue
  Pkts.read = 0;
  Pkts.write = 0;

}

void loop()
{
  Serial.print("\r\n=== loop #"); Serial.print(++loop_cnt); Serial.print(" === BT status "); Serial.print(BTconnected);
  Serial.print(" ==="); Serial.printf(" RSSI: %f", SoCData.rssi);
  Serial.print(" =================================================");
  print_state("loop - start - BLE Name: "); Serial.print(LB_NAME); Serial.print(", Version: "); Serial.print(LB_VERSION);
  Serial.print(", Platform: "); 
#ifdef RFD
  Serial.print("RFduino");
#else
  Serial.print("Simblee");
#endif

  // ====== change by Bert Roode
  time_loop_started = millis();
  // ====== end change by Bert Roode
  
  if (BatteryOK) {
    readAllData();
    if (NFCReady == 2) {
      print_state("After sensor read, NFCReady = ");
      Serial.print(NFCReady);
      dataTransferBLE();
    }
    else {
      print_state("No sensor data, ");
      Serial.print("NFCReady = ");
      Serial.print(NFCReady);
    }
  }
  else {
    print_state("low Battery - go sleep");
  }
  print_state("loop - end - ");
  Serial.print(" NFCReady = ");
  Serial.print(NFCReady);
   // ====== start change by Bert Roode
  time_elapsed = millis() - time_loop_started;
  print_state("sleep for ");
  Serial.print(60000 * runPeriod);
#ifdef RFD
  RFduino_ULPDelay((60000 * runPeriod) - time_elapsed) ;
#else
  Simblee_ULPDelay((60000 * runPeriod) - time_elapsed) ;
#endif
  
//  restartWDT();
   // ====== end change by Bert Roode
}


/* ********* BLE ************************ */
void dataTransferBLE()
{
  if ( BTconnected )
    print_state("we are connected - transfer data ...");
  else
    print_state("dataTransferBLE(), wait 40 s for BLE connect ...");
  for (int i = 0; i < 40; i++) {
    if (BTconnected) {
      if (protocolType == 1)      forLimiTTer();
      else if (protocolType == 2) forTransmiter1();
      else if (protocolType == 3) forTransmiter2();
      else if (protocolType == 4) forLibreCGM();
      break;
    }
    else {
      //        Serial.print("Not connected after ");
      //        Serial.println(i);
      //        Serial.printf("not connected after %d\rn", i);
      waitDoingServices(1000, 1);
    }
  }
  NFCReady = 1;

}

void setupBluetoothConnection()
{
  print_state("setupBluetoothConnection() - ");
#ifdef RFD
  if (protocolType == 1) RFduinoBLE.deviceName = LB_NAME;
  else if (protocolType == 2) RFduinoBLE.deviceName = "LimiTTer";
  else if (protocolType == 3) RFduinoBLE.deviceName = "Transmiter";
#ifdef T2
  Serial.print(" - setting Transmiter device");
  RFduinoBLE.advertisementData = "data";
  RFduinoBLE.customUUID = "c97433f0-be8f-4dc8-b6f0-5343e6100eb4";
#else /* T2 */
  // emulate a HM-11 module to be recognized like a LimiTTer running LBridge code
  Serial.print(" - setting LimiTTer/xbridge device");
  RFduinoBLE.advertisementData = LB_ADVERT;
  RFduinoBLE.customUUID = "0000ffe0-0000-1000-8000-00805f9b34fb";
#endif /* T2 */
  RFduinoBLE.advertisementInterval = MILLISECONDS(300);
  RFduinoBLE.txPowerLevel = 4;
  RFduinoBLE.begin();
#else /* RFD */
  if (protocolType == 1) SimbleeBLE.deviceName = LB_NAME;
  else if (protocolType == 2) SimbleeBLE.deviceName = "LimiTTer";
  else if (protocolType == 3) SimbleeBLE.deviceName = "Transmiter";
  // emulate a HM-11 module to be recognized like a LimiTTer running LBridge code
  Serial.print(" - setting LimiTTer/xbridge device");
  SimbleeBLE.advertisementData = LB_ADVERT;
  SimbleeBLE.customUUID = "0000ffe0-0000-1000-8000-00805f9b34fb";
  SimbleeBLE.advertisementInterval = MILLISECONDS(300);
  SimbleeBLE.txPowerLevel = 4;
  SimbleeBLE.begin();
#endif
  Serial.print(" - done");
}
/* ************ BLE ********************* */

/* ************ CR95HF ********************* */
//============================================================================================================
void NFC_wakeUP()
{
  print_state("NFC_wakeUp() - Send wake up pulse to CR95HF and configure to use SPI - ");
  digitalWrite(PIN_IRQ, HIGH);
  delay(10);
  digitalWrite(PIN_IRQ, LOW);
  delayMicroseconds(100);
  digitalWrite(PIN_IRQ, HIGH);
  delay(10);
  Serial.print("done");
}
void NFC_CheckWakeUpEventRegister()
{
  int length = 5;
  byte command[length];

  print_state("NFC_CheckWakeUpEventRegister()");
#ifdef RFD
  while ( RFduinoBLE.radioActive );
#else
  while ( SimbleeBLE.radioActive );
#endif

  command[ 0] = 0x08;
  command[ 1] = 0x03;
  command[ 2] = 0x62;
  command[ 3] = 0x01;
  command[ 4] = 0x00;
  send_NFC_PollReceive(command, sizeof(command));
  print_NFC_WakeUpRegisterResponse();

}
void send_NFC_PollReceive(byte *command, int commandLength)
{
//  print_state("send_NFC_PollReceive() - ");
  send_NFC_Command(command, commandLength);
  poll_NFC_UntilResponsIsReady();
  receive_NFC_Response();
}

#define NFCTIMEOUT 500
void new_send_NFC_PollReceive(byte *command, int commandLength)
{
//  print_state("send_NFC_PollReceive() - ");
  digitalWrite(PIN_SPI_SS, LOW);
  SPI.transfer(0x00);
  for (int i = 0; i < commandLength; i++)
  {
    SPI.transfer(command[i]);
  }
  digitalWrite(PIN_SPI_SS, HIGH);
  delay(1);

  unsigned long ms = millis();
  byte rb;
//  print_state("poll_NFC_UntilResponsIsReady() - ");
  digitalWrite(PIN_SPI_SS , LOW);
  while ( (resultBuffer[0] != 8) && ((millis() - ms) < NFCTIMEOUT) )
  {
    rb = resultBuffer[0] = SPI.transfer(0x03);
//    Serial.printf("SPI polling response byte:%x\r\n", resultBuffer[0]);
    resultBuffer[0] = resultBuffer[0] & 0x08;
  }
  digitalWrite(PIN_SPI_SS, HIGH);
  delay(1);
  if ( millis() - ms > NFCTIMEOUT ) {
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

//============================================================================================================
void send_NFC_Command(byte *commandArray, int length)
{
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
  unsigned long ms = millis();
  byte rb;
//  print_state("poll_NFC_UntilResponsIsReady() - ");
  digitalWrite(PIN_SPI_SS , LOW);
  while ( (resultBuffer[0] != 8) && ((millis() - ms) < NFCTIMEOUT) )
  {
    rb = resultBuffer[0] = SPI.transfer(0x03);
//    Serial.printf("SPI polling response byte:%x\r\n", resultBuffer[0]);
    resultBuffer[0] = resultBuffer[0] & 0x08;
  }
  digitalWrite(PIN_SPI_SS, HIGH);
  delay(1);
  if ( millis() - ms > NFCTIMEOUT ) {
    Serial.print("\r\n *** poll timeout *** -> response ");
    Serial.print(rb);
  }
//  else
//    Serial.print("done");
}
void receive_NFC_Response()
{
//  print_state("receive_NFC_Response()");
  digitalWrite(PIN_SPI_SS, LOW);
  SPI.transfer(0x02);
  resultBuffer[0] = SPI.transfer(0);
  resultBuffer[1] = SPI.transfer(0);
  for (byte i = 0; i < resultBuffer[1]; i++) resultBuffer[i + 2] = SPI.transfer(0);
  digitalWrite(PIN_SPI_SS, HIGH);
  delay(1);
}
//============================================================================================================
void print_NFC_WakeUpRegisterResponse()
{
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
  //    print_state("... finished printing wake-up register result.");
}
//============================================================================================================
void SetNFCprotocolCommand()
{
  for (int t = 0; t < 9; t++)
  {
    print_state("SetNFCprotocolCommand() - ");
    int length = 4;
    byte command[length];
    command[ 0] = 0x02;
    command[ 1] = 0x02;
    command[ 2] = 0x01;
    command[ 3] = 0x0F;
    //    command[ 3] = 0x0D;
    send_NFC_PollReceive(command, sizeof(command));
    Serial.print("resultBuffer: ");
    for (byte i = 0; i < 2; i++)
    {
      Serial.print(resultBuffer[i], HEX);
      Serial.print(" ");
    }
    if ((resultBuffer[0] == 0) & (resultBuffer[1] == 0))
    {
      Serial.print("Try=");
      Serial.print(t);
      Serial.print(" PROTOCOL SET - OK");
      NFCReady = 1;
      break;
    }
    else
    {
      Serial.print("Try=");
      Serial.print(t);
      Serial.print(" BAD RESPONSE TO SET PROTOCOL");
      NFCReady = 0; // NFC not ready
    }
  }
}
//============================================================================================================
void runIDNCommand(int maxTrials)
{
  byte command[2];
  print_state("runIDNCommand() - ");
  command[0] = 0x01;
  command[1] = 0x00;
  delay(10);
  Serial.printf("maxTrials: %d, RXBuffer[0]: %x", maxTrials, resultBuffer[0]);
  runIDNCommandUntilNoError(command, sizeof(command), maxTrials);
}

void runIDNCommandUntilNoError(byte *command, int length, int maxTrials)
{
  int count = 0;
  bool success;
//  print_state("runIDNCommandUntilNoError() - ");
  do
  {
    Serial.printf("Before: Count: %d, success: %b, resultBuffer[0]: %x", count, success, resultBuffer[0]);
    count++;
    memset(resultBuffer, 0, sizeof(resultBuffer));
    send_NFC_PollReceive(command, sizeof(command));
    success = idnResponseHasNoError();
    Serial.printf("\r\nAfter: Count: %d, success: %b, resultBuffer[0]: %x", count, success, resultBuffer[0]);
  } while ( !success && (count < maxTrials));
  delay(10);
  Serial.printf("Exiting at count: %d, resultBuffer[0]: %x", count, resultBuffer[0]);
}
bool idnResponseHasNoError()
{
  print_state("idnResponseHasNoError() - ");
  Serial.printf("IDN response is resultBuffer[0]: %x", resultBuffer[0]);
  if (resultBuffer[0] == 0x00)
  {
    return true;
  }
  return false;
}
IDNDataType idnDataFromIDNResponse()
{
  idnData.resultCode = resultBuffer[0];
  for (int i = 0; i < 13; i++)
  {
    idnData.deviceID[i] = resultBuffer[i + 2];
  }
  idnData.romCRC[0] = resultBuffer[13 + 2];
  idnData.romCRC[1] = resultBuffer[14 + 2];
  return idnData;
}
void printIDNData(IDNDataType idnData)
{
  print_state("printIDNData() - ");
  String nfc = "";
  //    Serial.println("Printing IDN data:");
  Serial.printf("Result code: %x", idnData.resultCode);
  Serial.printf(", NFC Device ID  [hex]: ");
  Serial.printf("%x", idnData.deviceID[0]);
  nfc += (char) idnData.deviceID[0];
  for (int i = 1; i < 12; i++)
  {
    Serial.printf(":%x", idnData.deviceID[i]);
    nfc += (char) idnData.deviceID[i];
  }
  Serial.println("");
  Serial.printf("NFC Device ID [char]: ");
  Serial.print(nfc);
  Serial.printf(", NFC Device CRC  %x:%x", idnData.romCRC[0], idnData.romCRC[1] );
  //    Serial.println("");
}
//============================================================================================================
void runSystemInformationCommandUntilNoError(int maxTrials)
{
  memset(resultBuffer, 0, sizeof(resultBuffer));
//  Serial.printf("maxTrials: %d, resultBuffer[0]: %x \r\n", maxTrials, resultBuffer[0]);
  byte command[4];
  command[0] = 0x04;
  command[1] = 0x02;
  command[2] = 0x03;
  command[3] = 0x2B;
  delay(10);
//  Serial.printf("maxTrials: %d, resultBuffer[0]: %x \r\n", maxTrials, resultBuffer[0]);
  runNFCcommandUntilNoError(command, sizeof(command), maxTrials);
}
void runNFCcommandUntilNoError(byte *command, int length, int maxTrials)
{
  int count = 0;
  bool success;
//  print_state("runNFCcommandUntilNoError() - ");
  do
  {
    delay(1);
//    Serial.printf("Before: Count: %d, success: %b, resultBuffer[0]: %x \r\n", count, success, resultBuffer[0]);
    count++;
    send_NFC_PollReceive(command, sizeof(command));
    success = responseHasNoError();
//    Serial.printf("After: Count: %d, success: %b, resultBuffer[0]: %x \r\n", count, success, resultBuffer[0]);
  } while ( !success && (count < maxTrials));
  delay(1);
//  Serial.printf("Exiting at count: %d, resultBuffer[0]: %x \r\n", count, resultBuffer[0]);
}

bool responseHasNoError()
{
//  Serial.printf("Response is resultBuffer[0]: %x, resultBuffer[2]: %x \r\n", resultBuffer[0], resultBuffer[2]);
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
    clearBuffer(systemInformationData.uid);
    systemInformationData.errorCode = resultBuffer[3];
  }
  return systemInformationData;
}

void printSystemInformationData(SystemInformationDataType systemInformationData)
{
  print_state("Printing system information data: ");
  Serial.printf("Result code: %x", systemInformationData.resultCode);
  Serial.printf(", Response flags: %x", systemInformationData.responseFlags);
/*
  Serial.printf("uid: %x", systemInformationData.uid[0]);
  for (int i = 1; i < 8; i++)
  {
    Serial.printf(":%x", systemInformationData.uid[i]);
  }
  Serial.println("");
*/
  Serial.print(", Sensor SN:");
  Serial.print(systemInformationData.sensorSN);
  Serial.printf(", Error code: %x", systemInformationData.errorCode);
}

void clearBuffer(byte *tmpBuffer)
{
  memset(tmpBuffer, 0, sizeof(tmpBuffer));
}
//============================================================================================================
bool readSensorData()
{
  byte resultCode = 0;
  int trials = 0;
  int maxTrials = 10;

  print_state("readSensorData() - ");

  clearBuffer(dataBuffer);
  for (int i = 0; i < 43; i++)
  {
    resultCode = ReadSingleBlockReturn(i);
//    printf("resultCode 0x%x - ", resultCode);
    if (resultCode != 0x80 && trials < maxTrials)
    {
//      printf("Error 0x%x\n\r", resultCode);
      i--;        // repeat same block if error occured, but
      trials++;   // not more than maxTrials times per block
    }
    else if (trials >= maxTrials)
    {
      break;
    }
    else
    {
      trials = 0;
      for (int j = 3; j < resultBuffer[1] + 3 - 4; j++)
      {
        dataBuffer[i * 8 + j - 3] = resultBuffer[j];
//        Serial.print(resultBuffer[j], HEX);
//        Serial.print(" ");
      }
//      Serial.println(" ");
    }
  }
  bool resultH = checkCRC16(dataBuffer, 0);
  bool resultB = checkCRC16(dataBuffer, 1);
  bool resultF = checkCRC16(dataBuffer, 2);
  bool crcResult = false;

  Serial.println();
  Serial.print(" CRC-H check = ");
  Serial.println(resultH);
  Serial.print(" CRC-B check = ");
  Serial.println(resultB);
  Serial.print(" CRC-F check = ");
  Serial.println(resultF);

  if (resultH && resultB && resultF) crcResult = true;
  else crcResult = false;
  Serial.print("CRC check ");
  Serial.print(crcResult);
  if (crcResult) NFCReady = 2;
  else NFCReady = 1;
  return crcResult;
}

byte ReadSingleBlockReturn(int blockNum)
{
  int length = 5;
  byte command[length];
  command[0] = 0x04;
  command[1] = 0x03;
  command[2] = 0x03;
  command[3] = 0x20;
  command[4] = blockNum;
  send_NFC_Command(command, 5);
  poll_NFC_UntilResponsIsReady();
  receive_NFC_Response();
  delay(1);
/*
  if (resultBuffer[0] == 128)
  {

    Serial.printf("The block #%d:", blockNum);
    for (byte i = 3; i < resultBuffer[1] + 3 - 4; i++)
    {
      Serial.print(resultBuffer[i], HEX);
      Serial.print(" ");
    }
    Serial.println(" ");
  }
  else
  {
    Serial.print("NO Single block available - ");
    Serial.print("RESPONSE CODE: ");
    Serial.println(resultBuffer[0], HEX);
  }
  Serial.println(" ");
*/
  return resultBuffer[0];
}
//============================================================================================================
void nfcInit()
{
  print_state("nfc_init()");
  configSPI();
  NFC_wakeUP();
  //=========== changed by Bert Roode
  // NFC_CheckWakeUpEventRegister();
  //=========== end change by Bert Roode

  NFCReady = 0;
  SetNFCprotocolCommand();
  runIDNCommand(10);
  idnData = idnDataFromIDNResponse();
  printIDNData(idnData);
}

void sendNFC_ToHibernate()
{
  print_state("sendNFC-ToHibernate()");
  // Bert Roode - length 16 instead of 17
  int length = 16;
  byte command[length];
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


/* ************ CR95HF ********************* */


/* ************ DataPrep ********************* */
void forLimiTTer()
{
  print_state("forLimiTTer()");
#ifdef T2
  noOfBuffersToTransmit = 1;
  TxBuffer1 = "";
  TxBuffer1 += String(sensorData.trend[0] * 100);
  TxBuffer1 += " ";
  TxBuffer1 += String(SoCData.voltage);
  TxBuffer1 += " ";
  TxBuffer1 += String(SoCData.voltagePercent);
  TxBuffer1 += " ";
  TxBuffer1 += String((int)(sensorData.minutesSinceStart / 10));
  int  LL = TxBuffer1.length();
  Serial.print("for LimiTTer >>");
  Serial.print(TxBuffer1);
  Serial.print("<< ");
  Serial.println(LL);
#ifdef RFD
  RFduinoBLE.send(TxBuffer1.cstr(), TxBuffer1.length());
#else
  SimbleeBLE.send(TxBuffer1.cstr(), TxBuffer1.length());
#endif
#else /* T2 */
  boolean resend_pkt = 0;
  while ((Pkts.read != Pkts.write) && BTconnected && ((millis() - pkt_time) < ((DXQUEUESIZE + 1) * 5000) )) {
    got_ack = 0;
    print_packet(&Pkts.buffer[Pkts.read]);
    // wait 10 s for ack
    int j;
    for ( j = 0 ; j < 10 ; j++ ) {
      waitDoingServicesInterruptible(1000, &got_ack, 1);
      if ( !BTconnected ) {
        print_state("connection lost during wait for ack, go to sleep");
        break;
      }
    }

    if (got_ack) {
      print_state("got ack for read position ");
      Serial.print(Pkts.read); Serial.print(F(" while write is "));
      Serial.print(Pkts.write); Serial.print(F(", incrementing read to "));
      if ( ++Pkts.read >= DXQUEUESIZE )
        Pkts.read = 0;     //increment read position since we got an ack for the last package
      Serial.print(Pkts.read);
      resend_pkt = 0;
    }
    else {
      if ( !resend_pkt ) {
        print_state("no ack received, try again");
        resend_pkt = 1;
      }
      else {
        print_state("no ack received, try again next wakeup");
        resend_pkt = 0;
        break;
      }
    }
  } /* while (send all packets available) */
#endif /* T2 */
}

void forTransmiter1()
{
  noOfBuffersToTransmit = 1;
  TxBuffer1 = "";
  TxBuffer1 += String(sensorData.trend[0] * 100);
  TxBuffer1 += " ";
  TxBuffer1 += String(SoCData.voltage);
  TxBuffer1 += " ";
  TxBuffer1 += String(SoCData.voltagePercent);
  TxBuffer1 += " ";
  TxBuffer1 += String((int)(SoCData.temperatureC * 10));
  int  LL = TxBuffer1.length();
  Serial.print("for Transmiter 1 >>");
  Serial.print(TxBuffer1);
  Serial.print("<< ");
  Serial.println(LL);
#ifdef RFD
  RFduinoBLE.send(TxBuffer1.cstr(), TxBuffer1.length());
#else
  SimbleeBLE.send(TxBuffer1.cstr(), TxBuffer1.length());
#endif
}

void forTransmiter2()
{
  noOfBuffersToTransmit = 0;
  String allHardware = "";          // systemInformationData.resultCode, systemInformationData.responseFlags,
  // rfduinoData.voltage, rfduinoData.voltagePercent, rfduinoData.temperatureC,
  //  RSSI
  String allData = "";              // sensorSN, sensorStatusByte, minutesHistoryOffset, minutesSinceStart,
  // all trend, all history
  allHardware +=  String(systemInformationData.resultCode) + " ";
  allHardware +=  String(systemInformationData.responseFlags) + " ";
  allHardware +=  String(SoCData.voltage) + " ";
  allHardware +=  String(SoCData.voltagePercent) + " ";
  allHardware +=  String((int)(SoCData.temperatureC * 10)) + " ";
  allHardware +=  String((int)(SoCData.rssi)) + " ";
  int  LLh = allHardware.length();
  allData += sensorData.sensorSN + " ";
  allData += String(sensorData.sensorStatusByte) + " ";
  allData += String(sensorData.minutesHistoryOffset) + " ";
  allData += String(sensorData.minutesSinceStart) + " ";
  for (int i = 0; i < 16; i++) allData += String(sensorData.trend[i]) + " ";
  for (int i = 0; i < 32; i++) allData += String(sensorData.history[i]) + " ";
  int  LLd = allData.length();
  Serial.println("for Transmiter 2 - Hardware data");
  Serial.print(">>");
  Serial.print(allHardware);
  Serial.print("<< ");
  Serial.println(LLh);
  Serial.println("for Transmiter 2 - Sensor data");
  Serial.print(">>");
  Serial.print(allData);
  Serial.print("<< ");
  Serial.println(LLd);
  String toTransfer = "M " + allHardware + " " + allData + " M";
  int lengthToTransfer = toTransfer.length();
  byte toCRC[lengthToTransfer];
  for (int i = 0; i < lengthToTransfer; i++) toCRC[i] = (byte) toTransfer[i];
  uint16_t CRC16toTransfer = computeCRC16(toCRC, lengthToTransfer);
  toTransfer += " *" + String(CRC16toTransfer, HEX) + "*";
  lengthToTransfer = toTransfer.length();
  for (int i = 0; i < (1 + ((int)(lengthToTransfer)) / 20); i++)
  {
    TxBuffer1 = "";
    TxBuffer1 = toTransfer.substring(20 * i, 20 * (i + 1));
#ifdef RFD
    RFduinoBLE.send(TxBuffer1.cstr(), TxBuffer1.length());
#else
    RFduinoBLE.send(TxBuffer1.cstr(), TxBuffer1.length());
#endif
    Serial.print("B-");
    Serial.print(i);
    Serial.print(" >>");
    Serial.print(TxBuffer1);
    Serial.println("<<");
  }
}

void forLibreCGM()
{
  Serial.println("Start BT transmission ...");
  Serial.printf("IDN Sizeof ist : %d\n", sizeof(systemInformationData));
  bool ergo = pumpViaBluetooth(SYSTEM_INFORMATION_DATA, UBP_TxFlagIsRPC, (char *) &systemInformationData, sizeof(SystemInformationDataType));
  printSystemInformationData(systemInformationData);
  Serial.println("----about to send all data bytes packet");
  for (int i = 0; i < sizeof(allBytes.allBytes); i++)
  {
    allBytes.allBytes[i] = 0;
  }
  for (int i = 0; i < 344; i++)
  {
    allBytes.allBytes[i] = dataBuffer[i];
  }
  Serial.printf("All data Sizeof ist : %d\n", sizeof(allBytes));
  bool success = UBP_queuePacketTransmission(ALL_BYTES, UBP_TxFlagIsRPC, (char *) &allBytes, sizeof(AllBytesDataType));
  if (success) Serial.println("----all data bytes packet queued successfully");
  else Serial.println("----Failed to enqueue all data bytes packet");
  while (UBP_isBusy() == true) UBP_pump();
  batteryData.voltage = (float) SoCData.voltage;
#ifdef RFD
  batteryData.temperature = RFduino_temperature(CELSIUS);
#else
  batteryData.temperature = Simblee_temperature(CELSIUS);
#endif
  Serial.printf("Battery voltage: %f\r\n", batteryData.voltage);
  Serial.printf("Bat Sizeof ist : %d\n", sizeof(batteryData));
  success = UBP_queuePacketTransmission(BATTERY_DATA, UBP_TxFlagIsRPC, (char *) &batteryData, sizeof(BatteryDataType));
  if (success) Serial.println("Battery data packet queued successfully");
  else Serial.println("Failed to enqueue battery data packet");
  while (UBP_isBusy() == true) UBP_pump();
  Serial.printf("Sent Battery voltage: %f\r\n", batteryData.voltage);
  ergo = pumpViaBluetooth(IDN_DATA, UBP_TxFlagNone, (char *) &idnData, sizeof(IDNDataType));
  success = UBP_queuePacketTransmission(IDN_DATA, UBP_TxFlagIsRPC, (char *) &idnData, sizeof(IDNDataType));
  delay(10);
  if (success) Serial.println("IDN data packet queued successfully");
  else Serial.println("Failed to enqueue IDN data packet");
  while (UBP_isBusy() == true) UBP_pump();
}

/* ************ DataPrep ********************* */

/* ************ RFDuino ********************* */
void configSPI()
{
  print_state("configSPI()");
  pinMode(PIN_SPI_SS, OUTPUT);
  pinMode(PIN_IRQ, OUTPUT);
  SPI.begin();
  SPI.setDataMode(SPI_MODE0);
  SPI.setBitOrder(MSBFIRST);
  SPI.setFrequency(1000);
}

void configWDT()
{
  print_state("configWDT()");
  NRF_WDT->CONFIG = (WDT_CONFIG_SLEEP_Run << WDT_CONFIG_SLEEP_Pos) | (WDT_CONFIG_HALT_Pause << WDT_CONFIG_HALT_Pos);
  NRF_WDT->CRV = 32768 * 60 * 11;         // 11 minut
  NRF_WDT->RREN |= WDT_RREN_RR0_Msk;
  NRF_WDT->TASKS_START = 1;
}
  //================== changed by Bert Roode
  /*
void restartWDT()
{
  NRF_WDT->RR[0] = WDT_RR_RR_Reload;
}
*/
  //================== end change by Bert Roode
void SoC_Data()
{
  print_state("SoC_Data(): ");

  analogReference(VBG);
  analogSelection(VDD_1_3_PS);
  int sensorValue = analogRead(1);

  Serial.print("raw analog Vin : "); Serial.print(sensorValue);
  
  SoCData.voltage = sensorValue * (360 / 1023.0) * 10;
  SoCData.voltagePercent = map(SoCData.voltage, MIN_VOLTAGE, MAX_VOLTAGE, 0, 100);
#ifdef RFD
  SoCData.temperatureC = RFduino_temperature(CELSIUS);
  SoCData.temperatureF = RFduino_temperature(FAHRENHEIT);
#else
  SoCData.temperatureC = Simblee_temperature(CELSIUS);
  SoCData.temperatureF = Simblee_temperature(FAHRENHEIT);
#endif
  if (SoCData.voltage < 2400) BatteryOK = false;
  else BatteryOK = true;

  Serial.print(" - U[mv]: ");
  Serial.print(SoCData.voltage);
  Serial.print(" - U[%]: ");
  Serial.print(SoCData.voltagePercent);
  Serial.print(" - Temp[C]: ");
  Serial.print(SoCData.temperatureC);
  Serial.print(" - Temp[F]: ");
  Serial.print(SoCData.temperatureF);
  Serial.print(" - RSSI: ");
  Serial.print(SoCData.rssi);
}

void readAllData()
{
//  print_state("readAllData()");

  NFC_wakeUP();
  //================== changed by Bert Roode
  //  NFC_CheckWakeUpEventRegister();
  //================== end change by Bert Roode
  NFCReady = 0;
  SetNFCprotocolCommand();

  runSystemInformationCommandUntilNoError(10);
  systemInformationData = systemInformationDataFromGetSystemInformationResponse();
  printSystemInformationData(systemInformationData);
  sensorData.sensorDataOK = readSensorData();
  if ( sensorData.sensorDataOK )
    decodeSensor();
  else
    print_state("no sensor data received");
  SoC_Data();
  sendNFC_ToHibernate();
}

void setupInitData()
{
  print_state("setupInitData() - ");
  if (p->marker == 'T')
  {
    protocolType = p->protocolType;
    runPeriod = p->runPeriod;
    Serial.print("Init data present at page ");
    Serial.print(MY_FLASH_PAGE);
    Serial.print(",  protocolType = ");
    Serial.print(p->protocolType);
    Serial.print(",  runPeriod = ");
    Serial.print(p->runPeriod);
    Serial.print(",  firmware = ");
    Serial.print(p->firmware);
  }
  else
  {
    eraseData();
    valueSetup.marker = 'T';
    valueSetup.protocolType = 1;                  // 1 - LimiTTer, 2 - Transmiter, 3 - LibreCGM, 4 - Transmiter II
    valueSetup.runPeriod = 5;
    valueSetup.firmware = 0x02;

    //    Serial.print("write ...");
    //    waitDoingServices(1000, 1);

    writeData();
    protocolType = p->protocolType;
    runPeriod = p->runPeriod;
    Serial.print("New init data stored at page ");
    Serial.print(MY_FLASH_PAGE);
    Serial.print(",  protocolType = ");
    Serial.print(p->protocolType);
    Serial.print(",  runPeriod = ");
    Serial.print(p->runPeriod, HEX);
    Serial.print(",  firmware = ");
    Serial.println(p->firmware, HEX);
  }
}

void eraseData()
{
  print_state("eraseData() - ");
  int rc;
  Serial.print("Attempting to erase flash page ");
  Serial.print(MY_FLASH_PAGE);
  rc = flashPageErase(PAGE_FROM_ADDRESS(p));
  if (rc == 0)
    Serial.println(" -> Success");
  else if (rc == 1)
    Serial.println(" -> Error - the flash page is reserved");
  else if (rc == 2)
    Serial.println(" -> Error - the flash page is used by the sketch");
}

void writeData()
{
  int rc;

  print_state("writeData() - ");

  Serial.print("Attempting to write data to flash page ");
  Serial.print(MY_FLASH_PAGE);
  valueSetup.marker = 'T';
  rc = flashWriteBlock(p, &valueSetup, sizeof(valueSetup));
  if (rc == 0)
    Serial.println(" -> Success");
  else if (rc == 1)
    Serial.println(" -> Error - the flash page is reserved");
  else if (rc == 2)
    Serial.println(" -> Error - the flash page is used by the sketch");
}

void displayData()
{
  Serial.print("The data stored in flash page ");
  Serial.print(MY_FLASH_PAGE);
  Serial.println(" contains: ");
  Serial.print("  protocolType = ");
  Serial.println(p->protocolType);
  Serial.print("  runPeriod = ");
  Serial.println(p->runPeriod);
  Serial.print("  firmware = ");
  Serial.println(p->firmware, HEX);
}

/*
 * handling of BLE transfer, incoming and outgoing data
 */

// BLE char buffer status
boolean bleCharAvailable(void)
{
  if ( bleBufRi == bleBufWi )
    return (0);
  else
    return (1);
}

// read one char form BLE incoming buffer
unsigned char bleBufRead(void)
{
  unsigned char ret = 0xFF;

  if ( bleBufRi != bleBufWi ) {
    ret = bleBuf[bleBufRi++];
    Serial.print(ret, HEX); Serial.print(" ");
    if ( bleBufRi >= BLEBUFLEN )
      bleBufRi = 0;
  }
  return (ret);
}

// write char into BEL char buffer, called from IRQ
void bleBufWrite(unsigned char c)
{
  bleBuf[bleBufWi++] = c;
  if ( bleBufWi >= BLEBUFLEN )
    bleBufWi = 0;
//  Serial.print(c, HEX);
}

#ifdef RFD
void RFduinoBLE_onReceive(char *data, int len)
{
  int i;
  for ( i = 0 ; i < len ; i++ )
    bleBufWrite(data[i]);
}

void RFduinoBLE_onDisconnect()
{
  BTconnected = false;
  print_state("+++++++++++++ BLE lost");
  _UBP_hostDisconnected();
}

void RFduinoBLE_onConnect()
{
  BTconnected = true;
  print_state("+++++++++++++ BLE conn");
  hostIsConnected = true;
  if (UBP_didConnect) UBP_didConnect();
}

void RFduinoBLE_onRSSI(int rssi)
{
  SoCData.rssi = rssi;
}

void RFduinoBLE_onAdvertisement(bool start) {
  if (UBP_didAdvertise) UBP_didAdvertise(start);
}

bool BLEconnected()
{
  return hostIsConnected;
}
#else /* RFD */
void SimbleeBLE_onReceive(char *data, int len)
{
  int i;
  for ( i = 0 ; i < len ; i++ )
    bleBufWrite(data[i]);
}

void SimbleeBLE_onDisconnect()
{
  BTconnected = false;
  print_state("+++++++++++++ BLE lost");
  _UBP_hostDisconnected();
}

void SimbleeBLE_onConnect()
{
  BTconnected = true;
  print_state("+++++++++++++ BLE conn");
  hostIsConnected = true;
  if (UBP_didConnect) UBP_didConnect();
}

void SimbleeBLE_onRSSI(int rssi)
{
  SoCData.rssi = rssi;
}

void SimbleeBLE_onAdvertisement(bool start) {
  if (UBP_didAdvertise) UBP_didAdvertise(start);
}

bool BLEconnected()
{
  return hostIsConnected;
}
#endif /* RFD */

/* ************ RFDuino ********************* */

/* ************ SLIP ********************* */
#define BUFFER_LENGTH 430
#define TX_CHUNK_SIZE 20
#define PACKET_ID_SIZE 2

// Serial Line IP (SLIP) escaping constants
#define ESCAPE_BYTE 0xDB
#define END_BYTE    0xC0
#define ESCAPED_ESCAPE_BYTE 0xDD
#define ESCAPED_END_BYTE    0xDC
const char escapeSequence[1] = {ESCAPE_BYTE};
const char endSequence[1] = {END_BYTE};
const char escapedEndSequence[2] = {ESCAPE_BYTE, ESCAPED_END_BYTE};
const char escapedEscapeSequence[2] = {ESCAPE_BYTE, ESCAPED_ESCAPE_BYTE};

// Buffers
char ubpTxBuffer[BUFFER_LENGTH];
int ubpTxBufferLength = 0;
int ubpTxBufferSentByteCount = 0;
char ubpRxBuffer[BUFFER_LENGTH];
int ubpRxBufferLength = 0;
char ubpUnescapedRxBufferBuffer[BUFFER_LENGTH];
int ubpUnescapedRxBufferBufferLength = 0;

// State Variables





bool UBP_isBusy()
{
  return UBP_isTxPending;
}

void UBP_pump() {

  _UBP_pumpTxQueue();
}

void _UBP_pumpTxQueue() {

  if (UBP_isTxPending) {

    char *nextByteToSend = ubpTxBuffer + ubpTxBufferSentByteCount;

    // Try sending the next chunk
    if (ubpTxBufferSentByteCount < ubpTxBufferLength && hostIsConnected) {  // Haven't already sent all the bytes

      int retryRemainingCount = 1000;  // Limit the number of times we retry sending to avoid getting into an infinite loop
      int remainingByteCount = ubpTxBufferLength - ubpTxBufferSentByteCount;

      if (remainingByteCount >= TX_CHUNK_SIZE) {  // Can fill the TX output buffer
#ifdef DEBUG
        Serial.printf("Buffer length is %d and i = %d\n", nextByteToSend, nextByteToSend);
#endif

#ifdef RFD
        while ( RFduinoBLE.send(nextByteToSend, TX_CHUNK_SIZE) == false && hostIsConnected && retryRemainingCount > 0) {
          retryRemainingCount--;
        };  // send() returns false if all tx buffers in use (can't enqueue, try again later)
#else
        while ( Simblee.send(nextByteToSend, TX_CHUNK_SIZE) == false && hostIsConnected && retryRemainingCount > 0) {
          retryRemainingCount--;
        };  // send() returns false if all tx buffers in use (can't enqueue, try again later)
#endif          

        //                delay(20); // 2016-07-12: try to get fewer transfer errors using a small delay
#ifdef DEBUG
        Serial.printf("Sending the bytes\n");
#endif
        ubpTxBufferSentByteCount += TX_CHUNK_SIZE;

      } else {  // Only partial TX buffer remaining to send

        // Send is queued (the ble stack delays send to the start of the next tx window)
#ifdef RFD
        while ( RFduinoBLE.send(nextByteToSend, remainingByteCount) == false && hostIsConnected && retryRemainingCount > 0) {
          retryRemainingCount--;
        };  // send() returns false if all tx buffers in use (can't enqueue, try again later)
#else
        while ( SimbleeBLE.send(nextByteToSend, remainingByteCount) == false && hostIsConnected && retryRemainingCount > 0) {
          retryRemainingCount--;
        };  // send() returns false if all tx buffers in use (can't enqueue, try again later)
#endif
        ubpTxBufferSentByteCount += remainingByteCount;

        //                delay(20); // 2016-07-12: try to get fewer transfer errors using a small delay
#ifdef DEBUG
        Serial.printf("Sending the remaining bytes\n");
#endif
        UBP_isTxPending = false;
      }
    }
  }
}

void _UBP_ingestRxBytes(char *receivedBytes, int byteCount) {

  Serial.print(byteCount);
  Serial.println(" bytes receieved");
  Serial.println(receivedBytes);


  return;
  // NOTE: Assuming not called unless len > 0

  // Determine what to do with incoming fragment
  if ( *receivedBytes == END_BYTE ) {  // Fragment has leading END byte, signals start of packet

    // Set fragment as the beginning of the reconstruction buffer
    memcpy(ubpRxBuffer, receivedBytes, byteCount);
    ubpRxBufferLength = byteCount;

  } else if (ubpRxBufferLength > 0) {  // Already have fragments in the reconstruction buffer

    // Append fragment to reconstruction buffer
    memcpy(ubpRxBuffer + ubpRxBufferLength, receivedBytes, byteCount);
    ubpRxBufferLength += byteCount;

  }


  // Check RX buffer for trailing END byte
  if ( *(ubpRxBuffer + ubpRxBufferLength - 1) == END_BYTE) {  // RX buffer ends with END byte, looks like packet is complete

    byte firstNonControlIndex = 1;
    byte escapedDataLength = ubpRxBufferLength - 2;  // "- 2" for leading/trailing control chars

    // Un-escape the incoming payload
    ubpUnescapedRxBufferBufferLength = _UBP_makeUnEscapedCopy(ubpRxBuffer + firstNonControlIndex, escapedDataLength, ubpUnescapedRxBufferBuffer);
    byte payloadDataLength = ubpUnescapedRxBufferBufferLength - 1;  // -1 to account for checksum

    // Calculate checksum over payload, i.e. all bytes except for last checksum byte)
    char calculatedChecksum = CRC8(ubpUnescapedRxBufferBuffer, payloadDataLength * sizeof(byte));

    // Extract embedded checksum value
    char receivedChecksum = *(ubpUnescapedRxBufferBuffer + payloadDataLength);  // NOTE: Omitting '-1' because checksum byte comes just after payloadDataLength

    // Verify the checksum
    if (calculatedChecksum == receivedChecksum) {

      unsigned short packetIdentifier = *(ubpUnescapedRxBufferBuffer);
      UBP_TxFlags txFlags = (UBP_TxFlags) * (ubpUnescapedRxBufferBuffer + PACKET_ID_SIZE);

      if (UBP_receivedPacket) {

        void *packetBuffer = (ubpUnescapedRxBufferBuffer + PACKET_ID_SIZE + 1);  // skip <identifier length> + <tx flags length>
        UBP_receivedPacket(packetIdentifier, txFlags, packetBuffer);
      }

    } else {

      Serial.println("Incoming packet checksum invalid");

      // Reset
      ubpRxBufferLength = 0;
      ubpUnescapedRxBufferBufferLength = 0;

      if (UBP_incomingChecksumFailed) {

        UBP_incomingChecksumFailed();
      }
    }

  }  // else haven't RX'd final fragment yet, keep waiting
}

bool UBP_queuePacketTransmission(unsigned short packetIdentifier, UBP_TxFlags txFlags, const char *packetBytes, unsigned short byteCount) {

  if (UBP_isTxPending) {  // Preexisting transmission still in progress

#ifdef DEBUG
    Serial.println("Could not queue packet because preexisting transmission is still in progress");
#endif
    return false;

  } else {  // Ready to queue a new transmission

    if (hostIsConnected == false) {
#ifdef DEBUG
      Serial.println("Host not connected");
#endif
      return false;
    }

#ifdef DEBUG
    Serial.println("ready to queue a new transmission");
#endif
    ubpTxBufferLength = 0;

    // Start off with the END_BYTE as required for SLIP
    ubpTxBuffer[0] = END_BYTE;
    ubpTxBufferLength++;

    // Prepend the packet identifier
    memcpy(ubpTxBuffer + ubpTxBufferLength, &packetIdentifier, sizeof(packetIdentifier));  // TODO: Escape the identifier
    ubpTxBufferLength += sizeof(packetIdentifier);

    // Append the transmission flags
    ubpTxBuffer[ubpTxBufferLength] = txFlags;
    ubpTxBufferLength++;
#ifdef DEBUG
    Serial.println("appended the transmission flags");
#endif

    // Copy the escaped contents of packetBytes into the TX buffer following the packet identifier
    int escapedByteCount = _UBP_makeEscapedCopy(packetBytes, byteCount, ubpTxBuffer + ubpTxBufferLength, BUFFER_LENGTH);
#ifdef DEBUG
    Serial.println("made escaped copy");
#endif

    if (escapedByteCount != -1) {  // Escaping succeeded

      ubpTxBufferLength += escapedByteCount;
      int payloadLength = ubpTxBufferLength - 1;  // Length so far minus leading END byte    //sizeof(packetIdentifier) + escapedByteCount;  // <identifier length> + <escaped content length>

      // Calculate and append checksum
      byte checksumValue = CRC8(ubpTxBuffer + 1, payloadLength);  // Checksum over all payload bytes (minus the leading END byte, checksum, and trailing END byte)
      *(ubpTxBuffer + ubpTxBufferLength) = checksumValue;
      ubpTxBufferLength++;

      // Append trailing END byte
      *(ubpTxBuffer + ubpTxBufferLength) = END_BYTE;
      ubpTxBufferLength++;

      // Mark as ready to begin transmission
      ubpTxBufferSentByteCount = 0;
      UBP_isTxPending = true;

    } else {

#ifdef DEBUG
      Serial.println("Couldn't escape the content because it was going to overflow the output buffer");
#endif
      return false;  // Return false if we couldn't escape the content because it was going to overflow the output buffer
    }
  }
}

int _UBP_makeEscapedCopy(const char *inputBuffer, unsigned short inputBufferLength, char *outputBuffer, unsigned short outputBufferLength) {

  unsigned int bytesCopied = 0;
  const char *inputBytes = inputBuffer;  // Cast here to avoid compiler warnings later

  // 2016-07-24: changed i from char to int to avoid buffer over flow that had happend for more numbers larger than 256
  //for (char i = 0; i < inputBufferLength; i++) {  // For each byte to append
  for (int i = 0; i < inputBufferLength; i++) {  // For each byte to append

#ifdef DEBUG
    Serial.printf("Input Buffer length is %d and i = %d\n", inputBufferLength, i);
#endif
    // Escape any control characters. Refer to Serial Line IP (SLIP) spec.
    char aByte = *(inputBytes + i);
    if (aByte == ESCAPE_BYTE) {  // Escape an ESCAPE_BYTE

#ifdef DEBUG
      Serial.printf("%d:%x ESC\n", bytesCopied, aByte);
#endif
      if (bytesCopied + 1 >= outputBufferLength) return -1;  // Would overflow destination buffer
      else {
        *(outputBuffer + bytesCopied++) = ESCAPE_BYTE;  // Write ESCAPE_BYTE to buffer and increment offset
        *(outputBuffer + bytesCopied++) = ESCAPED_ESCAPE_BYTE;  // Write escaped ESCAPE_BYTE to buffer and increment offset
      }

    } else if (aByte == END_BYTE) {  // Escape an END_BYTE

#ifdef DEBUG
      Serial.printf("%d:%x END\n", bytesCopied, aByte);
#endif
      if (bytesCopied + 1 >= outputBufferLength) return -1;  // Would overflow destination buffer
      else {
        *(outputBuffer + bytesCopied++) = ESCAPE_BYTE;  // Write ESCAPE_BYTE to buffer and increment offset
        *(outputBuffer + bytesCopied++) = ESCAPED_END_BYTE;  // Write escaped END_BYTE to buffer and increment offset
      }

    } else {  // Not a control character

#ifdef DEBUG
      Serial.printf("%d:%x\n", bytesCopied, aByte);
#endif
      if (bytesCopied >= outputBufferLength) return -1;  // Would overflow destination buffer
      else *(outputBuffer + bytesCopied++) = aByte;  // Copy the unmolested byte to the buffer and increment offset
    }
  }

  return bytesCopied;
}

int _UBP_makeUnEscapedCopy(const char *inputBuffer, unsigned short inputBufferLength, char *outputBuffer) {

  bool done = false;
  char * destinationBufferPtr = outputBuffer;
  const char * sourceBufferPtr = inputBuffer;

  // UNESCAPE END SEQUENCE
  while (!done && (sourceBufferPtr - inputBuffer) < inputBufferLength) {

    char * substringPtr = strstr(sourceBufferPtr, escapedEndSequence);
    if (substringPtr == NULL) done = true;
    else {

      // Copy bytes between last-copied byte and next escape byte
      char lengthToCopy = substringPtr - sourceBufferPtr;  // How many bytes between last byte copied and next escape byte
      memcpy(destinationBufferPtr, sourceBufferPtr, lengthToCopy);
      destinationBufferPtr += lengthToCopy;
      sourceBufferPtr += lengthToCopy;

      // Replace escaped source sequence with unescaped version during copy, increment pointer
      memcpy(destinationBufferPtr, endSequence, sizeof(endSequence));
      destinationBufferPtr += sizeof(endSequence);

      // Increment pointer past escaped end sequence
      sourceBufferPtr += sizeof(escapedEndSequence);
    }
  }

  // UNESCAPE ESCAPE SEQUENCE
  done = false;
  while (!done && (sourceBufferPtr - inputBuffer) < inputBufferLength) {

    char * substringPtr = strstr(sourceBufferPtr, escapedEscapeSequence);
    if (substringPtr == NULL) done = true;
    else {

      // Copy bytes between last-copied byte and next escape byte
      char lengthToCopy = substringPtr - sourceBufferPtr;  // How many bytes between last byte copied and next escape byte
      memcpy(destinationBufferPtr, sourceBufferPtr, lengthToCopy);
      destinationBufferPtr += lengthToCopy;
      sourceBufferPtr += lengthToCopy;

      // Replace escaped source sequence with unescaped version during copy, increment pointer
      memcpy(destinationBufferPtr, escapeSequence, sizeof(escapeSequence));
      destinationBufferPtr += sizeof(escapeSequence);

      // Increment pointer past escaped end sequence
      sourceBufferPtr += sizeof(escapedEscapeSequence);
    }
  }

  // COPY ANY TRAILING BYTES
  char lengthToCopy = (inputBuffer + inputBufferLength) - sourceBufferPtr;  // How many bytes remain to be copied
  memcpy(destinationBufferPtr, sourceBufferPtr, lengthToCopy);
  destinationBufferPtr += lengthToCopy;
  sourceBufferPtr += lengthToCopy;

  return (destinationBufferPtr - outputBuffer);  // Return the total number of bytes copied to the destination buffer
}

void _UBP_hostDisconnected() {

  hostIsConnected = false;

  // Reset TX subsystem
  UBP_isTxPending = false;

  // Reset RX subsystem
  ubpRxBufferLength = 0;

  // Invoke user callback
  if (UBP_didDisconnect) UBP_didDisconnect();
}

bool pumpViaBluetooth(unsigned short packetIdentifier, UBP_TxFlags txFlags, const char *packetBytes, unsigned short byteCount)
{
  bool success = UBP_queuePacketTransmission(packetIdentifier, txFlags, packetBytes, byteCount);
  delay(1);
#ifdef DEBUG
  if (success) Serial.println("Packet queued successfully");
  else Serial.println("Failed to enqueue packet");
#endif
  while (UBP_isBusy() == true) UBP_pump();
}

/* ************ SLIP ********************* */

/* ************ Sensor ********************* */
String decodeSN(byte *data)
{
  byte uuid[8];
  String lookupTable[32] =
  {
    "0", "1", "2", "3", "4", "5", "6", "7", "8", "9",
    "A", "C", "D", "E", "F", "G", "H", "J", "K", "L",
    "M", "N", "P", "Q", "R", "T", "U", "V", "W", "X",
    "Y", "Z"
  };
  byte uuidShort[8];
  for (int i = 2; i < 8; i++)  uuidShort[i - 2] = data[i];
  uuidShort[6] = 0x00;
  uuidShort[7] = 0x00;
  String binary = "";
  String binS = "";
  for (int i = 0; i < 8; i++)
  {
    binS = String(uuidShort[i], BIN);
    int l = binS.length();
    if (l == 1) binS = "0000000" + binS;
    else if (l == 6) binS = "00" + binS;
    else if (l == 7) binS = "0" + binS;
    binary += binS;
  }
  String v = "0";
  char pozS[5];
  for (int i = 0; i < 10; i++)
  {
    for (int k = 0; k < 5; k++) pozS[k] = binary[(5 * i) + k];
    int value = (pozS[0] - '0') * 16 + (pozS[1] - '0') * 8 + (pozS[2] - '0') * 4 + (pozS[3] - '0') * 2 + (pozS[4] - '0') * 1;
    v += lookupTable[value];
  }
  return v;
}
void decodeSensor()
{
//  print_state("decodeSensor()");

  for (int i = 0; i < 24; i++) sensorDataHeader[i] = dataBuffer[i];
  for (int i = 24; i < 320; i++) sensorDataBody[i - 24] = dataBuffer[i];
  for (int i = 320; i < 344; i++) sensorDataFooter[i - 320] = dataBuffer[i];
  decodeSensorHeader();
  decodeSensorBody();
  decodeSensorFooter();
  displaySensorData();
}

void decodeSensorHeader()
{
//  print_state("decodeSensorHeader()");
  sensorData.sensorStatusByte = sensorDataHeader[4];
  switch (sensorData.sensorStatusByte)
  {
    case 0x01:
      sensorData.sensorStatusString = "not yet started";
      break;
    case 0x02:
      sensorData.sensorStatusString = "starting";
      break;
    case 0x03:
      sensorData.sensorStatusString = "ready";
      break;
    case 0x04:
      sensorData.sensorStatusString = "expired";
      break;
    case 0x05:
      sensorData.sensorStatusString = "shutdown";
      break;
    case 0x06:
      sensorData.sensorStatusString = "failure";
      break;
    default:
      sensorData.sensorStatusString = "unknown state";
      break;
  }
}

//
// sensor body data format format documented here: 
// https://github.com/vicktor/FreeStyleLibre-NFC-Reader/wiki/Progress-&-ToDo
//

void show_4_bytes(byte *buf)
{
  Serial.printf(", %x(%d)", buf[0], buf[0]);
  Serial.printf(", %x(%d)", buf[1], buf[1]);
  Serial.printf(", %x(%d)", buf[2], buf[2]);
  Serial.printf(", %x(%d)", ((buf[2] << 8) & 0xFF00) + buf[1], ((buf[2] << 8) & 0xFF00) + buf[1]);
  Serial.printf(", %x(%d)", buf[3], buf[3]);

  int rawTmp =    ((buf[1]&0x0F) << 8) + buf[0];
  int rawBGtemp = ((buf[3]&0x0F) << 8) + buf[2];
  int rawBGhugo1 = (buf[1]&0xF0)>>4;
  int rawBGhugo2 = (buf[3]&0xF0)>>4;

  Serial.printf(", rawTmp: %x(%d)", rawTmp, rawTmp);
  Serial.printf(", rawBGtemp: %x(%d)", rawBGtemp, rawBGtemp);
  Serial.printf(", rawBGhugo1: %x(%d)", rawBGhugo1, rawBGhugo1);
  Serial.printf(", rawBGhugo2: %x(%d)", rawBGhugo2, rawBGhugo2);
}

void decodeSensorBody()
{
//  print_state("decodeSensorBody() - ");

  int i;
//  Serial.println("");
/*
  for ( i = 0 ; i < (320-24) ; i++) {
    if ( (i%8) == 0 )
      Serial.println(F(""));
    if ( sensorDataBody[i] <= 0x0f )
      Serial.print("0");    
    Serial.print(sensorDataBody[i], HEX);
    Serial.print(" ");
  }
*/
/*
  // 16 values 15 mins trend
  print_state("block number that will be next written for 15 mins trend: "); Serial.print(sensorDataBody[2]);
  for ( i = 4 ; i < 100 ; i++ ) {
    if ( ((i-4)%6) == 0 ) {
      Serial.print("\r\nBlock #"); Serial.print((i-4)/6);
      Serial.print(", byte #"); Serial.print(i); Serial.print(" to #"); Serial.print(i+6); Serial.print(" ");
    }
    if ( sensorDataBody[i] <= 0x0f )
      Serial.print("0");    
    Serial.print(sensorDataBody[i], HEX);
    Serial.print(" ");        
  }
  // 32 values historic data
  print_state("block number that will be next written for historic data: "); Serial.print(sensorDataBody[3]);
  for ( i = 100 ; i < 292 ; i++ ) {
    if ( ((i-100)%6) == 0 ) {
      Serial.print("\r\nBlock #"); Serial.print((i-100)/6);
      Serial.print(", byte #"); Serial.print(i); Serial.print(" to #"); Serial.print(i+6); Serial.print(" ");
    }
    if ( sensorDataBody[i] <= 0x0f )
      Serial.print("0");    
    Serial.print(sensorDataBody[i], HEX);
    Serial.print(" ");        
  }
  // 4 tailing bytes
  print_state("tail bytes, first are the elapsed minutes: ");
  Serial.print("byte #292 to #296 ");
  for ( i = 292 ; i < 296 ; i++ ) {
    if ( sensorDataBody[i] <= 0x0f )
      Serial.print("0");    
    Serial.print(sensorDataBody[i], HEX);
    Serial.print(" ");        
  }
*/
  sensorData.nextTrend = sensorDataBody[2];
  sensorData.nextHistory = sensorDataBody[3];
//  print_state("next trend block / next history block: "); Serial.printf("%d %d", sensorData.nextTrend, sensorData.nextHistory);
  
  byte minut[2];
  minut[0] = sensorDataBody[293];
  minut[1] = sensorDataBody[292];

//  print_state("Sensor minutes: "); Serial.printf("%x %x", minut[0], minut[1]);

  sensorData.minutesSinceStart = minut[0] * 256 + minut[1];
  sensorData.minutesHistoryOffset = (sensorData.minutesSinceStart - 3) % 15 + 3 ;

//  print_state("Sensor minutes : "); Serial.print(sensorData.minutesSinceStart);
  Serial.print(", minutes history offset: "); Serial.print(sensorData.minutesHistoryOffset);

  int index = 0;
  
  for (int i = 0; i < 16; i++)                                // 16 bloków co 1 minutę
  {
    index = 4 + (sensorData.nextTrend - 1 - i) * 6;
    if (index < 4) index = index + 96;
    byte pomiar[6];
    for (int k = index; k < index + 6; k++) pomiar[k - index] = sensorDataBody[k];
/*
    print_state("Block #"); Serial.print((index-4)/6); Serial.print(" ");
//    Serial.print(", pomiar[] = ");
    int j;
    for ( j = 0 ; j < 6 ; j++ ) {
      if ( pomiar[j] <= 0x0F )
        Serial.print("0");
      Serial.print(pomiar[j], HEX);
      Serial.print(" ");
    }
*/    
    sensorData.trend[i] = ((pomiar[1] << 8) & 0x0F00) + pomiar[0];
/*
    Serial.print(", #["); Serial.print(i); Serial.print("] "); Serial.print(sensorData.trend[i], HEX);
    Serial.print("("); Serial.print(sensorData.trend[i]); Serial.print("(");
     show_4_bytes(&pomiar[2]);
*/      
  }

  get_packet(&Pkts.buffer[Pkts.write] , sensorData.trend[0]);
  print_state("got packet, stored at position ");
  Serial.print(Pkts.write);
  Serial.print(F(", incrementing write to "));
  // so increment write position for next round...
  if ( ++Pkts.write >= DXQUEUESIZE )
    Pkts.write = 0;
  Serial.print(Pkts.write);
  if (Pkts.read == Pkts.write) {
    print_state("queue overflow, incrementing read overwriting oldest entry");
    if ( ++Pkts.read >= DXQUEUESIZE ) //overflow in ringbuffer, overwriting oldest entry, thus move read one up
      Pkts.read = 0;
  }

  index = 0;
  for (int i = 0; i < 32; i++)                                // 32 bloki co 15 minut
  {
    index = 100 + (sensorData.nextHistory - 1 - i) * 6;
    if (index < 100) index = index + 192;
    byte pomiar[6];
    for (int k = index; k < index + 6; k++) pomiar[k - index] = sensorDataBody[k];
    sensorData.history[i] = ((pomiar[1] << 8) & 0x0F00) + pomiar[0];
    //    Int16 rawTmp = Convert.ToInt16(((Convert.ToInt16(pomiar[1]) << 8) & 0x0F00) + Convert.ToInt16(pomiar[0]));
    //    int rawBGtemp = ((Convert.ToInt16(pomiar[5]) << 8) & 0x0F) + Convert.ToInt16(pomiar[4]);
    //    int rawBGhugo = Convert.ToInt16(pomiar[3]);
  }
}
void decodeSensorFooter()
{

}

void displaySensorData()
{
  if (!sensorData.sensorDataOK) 
    print_state("Sensor data error");
  else
  {
    print_state("Sensor data OK, S/N "); Serial.print(sensorData.sensorSN);
    Serial.print(", Sensor status: ");
    Serial.print(sensorData.sensorStatusByte, HEX); Serial.print(" - "); Serial.print(sensorData.sensorStatusString);
    print_state("Next trend position: ");
    Serial.print(sensorData.nextTrend);
    Serial.print(", Next history position: ");
    Serial.print(sensorData.nextHistory);
    Serial.print(", Minutes since sensor start: ");
    Serial.print(sensorData.minutesSinceStart);
    Serial.print(", Minutes trend to history offset: ");
    Serial.print(sensorData.minutesHistoryOffset);
    print_state("BG trend: ");
    for (int i = 0; i < 16; i++) Serial.printf(" %f", (float)(sensorData.trend[i] / 10.0f));
    Serial.println("");
    Serial.print("BG history: ");
    for (int i = 0; i < 32; i++) {
      Serial.printf(" %f", (float)(sensorData.history[i] / 10.0f));
      if ( i == 16 )
        Serial.println("");
    }
    //      Serial.println("");
  }
}



/* ************ Sensor ********************* */

/* ************ crc16 ********************* */
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

/* ************ crc16 ********************* */

/* ************ crc8 ********************* */
#define CRC8INIT  0x00
#define CRC8POLY  0x18      // 0X18 = X^8+X^5+X^4+X^0

byte CRC8(void *bytes, byte number_of_bytes_to_read) {

  byte *data_in = (byte*) bytes;
  byte  crc;
  uint16_t loop_count;
  byte  bit_counter;
  byte  data;
  byte  feedback_bit;

  crc = CRC8INIT;

  for (loop_count = 0; loop_count != number_of_bytes_to_read; loop_count++)
  {
    data = data_in[loop_count];

    bit_counter = 8;
    do {
      feedback_bit = (crc ^ data) & 0x01;

      if ( feedback_bit == 0x01 ) {
        crc = crc ^ CRC8POLY;
      }
      crc = (crc >> 1) & 0x7F;
      if ( feedback_bit == 0x01 ) {
        crc = crc | 0x80;
      }

      data = data >> 1;
      bit_counter--;

    } while (bit_counter > 0);
  }

  return crc;
}

/* ************ crc8 ********************* */

/* ************************ LBridge code *********************** */
/* ********************************************* */
/*  help functions */
/* *********************************************** */

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

// print timestamp and current status/action
void print_state(String str)
{
  unsigned long ms = millis();
  unsigned long val;
  int i;

  Serial.println("");
  Serial.print("[");
  Serial.print(ms / 60000);
  Serial.print("][");
  if ( (val = ((ms / 1000) % 60)) < 10 )
    Serial.print("0");
  Serial.print(val);
  Serial.print("][");
  if ( (val = (ms % 1000)) < 10 )
    Serial.print("00");
  else if ( val < 100 )
    Serial.print("0");
  Serial.print(val);
  Serial.print("] ");

/*
  Serial.print(" -");
  for ( i = 0 ; i < nl ; i++ )
    Serial.print("-");
  Serial.print(" ");
*/
  Serial.print(str);
}

// send data to BLE
void send_data(unsigned char *msg, unsigned char len)
{
  unsigned char i = 0;

  last_ble_send_time = millis();
  crlf_printed = 0;
#ifdef RFD
  RFduinoBLE.send((char *)msg, len);
#else
  SimbleeBLE.send((char *)msg, len);
#endif
  delay(100);

  print_state("sending: <");
  for ( i = 0 ; i < len ; i++ )
    Serial.printf("%x ", msg[i]);
  Serial.print(">");
  print_state("response: ");
}

// due to Arduino cast problems between String and char * use extra function
int send_ble_string(String cmd, boolean connect_state)
{
  int i;
  unsigned long now;
  boolean timeout;

  if ( !connect_state ) {
    for ( i = 0 ; i < 3 ; i++ ) {
      if ( show_ble ) {
        print_state(" ->(");
        Serial.print(cmd);
        Serial.print(F(") - "));
      }

      last_ble_send_time = millis();
      crlf_printed = 0;
#ifdef RFD
      RFduinoBLE.send(cmd.cstr(), strlen(cmd.cstr()));
#else
      SimbleeBLE.send(cmd.cstr(), strlen(cmd.cstr()));
#endif
      //      ble_Serial.print(cmd);

      now = millis();
      ble_answer = 0;
      timeout = 1;

      while ( (millis() - now) < 3000 ) {
        waitDoingServices(30, 1);
        if ( ble_answer == 2 ) {
          ble_answer = 0;
          timeout = 0;
          break;
        }
      }
      waitDoingServices(500, 1);

    }
    return (0);
  }
  return (0);
}

void send_string(String cmd, int dly)
{
  last_ble_send_time = millis();
  crlf_printed = 0;
  //  ble_Serial.print(cmd);
#ifdef RFD
  RFduinoBLE.send(cmd.cstr(), strlen(cmd.cstr()));
#else
  SimbleeBLE.send(cmd.cstr(), strlen(cmd.cstr()));
#endif
  if ( show_ble ) {
    print_state(" ->(");
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
  delay(200);
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
int commandBuffIs(char* command)
{
  unsigned char len = strlen(command);
  if (len != command_buff.nCurReadPos)
    return (0);
  return ( memcmp(command, command_buff.commandBuffer, len) == 0 );
}

// decode incoming serial/USB or BLE data commands
int doCommand(void)
{
  // TXID packet?
  if (command_buff.commandBuffer[1] == 0x01 && command_buff.commandBuffer[0] == 0x06)
  {
    memcpy(&settings.dex_tx_id, &command_buff.commandBuffer[2], sizeof(settings.dex_tx_id));
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
    print_state("DataRequestPacket received");
    got_drp = 1;
    init_command_buff(&command_buff);
    return (0);
  }
#endif
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
    if ( show_state ) {
      print_state("S command - dont show states");
      show_state = 0;
    }
    else {
      show_state = 1;
      print_state("S command - show states");
    }
  }  // "OK+..." answer from BLE?
  if ( commandBuffIs("OK") )
  {
    got_ok = 1;
    //    print_state("got_ok = 1");
    return (0);
  }
  // we don't respond to unrecognised commands.
  return (1);
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
  if (command_buff.nCurReadPos > 0 && (millis() - cmd_to) > 2000)
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
  // ???
  //  while( (Serial.available() || ble_Serial.available()) && command_buff.nCurReadPos < COMMAND_MAXLEN) {
  while ( (Serial.available() || bleCharAvailable()) && command_buff.nCurReadPos < COMMAND_MAXLEN) {

    if ( ble_answer == 0 ) {
      //      Serial.print("-1-");
      ble_answer = 1;
    }

    // BLE receive which is not 3 s after last sending?
    if ( ((millis() - last_ble_send_time) > 3000) && !crlf_printed ) {
      //        print_state(F(" - CRLF printed"));
      Serial.print(F("\r\n"));
      crlf_printed = 1;
    }

    if ( bleCharAvailable() )
      b = bleBufRead();

    command_buff.commandBuffer[command_buff.nCurReadPos] = b;
    command_buff.nCurReadPos++;
    cmd_to = millis();
    // if it is the end for the byte string, we need to process the command
    // valid data packet or "OK" received?
    if (      command_buff.nCurReadPos == command_buff.commandBuffer[0] \
              || (command_buff.commandBuffer[0] == 'O' && command_buff.commandBuffer[1] == 'K' ) \
              || (command_buff.nCurReadPos == 1 && toupper(command_buff.commandBuffer[0]) == 'B') \
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
    //    Serial.print("-2-");
    ble_answer = 2;
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


// wait for a NFC reading and put it into Dexcom_packet
int get_packet(Dexcom_packet* pPkt, int actGlucose)
{
  print_state("get_packet() - ");
//  pPkt->raw = actGlucose * 100;
  pPkt->raw = (actGlucose/8.5)*1000;
  pPkt->ms = millis();
  pkt_time = pPkt->ms;
  Serial.print("packet read at ");
  Serial.print(pPkt->ms);
  Serial.print(F(" ms"));
  /*
    print_state("last packet read ");
    Serial.print((abs_millis() - last_abs_pkt_time)/1000);
    Serial.print(F(" s before"));
  */
  return (1);
}

//function to format and send the passed Dexom_packet.
int print_packet(Dexcom_packet* pPkt)
{
  nRawRecord msg;
  unsigned char msgbuf[80];

//  print_state("print_packet()");

  if ( !BTconnected ) {
    print_state("no BT connection, skip");
    return (0);
  }

  //prepare the message
  // count every memeber as structs seems to be long aligned (adding 0x00 0x20 to gaps)
  msg.size = sizeof(msg.size) + sizeof(msg.cmd_code) + sizeof(msg.raw) + sizeof(msg.filtered)
             + sizeof(msg.dex_battery) + sizeof(msg.my_battery) + sizeof(msg.dex_src_id)
             + sizeof(msg.delay) + sizeof(msg.function);
  msg.cmd_code = 0x00;
  msg.raw = pPkt->raw;
  msg.filtered = pPkt->raw;
  msg.dex_battery = 214;  // simulate good dexcom transmitter battery
  msg.my_battery = SoCData.voltagePercent;
  msg.dex_src_id = settings.dex_tx_id;
  msg.delay = millis() - pPkt->ms;
  msg.function = DEXBRIDGE_PROTO_LEVEL; // basic functionality, data packet (with ack), TXID packet, beacon packet (also TXID ack).
  /*
    Serial.print("Values to send: ");
    Serial.print(msg.size, HEX);
    Serial.print(" ");
    Serial.print(msg.cmd_code, HEX);
    Serial.print(" [");
    Serial.print(msg.raw, HEX);
    Serial.print(" ");
    Serial.print(msg.filtered, HEX);
    Serial.print("] (");
    Serial.print(msg.dex_battery, HEX);
    Serial.print(" ");
    Serial.print(msg.my_battery, HEX);
    Serial.print(") [");
    Serial.print(msg.dex_src_id, HEX);
    Serial.print("] ");
    Serial.print(msg.delay, HEX);
    Serial.print(" ");
    Serial.println(msg.function, HEX);
  */
  *(unsigned char *)&msgbuf[0] = msg.size;
  *(unsigned char *)&msgbuf[1] = msg.cmd_code;
  *(unsigned long *)&msgbuf[2] = msg.raw;
  *(unsigned long *)&msgbuf[6] = msg.filtered;
  *(unsigned char *)&msgbuf[10] = msg.dex_battery;
  *(unsigned char *)&msgbuf[11] = msg.my_battery;
  *(unsigned long *)&msgbuf[12] = msg.dex_src_id;
  *(unsigned long *)&msgbuf[16] = msg.delay;
  *(unsigned char *)&msgbuf[20] = msg.function;

  //  strcpy((char *)msgbuf, "171882 3891 58 21683");
  //  send_data((unsigned char *)&msgbuf, 20);

  send_data((unsigned char *)&msgbuf, msg.size);

  /*
    print_state("sending packet with a delay of ");
    Serial.print(msg.delay);
    Serial.print(" ms");
  */
  return (1);
}

/* ************************ end of LBridge code **************** */
