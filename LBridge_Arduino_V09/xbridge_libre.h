//define the xBridge Version
#define VERSION ("12.46")
// defines the xBridge protocol functional level.  Sent in each packet as the last byte.
#define DEXBRIDGE_PROTO_LEVEL (0x01)


typedef struct __attribute__((__packed__)) _beacon_packet
{
  uint8_t len;
  uint8_t cmd_code; // code for this data packet.  0xF1
  uint32_t  dex_src_id;   //raw TXID of the Dexcom Transmitter
  uint8_t function;// Byte representing the xBridge code funcitonality.  01 = this level.
} Beacon_packet;


//TXID packet - App to Bridge.  Sends the TXID the App wants the bridge to filter on.  In response to a Data packet or beacon packet being wrong.
//  0x06  - Length of the packet.
//  0x01  - Packet Type (01 means TXID packet).
//  uint32  - Dexcom encoded TXID.
typedef struct __attribute__((__packed__)) _txid_packet
{
  uint8_t len; //Length of the packet.
  uint8_t cmd_code; // Packet Type (01 means TXID packet).
  uint32_t  dex_src_id;   //Dexcom encoded TXID
};


// structure of a raw record we will send.
//Data Packet - Bridge to App.  Sends the Dexcom transmitter data, and the bridge battery volts.
//  0x11  - length of packet.
//  0x00  - Packet type (00 means data packet)
//  uint32  - Dexcom Raw value.
//  uint32  - Dexcom Filtered value.
//  uint8_t   dex_battery;  //battery value
//  uint8_t   my_battery; //xBridge battery value
//  uint32_t  dex_src_id;   //raw TXID of the Dexcom Transmitter
//  uint8_t  function; // Byte representing the xBridge code funcitonality.  01 = this level.
typedef struct __attribute__((__packed__)) _RawRecord
{
  uint8_t   size; //size of the packet (17)
  uint8_t   cmd_code; // code for this data packet.  Always 00 for a Dexcom data packet.
  uint32_t  raw;  //"raw" BGL value.
  uint32_t  filtered; //"filtered" BGL value 
  uint8_t   dex_battery;  //battery value
  uint8_t   my_battery; //xBridge battery value
  uint32_t  dex_src_id;   //raw TXID of the Dexcom Transmitter
  uint8_t   function; // Byte representing the xBridge code funcitonality.  01 = this level.
};


//buffer[1] == 0x02 && buffer[2] == 0x0 request data now - (status or historical readings on demand)
typedef struct __attribute__((__packed__)) _DataRequestPacket
{
  uint8_t   size; //size of the packet (12)
  uint8_t   cmd_code; // Always 0x02 for a xbridgeEXT protocol packet.
  uint8_t   sub_code; // Subtype code. 0x00 for a request packet.
  uint8_t   requested_sub_code; //0x00 xbridge Data Packet, 0x01 status packet, 0x02  last 15 minutes readings part1, 0x03 last 15 minutes readings part2, 0x04 last 8 hours readings part1, 0x05 last 8 hours readings part2
  uint32_t  dex_src_id; //raw TXID of the requested Dexcom Transmitter
  uint32_t  timestamp; //current timestamp
};


//buffer[1] == 0x02 && buffer[2] == 0x1 get sensor status details(battery,serial number, age...)
typedef struct __attribute__((__packed__)) _StatusPacket
{
  uint8_t   size; //size of the packet (19)
  uint8_t   cmd_code; // Always 0x02 for a xbridgeEXT protocol packet.
  uint8_t   sub_code; // Subtype code. 0x01 for a status packet.
  uint8_t   sensorStatusByte;
  char      sensorSN[11]; //0M00000RG3D
  uint16_t  minutesSinceStart;
  uint8_t   temperatureC;
  uint8_t   rssi;
};


//buffer[1] == 0x02 && buffer[2] == 0x2 get last 15 minutes readings - part1 (current and last 7 readings, 1 minute interval)
typedef struct __attribute__((__packed__)) _QuarterPacket1
{
  uint8_t   size; //size of the packet (19)
  uint8_t   cmd_code; // Always 0x02 for a xbridgeEXT protocol packet.
  uint8_t   sub_code; // Subtype code. 0x02 for a last quarter hour packet part1.
  uint16_t  trend[8]; //BG readings in the current minute, previous minute, 2, 3, 4, 5, 6, 7 minutes ago
};
//buffer[1] == 0x02 && buffer[2] == 0x3 get last 15 minutes readings - part1 (next 8 readings, 1 minute interval)
typedef struct __attribute__((__packed__)) _QuarterPacket2
{
  uint8_t   size; //size of the packet (19)
  uint8_t   cmd_code; // Always 0x02 for a xbridgeEXT protocol packet.
  uint8_t   sub_code; // Subtype code. 0x03 for a last quarter hour packet part2..
  uint16_t  trend[8]; //BG readings 8, 9, 10, 11, 12, 13, 14, 15 minutes ago
};


//buffer[1] == 0x02 && buffer[2] == 0x4 get last 8 hours readings - part1 (last 8 readings, 15 minutes interval)
typedef struct __attribute__((__packed__)) _HistoryPacket1
{
  uint8_t   size; //size of the packet (19)
  uint8_t   cmd_code; // Always 0x02 for a xbridgeEXT protocol packet.
  uint8_t   sub_code; // Subtype code. 0x04 for a last 8 hour, packet part1.
  uint16_t  history[8]; //BG readings 15, 30, 45, 60, 75, 90, 105, 120 minutes ago
};
//buffer[1] == 0x02 && buffer[2] == 0x5 get last 8 hours readings - part2 (next 8 readings, 15 minutes interval)
typedef struct __attribute__((__packed__)) _HistoryPacket2
{
  uint8_t   size; //size of the packet (19)
  uint8_t   cmd_code; // Always 0x02 for a xbridgeEXT protocol packet.
  uint8_t   sub_code; // Subtype code. 0x05 for a last 8 hour, packet part1.
  uint16_t  history[8]; //BG readings 135, 150, 165, 180, 195, 210, 225, 240 minutes ago
};
//buffer[1] == 0x02 && buffer[2] == 0x6 get last 8 hours readings - part3 (next 8 readings, 15 minutes interval)
typedef struct __attribute__((__packed__)) _HistoryPacket3
{
  uint8_t   size; //size of the packet (19)
  uint8_t   cmd_code; // Always 0x02 for a xbridgeEXT protocol packet.
  uint8_t   sub_code; // Subtype code. 0x06 for a last 8 hour, packet part1.
  uint16_t  history[8]; //BG readings 255, 270, 285, 300, 315, 330, 345, 360 minutes ago
};
//buffer[1] == 0x02 && buffer[2] == 0x7 get last 8 hours readings - part4 (next 8 readings, 15 minutes interval)
typedef struct __attribute__((__packed__)) _HistoryPacket4
{
  uint8_t   size; //size of the packet (19)
  uint8_t   cmd_code; // Always 0x02 for a xbridgeEXT protocol packet.
  uint8_t   sub_code; // Subtype code. 0x07 for a last 8 hour, packet part4.
  uint16_t  history[8]; //BG readings 375, 390, 405, 420, 435,450, 465, 480 minutes ago
};



