#include <Arduino.h>

/* COMPILE FOR
  --------------------
  Board: Arduino NANO
  Processor: ATMEGA328P (Old Booatloader)
  --------------------

  Manual_IP_switch_MK2 for Arduino rev.0.2
-----------------------------------------------------------
  https://remoteqth.com/wiki/index.php?page=Band+decoder+MK2
  2019-01 by OK1HRA

  ___               _        ___ _____ _  _
 | _ \___ _ __  ___| |_ ___ / _ \_   _| || |  __ ___ _ __
 |   / -_) '  \/ _ \  _/ -_) (_) || | | __ |_/ _/ _ \ '  \
 |_|_\___|_|_|_\___/\__\___|\__\_\|_| |_||_(_)__\___/_|_|_|


    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 2 of the License, or
    (at your option) any later version.
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.

Features:
  * control IP relay https://remoteqth.com/wiki/index.php?page=IP+Switch+with+ESP32-GATEWAY
  * low latency
  * 8 buttons ON/OFF and up to 16 encoder switch (one from)
  * automatic nework pair by ID
  * inhibit by PTT
  * ethernet plug detect with auto reinitialization

  Changelog
  ---------
  2019-04 - group button support (idea TNX SM0MDG)
  2019-03 - redesign CLI
          - multi control support (idea TNX SM0MDG)
          - PTT trasfer as one button
  2019-01 - rx encoder range and relay set
          - ptt lock
          - NET-ID prefix/sufix support
  2018-11 - rebuild ethernet and some bug fix
  2018-06 - support IP relay
  2018-05 - mk2 initial release

  ToDo
  ----
  - select ID by band decoder
  - encoder show ANT fullname
*/
const char* REV = "20190406";

//=====[ Settings ]===========================================================================================

#define LcdI2Caddress  0x27   // 0x27 0x3F - may be find with I2C scanner https://playground.arduino.cc/Main/I2cScanner
#define LCD_PCF8574           // If LCD uses PCF8574 chip
// #define LCD_PCF8574T          // If LCD uses PCF8574T chip
// #define LCD_PCF8574AT         // If LCD uses PCF8574AT chip
// #define SERIAL_debug          // Enable debuging on serial terminal, enable with send *
// #define FastBoot              // Enable fast power up, without shown LCD information

// You can set remote relay IP address, from outside local network
byte DetectedRemoteSw[16][5]{
  {0,0,0,0, 0},   // IP:port ID 0
  {0,0,0,0, 0},   // IP:port ID 1
  {0,0,0,0, 0},   // IP:port ID 2
  {0,0,0,0, 0},   // IP:port ID 3
  {0,0,0,0, 0},   // IP:port ID 4
  {0,0,0,0, 0},   // IP:port ID 5
  {0,0,0,0, 0},   // IP:port ID 6
  {78,111,124,210, 88},   // IP:port ID 7 - remoteqth test point
  {0,0,0,0, 0},   // IP:port ID 8
  {0,0,0,0, 0},   // IP:port ID 9
  {0,0,0,0, 0},   // IP:port ID A
  {0,0,0,0, 0},   // IP:port ID B
  {0,0,0,0, 0},   // IP:port ID C
  {0,0,0,0, 0},   // IP:port ID D
  {0,0,0,0, 0},   // IP:port ID E
  {0,0,0,0, 0},   // IP:port ID F
};
//==================================================================================================

//=====[ OBSOLETE ]=============================================================================================

// #define YAESU_BCD          // TTL BCD in A
// #define ICOM_ACC           // voltage 0-8V on pin4 ACC(2) connector - need calibrate table
// #define INPUT_SERIAL       // telnet ascii input - cvs format [band],[freq]\n
// #define ICOM_CIV           // read frequency from CIV

// #define KENWOOD_PC         // RS232 CAT
// #define YAESU_CAT          // RS232 CAT YAESU CAT since 2015 ascii format
// #define YAESU_CAT_OLD      // Old binary format RS232 CAT ** tested on FT-817 **

//=====[ Outputs ]============================================================================================

// #define REMOTE_RELAY       // TCP/IP remote relay - need install and configure TCP232 module
// #define SERIAL_echo        // Feedback on serial line in same baudrate, CVS format <[band],[freq]>\n
// #define ICOM_CIV_OUT       // send frequency to CIV ** you must set TRX CIV_ADRESS, and disable ICOM_CIV **
// #define KENWOOD_PC_OUT     // send frequency to RS232 CAT ** for operation must disable REQUEST **
// #define YAESU_CAT_OUT      // send frequency to RS232 CAT ** for operation must disable REQUEST **

#define LCD                   // Uncoment to Enable I2C LCD
bool PttTransfer = 0;
byte ButtonUseForPttTransfer= 8;  // 1-8
bool HW_BCD_SW = 0;              // enable hardware ID board bcd switch (disable if not installed)
bool SET_RX_DATA = 0;         // multi control
bool Need_SET_RX_DATA = 1;
bool GroupButton;             // group button
bool DEBUG = 1;
long HW_BCD_SWTimer[2]{0,3000};
int NumberOfEncoderOutputs = 8;  // 2-16
int EncoderCount = 0;
const int SERIAL_BAUDRATE = 115200;
int incomingByte = 0;   // for incoming serial data
long GetNetIdTimer[2]{0,2000};
long ButtDebounceTimer[2]{0,200};
long EncEndTimer[2]{0,1000};
int EncEndStatus = 0;
const int ButtonMapping[8]={3,2,1,0,7,6,5,4}; // 1-8

#define EthModule             // enable Ethernet module if installed
#define __USE_DHCP__          // enable DHCP
// #define BcdToIP               // control IP relay in BCD format
bool EthLinkStatus = 0;
long EthLinkStatusTimer[2]{1500,1000};

#define SERBAUD        115200   // [baud] Serial port in/out baudrate
#define WATCHDOG       10     // [sec] determines the time, after which the all relay OFF, if missed next input data - uncomment for the enabled
#define REQUEST        500    // [ms] use TXD output for sending frequency request
#define CIV_ADRESS   0x56     // CIV input HEX Icom adress (0x is prefix)
#define CIV_ADR_OUT  0x56     // CIV output HEX Icom adress (0x is prefix)
// #define ENABLE_DIVIDER     // for highest voltage D-SUB pin 13 inputs up to 24V - need short JP9

#if defined(DUMMY)

  //=====[ FREQUEN RULES ]===========================================================================================
  const long Freq2Band[16][2] = {/*
  Freq Hz from       to   Band number
  */   {1810000,   2000000},  // #1 [160m]
       {3500000,   3800000},  // #2  [80m]
       {7000000,   7200000},  // #3  [40m]
      {10100000,  10150000},  // #4  [30m]
      {14000000,  14350000},  // #5  [20m]
      {18068000,  18168000},  // #6  [17m]
      {21000000,  21450000},  // #7  [15m]
      {24890000,  24990000},  // #8  [12m]
      {28000000,  29700000},  // #9  [10m]
      {50000000,  52000000},  // #10  [6m]
     {144000000, 146000000},  // #11  [2m]
     {430000000, 440000000},  // #12  [70cm]
     {1240000000, 1300000000},  // #13  [23cm]
     {2300000000, 2450000000},  // #14  [13cm]
     {3300000000, 3500000000},  // #15  [9cm]
     {5650000000, 5850000000},  // #16  [6cm]
  };

  //=====[ Sets band -->  to output in MATRIX table ]===========================================================

          const boolean matrix[17][16] = { /*

          Band 0 --> */ { 0,  0,  0,  0,  0,  0,  0,  0,    0,  0,  0,  0,  0,  0,  0,  1 }, /* first eight shift register board
  \       Band 1 --> */ { 1,  0,  0,  0,  0,  0,  0,  0,    0,  0,  0,  0,  0,  0,  0,  0 }, /*
   \      Band 2 --> */ { 0,  1,  0,  0,  0,  0,  0,  0,    0,  0,  0,  0,  0,  0,  0,  0 }, /*
    \     Band 3 --> */ { 0,  0,  1,  0,  0,  0,  0,  0,    0,  0,  0,  0,  0,  0,  0,  0 }, /*
     \    Band 4 --> */ { 0,  0,  0,  1,  0,  0,  0,  0,    0,  0,  0,  0,  0,  0,  0,  0 }, /*
      \   Band 5 --> */ { 0,  0,  0,  0,  1,  0,  0,  0,    0,  0,  0,  0,  0,  0,  0,  0 }, /*
       \  Band 6 --> */ { 0,  0,  0,  0,  0,  1,  0,  0,    0,  0,  0,  0,  0,  0,  0,  0 }, /*
  IN    ) Band 7 --> */ { 0,  0,  0,  0,  0,  0,  1,  0,    0,  0,  0,  0,  0,  0,  0,  0 }, /*
       /  Band 8 --> */ { 0,  0,  0,  0,  0,  0,  0,  1,    0,  0,  0,  0,  0,  0,  0,  0 }, /*

      /   Band 9 --> */ { 0,  0,  0,  0,  0,  0,  0,  0,    1,  0,  0,  0,  0,  0,  0,  0 }, /* second eight shift register board
     /    Band 10 -> */ { 0,  0,  0,  0,  0,  0,  0,  0,    0,  1,  0,  0,  0,  0,  0,  0 }, /* (optional)
    /     Band 11 -> */ { 0,  0,  0,  0,  0,  0,  0,  0,    0,  0,  1,  0,  0,  0,  0,  0 }, /*
   /      Band 12 -> */ { 0,  0,  0,  0,  0,  0,  0,  0,    0,  0,  0,  1,  0,  0,  0,  0 }, /*
  /       Band 13 -> */ { 0,  0,  0,  0,  0,  0,  0,  0,    0,  0,  0,  0,  1,  0,  0,  0 }, /*
          Band 14 -> */ { 0,  0,  0,  0,  0,  0,  0,  0,    0,  0,  0,  0,  0,  1,  0,  0 }, /*
          Band 15 -> */ { 0,  0,  0,  0,  0,  0,  0,  0,    0,  0,  0,  0,  0,  0,  1,  0 }, /*
          Band 16 -> */ { 0,  0,  0,  0,  0,  0,  0,  0,    0,  0,  0,  0,  0,  0,  0,  1 }, /*
                          |   |   |   |   |   |   |   |     |   |   |   |   |   |   |   |
                          V   V   V   V   V   V   V   V     V   V   V   V   V   V   V   V
                       ----------------------------------  ---------------------------------
                       |  1   2   3   4   5   6   7   8     9  10  11  12  13  14  15  16  |
                       ----------------------------------  ---------------------------------
                                                     OUTPUTS
                                      (for second eight need aditional board)*/
          };

  //=====[ BCD OUT ]===========================================================================================

          const boolean BCDmatrixOUT[4][16] = { /*
          --------------------------------------------------------------------
          Band # to output relay   0   1   2   3   4   5   6   7   8   9  10
          (Yaesu BCD)                 160 80  40  30  20  17  15  12  10  6m
          --------------------------------------------------------------------
                                   |   |   |   |   |   |   |   |   |   |   |
                                   V   V   V   V   V   V   V   V   V   V   V
                              */ { 0,  1,  0,  1,  0,  1,  0,  1,  0,  1,  0, 1, 0, 1, 0, 1 }, /* --> DB25 Pin 11
                              */ { 0,  0,  1,  1,  0,  0,  1,  1,  0,  0,  1, 1, 0, 0, 1, 1 }, /* --> DB25 Pin 24
                              */ { 0,  0,  0,  0,  1,  1,  1,  1,  0,  0,  0, 0, 1, 1, 1, 1 }, /* --> DB25 Pin 12
                              */ { 0,  0,  0,  0,  0,  0,  0,  0,  1,  1,  1, 1, 1, 1, 1, 1 }, /* --> DB25 Pin 25
          */};

  //============================================================================================================

  // #define UdpBroadcastDebug_debug
#endif

#if defined(LCD)
  #include <Wire.h>

  #if defined(LCD_PCF8574T) || defined(LCD_PCF8574)
    #include <LiquidCrystal_PCF8574.h>
    LiquidCrystal_PCF8574 lcd(LcdI2Caddress);
  #endif
  #if defined(LCD_PCF8574AT)
    #include <LiquidCrystal_I2C.h>
    LiquidCrystal_I2C lcd(LcdI2Caddress,16,2);
  #endif

  long LcdRefresh[2]{0,500};
  // const char* ANTname[17] = {
  //     "Out of band",  // Band 0 (no data)
  //     "Dipole",       // Band 1
  //     "Vertical",     // Band 2
  //     "3el Yagi",     // Band 3
  //     "Windom",       // Band 4
  //     "DeltaLoop",    // Band 5
  //     "20m Stack",    // Band 6
  //     "DeltaLoop",    // Band 7
  //     "HB9",          // Band 8
  //     "Dipole",       // Band 9
  //     "5el Yagi",     // Band 10
  //     "7el Yagi",     // Band 11
  //     "24el",         // Band 12
  //     "20el quad",    // Band 13
  //     "Dish 1.2m",    // Band 14
  //     "Dish 1.2m",    // Band 15
  //     "Dish 1m",      // Band 16
  // };
  // byte LockChar[8] = {0b00100, 0b01010, 0b01010, 0b11111, 0b11011, 0b11011, 0b11111, 0b00000};
  uint8_t LockChar[8] = {0x4,0xa,0xa,0x1f,0x1b,0x1b,0x1f,0x0};
  // byte EthChar[8] = {0b00000, 0b00000, 0b11111, 0b10001, 0b10001, 0b11011, 0b11111, 0b00000};
  uint8_t EthChar[8] = {0x0,0x0,0x1f,0x11,0x11,0x1b,0x1f,0x0};
  uint8_t DotChar[8] = {0x0,0x0,0x0,0x4,0x0,0x0,0x0,0x0};
  const byte rq[2][13] = {
    {0x49, 0x50, 0x20, 0x63, 0x6f, 0x6e, 0x74, 0x72, 0x6f, 0x6c, 0x20, 0x62, 0x79},
    {0x52, 0x65, 0x6d, 0x6f, 0x74, 0x65, 0x51, 0x54, 0x48, 0x2e, 0x63, 0x6f, 0x6d},
  };
  bool LcdNeedRefresh = false;
#endif

#if defined(EthModule)
  const byte RemoteDevice = 's';
  const byte ThisDevice = 'm';
  byte NET_ID = 0x00;         // NetID [hex] MUST BE UNIQUE IN NETWORK - replace by P6 board encoder
  bool EnableEthernet = 1;
  bool EnableDHCP     = 1;
  #include <Ethernet.h>
  #include <EthernetUdp.h>
  // #include <Ethernet2.h>
  // #include <EthernetUdp2.h>
  //  #include <util.h>
  // #include <Dhcp.h>
  // #include <EthernetServer.h>
  #include <SPI.h>
  byte LastMac = 0x00 + NET_ID;

  byte mac[] = {0xDE, 0xAD, 0xBE, 0xEF, 0x23, LastMac};
  // byte mac[] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, LastMac};
  IPAddress ip(192, 168, 1, 222);         // IP
  IPAddress gateway(192, 168, 1, 200);    // GATE
  IPAddress subnet(255, 255, 255, 0);     // MASK
  IPAddress myDns(8, 8, 8, 8);            // DNS (google pub)
  EthernetServer server(80);              // Web server PORT
  String HTTP_req;

  unsigned int UdpCommandPort = 88;       // local UDP port listen to command
  #define UDP_TX_PACKET_MAX_SIZE 40       // MIN 30
  unsigned char packetBuffer[UDP_TX_PACKET_MAX_SIZE]; //buffer to hold incoming packet,
  // unsigned char packetBuffer[12]; //buffer to hold incoming packet,
  int UDPpacketSize;
  EthernetUDP UdpCommand; // An EthernetUDP instance to let us send and receive packets over UDP
  IPAddress BroadcastIP(0, 0, 0, 0);        // Broadcast IP address
  int BroadcastPort       = 88;             // destination broadcast packet port
  IPAddress RemoteSwIP(0, 0, 0, 0);         // remote UDP IP switch - set from UDP DetectRemote array
  int RemoteSwPort         = 0;             // remote UDP IP switch port
  int BandDecoderChange    = 0;             // If band change, send query packet to Remote IP switch
  long RemoteSwLatency[2];                  // start, stop mark
  byte RemoteSwLatencyAnsw = 0;             // answer (offline) detect
  byte TxUdpBuffer[8];
  long IpTimeout[1][2] = {0, 60000};          // UDP Broadcast packet [8][0-timer/1-timeout]
#endif

// PINOUTS
const int VoltagePin = A0;        // measure input voltage
  const int EncBPin = A0;         // Encoder B
const int Id1Pin = A1;            // ID switch
const int BcdIn1Pin = A2;         // BCD-1 in/out
  const int ShiftInLatchPin = A2; // [ShiftIn]
const int ADPin = A3;             // [BD/ROT]
  const int SequencerPin = A3;    // [IPsw/BD]
// const int SdaPin = A4;         // [LCD]
  const int Id4Pin = A4;          //
// const int SclPin = A5;         // [LCD]
const int Id2Pin = A6;            //
const int Id3Pin = A7;            //
const int PttDetectorPin = 2;     // PTT in - [interrupt]
const int BcdIn4Pin = 3;          // BCD-4 in/out
  const int EncAShiftInPin = 3;   // Encoder+Keyboard - [interrupt]
const int BcdIn3Pin = 4;          // BCD-3 in/out
  const int ShiftInClockPin = 4;   // [ShiftIn]
const int BcdIn2Pin = 5;          // BCD-2 in/out
  const int ShiftInDataPin = 5;   // [ShiftIn]
const int PttOffPin = 6;          // PTT out OFF switch
const int ShiftOutDataPin = 7;    // DATA
const int ShiftOutLatchPin = 8;   // LATCH
const int ShiftOutClockPin = 9;   // CLOCK
// boolean rxShiftInRead;
byte rxShiftInButton[3]{0,0,0};  // three button bank: 1-8 switch, 9-16 one from, encoder...

int BAND = 0;
int previousBAND = -1;
long freq = 0;
bool PTT = false;
long PttTiming[2]={0, 10};            // debouncing time and also minimal PTT on time in ms
float DCinVoltage;
float ResistorCoeficient = 6.0;
long VoltageRefresh[2] = {0, 3000};   // refresh in ms
float ArefVoltage = 4.303;            // Measure on Aref pin 20 for calibrate
float Divider = 1;

int NumberOfBoards = 1;    // number of eight byte shift register 0-x
byte ShiftByte[5];

// int SelectOut = 0;
// int x;
long RequestTimeout[2]={0, REQUEST};
int watchdog2 = 500;     // REQUEST refresh time [ms]
int previous2;
int timeout2;

#if defined(WATCHDOG)
    int previous;
    int timeout;
    long WatchdogTimeout[2] = {-WATCHDOG*1000, WATCHDOG*1000};
#endif
#if defined(ICOM_ACC)
    int VALUE = 0;
    int prevVALUE=0;
    float VOLTAGE = 0;
    int band = 0;
    int counter = 0;
#endif
#if defined(YAESU_BCD)
    long BcdInRefresh[2] = {0, 1000};   // refresh in ms
#endif
#if defined(REMOTE_RELAY)
    int watchdog3 = 1000;     // send command to relay refresh time [ms]
    int previous3;
    int timeout3;
#endif
#if defined(KENWOOD_PC) || defined(YAESU_CAT)
    int lf = 59;  // 59 = ;
#endif
#if defined(KENWOOD_PC)
    char rdK[37];   //read data kenwood
    String rdKS;    //read data kenwood string
#endif
#if defined(YAESU_CAT)
    char rdY[37];   //read data yaesu
    String rdYS;    //read data yaesu string
#endif
#if defined(YAESU_CAT_OLD)
    byte rdYO[37];   //read data yaesu
    String rdYOS;    //read data yaesu string
#endif
#if defined(ICOM_CIV) || defined(ICOM_CIV_OUT)
    int fromAdress = 0xE0;              // 0E
    byte rdI[10];   //read data icom
    String rdIS;    //read data icom string
    long freqPrev1;
    byte incomingByte = 0;
    int state = 1;  // state machine
#endif
#if defined(KENWOOD_PC_OUT) || defined(YAESU_CAT_OUT)
    long freqPrev2;
#endif
#if defined(BCD_OUT)
    char BCDout;
#endif

#include <EEPROM.h>
/*
0 - NumberOfEncoderOutputs
1 - NET_ID
2 - Set RX data
3 - HW BCD switch ON/OFF
*/
byte ReadEepromValue[1];

//---------------------------------------------------------------------------------------------------------

void setup() {
  Serial.begin(115200);

  // 1-net ID
  if(HW_BCD_SW==true){
    bitClear(NET_ID, 0);
    bitClear(NET_ID, 1);
    bitClear(NET_ID, 2);
    bitClear(NET_ID, 3);
    NET_ID = NET_ID | GetBoardId();
    TxUdpBuffer[0] = NET_ID;
  }else{
      NET_ID = EEPROM.read(1);
      TxUdpBuffer[0] = NET_ID;
  }

  // 2-PTT transfer
  Serial.print("PTT transfer ");
  if(EEPROM.read(2)<2){
    PttTransfer = EEPROM.read(2);
  }else{
    EEPROM.write(2, PttTransfer);
  }
  if(PttTransfer==1){
    Serial.println("[ON]");
  }else{
    Serial.println("[OFF]");
  }

  // 3-HW_BCD_SW
  Serial.print("HW BCD");
  if(EEPROM.read(3)<2){
    HW_BCD_SW = EEPROM.read(3);
    Serial.println(" read from EEPROM");
  }else{
    Serial.println(" set to OFF");
  }

  pinMode(EncBPin, INPUT);
  pinMode(EncAShiftInPin, INPUT);
  pinMode(ShiftInClockPin, OUTPUT);
  pinMode(ShiftInLatchPin, OUTPUT);
  pinMode(ShiftInDataPin, INPUT);
  pinMode(ShiftOutDataPin, OUTPUT);
  pinMode(ShiftOutLatchPin, OUTPUT);
  pinMode(ShiftOutClockPin, OUTPUT);

  #if defined(LCD)
    #if defined(LCD_PCF8574T) || defined(LCD_PCF8574)
      lcd.begin(16, 2); // initialize the lcd PFC8574
      lcd.setBacklight(1);
    #else
      //------------------------------------------------------
      // Enable begin or init in dependence on the GUI version
      // lcd.begin();
      lcd.init();
      lcd.backlight();
      //------------------------------------------------------
    #endif
    lcd.createChar(0, LockChar);
    lcd.createChar(1, EthChar);
    lcd.createChar(2, DotChar);
    lcd.clear();
  #endif

    ReadEepromValue[0]=EEPROM.read(0);
      if(ReadEepromValue[0]>0x0f){
        // use default value
      }else{
        NumberOfEncoderOutputs=ReadEepromValue[0];
      }
      #if defined(LCD)
        lcd.setCursor(1, 0);
        lcd.print("Version");
        lcd.setCursor(1, 1);
        lcd.print(REV);
        // delay(350);  // 350 is max, then eth not initialize
      #endif

  #if defined(EthModule)
    pinMode(Id1Pin, INPUT_PULLUP);
    pinMode(Id2Pin, INPUT_PULLUP);
    pinMode(Id3Pin, INPUT_PULLUP);
    // must disable because use as i2c // pinMode(Id4Pin, INPUT_PULLUP);
    LastMac = 0x00 + NET_ID;
    mac[5] = LastMac;
    EthernetCheck();
  #endif
  #if defined(LCD)
    lcd.clear();
    lcd.setCursor(1, 0); for (int i = 0; i < 13; i++) {lcd.write(rq[0][i]);}
    lcd.setCursor(1, 1); for (int i = 0; i < 13; i++) {lcd.write(rq[1][i]);}
    #if !defined(FastBoot)
      delay(3000);
    #endif
    lcd.clear();
  #endif
  InterruptON(1,1); // ptt, enc
  ListCommands();
}
//---------------------------------------------------------------------------------------------------------

void loop() {
  EthernetCheck();
  LcdDisplay();
  RX_UDP(RemoteDevice, ThisDevice);
  PttOff();
  SerialCLI();
  CheckNetId(); // Live change ID/BAND

  // WebServer();
  // BandDecoderInput();
  // BandDecoderOutput();
  // watchDog();
  // FrequencyRequest();
  // ShiftOutTest();
}

// SUBROUTINES ---------------------------------------------------------------------------------------------------------

void ShiftOutTest(){

  for (int i = 0; i < 8; i++) {   // outputs 9-16
    ShiftByte[0] = 0;
    ShiftByte[0] = ShiftByte[0] | (1<<i);
    digitalWrite(ShiftOutLatchPin, LOW);    // ready for receive data
      // if(NumberOfBoards > 1){ shiftOut(ShiftOutDataPin, ShiftOutClockPin, LSBFIRST, ShiftByte[1]); }
                              shiftOut(ShiftOutDataPin, ShiftOutClockPin, LSBFIRST, ShiftByte[0]);
    digitalWrite(ShiftOutLatchPin, HIGH);    // switch to output pin
    Serial.println(ShiftByte[0], BIN);
    delay(500);
  }
}
//---------------------------------------------------------------------------------------------------------
void CheckNetId(){
  if (HW_BCD_SW==true){
    if(millis()-HW_BCD_SWTimer[0]>HW_BCD_SWTimer[1]){
      bitClear(NET_ID, 0);
      bitClear(NET_ID, 1);
      bitClear(NET_ID, 2);
      bitClear(NET_ID, 3);
      NET_ID = NET_ID | GetBoardId();
      if(NET_ID!=TxUdpBuffer[0]){
        TxUdpBuffer[0] = NET_ID;
        EEPROM.write(1, NET_ID); // address, value
        // EEPROM.commit();
        Serial.print("** Now NET-ID change to 0x");
        if(NET_ID <=0x0f){
          Serial.print(F("0"));
        }
        Serial.print(NET_ID, HEX);
        Serial.println(" **");
        #if defined(SERIAL_debug)
          if(DEBUG==1){
            Serial.print("EEPROM read [");
            Serial.print(EEPROM.read(1));
            Serial.println("]");
          }
        #endif
        TxUDP(ThisDevice, RemoteDevice, 'b', 'r', 'o');
      }
      HW_BCD_SWTimer[0]=millis();
    }
  }
}
//---------------------------------------------------------------------------------------------------------

byte IdPrefix(byte ID){
  bitClear(ID, 0);  // ->
  bitClear(ID, 1);
  bitClear(ID, 2);
  bitClear(ID, 3);
  ID = ID >> 4;
  return(ID);
}

//---------------------------------------------------------------------------------------------------------

byte IdSufix(byte ID){
  bitClear(ID, 4);
  bitClear(ID, 5);
  bitClear(ID, 6);
  bitClear(ID, 7);
  return(ID);
}
//---------------------------------------------------------------------------------------------------------

void SerialCLI(){
  if (Serial.available() > 0) {
          incomingByte = Serial.read();

      // .
      if(incomingByte==46){
          Serial.print(F("RX ["));
          Serial.print(packetBuffer[0], HEX);
          for(int i=1; i<8; i++){
            Serial.print(char(packetBuffer[i]));
          }
          Serial.print(F("] "));
          Serial.print(UdpCommand.remoteIP());
          Serial.print(":");
          Serial.println(UdpCommand.remotePort());
          Serial.println(F("List detected switch by ID sufix"));
          for (int i = 0; i < 16; i++) {
            Serial.print(i, HEX);
            Serial.print(F("  "));
            Serial.print(DetectedRemoteSw [i] [0]);
            Serial.print(F("."));
            Serial.print(DetectedRemoteSw [i] [1]);
            Serial.print(F("."));
            Serial.print(DetectedRemoteSw [i] [2]);
            Serial.print(F("."));
            Serial.print(DetectedRemoteSw [i] [3]);
            Serial.print(F(":"));
            Serial.println(DetectedRemoteSw [i] [4]);
          }

        // *
        #if defined(SERIAL_debug)
          }else if(incomingByte==42){
            DEBUG = !DEBUG;
            Serial.print(F("** Serial DEBUG "));
            if(DEBUG==1){
              Serial.println(F("ENABLE **"));
            }else{
              Serial.println(F("DISABLE **"));
            }
            #endif

        // /
        }else if(incomingByte==47){
            PttTransfer = !PttTransfer;
            EEPROM.write(2, PttTransfer);
            // reinit ptt interrupt
            InterruptON(0,1); // ptt, enc
            InterruptON(1,1); // ptt, enc
            Serial.print(F("** PTT trasfer "));
            if(PttTransfer==1){
              Serial.println(F("ENABLE **"));
            }else{
              Serial.println(F("DISABLE **"));
            }

        // ?
        }else if(incomingByte==63){
          ListCommands();

          // #
          }else if(incomingByte==35){
                Serial.println("Press NET-ID X_ prefix 0-f...");
              Serial.print("> ");
              while (Serial.available() == 0) {
                // Wait
              }
              incomingByte = Serial.read();

              if( (incomingByte>=48 && incomingByte<=57) || (incomingByte>=97 && incomingByte<=102) ){
                // prefix
                  bitClear(NET_ID, 4);
                  bitClear(NET_ID, 5);
                  bitClear(NET_ID, 6);
                  bitClear(NET_ID, 7);
                  Serial.write(incomingByte);
                  Serial.println();
                  if(incomingByte>=48 && incomingByte<=57){
                    incomingByte = incomingByte-48;
                    incomingByte = (byte)incomingByte << 4;
                    NET_ID = NET_ID | incomingByte;
                    TxUdpBuffer[0] = NET_ID;
                  }else if(incomingByte>=97 && incomingByte<=102){
                    incomingByte = incomingByte-87;
                    incomingByte = (byte)incomingByte << 4;
                    NET_ID = NET_ID | incomingByte;
                    TxUdpBuffer[0] = NET_ID;
                  }
                // sufix
                if(HW_BCD_SW==false){
                    Serial.println("Press NET-ID _X sufix 0-f...");
                    Serial.print("> ");
                    while (Serial.available() == 0) {
                      // Wait
                    }
                    incomingByte = Serial.read();

                  if( (incomingByte>=48 && incomingByte<=57) || (incomingByte>=97 && incomingByte<=102) ){
                    bitClear(NET_ID, 0);
                    bitClear(NET_ID, 1);
                    bitClear(NET_ID, 2);
                    bitClear(NET_ID, 3);
                    Serial.write(incomingByte);
                    Serial.println();
                    if(incomingByte>=48 && incomingByte<=57){
                      incomingByte = incomingByte-48;
                      NET_ID = NET_ID | incomingByte;
                      TxUdpBuffer[0] = NET_ID;
                    }else if(incomingByte>=97 && incomingByte<=102){
                      incomingByte = incomingByte-87;
                      NET_ID = NET_ID | incomingByte;
                      TxUdpBuffer[0] = NET_ID;
                    }
                // #endif
                    EEPROM.write(1, NET_ID); // address, value
                    // EEPROM.commit();
                    Serial.print("** Now NET-ID change to 0x");
                    if(NET_ID <=0x0f){
                      Serial.print(F("0"));
                    }
                    Serial.print(NET_ID, HEX);
                    Serial.println(" **");
                    #if defined(SERIAL_debug)
                      if(DEBUG==1){
                        Serial.print("EEPROM read [");
                        Serial.print(EEPROM.read(1), HEX);
                        Serial.println("]");
                      }
                    #endif
                    TxUDP(ThisDevice, RemoteDevice, 'b', 'r', 'o');
                    // if(TxUdpBuffer[2] == 'm'){
                    //   TxUDP('s', packetBuffer[2], ShiftOutByte[0], ShiftOutByte[1], ShiftOutByte[2], 0);
                    // }
                // #if !defined(HW_BCD_SW)
                  }else{
                    Serial.println(" accepts 0-f, exit");
                  }
                // #endif
                }
              }else{
                Serial.println(" accepts 0-f, exit");
              }


          // +
          }else if(incomingByte==43){
              HW_BCD_SW=!HW_BCD_SW;
              Serial.print("** Net ID sufix by ");
              EEPROM.write(3, HW_BCD_SW);
              // EEPROM.commit();
              if(HW_BCD_SW==true){
                Serial.println("EEPROM/[BCD switch] **");
                bitClear(NET_ID, 0);
                bitClear(NET_ID, 1);
                bitClear(NET_ID, 2);
                bitClear(NET_ID, 3);
                NET_ID = NET_ID | GetBoardId();
                TxUdpBuffer[0] = NET_ID;
              }else{
                NET_ID = EEPROM.read(1);
                TxUdpBuffer[0] = NET_ID;
                Serial.println("[EEPROM]/BCD switch **");
              }

          }else{
            Serial.print(F(" ["));
            Serial.write(incomingByte); //, DEC);
            Serial.println(F("] unknown command - for more info press 'h'"));
            ListCommands();
          }
  }
}
//---------------------------------------------------------------------------------------------------------
void ListCommands(){
  Serial.println();
  Serial.println(F("  =========================="));
  Serial.print(F("   IP SW MASTER NET ID: 0x"));
  if(NET_ID <=0x0f){
    Serial.print(F("0"));
  }
  Serial.print(NET_ID, HEX);
  if(HW_BCD_SW==true){
    Serial.print(" [BCD-");
    Serial.print(digitalRead(Id1Pin));
    Serial.print(digitalRead(Id2Pin));
    Serial.print(digitalRead(Id3Pin));
    Serial.print("]");
  }
    Serial.println();
  Serial.println(F("  =========================="));
  Serial.print(F("  Version: "));
  Serial.println(REV);
  Serial.print(F("  IP address:"));
  Serial.println(Ethernet.localIP());
  Serial.print(F("  IP switch: "));
  Serial.print(RemoteSwIP);
  Serial.print(F(":"));
  Serial.println(RemoteSwPort);
  Serial.print(F("  Encoder range: "));
  Serial.println(NumberOfEncoderOutputs);
  Serial.print(F("  Set RX data (multi control) "));
  if(SET_RX_DATA==1){
    Serial.println(F("[ON]"));
  }else{
    Serial.println(F("[OFF]"));
  }

  Serial.println(F("  -----------------------------"));
  Serial.println(F("      ? for info"));
  #if defined(SERIAL_debug)
    Serial.print(F("      * serial debug ["));
    if(DEBUG==1){
      Serial.println(F("ON]"));
    }else{
      Serial.println(F("OFF]"));
    }
  #endif
  Serial.print(F("      / PTT transfer"));
  if(PttTransfer==1){
    Serial.print(F(" as button number "));
    Serial.print(ButtonUseForPttTransfer);
    Serial.println(F(" [ON]"));
  }else{
    Serial.println(F(" [OFF]"));
  }
  Serial.println(F("      . Listing detected/stored ip-relay by ID sufix"));
  Serial.print("      + Net ID sufix by ");
    if(HW_BCD_SW==true){
      Serial.println("EEPROM/[BCD switch]");
    }else{
      Serial.println("[EEPROM]/BCD switch");
    }
    Serial.print("      # network ID prefix [");
    Serial.print(IdPrefix(NET_ID), HEX);
    if(SET_RX_DATA==1){
      Serial.println("] hex, diffrent for each device in multi control");
    }else{
      Serial.println("] hex, expanded range of network ID");
    }

    if(HW_BCD_SW==false){
      Serial.print("        +network ID sufix [");
      Serial.print(IdSufix(NET_ID), HEX);
      if(SET_RX_DATA==1){
        Serial.println("] hex, in multi control same, for all shared devices");
      }else{
        Serial.println("] hex, diffrent for each device");
      }
    }
    Serial.println(F("  -----------------------------"));
}

//---------------------------------------------------------------------------------------------------------

void InterruptON(int ptt, int enc){
  if(ptt==0){
    detachInterrupt(digitalPinToInterrupt(PttDetectorPin));
  }else if(ptt==1){
    if(PttTransfer==0){
      attachInterrupt(digitalPinToInterrupt(PttDetectorPin), PttDetector, FALLING);
    }else{
      attachInterrupt(digitalPinToInterrupt(PttDetectorPin), PttDetectorTransfer, CHANGE);
    }
  }
  if(enc==0){
    detachInterrupt(digitalPinToInterrupt(EncAShiftInPin));
  }else if(enc==1){
    attachInterrupt(digitalPinToInterrupt(EncAShiftInPin), EncoderInterrupt, FALLING);  // need detachInterrupt in RX_UDP() subroutine
  }
}
//---------------------------------------------------------------------------------------------------------

void EncoderInterrupt(){
  InterruptON(1,0); // ptt, enc
  boolean UseButt = 0;
  if(PTT==false){
    rxShiftInButton[1]=0;
    rxShiftInButton[2]=0;

    // Encoder and buttons
    if(digitalRead(EncBPin)==0){
      if(EncoderCount<NumberOfEncoderOutputs-1){
        EncoderCount++;
      }else{
        EncoderCount=0;
      }
      EncEndStatus=1;
      EncEndTimer[0]=millis();
    }else{
      if(AccKeyboardShift()==true){
        UseButt=1;
        // Serial.print(rxShiftInButton[0], BIN);
        // Serial.print(" rxSHIFT ");
        // Serial.println(millis());
      }else{
        if(EncoderCount>0){
          EncoderCount--;
        }else{
          EncoderCount=NumberOfEncoderOutputs-1;
        }
        EncEndStatus=1;
        EncEndTimer[0]=millis();
      }
    }

    // Over/under count
    if(EncoderCount<8){
      rxShiftInButton[1]=rxShiftInButton[1] | (1<<EncoderCount);
    }else{
      rxShiftInButton[2]=rxShiftInButton[2] | (1<<EncoderCount-8);
    }

    if( (millis()-ButtDebounceTimer[0]>ButtDebounceTimer[1] && UseButt==1) || UseButt==0 ){
      TxUDP(ThisDevice, RemoteDevice, rxShiftInButton[0], rxShiftInButton[1], rxShiftInButton[2]);
      if(UseButt==1){
        ButtDebounceTimer[0]=millis();
      }
    }
    #if defined(LCD)
      LcdNeedRefresh = true;
    #endif
  }
  GetNetIdTimer[0]=millis();
  InterruptON(1,1); // ptt, enc
}
//-------------------------------------------------------------------------------------------------------

bool AccKeyboardShift(){    // run from interrupt
    digitalWrite(ShiftInLatchPin,1);   //Set latch pin to 1 to get recent data into the CD4021
    delayMicroseconds(15);
    digitalWrite(ShiftInLatchPin,0);     //Set latch pin to 0 to get data from the CD4021
    bool KeyboardDetect = false;
    for (int i=0; i<8; i++){                // 16 = two bank
      digitalWrite(ShiftInClockPin, 0);
      bool rxShiftInRead = digitalRead(ShiftInDataPin);

        if(rxShiftInRead==0){
          if(millis()-ButtDebounceTimer[0]>ButtDebounceTimer[1]){
            rxShiftInButton[0] = rxShiftInButton[0] ^ (1<<ButtonMapping[i]);  // invert n-th bit
          }
          KeyboardDetect = true;
        }
      digitalWrite(ShiftInClockPin, 1);
    }
    return KeyboardDetect;
}
//---------------------------------------------------------------------------------------------------------

void NetId(){
  if(millis()-GetNetIdTimer[0]>GetNetIdTimer[1]){
    if(NET_ID != GetBoardId()){
      NET_ID = GetBoardId();
      TxUDP(ThisDevice, RemoteDevice, 'b', 'r', 'o');
    }
  }
}
//---------------------------------------------------------------------------------------------------------

#if defined(EthModule)
  byte GetBoardId(){
    byte GetBcd = 0;
    if(analogRead(Id1Pin)<50){  // 17 1023
      GetBcd = GetBcd | (1<<0);    // Set the n-th bit
    }
    if(analogRead(Id2Pin)<50){ // 0 170-970
      GetBcd = GetBcd | (1<<1);    // Set the n-th bit
    }
    if(analogRead(Id3Pin)<50){  // 0 290-830
      GetBcd = GetBcd | (1<<2);    // Set the n-th bit
    }
    return(GetBcd);
    // if(digitalRead(Id4Pin)==0){
    //   NET_ID = NET_ID | (1<<3);    // Set the n-th bit
    // }
  }
#endif
//---------------------------------------------------------------------------------------------------------

void LcdDisplay(){
  #if defined(LCD)
    if(millis()-LcdRefresh[0]>LcdRefresh[1] || LcdNeedRefresh == true){

      if(RemoteSwLatencyAnsw==1 || (RemoteSwLatencyAnsw==0 && millis() < RemoteSwLatency[0]+2000)){ // if answer ok, or latency measure nod end
        if(millis()-EncEndTimer[0]>EncEndTimer[1] && EncEndStatus==1 && digitalRead(EncBPin)==1 && digitalRead(EncAShiftInPin)==1){
          EncEndStatus = 0;
        }
        if(millis()-EncEndTimer[0]>EncEndTimer[1] && EncEndStatus==1 && (digitalRead(EncBPin)==0 || digitalRead(EncAShiftInPin)==0)){

          #if defined(LCD_PCF8574T) || defined(LCD_PCF8574)
            lcd.setBacklight(0);
          #else
            lcd.noBacklight();
          #endif
          lcd.setCursor(0,0);
          lcd.print("Intermediate    ");
          lcd.setCursor(0,1);
          lcd.print("position!");
          delay(200);
          #if defined(LCD_PCF8574T) || defined(LCD_PCF8574)
            lcd.setBacklight(1);
          #else
            lcd.backlight();
          #endif
        }else{
          int LcdSpace = (16-NumberOfEncoderOutputs)/2;
          int PositionCounter = 0;
          lcd.setCursor(0,0);
          for (int i=0; i<LcdSpace; i++){
            lcd.print(" ");
            PositionCounter++;
          }
          // Encoder
          if( (EncoderCount+1) > (NumberOfEncoderOutputs-1) ){
            EncoderCount = NumberOfEncoderOutputs-1;
          }
          for (int i=0; i<NumberOfEncoderOutputs; i++){
            lcd.setCursor(i+LcdSpace,0);
            if(i==EncoderCount){
              if(EncoderCount>8){
                lcd.setCursor(i-1+LcdSpace,0);
                lcd.print(EncoderCount+1, DEC);
              }else{
                lcd.print(EncoderCount+1, DEC);
              }
            }else{
              // lcd.print(".");
              lcd.print((char)2);
            }
            PositionCounter++;
          }
          for (int i=PositionCounter; i<16; i++){
            lcd.print(" ");
          }
          // Keyboard
          for (int i=0; i<8; i++){
            lcd.setCursor(i,1);
            if (rxShiftInButton[0] & (1<<i)) {
              lcd.print(i+1);
            }else{
              lcd.print(' ');
            }
          }

          lcd.setCursor(8,1);
          // PTT
          if(PTT==true){
            // lcd.write(byte(0));        // Lock icon
            lcd.print((char)0);
          }else{
            lcd.print("|");
          }
        }

        if(EthLinkStatus==1){
          if(RemoteSwLatencyAnsw==1){ // if answer ok, or latency measure nod end
            // lcd.print("*");
            lcd.print((char)1);   // EthChar
          }else if(RemoteSwLatencyAnsw==0 && millis() < RemoteSwLatency[0]+2000){
              lcd.print(" ");
          }else{
            lcd.print("!");
          }
        }else{
          lcd.print("x");
        }
        if(SET_RX_DATA==1){
          lcd.print("m");
        }else{
          lcd.print(" ");
        }
        lcd.print("ID-");
        if(NET_ID < 0x0f){
          lcd.print(F("0"));
        }
        lcd.print(NET_ID, HEX);
      }else if(EthLinkStatus==0){
        lcd.setCursor(0, 0);
        lcd.print((char)1);   // EthChar
        lcd.print(F(" Please connect"));
        lcd.setCursor(0, 1);
        lcd.print(F("  ethernet      "));
      }else{
        lcd.setCursor(0, 0);
        lcd.print(F(" IPswitch-ID: "));
        if(NET_ID < 0x0f){
          lcd.print(F("0"));
        }
        lcd.print(NET_ID, HEX);
        lcd.setCursor(0, 1);
        lcd.print(F("  not detected  "));
        digitalWrite(ShiftOutLatchPin, LOW);  // ready for receive data
        shiftOut(ShiftOutDataPin, ShiftOutClockPin, LSBFIRST, 0x00);    // buttons
        shiftOut(ShiftOutDataPin, ShiftOutClockPin, LSBFIRST, 0x00);    // buttons
        digitalWrite(ShiftOutLatchPin, HIGH);    // switch to output pin
      }

      LcdRefresh[0]=millis();
      LcdNeedRefresh = false;
    }
  #endif
}
//-------------------------------------------------------------------------------------------------------
/*
ID FROM TO : BRO ;
ID FROM TO : A B C ;

TX  0ms:b;
TX  0ms:123;
*/

void TxUDP(byte FROM, byte TO, byte A, byte B, byte C){
  #if defined(EthModule)
    InterruptON(0,0); // ptt, enc

    // TxUdpBuffer[0] = NET_ID;
    TxUdpBuffer[1] = FROM;
    TxUdpBuffer[2] = TO;
    TxUdpBuffer[3] = B00000000;           // :
    // TxUdpBuffer[3] = B00111010;           // :
    TxUdpBuffer[4] = A;
    TxUdpBuffer[5] = B;
    TxUdpBuffer[6] = C;
    TxUdpBuffer[7] = B00111011;           // ;

    // BROADCAST
    if(A=='b' && B=='r' && C=='o'){  // b r o
      Need_SET_RX_DATA=1;
    // if(A==B01100010 && B==B01110010 && C==B01101111){  // b r o
      // direct
      for (int i=0; i<15; i++){
        if( DetectedRemoteSw[i][4]!=0 && IdSufix(NET_ID)==i ){
          RemoteSwIP = DetectedRemoteSw[i];
          RemoteSwPort = DetectedRemoteSw[i][4];
          UdpCommand.beginPacket(RemoteSwIP, RemoteSwPort);
            UdpCommand.write(TxUdpBuffer, sizeof(TxUdpBuffer));   // send buffer
          UdpCommand.endPacket();
          RemoteSwLatency[0] = millis(); // set START time mark UDP command latency
          RemoteSwLatencyAnsw = 0;   // send command, wait to answer

          #if defined(SERIAL_debug)
            if(DEBUG==1){
              Serial.print(F("TX direct ID-x"));
              Serial.print(i);
              Serial.print(F(" "));
              Serial.print(RemoteSwIP);
              Serial.print(F(":"));
              Serial.print(RemoteSwPort);
              Serial.print(F(" ["));
              Serial.print(TxUdpBuffer[0], HEX);
              for (int i=1; i<8; i++){
                Serial.print(char(TxUdpBuffer[i]));
                // Serial.print(F(" "));
              }
              Serial.println("]");
            }
          #endif
        }
      }

      // broadcast
      BroadcastIP = ~Ethernet.subnetMask() | Ethernet.gatewayIP();
      UdpCommand.beginPacket(BroadcastIP, BroadcastPort);   // Send to IP and port from recived UDP command
        UdpCommand.write(TxUdpBuffer, sizeof(TxUdpBuffer));   // send buffer
      UdpCommand.endPacket();
      IpTimeout[0][0] = millis();                      // set time mark
      RemoteSwLatency[0] = millis(); // set START time mark UDP command latency
      RemoteSwLatencyAnsw = 0;   // send command, wait to answer

        #if defined(SERIAL_debug)
          if(DEBUG==1){
            Serial.print(F("TX broadcast "));
            Serial.print(BroadcastIP);
            Serial.print(F(":"));
            Serial.print(BroadcastPort);
            Serial.print(F(" ["));
            Serial.print(TxUdpBuffer[0], HEX);
            for (int i=1; i<8; i++){
              Serial.print(char(TxUdpBuffer[i]));
              // Serial.print(F(" "));
            }
            Serial.println("]");
          }
        #endif

    // DATA
    }else{
      if(DetectedRemoteSw[IdSufix(NET_ID)][4]!=0 && (PTT==false || PTT==true && PttTransfer==1) ){
        RemoteSwIP = DetectedRemoteSw[IdSufix(NET_ID)];
        RemoteSwPort = DetectedRemoteSw[IdSufix(NET_ID)][4];
        UdpCommand.beginPacket(RemoteSwIP, RemoteSwPort);
          UdpCommand.write(TxUdpBuffer, sizeof(TxUdpBuffer));   // send buffer
        UdpCommand.endPacket();
        RemoteSwLatency[0] = millis(); // set START time mark UDP command latency
        if(A!='c' && B!='f' && C!='m'){
          RemoteSwLatencyAnsw = 0;   // send command, wait to answer
        }
        #if defined(SERIAL_debug)
          if(DEBUG==1){
            Serial.println();
            Serial.print(F("TX ["));
            Serial.print(TxUdpBuffer[0], HEX);
            for (int i=1; i<4; i++){
              Serial.print(char(TxUdpBuffer[i]));
            }
            Serial.print(TxUdpBuffer[4], BIN);
            Serial.print(F("|"));
            Serial.print(TxUdpBuffer[5], BIN);
            Serial.print(F("|"));
            Serial.print(TxUdpBuffer[6], BIN);
            Serial.print(char(TxUdpBuffer[7]));
            Serial.print(F("] "));
            Serial.print(RemoteSwIP);
            Serial.print(F(":"));
            Serial.println(RemoteSwPort);
          }
        #endif
      }
    }
  InterruptON(1,1); // ptt, enc
  #endif
}
//-------------------------------------------------------------------------------------------------------
/*
ID FROM TO : CFM ;
ID FROM TO : A B C ;

RX  0sm:c;
RX  0sm:123;
*/
byte AsciiToHex(int ASCII){
  if(ASCII>=48 && ASCII<=57){
    return(ASCII-48);
  }else if(ASCII>=97 && ASCII<=102){
    return(ASCII-87);
  }else{
    return(255);
  }
}
//-------------------------------------------------------------------------------------------------------

void RX_UDP(char FROM, char TO){
  #if defined(EthModule)
    InterruptON(0,0); // ptt, enc
    UDPpacketSize = UdpCommand.parsePacket();    // if there's data available, read a packet
    if (UDPpacketSize){
      UdpCommand.read(packetBuffer, UDP_TX_PACKET_MAX_SIZE);      // read the packet into packetBufffer
      // UdpCommand.read(packetBuffer, 12);      // read the packet into packetBufffer
      // Print RAW
      #if defined(SERIAL_debug)
        if(DEBUG==1){
          Serial.print(F("RXraw "));
          Serial.print(packetBuffer[0], HEX);
          for(int i=1; i<8; i++){
            Serial.print(char(packetBuffer[i]));
          }
          Serial.print(F(" "));
          Serial.print(UdpCommand.remoteIP());
          Serial.print(":");
          Serial.print(UdpCommand.remotePort());
          Serial.println();
        }
      #endif
      // ID-FROM-TO filter
      if(String(packetBuffer[0], DEC).toInt()==NET_ID
        // && packetBuffer[1]== 's'  // FROM Switch
        // && packetBuffer[2]== 'm'  // TO
        && packetBuffer[1]== FROM
        && packetBuffer[2]== TO
        && packetBuffer[3]<4  // B000000??
        // && (packetBuffer[3]== ':' || packetBuffer[3]== '-')
        && packetBuffer[7]== ';'
      ){
        RemoteSwLatency[1] = (millis()-RemoteSwLatency[0])/2; // set latency (half path in ms us/2/1000)
        RemoteSwLatencyAnsw = 1;           // answer packet received

        SET_RX_DATA=bitRead(packetBuffer[3], 0);    // set multi control
        // if(packetBuffer[3]== ':'){
        //   SET_RX_DATA=0;    // set multi control
        // }else if(packetBuffer[3]== '-'){
        //   SET_RX_DATA=1;    // set multi control
        // }

        GroupButton=bitRead(packetBuffer[3], 1);    // set group button

        // RX Broadcast / CFM
        if((packetBuffer[4]== 'b' && packetBuffer[5]== 'r') // && packetBuffer[6]== 'o')
          || (packetBuffer[4]== 'c' && packetBuffer[5]== 'f') // && packetBuffer[6]== 'm')
          ){
          NumberOfEncoderOutputs = String(packetBuffer[6], DEC).toInt()+1;
          IPAddress TmpAddr = UdpCommand.remoteIP();
          DetectedRemoteSw [hexToDecBy4bit(IdSufix(NET_ID))] [0]=TmpAddr[0];     // Switch IP addres storage to array
          DetectedRemoteSw [hexToDecBy4bit(IdSufix(NET_ID))] [1]=TmpAddr[1];
          DetectedRemoteSw [hexToDecBy4bit(IdSufix(NET_ID))] [2]=TmpAddr[2];
          DetectedRemoteSw [hexToDecBy4bit(IdSufix(NET_ID))] [3]=TmpAddr[3];
          DetectedRemoteSw [hexToDecBy4bit(IdSufix(NET_ID))] [4]=UdpCommand.remotePort();
          // DetectedRemoteSw [hexToDecBy4bit(packetBuffer[0])] [0]=TmpAddr[0];     // Switch IP addres storage to array
          // DetectedRemoteSw [hexToDecBy4bit(packetBuffer[0])] [1]=TmpAddr[1];
          // DetectedRemoteSw [hexToDecBy4bit(packetBuffer[0])] [2]=TmpAddr[2];
          // DetectedRemoteSw [hexToDecBy4bit(packetBuffer[0])] [3]=TmpAddr[3];
          // DetectedRemoteSw [hexToDecBy4bit(packetBuffer[0])] [4]=UdpCommand.remotePort();

          lcd.clear();
          lcd.setCursor(0, 0);
          lcd.print(F("Detect SW #"));
          lcd.print(IdPrefix(NET_ID), HEX);
          lcd.print(IdSufix(NET_ID), HEX);
          // lcd.print(packetBuffer[0], HEX);
          lcd.setCursor(0, 1);
          lcd.print(DetectedRemoteSw [hexToDecBy4bit(IdSufix(NET_ID))] [0]);
          // lcd.print(DetectedRemoteSw [hexToDecBy4bit(packetBuffer[0])] [0]);
          lcd.print(F("."));
          lcd.print(DetectedRemoteSw [hexToDecBy4bit(IdSufix(NET_ID))] [1]);
          // lcd.print(DetectedRemoteSw [hexToDecBy4bit(packetBuffer[0])] [1]);
          lcd.print(F("."));
          lcd.print(DetectedRemoteSw [hexToDecBy4bit(IdSufix(NET_ID))] [2]);
          // lcd.print(DetectedRemoteSw [hexToDecBy4bit(packetBuffer[0])] [2]);
          lcd.print(F("."));
          lcd.print(DetectedRemoteSw [hexToDecBy4bit(IdSufix(NET_ID))] [3]);
          // lcd.print(DetectedRemoteSw [hexToDecBy4bit(packetBuffer[0])] [3]);
          lcd.print(F(":"));
          lcd.print(DetectedRemoteSw [hexToDecBy4bit(IdSufix(NET_ID))] [4]);
          // lcd.print(DetectedRemoteSw [hexToDecBy4bit(packetBuffer[0])] [4]);
          #if !defined(FastBoot)
            delay(4000);
          #endif
          lcd.clear();
          lcd.setCursor(0, 0);
          lcd.print(F(" Encoder range "));
          lcd.setCursor(7, 1);
          lcd.print(NumberOfEncoderOutputs);
          #if !defined(FastBoot)
            delay(1500);
          #endif
          LcdNeedRefresh = true;
          lcd.clear();

          #if defined(SERIAL_debug)
          if(DEBUG==1){
            Serial.print(F("RX ["));
            Serial.print(packetBuffer[0], HEX);
            for(int i=1; i<8; i++){
              Serial.print(char(packetBuffer[i]));
            }
            Serial.print(F("] "));
            Serial.print(UdpCommand.remoteIP());
            Serial.print(":");
            Serial.println(UdpCommand.remotePort());
            for (int i = 0; i < 16; i++) {
              Serial.print(i, HEX);
              Serial.print(F("  "));
              Serial.print(DetectedRemoteSw [i] [0]);
              Serial.print(F("."));
              Serial.print(DetectedRemoteSw [i] [1]);
              Serial.print(F("."));
              Serial.print(DetectedRemoteSw [i] [2]);
              Serial.print(F("."));
              Serial.print(DetectedRemoteSw [i] [3]);
              Serial.print(F(":"));
              Serial.println(DetectedRemoteSw [i] [4]);
            }
          }
          #endif
          TxUDP(ThisDevice, RemoteDevice, 'c', 'f', 'm');

        // RX DATA
        }else{
          byte ButtonSequence = 0;
          // 4bit shift left OR 4bit shift right = 4bit shift rotate
          ButtonSequence = (byte)packetBuffer[4] >> 4 | (byte)packetBuffer[4] << 4;
          digitalWrite(ShiftOutLatchPin, LOW);  // ready for receive data
          shiftOut(ShiftOutDataPin, ShiftOutClockPin, LSBFIRST, ButtonSequence);    // buttons
          shiftOut(ShiftOutDataPin, ShiftOutClockPin, LSBFIRST, ButtonSequence);    // buttons
          digitalWrite(ShiftOutLatchPin, HIGH);    // switch to output pin

          #if defined(SERIAL_debug)
            if(DEBUG==1){
              Serial.print(F("RX ["));
              Serial.print(packetBuffer[0], HEX);
              for(int i=1; i<4; i++){
                Serial.print(char(packetBuffer[i]));
              }
              Serial.print((byte)packetBuffer[4], BIN);
              Serial.print(F("|"));
              Serial.print((byte)packetBuffer[5], BIN);
              Serial.print(F("|"));
              Serial.print((byte)packetBuffer[6], BIN);
              Serial.print(F(";] "));
              Serial.print(UdpCommand.remoteIP());
              Serial.print(":");
              Serial.print(UdpCommand.remotePort());
              Serial.print(F(" Latency: "));
              Serial.println(RemoteSwLatency[1]);
            }
          #endif

          // Check if RX different data
          int EncoderCountTmp =0;
          for(int i=0; i<16; i++){
            if(i<8 && bitRead(packetBuffer[5], i) ){
              EncoderCountTmp=i;
              break;
            }
            if(i>7 && bitRead(packetBuffer[6], i-8) ){
              EncoderCountTmp=i;
              break;
            }
          }
          if( (SET_RX_DATA==1 || Need_SET_RX_DATA==1) && ( rxShiftInButton[0]!=packetBuffer[4] || EncoderCountTmp!=EncoderCount ) ){
            rxShiftInButton[0]=packetBuffer[4];
            EncoderCount=EncoderCountTmp;
            #if defined(SERIAL_debug)
              if(DEBUG==1){
                Serial.println(F("RX different data > SET it "));
              }
            #endif
            if(Need_SET_RX_DATA==1){
              Need_SET_RX_DATA=0;
            }
          }
          // group button bit
          if(GroupButton==true && rxShiftInButton[0]!=packetBuffer[4]){
            rxShiftInButton[0]=packetBuffer[4];
          }
          LcdNeedRefresh = true;
        } // else (rx data) end
      } // filtered end
      else{
        #if defined(SERIAL_debug)
          if(DEBUG==1){
            Serial.println(F("   Different NET-ID, or bad packet format"));
          }
        #endif
      }
      // memset(packetBuffer, 0, 12);   // Clear contents of Buffer
      memset(packetBuffer, 0, sizeof(packetBuffer));   // Clear contents of Buffer
    } //end IfUdpPacketSice
    InterruptON(1,1); // ptt, enc
  #endif
}
//---------------------------------------------------------------------------------------------------------

void PttDetector(){   // call from interupt
  // digitalWrite(PttOffPin, HIGH);
  PTT = true;
  #if defined(LCD)
    LcdNeedRefresh = true;
  #endif
  PttTiming[0]=millis();
}
//---------------------------------------------------------------------------------------------------------

void PttDetectorTransfer(){   // call from interupt
  if(digitalRead(PttDetectorPin)==LOW){
    PTT = true;
    bitSet(rxShiftInButton[0], ButtonUseForPttTransfer-1);
    TxUDP(ThisDevice, RemoteDevice, rxShiftInButton[0], rxShiftInButton[1], rxShiftInButton[2]);
  }else{
    PTT = false;
    bitClear(rxShiftInButton[0], ButtonUseForPttTransfer-1);
    TxUDP(ThisDevice, RemoteDevice, rxShiftInButton[0], rxShiftInButton[1], rxShiftInButton[2]);
  }
  #if defined(LCD)
    LcdNeedRefresh = true;
  #endif
}
//-------------------------------------------------------------------------------------------------------

void EthernetCheck(){
  if(millis()-EthLinkStatusTimer[0]>EthLinkStatusTimer[1] && EnableEthernet==1){
    if ((Ethernet.linkStatus() == Unknown || Ethernet.linkStatus() == LinkOFF) && EthLinkStatus==1) {
      EthLinkStatus=0;
      #if defined(SERIAL_debug)
      // if(DEBUG==1){
        Serial.println(F("Ethernet DISCONNECTED"));
      // }
      #endif
    }else if (Ethernet.linkStatus() == LinkON && EthLinkStatus==0) {
      EthLinkStatus=1;
      #if defined(SERIAL_debug)
      // if(DEBUG==1){
        Serial.println(F("Ethernet CONNECTED"));
      // }
      #endif

      lcd.clear();
      lcd.setCursor(1, 0);
      lcd.print(F("Net-ID: "));
      lcd.print(IdPrefix(NET_ID), HEX);
      lcd.print(IdSufix(NET_ID), HEX);
      lcd.setCursor(1, 1);
      lcd.print(F("[DHCP-"));
      if(EnableDHCP==1){
          lcd.print(F("ON]..."));
          Ethernet.begin(mac);
          IPAddress CheckIP = Ethernet.localIP();
          if( CheckIP[0]==0 && CheckIP[1]==0 && CheckIP[2]==0 && CheckIP[3]==0 ){
            lcd.clear();
            lcd.setCursor(1, 0);
            lcd.print(F("DHCP FAIL"));
            lcd.setCursor(1, 1);
            lcd.print(F("please restart"));
            while(1) {
              // infinite loop
            }
          }
      }else{
        lcd.print(F("OFF]"));
        Ethernet.begin(mac, ip, myDns, gateway, subnet);
      }

      #if !defined(FastBoot)
        delay(2000);
      #endif
        lcd.clear();
        lcd.setCursor(1, 0);
        lcd.print(F("IP address:"));
        lcd.setCursor(1, 1);
        lcd.print(Ethernet.localIP());
      #if !defined(FastBoot)
        delay(2500);
      #endif
        lcd.clear();

      server.begin();                     // Web
      UdpCommand.begin(UdpCommandPort);   // UDP
      TxUDP(ThisDevice, RemoteDevice, 'b', 'r', 'o');
    }
    EthLinkStatusTimer[0]=millis();
  }
}
//---------------------------------------------------------------------------------------------------------

void TxBroadcastUdp(String MSG){
  #if defined(EthModule)
    InterruptON(0,0); // ptt, enc
    BroadcastIP = ~Ethernet.subnetMask() | Ethernet.gatewayIP();

    UdpCommand.beginPacket(BroadcastIP, BroadcastPort);   // Send to IP and port from recived UDP command
      UdpCommand.print(MSG);
    UdpCommand.endPacket();

    InterruptON(1,1); // ptt, enc
  #endif
}
//-------------------------------------------------------------------------------------------------------

unsigned char hexToDecBy4bit(unsigned char hex)
// convert a character representation of a hexidecimal digit into the actual hexidecimal value
{
  if(hex > 0x39) hex -= 7; // adjust for hex letters upper or lower case
  return(hex & 0xf);
}

//---------------------------------------------------------------------------------------------------------

void PttOff(){
  if(PTT==true && millis()-PttTiming[0] > PttTiming[1] && digitalRead(PttDetectorPin)==HIGH && PttTransfer==0){

  #if defined(EthModule)
    if(DetectedRemoteSw[IdSufix(NET_ID)][4]!=0 && RemoteSwLatencyAnsw==1){
  #endif

    // digitalWrite(PttOffPin, LOW);
    #if defined(EthModule) && defined(UdpBroadcastDebug_debug)
      // TxBroadcastUdp("PttOff-" + String(DetectedRemoteSw[NET_ID][4]) + "-" + String(RemoteSwLatencyAnsw) );
    #endif
    PTT = false;
    #if defined(LCD)
      LcdNeedRefresh = true;
    #endif

  #if defined(EthModule)
    }
  #endif

  }
}

//---------------------------------------------------------------------------------------------------------

void FrequencyRequest(){
  if(REQUEST > 0 && (millis() - RequestTimeout[0] > RequestTimeout[1])){

    #if defined(ICOM_CIV)
      txCIV(3, 0, CIV_ADRESS);  // ([command], [freq]) 3=read
    #endif

    #if defined(KENWOOD_PC)
          Serial.print("IF;");
          Serial.flush();       // Waits for the transmission of outgoing serial data to complete
    #endif
    RequestTimeout[0]=millis();
  }
}
//---------------------------------------------------------------------------------------------------------

#if defined(LCD)
  void Space(int MAX, int LENGHT, char CHARACTER){
    int NumberOfSpace = MAX-LENGHT;
    if(NumberOfSpace>0){
      for (int i=0; i<NumberOfSpace; i++){
        lcd.print(CHARACTER);
      }
    }
  }
#endif
//---------------------------------------------------------------------------------------------------------

#if defined(LCD)
  void PrintFreq(){
    int longer=String(freq/1000).length();
    if(longer<4){
      lcd.print(" ");
      lcd.print(freq);
    }else{
      lcd.print(String(freq/1000).substring(0, longer-3));
      lcd.print(".");
      lcd.print(String(freq/1000).substring(longer-3, longer));
    }
  }
#endif
//---------------------------------------------------------------------------------------------------------

void WebServer(){
  #if defined(EthModule)
    EthernetClient client = server.available();
    if (client) {
      boolean currentLineIsBlank = true;
      while (client.connected()) {
        if (client.available()) {
          char c = client.read();
          HTTP_req += c;
          if (c == '\n' && currentLineIsBlank) {
            client.println(F("HTTP/1.1 200 OK"));
            client.println(F("Content-Type: text/html"));
            client.println(F("Connection: close"));
            client.println();
            client.println(F("<!DOCTYPE html>"));
            client.println(F("<html>"));
            client.println(F("<head>"));
            client.print(F("<title>"));
            client.println(F("Band Decoder</title>"));
            client.print(F("<meta http-equiv=\"refresh\" content=\"10;url=http://"));
            client.print(Ethernet.localIP());
            client.println(F("\">"));
            client.println(F("<link href='http://fonts.googleapis.com/css?family=Roboto+Condensed:300italic,400italic,700italic,400,700,300&subset=latin-ext' rel='stylesheet' type='text/css'>"));
            client.println(F("<meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0, user-scalable=no\">"));
            client.println(F("<meta name=\"mobile-web-app-capable\" content=\"yes\">"));
            client.println(F("<style type=\"text/css\">"));
            client.println(F("body {font-family: 'Roboto Condensed',sans-serif,Arial,Tahoma,Verdana;background: #ccc;}"));
            client.println(F("a:link  {color: #888;font-weight: bold;text-decoration: none;}"));
            client.println(F("a:visited  {color: #888;font-weight: bold;text-decoration: none;}"));
            client.println(F("a:hover  {color: #888;font-weight: bold;text-decoration: none;}"));
            client.println(F("input {border: 2px solid #ccc;background: #fff;margin: 10px 5px 0 0;-webkit-border-radius: 5px;-moz-border-radius: 5px;border-radius: 5px;color : #333;}"));
            client.println(F("input:hover {border: 2px solid #080;}"));
            client.println(F("input.g {background: #080;color: #fff;}"));
            client.println(F("input.gr {background: #800;color: #fff;}"));
            client.println(F(".box {border: 2px solid #080;background: #ccc;  line-height: 2; margin: 10px 5px 0 5px;padding: 1px 7px 1px 7px;-webkit-border-radius: 5px;-moz-border-radius: 5px;border-radius: 5px;color : #000;}"));
            client.println(F(".boxr {border: 2px solid #800;background: #ccc; line-height: 2; margin: 10px 5px 0 5px;padding: 1px 7px 1px 7px;-webkit-border-radius: 5px;-moz-border-radius: 5px;border-radius: 5px;color : #000;}"));
            // client.println(F(".ptt {border: 2px solid #800;background: #ccc;margin: 10px 15px 0 10px;padding: 1px 7px 1px 7px;-webkit-border-radius: 5px;-moz-border-radius: 5px;border-radius: 5px;color : #800;}"));
            client.println(F("</style></head><body><p>Input TRX <span class=\"boxr\">"));
            #if defined(INPUT_SERIAL)
              client.print(F("Input serial"));
            #endif
            #if defined(ICOM_CIV)
              client.print(F("ICOM CI-V</span><br>CI-V address <span class=\"box\">"));
              client.print(CIV_ADRESS);
              client.print(F("h"));
            #endif
            #if defined(KENWOOD_PC)
              client.print(F("KENWOOD"));
            #endif
            #if defined(YAESU_CAT)
              client.print(F("YAESU"));
            #endif
            #if defined(YAESU_CAT_OLD)
              client.print(F("YAESU [Old]"));
            #endif
            #if defined(YAESU_BCD)
              client.print(F("BCD"));
            #endif
            #if defined(ICOM_ACC)
              client.print(F("ICOM ACC voltage"));
            #endif
            client.print(F("</span><br>Baudrate <span class=\"box\">"));
            client.print(SERBAUD);
            client.print(F("</span><br>Watchdog second <span class=\"box\">"));
            #if defined(WATCHDOG)
              client.print(WATCHDOG);
              client.print(F("</span><br>Request <span class=\"box\">"));
            #endif
            client.print(REQUEST);
            client.print(F("ms</span><br>Band <span class=\"box\">"));
            client.print(BAND);
            client.print(F("</span><br>Frequency <span class=\"boxr\">"));
            client.print(freq);
            client.print(F("Hz</span><br>Power voltage: <span class=\"box\">"));
            client.print(volt(analogRead(VoltagePin),ResistorCoeficient));
            client.println(F(" V</span></p>"));
            client.println(F("<p><a href=\".\" onclick=\"window.open( this.href, this.href, 'width=220,height=350,left=0,top=0,menubar=no,location=no,status=no' ); return false;\" > split&#8599;</a></p>"));
            client.println(F("</body></html>"));

            // Serial.print(HTTP_req);
            HTTP_req = "";
            break;
          }
          if (c == '\n') {
            currentLineIsBlank = true;
          }
          else if (c != '\r') {
            currentLineIsBlank = false;
          }
        }
      }
      delay(1);
      client.stop();
    }
  #endif
}
//---------------------------------------------------------------------------------------------------------

float volt(int raw, float divider) {
  // float voltage = (raw * 5.0) / 1024.0 * ResistorCoeficient;
  float voltage = float(raw) * ArefVoltage * divider / 1023.0;
  #if defined(SERIAL_debug)
  if(DEBUG==1){
    Serial.print(F("Voltage "));
    Serial.println(voltage);
  }
  #endif
  return voltage;
}
//-------------------------------------------------------------------------------------------------------

void DCinMeasure(){
  if (millis() - VoltageRefresh[0] > VoltageRefresh[1]){
    DCinVoltage = volt(analogRead(VoltagePin), ResistorCoeficient);
    #if defined(LCD)
      if (DCinVoltage<7){
        lcd.setCursor(3, 0);
        lcd.print(" Power LOW!");
      }else if (DCinVoltage>15){
        lcd.setCursor(2, 0);
        lcd.print("Power HIGH!");
      }
    #endif
    VoltageRefresh[0] = millis();                      // set time mark
  }
}
//---------------------------------------------------------------------------------------------------------

void BandDecoderInput(){
  #if !defined(YAESU_BCD)
    InterruptON(0,0); // ptt, enc
  #endif

  //----------------------------------- Input Serial
  #if defined(INPUT_SERIAL)
    while (Serial.available() > 0) {
        BAND = Serial.parseInt();
        freq = Serial.parseInt();
        if (Serial.read() == '\n') {
            bandSET();
            #if defined(SERIAL_echo)
                serialEcho();
            #endif
        }
    }
  #endif

  //----------------------------------- Icom ACC
  #if defined(ICOM_ACC)
    VALUE = analogRead(ADPin);
    #if defined(ENABLE_DIVIDER)
      Divider = 6;
    #endif
    if (counter == 5) {
        VOLTAGE = float(VALUE) * ArefVoltage * Divider / 1023.0;

        //=====[ Icom ACC voltage range ]===========================================================

        if (VOLTAGE > 0.73 && VOLTAGE < 1.00 ) {BAND=10;}  //   6m   * * * * * * * * * * * * * * * *
        if (VOLTAGE > 1.00 && VOLTAGE < 1.09 ) {BAND=9;}   //  10m   *           Need              *
        if (VOLTAGE > 1.09 && VOLTAGE < 1.32 ) {BAND=8;}   //  12m   *    calibrated to your       *
        if (VOLTAGE > 1.32 && VOLTAGE < 1.55 ) {BAND=7;}   //  15m   *         own ICOM            *
        if (VOLTAGE > 1.55 && VOLTAGE < 1.77 ) {BAND=6;}   //  17m   *     ----------------        *
        if (VOLTAGE > 1.77 && VOLTAGE < 2.24 ) {BAND=5;}   //  20m   *    (These values have       *
        if (VOLTAGE > 0.10 && VOLTAGE < 0.50 ) {BAND=4;}   //  30m   *   been measured by any)     *
        if (VOLTAGE > 2.24 && VOLTAGE < 2.73 ) {BAND=3;}   //  40m   *          ic-746             *
        if (VOLTAGE > 2.73 && VOLTAGE < 2.99 ) {BAND=2;}   //  80m   *                             *
        if (VOLTAGE > 2.99 && VOLTAGE < 4.00 ) {BAND=1;}   // 160m   * * * * * * * * * * * * * * * *
        if (VOLTAGE > 0.00 && VOLTAGE < 0.10 ) {BAND=0;}   // parking

        //==========================================================================================

        bandSET();                                // set outputs
        delay (20);
    }else{
        if (abs(prevVALUE-VALUE)>10) {            // average
            //means change or spurious number
            prevVALUE=VALUE;
        }else {
            counter++;
            prevVALUE=VALUE;
        }
    }
    #if defined(SERIAL_echo)
        serialEcho();
        Serial.print(VOLTAGE);
        Serial.println(" V");
        Serial.flush();
    #endif

    delay(500);                                   // refresh time
  #endif

  //----------------------------------- Icom CIV
  #if defined(ICOM_CIV)
    if (Serial.available() > 0) {
        incomingByte = Serial.read();
        icomSM(incomingByte);
        rdIS="";
        if(rdI[10]==0xFD){                    // state machine end
          for (int i=9; i>=5; i-- ){
              if (rdI[i] < 10) {            // leading zero
                  rdIS = rdIS + 0;
              }
              rdIS = rdIS + String(rdI[i], HEX);  // append BCD digit from HEX variable to string
          }
          freq = rdIS.toInt();
          FreqToBandRules();
          bandSET();

          #if defined(SERIAL_echo)
              serialEcho();
          #endif
          RequestTimeout[0]=millis();
        }
    }
  #endif

  //----------------------------------- Yaesu BCD
  #if defined(YAESU_BCD)
    if (millis() - BcdInRefresh[0] > BcdInRefresh[1]){
      BAND = 0;
      if(digitalRead(BcdIn1Pin)==0){
        BAND = BAND | (1<<3);    // Set the n-th bit
      }
      if(digitalRead(BcdIn2Pin)==0){
        BAND = BAND | (1<<2);
      }
      if(digitalRead(BcdIn3Pin)==0){
        BAND = BAND | (1<<1);
      }
      if(digitalRead(BcdIn4Pin)==0){
        BAND = BAND | (1<<0);
      }
      bandSET();
      #if defined(SERIAL_echo)
          serialEcho();
      #endif
      BcdInRefresh[0]=millis();
    }
  #endif

  //----------------------------------- Kenwood
  #if defined(KENWOOD_PC)
    // Data exapmple
    // IF00007151074      000000000030000080;
    // IF00007032327      000000000030000080;
    while (Serial.available()) {
        rdKS="";
        Serial.readBytesUntil(lf, rdK, 38);       // fill array from serial
            if (rdK[0] == 73 && rdK[1] == 70){     // filter
                for (int i=2; i<=12; i++){          // 3-13 position to freq
                    rdKS = rdKS + String(rdK[i]);   // append variable to string
                }
                freq = rdKS.toInt();
                FreqToBandRules();
                bandSET();                                              // set outputs relay

                #if defined(SERIAL_echo)
                    serialEcho();
                #endif
            }
            memset(rdK, 0, sizeof(rdK));   // Clear contents of Buffer
    }
  #endif

  //----------------------------------- Yaesu CAT
  #if defined(YAESU_CAT)
      #include "yaesu_cat.h"
  #endif
  #if defined(YAESU_CAT_OLD)
      #include "yaesu_cat_old.h"
  #endif

  #if !defined(YAESU_BCD)
    InterruptON(1,1); // ptt, enc
  #endif
}
//---------------------------------------------------------------------------------------------------------

void BandDecoderOutput(){

  //=====[ Output Remote relay ]=======================
  #if defined(REMOTE_RELAY)
      timeout3 = millis()-previous3;                  // check timeout
      if (timeout3>(watchdog3)){
          remoteRelay();
          previous3 = millis();                       // set time mark
      }
  #endif
  //=====[ Output Icom CIV ]=======================
  #if defined(ICOM_CIV_OUT)
      if(freq!= freqPrev1){                    // if change
          txCIV(0, freq, CIV_ADR_OUT);         // 0 - set freq
          freqPrev1 = freq;
      }
  #endif
  //=====[ Output Kenwood PC ]=====================
  #if !defined(REQUEST) && defined(KENWOOD_PC_OUT)
      if(freq != freqPrev2){                     // if change
          String freqPCtx = String(freq);        // to string
          while (freqPCtx.length() < 11) {       // leding zeros
              freqPCtx = 0 + freqPCtx;
          }
         Serial.print("FA" + freqPCtx + ";");    // sets both VFO
         Serial.print("FB" + freqPCtx + ";");
//          Serial.print("FA" + freqPCtx + ";");    // first packet not read every time
         Serial.flush();
         freqPrev2 = freq;
      }
  #endif
  //=====[ Output Yaesu CAT ]=====================
  #if !defined(REQUEST) && defined(YAESU_CAT_OUT)
      if(freq != freqPrev2){                     // if change
          String freqPCtx = String(freq);        // to string
          while (freqPCtx.length() < 8) {        // leding zeros
              freqPCtx = 0 + freqPCtx;
          }
         Serial.print("FA" + freqPCtx + ";");    // sets both VFO
         Serial.print("FB" + freqPCtx + ";");
         Serial.flush();
         freqPrev2 = freq;
      }
  #endif
  //=====[ Output Yaesu CAT OLD ]=================
  #if !defined(REQUEST) && defined(YAESU_CAT_OUT_OLD)
      if(freq != freqPrev2){                     // if change
          String freqPCtx = String(freq);        // to string
          while (freqPCtx.length() < 8) {        // leding zeros
              freqPCtx = 0 + freqPCtx;
         }
         Serial.write(1);                        // set freq
         Serial.flush();
         freqPrev2 = freq;
      }
  #endif

}
//---------------------------------------------------------------------------------------------------------

// void bandSET() {                                               // set outputs by BAND variable
//   if(BAND==0 && previousBAND != 0){    // deactivate PTT
//     digitalWrite(PttOffPin, HIGH);
//     PTT = true;
//     #if defined(LCD)
//       LcdNeedRefresh = true;
//     #endif
//   }
//
//
//   if((PTT==false && previousBAND != 0 ) || (PTT==true && previousBAND == 0)){
//     ShiftByte[0] = B00000000;
//     ShiftByte[1] = B00000000;
//
//     if(BAND > 0 && BAND < 9){
//       ShiftByte[0] = ShiftByte[0] | (1<<BAND-1);    // Set the n-th bit
//     }
//     if(BAND > 7 && BAND < 17){
//       ShiftByte[1] = ShiftByte[1] | (1<<BAND-9);    // Set the n-th bit
//     }
//     digitalWrite(ShiftOutLatchPin, LOW);    // ready for receive data
//     if(NumberOfBoards > 1){ shiftOut(ShiftOutDataPin, ShiftOutClockPin, LSBFIRST, ShiftByte[1]); }
//                             shiftOut(ShiftOutDataPin, ShiftOutClockPin, LSBFIRST, ShiftByte[0]);
//     digitalWrite(ShiftOutLatchPin, HIGH);    // switch to output pin
//
//       #if !defined(YAESU_BCD)
//           bcdOut();
//       #endif
//       #if defined(LCD)
//         LcdNeedRefresh = true;
//       #endif
//   }
//
//   #if defined(EthModule)
//     if(DetectedRemoteSw[IdSufix(NET_ID)][4]==0 || RemoteSwLatencyAnsw==0){
//       //  && millis() < RemoteSwLatency[0]+RemoteSwLatency[1]*5) ){
//       digitalWrite(PttOffPin, HIGH);
//       #if defined(UdpBroadcastDebug_debug)
//         TxBroadcastUdp("BandSet-" + String(DetectedRemoteSw[IdSufix(NET_ID)][4]) + "-" + String(RemoteSwLatencyAnsw) );
//       #endif
//       PTT = true;
//       #if defined(LCD)
//       LcdNeedRefresh = true;
//       #endif
//     }
//   #endif
//   TxUDP(ThisDevice, RemoteDevice, rxShiftInButton[0], rxShiftInButton[1], rxShiftInButton[2]);
//
//   previousBAND = BAND;
//   #if defined(WATCHDOG)
//     WatchdogTimeout[0] = millis();                   // set time mark
//   #endif
// }
//---------------------------------------------------------------------------------------------------------

// void remoteRelay() {
//     Serial.print(1);
//     Serial.print(',');
//     Serial.print(BAND, DEC);
//     Serial.print('\n');
//     Serial.flush();
// }
// //---------------------------------------------------------------------------------------------------------
//
// void serialEcho() {
//     Serial.print("<");
//     Serial.print(BAND);
//     Serial.print(",");
//     Serial.print(freq);
//     Serial.println(">");
//     Serial.flush();
// }
//---------------------------------------------------------------------------------------------------------

// #if !defined(YAESU_BCD)
//     void bcdOut(){
//         if (BCDmatrixOUT[0][BAND] == 1){ digitalWrite(BcdIn1Pin, HIGH); }else{ digitalWrite(BcdIn1Pin, LOW);}
//         if (BCDmatrixOUT[1][BAND] == 1){ digitalWrite(BcdIn2Pin, HIGH); }else{ digitalWrite(BcdIn2Pin, LOW);}
//         if (BCDmatrixOUT[2][BAND] == 1){ digitalWrite(BcdIn3Pin, HIGH); }else{ digitalWrite(BcdIn3Pin, LOW);}
//         if (BCDmatrixOUT[3][BAND] == 1){ digitalWrite(BcdIn4Pin, HIGH); }else{ digitalWrite(BcdIn4Pin, LOW);}
//     }
// #endif
//---------------------------------------------------------------------------------------------------------

// void watchDog() {
//   #if defined(WATCHDOG)
//     if((millis() - WatchdogTimeout[0]) > WatchdogTimeout[1]) {
//       BAND=0;
//       freq=0;
//       bandSET();                                 // set outputs
//     }
//   #endif
// }
//---------------------------------------------------------------------------------------------------------

#if defined(ICOM_CIV) || defined(ICOM_CIV_OUT)

    int icomSM(byte b){      // state machine
        // This filter solves read from 0x00 0x05 0x03 commands and 00 E0 F1 address used by software
        switch (state) {
            case 1: if( b == 0xFE ){ state = 2; rdI[0]=b; }; break;
            case 2: if( b == 0xFE ){ state = 3; rdI[1]=b; }else{ state = 1;}; break;
            // addresses that use different software 00-trx, e0-pc-ale, winlinkRMS, f1-winlink trimode
            case 3: if( b == 0x00 || b == 0xE0 || b == 0xF1 ){ state = 4; rdI[2]=b;                             // choose command $03
              }else if( b == CIV_ADRESS ){ state = 6; rdI[2]=b;}else{ state = 1;}; break;                       // or $05

            case 4: if( b == CIV_ADRESS ){ state = 5; rdI[3]=b; }else{ state = 1;}; break;                      // select command $03
            case 5: if( b == 0x00 || b == 0x03 ){state = 8; rdI[4]=b; }else{ state = 1;}; break;

            case 6: if( b == 0x00 || b == 0xE0 || b == 0xF1 ){ state = 7; rdI[3]=b; }else{ state = 1;}; break;  // select command $05
            case 7: if( b == 0x00 || b == 0x05 ){ state = 8; rdI[4]=b; }else{ state = 1;}; break;

            case 8: if( b <= 0x99 ){state = 9; rdI[5]=b; }else{state = 1;}; break;
            case 9: if( b <= 0x99 ){state = 10; rdI[6]=b; }else{state = 1;}; break;
           case 10: if( b <= 0x99 ){state = 11; rdI[7]=b; }else{state = 1;}; break;
           case 11: if( b <= 0x99 ){state = 12; rdI[8]=b; }else{state = 1;}; break;
           case 12: if( b <= 0x99 ){state = 13; rdI[9]=b; }else{state = 1;}; break;
           case 13: if( b == 0xFD ){state = 1; rdI[10]=b; }else{state = 1; rdI[10] = 0;}; break;
        }
    }
    //---------------------------------------------------------------------------------------------------------

    int txCIV(int commandCIV, long dataCIVtx, int toAddress) {
        //Serial.flush();
        Serial.write(254);                                    // FE
        Serial.write(254);                                    // FE
        Serial.write(toAddress);                              // to adress
        Serial.write(fromAdress);                             // from OE
        Serial.write(commandCIV);                             // data
        if (dataCIVtx != 0){
            String freqCIVtx = String(dataCIVtx);             // to string
            String freqCIVtxPart;
            while (freqCIVtx.length() < 10) {                 // leding zeros
                freqCIVtx = 0 + freqCIVtx;
            }
            for (int x=8; x>=0; x=x-2){                       // loop for 5x2 char [xx xx xx xx xx]
                freqCIVtxPart = freqCIVtx.substring(x,x+2);   // cut freq to five part
                    Serial.write(hexToDec(freqCIVtxPart));    // HEX to DEC, because write as DEC format from HEX variable
            }
        }
        Serial.write(253);                                    // FD
        // Serial.flush();
        while(Serial.available()){        // clear buffer
          Serial.read();
        }
    }
    //---------------------------------------------------------------------------------------------------------

    unsigned int hexToDec(String hexString) {
        unsigned int decValue = 0;
        int nextInt;
        for (int i = 0; i < hexString.length(); i++) {
            nextInt = int(hexString.charAt(i));
            if (nextInt >= 48 && nextInt <= 57) nextInt = map(nextInt, 48, 57, 0, 9);
            if (nextInt >= 65 && nextInt <= 70) nextInt = map(nextInt, 65, 70, 10, 15);
            if (nextInt >= 97 && nextInt <= 102) nextInt = map(nextInt, 97, 102, 10, 15);
            nextInt = constrain(nextInt, 0, 15);
            decValue = (decValue * 16) + nextInt;
        }
        return decValue;
    }
    //---------------------------------------------------------------------------------------------------------
#endif

// void FreqToBandRules(){
//          if (freq >=Freq2Band[0][0] && freq <=Freq2Band[0][1] )  {BAND=1;}  // 160m
//     else if (freq >=Freq2Band[1][0] && freq <=Freq2Band[1][1] )  {BAND=2;}  //  80m
//     else if (freq >=Freq2Band[2][0] && freq <=Freq2Band[2][1] )  {BAND=3;}  //  40m
//     else if (freq >=Freq2Band[3][0] && freq <=Freq2Band[3][1] )  {BAND=4;}  //  30m
//     else if (freq >=Freq2Band[4][0] && freq <=Freq2Band[4][1] )  {BAND=5;}  //  20m
//     else if (freq >=Freq2Band[5][0] && freq <=Freq2Band[5][1] )  {BAND=6;}  //  17m
//     else if (freq >=Freq2Band[6][0] && freq <=Freq2Band[6][1] )  {BAND=7;}  //  15m
//     else if (freq >=Freq2Band[7][0] && freq <=Freq2Band[7][1] )  {BAND=8;}  //  12m
//     else if (freq >=Freq2Band[8][0] && freq <=Freq2Band[8][1] )  {BAND=9;}  //  10m
//     else if (freq >=Freq2Band[9][0] && freq <=Freq2Band[9][1] ) {BAND=10;}  //   6m
//     else if (freq >=Freq2Band[10][0] && freq <=Freq2Band[10][1] ) {BAND=11;}  //   2m
//     else {BAND=0;}                                                // out of range
// }
