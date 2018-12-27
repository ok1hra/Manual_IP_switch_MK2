#include <Arduino.h>

/* COMPILE FOR
  --------------------
  Board: Arduino NANO
  Processor: ATMEGA328P (Old Booatloader)
  --------------------

  Manual_IP_switch_MK2 for Arduino rev.0.2
-----------------------------------------------------------
  https://remoteqth.com/wiki/index.php?page=Band+decoder+MK2
  2018-05 by OK1HRA

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
  2018-11 rebuild ethernet and some bug fix
  2018-06 support IP relay
  2018-05 mk2 initial release

  ToDo
  ----
  - select ID by band decoder
  - encoder show ANT fullname

*/
//=====[ Inputs ]=============================================================================================

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

//=====[ Hardware ]=============================================================================================
const char* REV = "20181216";

#define SERIAL_debug
bool DEBUG = 1;
bool EnableEthernet = 1;
bool EnableDHCP     = 1;
byte DetectedRemoteSw[16][5]{             // detect by RX broadcast packet - storage by ID (ID=rows)
  {0,0,0,0, 0},   // IP:port ID 0
  {0,0,0,0, 0},   // IP:port ID 1
  {0,0,0,0, 0},   // IP:port ID 2
  {0,0,0,0, 0},   // IP:port ID 3
  {0,0,0,0, 0},   // IP:port ID 4
  {0,0,0,0, 0},   // IP:port ID 5
  {0,0,0,0, 0},   // IP:port ID 6
  {0,0,0,0, 0},   // IP:port ID 7
  {0,0,0,0, 0},   // IP:port ID 8
  {0,0,0,0, 0},   // IP:port ID 9
  {0,0,0,0, 0},   // IP:port ID A
  {0,0,0,0, 0},   // IP:port ID B
  {0,0,0,0, 0},   // IP:port ID C
  {0,0,0,0, 0},   // IP:port ID D
  {0,0,0,0, 0},   // IP:port ID E
};

const int NumberOfEncoderOutputs = 16;
int EncoderCount = 0;
const int SERIAL_BAUDRATE = 115200;
int incomingByte = 0;   // for incoming serial data
#define LCD                   // Uncoment to Enable I2C LCD
long GetNetIdTimer[2]{0,2000};
long ButtDebounceTimer[2]{0,200};
long EncEndTimer[2]{0,1000};
int EncEndStatus = 0;
const int ButtonMapping[8]={3,2,1,0,7,6,5,4}; // 1-8

#define LcdI2Caddress  0x27   // 0x27 0x3F - may be find with I2C scanner https://playground.arduino.cc/Main/I2cScanner
#define EthModule             // enable Ethernet module if installed
#define __USE_DHCP__          // enable DHCP
byte BOARD_ID = 0x00;         // NetID [hex] MUST BE UNIQUE IN NETWORK - replace by P6 board encoder
// #define BcdToIP               // control IP relay in BCD format
bool EthLinkStatus = 0;
long EthLinkStatusTimer[2]{1500,1000};

//=====[ Settings ]===========================================================================================

#define SERBAUD        115200   // [baud] Serial port in/out baudrate
#define WATCHDOG       10     // [sec] determines the time, after which the all relay OFF, if missed next input data - uncomment for the enabled
#define REQUEST        500    // [ms] use TXD output for sending frequency request
#define CIV_ADRESS   0x56     // CIV input HEX Icom adress (0x is prefix)
#define CIV_ADR_OUT  0x56     // CIV output HEX Icom adress (0x is prefix)
// #define ENABLE_DIVIDER     // for highest voltage D-SUB pin 13 inputs up to 24V - need short JP9

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

#if defined(LCD)
  #include <Wire.h>
  #include <LiquidCrystal_I2C.h>
  LiquidCrystal_I2C lcd(LcdI2Caddress,16,2);
  long LcdRefresh[2]{0,500};
  const char* ANTname[17] = {
      "Out of band",  // Band 0 (no data)
      "Dipole",       // Band 1
      "Vertical",     // Band 2
      "3el Yagi",     // Band 3
      "Windom",       // Band 4
      "DeltaLoop",    // Band 5
      "20m Stack",    // Band 6
      "DeltaLoop",    // Band 7
      "HB9",          // Band 8
      "Dipole",       // Band 9
      "5el Yagi",     // Band 10
      "7el Yagi",     // Band 11
      "24el",         // Band 12
      "20el quad",    // Band 13
      "Dish 1.2m",    // Band 14
      "Dish 1.2m",    // Band 15
      "Dish 1m",      // Band 16
  };
  // byte LockChar[8] = {0b00100, 0b01010, 0b01010, 0b11111, 0b11011, 0b11011, 0b11111, 0b00000};
  uint8_t LockChar[8] = {0x4,0xa,0xa,0x1f,0x1b,0x1b,0x1f,0x0};
  // byte EthChar[8] = {0b00000, 0b00000, 0b11111, 0b10001, 0b10001, 0b11011, 0b11111, 0b00000};
  uint8_t EthChar[8] = {0x0,0x0,0x1f,0x11,0x11,0x1b,0x1f,0x0};

  bool LcdNeedRefresh = false;
#endif

#if defined(EthModule)
  #include <Ethernet.h>
  #include <EthernetUdp.h>
  // #include <Ethernet2.h>
  // #include <EthernetUdp2.h>
  //  #include <util.h>
  // #include <Dhcp.h>
  // #include <EthernetServer.h>
  #include <SPI.h>
  byte LastMac = 0x00 + BOARD_ID;

  byte mac[] = {0xDE, 0xAD, 0xBE, 0xEF, 0x22, LastMac};
  // byte mac[] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, LastMac};
  IPAddress ip(192, 168, 1, 222);         // IP
  IPAddress gateway(192, 168, 1, 200);    // GATE
  IPAddress subnet(255, 255, 255, 0);     // MASK
  IPAddress myDns(8, 8, 8, 8);            // DNS (google pub)
  EthernetServer server(80);              // Web server PORT
  String HTTP_req;

  unsigned int UdpCommandPort = 88;       // local UDP port listen to command
  #define UDP_TX_PACKET_MAX_SIZE 40       // MIN 30
  char packetBuffer[UDP_TX_PACKET_MAX_SIZE]; //buffer to hold incoming packet,
  int UDPpacketSize;
  EthernetUDP UdpCommand; // An EthernetUDP instance to let us send and receive packets over UDP
  IPAddress BroadcastIP(0, 0, 0, 0);        // Broadcast IP address
  int BroadcastPort       = 88;             // destination broadcast packet port
  IPAddress RemoteSwIP(0, 0, 0, 0);         // remote UDP IP switch - set from UDP DetectRemote array
  int RemoteSwPort         = 0;             // remote UDP IP switch port
  int BandDecoderChange    = 0;             // If band change, send query packet to Remote IP switch
  long RemoteSwLatency[2];                  // start, stop mark
  byte RemoteSwLatencyAnsw = 0;             // answer (offline) detect
  byte TxUdpBuffer[10];
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

//---------------------------------------------------------------------------------------------------------

void setup() {
  BOARD_ID = GetBoardId();
  Serial.begin(115200);
  #if defined(SERIAL_debug)
  if(DEBUG==1){
    Serial.setTimeout(10);
    Serial.println();
    Serial.println(F("==================="));
    Serial.print(F("IP SW MASTER NET ID: "));
    Serial.println(BOARD_ID, HEX);
    Serial.println(F("==================="));
    Serial.println();
  }
  #endif

  pinMode(EncBPin, INPUT);
  pinMode(EncAShiftInPin, INPUT);
  pinMode(ShiftInClockPin, OUTPUT);
  pinMode(ShiftInLatchPin, OUTPUT);
  pinMode(ShiftInDataPin, INPUT);
  pinMode(ShiftOutDataPin, OUTPUT);
  pinMode(ShiftOutLatchPin, OUTPUT);
  pinMode(ShiftOutClockPin, OUTPUT);

  #if defined(LCD)
    lcd.backlight();
    // lcd.begin();
    lcd.init();
    lcd.createChar(0, LockChar);
    lcd.createChar(1, EthChar);
    lcd.clear();
    lcd.setCursor(1, 0);
    lcd.print("Version: ");
    lcd.setCursor(8, 1);
    lcd.print(REV);
  #endif


  #if defined(EthModule)
    pinMode(Id1Pin, INPUT_PULLUP);
    pinMode(Id2Pin, INPUT_PULLUP);
    pinMode(Id3Pin, INPUT_PULLUP);
    // must disable because use as i2c // pinMode(Id4Pin, INPUT_PULLUP);
    LastMac = 0x00 + BOARD_ID;
    mac[5] = LastMac;

    EthernetCheck();

  #endif
  // ANTname[0] = " [timeout]  ";
  InterruptON(1,1); // ptt, enc
}
//---------------------------------------------------------------------------------------------------------

void loop() {
  EthernetCheck();
  LcdDisplay();
  IncomingUDP();
  PttOff();
  // NetId();   // Live change ID/BAND

  if (Serial.available() > 0) {
          incomingByte = Serial.read();

         if(incomingByte==100){
            DEBUG = !DEBUG;
            Serial.print(F("* Now serial debug "));
            if(DEBUG==1){
              Serial.println(F("ON *"));
            }else{
              Serial.println(F("OFF *"));
            }
          }else if(incomingByte==104){  // h
            Serial.println(F("-----------------------"));
            Serial.print(F("Version: "));
            Serial.println(REV);
            Serial.print(F("MASTER DEVICE ID: "));
            Serial.println(BOARD_ID, HEX);
            Serial.print(F("IP address:"));
            Serial.println(Ethernet.localIP());
            Serial.print(F("Slave IP: "));
            Serial.print(RemoteSwIP);
            Serial.print(F(":"));
            Serial.println(RemoteSwPort);
            Serial.print(F("Serial debug: "));
            if(DEBUG==1){
              Serial.println(F("ON"));
            }else{
              Serial.println(F("OFF"));
            }
            Serial.println("-----------------------");
          }else{
            Serial.print(F(" ["));
            Serial.write(incomingByte); //, DEC);
            Serial.println(F("] unknown command - for more info press 'h'"));
          }
  }

// ToDo
//  - netid
//  - PTT
//  - cat

  // WebServer();

  // BandDecoderInput();
  // BandDecoderOutput();

  // Serial.println(AccKeyboardShift());
  // delay(500);

  // watchDog();
  // FrequencyRequest();
  // ShiftOutTest();

}
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
// SUBROUTINES ---------------------------------------------------------------------------------------------------------

void InterruptON(int ptt, int enc){
  if(ptt==0){
    detachInterrupt(digitalPinToInterrupt(PttDetectorPin));
  }else if(ptt==1){
    attachInterrupt(digitalPinToInterrupt(PttDetectorPin), PttDetector, FALLING);  // need detachInterrupt in IncomingUDP() subroutine
  }
  if(enc==0){
    detachInterrupt(digitalPinToInterrupt(EncAShiftInPin));
  }else if(enc==1){
    attachInterrupt(digitalPinToInterrupt(EncAShiftInPin), EncoderInterrupt, FALLING);  // need detachInterrupt in IncomingUDP() subroutine
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
      TxUDP();
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
      //   switch (rxShiftInRead) {
      //     case 0:
      //         KeyboardDetect = true;
      //         switch (i) {
      //           case 1: rxShiftInButton[0] = rxShiftInButton[0] ^ (1<<3); break;  // invert n-th bit
      //           case 2: rxShiftInButton[0] = rxShiftInButton[0] ^ (1<<2); break;
      //           case 3: rxShiftInButton[0] = rxShiftInButton[0] ^ (1<<1); break;
      //           case 4: rxShiftInButton[0] = rxShiftInButton[0] ^ (1<<0); break;
      //           case 5: rxShiftInButton[0] = rxShiftInButton[0] ^ (1<<7); break;
      //           case 6: rxShiftInButton[0] = rxShiftInButton[0] ^ (1<<6); break;
      //           case 7: rxShiftInButton[0] = rxShiftInButton[0] ^ (1<<5); break;
      //           case 8: rxShiftInButton[0] = rxShiftInButton[0] ^ (1<<4); break;
      //           default:
      //             // if nothing else matches, do the default
      //           break;
      //         }
      //       break;
      //     case 1: break;
      //   }

      digitalWrite(ShiftInClockPin, 1);
    }
    return KeyboardDetect;
}
//---------------------------------------------------------------------------------------------------------

void NetId(){
  if(millis()-GetNetIdTimer[0]>GetNetIdTimer[1]){
    if(BOARD_ID != GetBoardId()){
      BOARD_ID = GetBoardId();
      SendBroadcastUdp();
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
    //   BOARD_ID = BOARD_ID | (1<<3);    // Set the n-th bit
    // }
  }
#endif
//---------------------------------------------------------------------------------------------------------

void LcdDisplay(){
  #if defined(LCD)
    if(millis()-LcdRefresh[0]>LcdRefresh[1] || LcdNeedRefresh == true){

      if(RemoteSwLatencyAnsw==1 || (RemoteSwLatencyAnsw==0 && millis() < RemoteSwLatency[0]+RemoteSwLatency[1]*5)){ // if answer ok, or latency measure nod end
        if(millis()-EncEndTimer[0]>EncEndTimer[1] && EncEndStatus==1 && digitalRead(EncBPin)==1 && digitalRead(EncAShiftInPin)==1){
          EncEndStatus = 0;
        }
        if(millis()-EncEndTimer[0]>EncEndTimer[1] && EncEndStatus==1 && (digitalRead(EncBPin)==0 || digitalRead(EncAShiftInPin)==0)){
          lcd.noBacklight();
          lcd.setCursor(0,0);
          lcd.print("Intermediate    ");
          lcd.setCursor(0,1);
          lcd.print("position!");
          delay(200);
          lcd.backlight();
        }else{
          // Encoder
          for (int i=0; i<NumberOfEncoderOutputs; i++){
            lcd.setCursor(i,0);
            if(i==EncoderCount){
              if(EncoderCount>8){
                lcd.setCursor(i-1,0);
                lcd.print(EncoderCount+1, DEC);
              }else{
                lcd.print(EncoderCount+1, DEC);
              }
            }else{
              lcd.print(' ');
            }
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
          // PTT
          lcd.setCursor(8,1);
          if(PTT==true){
            // lcd.write(byte(0));        // Lock icon
            lcd.print((char)0);
          }else{
            lcd.print("|");
          }
        }

        lcd.print(" ");
        if(EthLinkStatus==1){
          if(RemoteSwLatencyAnsw==1){ // if answer ok, or latency measure nod end
            // lcd.print("*");
            lcd.print((char)1);   // EthChar
          }else if(RemoteSwLatencyAnsw==0 && millis() < RemoteSwLatency[0]+RemoteSwLatency[1]*5){
              lcd.print(" ");
          }else{
            lcd.print("!");
          }
        }else{
          lcd.print("x");
        }
        lcd.print(" ID-");
        lcd.print(BOARD_ID, HEX);
      }else if(EthLinkStatus==0){
        lcd.setCursor(0, 0);
        lcd.print((char)1);   // EthChar
        lcd.print(F(" Please connect"));
        lcd.setCursor(0, 1);
        lcd.print(F("  ethernet      "));
      }else{
        lcd.setCursor(0, 0);
        lcd.print(F(" IPswitch-ID: "));
        lcd.print(BOARD_ID, HEX);
        lcd.setCursor(0, 1);
        lcd.print(F("  not detected  "));
      }

      LcdRefresh[0]=millis();
      LcdNeedRefresh = false;
    }
  #endif
}
//-------------------------------------------------------------------------------------------------------

void SendBroadcastUdp(){
  #if defined(EthModule)
    InterruptON(0,0); // ptt, enc
    for (int i=0; i<15; i++){
      if(DetectedRemoteSw[i][4]!=0){
        RemoteSwIP = DetectedRemoteSw[i];
        RemoteSwPort = DetectedRemoteSw[i][4];

        UdpCommand.beginPacket(RemoteSwIP, RemoteSwPort);
          UdpCommand.print("b:m");
          UdpCommand.print(BOARD_ID);
          UdpCommand.print(";");
        UdpCommand.endPacket();
        RemoteSwLatencyAnsw = 0;   // send command, wait to answer

        #if defined(SERIAL_debug)
        if(DEBUG==1){
          Serial.print(i);
          Serial.print(F(" - TX direct "));
          Serial.print(RemoteSwIP);
          Serial.print(F(":"));
          Serial.print(RemoteSwPort);
          Serial.print(F(" \""));
          Serial.print(F("b:m"));
          Serial.print(i);
          Serial.println(F(";"));
        }
        #endif
      }
    }

    BroadcastIP = ~Ethernet.subnetMask() | Ethernet.gatewayIP();

    UdpCommand.beginPacket(BroadcastIP, BroadcastPort);   // Send to IP and port from recived UDP command
    // UdpCommand.beginMulticast(UdpCommand.BroadcastIP(), BroadcastPort, ETH.localIP()).
      UdpCommand.print("b:m");
      UdpCommand.print(BOARD_ID);
      UdpCommand.print(";");
    UdpCommand.endPacket();

    #if defined(SERIAL_debug)
    if(DEBUG==1){
      Serial.print(F("TX Broadcast "));
      Serial.print(BroadcastIP);
      Serial.print(F(":"));
      Serial.print(BroadcastPort);
      Serial.print(F(" \""));
      Serial.print(F("b:m"));
      Serial.print(BOARD_ID);
      Serial.print(F(";"));
      Serial.print(F("\"  ms "));
      Serial.println(IpTimeout[0][0]);
    }
    #endif
    IpTimeout[0][0] = millis();                      // set time mark
    InterruptON(1,1); // ptt, enc
  #endif
}
//---------------------------------------------------------------------------------------------------------

void IncomingUDP(){
  #if defined(EthModule)
    InterruptON(0,0); // ptt, enc
      // COMMANDS [port 88]
      UDPpacketSize = UdpCommand.parsePacket();    // if there's data available, read a packet
      if (UDPpacketSize){
        UdpCommand.read(packetBuffer, UDP_TX_PACKET_MAX_SIZE);      // read the packet into packetBufffer

        //- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
        // Switch BROADCAST - STORAGE IP by received ID in 'DetectedRemoteSw' array (rows = Switch ID)
        if (packetBuffer[0] == 'b' && packetBuffer[1] == ':' && packetBuffer[2] == 's' && packetBuffer[3] == 'm' && packetBuffer[5] == ';'){
          RemoteSwLatency[1] = (millis()-RemoteSwLatency[0])/2; // set latency (half path in ms us/2/1000)
          RemoteSwLatencyAnsw = 1;           // answer packet received
          IPAddress TmpAddr = UdpCommand.remoteIP();
          // DetectedRemoteSw [(int)packetBuffer[3] - 48] [0]=TmpAddr[0];     // Switch IP addres storage to array
          // DetectedRemoteSw [(int)packetBuffer[3] - 48] [1]=TmpAddr[1];     // (int)packetBuffer[3] - 48 = convert char number to DEC
          // DetectedRemoteSw [(int)packetBuffer[3] - 48] [2]=TmpAddr[2];
          // DetectedRemoteSw [(int)packetBuffer[3] - 48] [3]=TmpAddr[3];
          // DetectedRemoteSw [(int)packetBuffer[3] - 48] [4]=UdpCommand.remotePort();
          DetectedRemoteSw [hexToDecBy4bit(packetBuffer[4])] [0]=TmpAddr[0];     // Switch IP addres storage to array
          DetectedRemoteSw [hexToDecBy4bit(packetBuffer[4])] [1]=TmpAddr[1];
          DetectedRemoteSw [hexToDecBy4bit(packetBuffer[4])] [2]=TmpAddr[2];
          DetectedRemoteSw [hexToDecBy4bit(packetBuffer[4])] [3]=TmpAddr[3];
          DetectedRemoteSw [hexToDecBy4bit(packetBuffer[4])] [4]=UdpCommand.remotePort();

          lcd.clear();
          lcd.setCursor(0, 0);
          lcd.print(F("Detect SW #"));
          lcd.print(packetBuffer[4]);
          lcd.setCursor(0, 1);
          lcd.print(DetectedRemoteSw [hexToDecBy4bit(packetBuffer[4])] [0]);
          lcd.print(F("."));
          lcd.print(DetectedRemoteSw [hexToDecBy4bit(packetBuffer[4])] [1]);
          lcd.print(F("."));
          lcd.print(DetectedRemoteSw [hexToDecBy4bit(packetBuffer[4])] [2]);
          lcd.print(F("."));
          lcd.print(DetectedRemoteSw [hexToDecBy4bit(packetBuffer[4])] [3]);
          lcd.print(F(":"));
          lcd.print(DetectedRemoteSw [hexToDecBy4bit(packetBuffer[4])] [4]);
          delay(4000);
          LcdNeedRefresh = true;
          lcd.clear();

          #if defined(SERIAL_debug)
          if(DEBUG==1){
            Serial.println();
            Serial.print(F("RX b:s"));
            Serial.print(packetBuffer[3]);
            Serial.print(hexToDecBy4bit(packetBuffer[4]), HEX);
            Serial.println(F(";"));

            // Serial.print(packetBuffer[4], DEC);
            // Serial.println(" ");
            // Serial.println((int)packetBuffer[4] - 48, DEC);

            for (int i = 0; i < 16; i++) {
              Serial.print(i);
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
        }

        //- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
        // Switch ANSWER > SET LED - Bank0-2
        if (packetBuffer[0] == 'm' && packetBuffer[1] == ':'){
          RemoteSwLatency[1] = (millis()-RemoteSwLatency[0])/2; // set latency (half path in ms us/2/1000)
          RemoteSwLatencyAnsw = 1;           // answer packet received

          // if (ACC_KEYBOARD==1 && KeyboardAnswLed==1 && HowRemoteSwitchID()<7){ // and ID < 7 (bank A/B)
            // Serial2.print("Remote IP switch ID: ");
            // Serial2.println(HowRemoteSwitchID());

            // need if RX answer from band change query
            // rxShiftInButton[0] = packetBuffer[2];
            // rxShiftInButton[1] = packetBuffer[3];
            // rxShiftInButton[2] = packetBuffer[4];

            // Serial.print((byte)packetBuffer[2], BIN);
            // Serial.print(" rxUDP   ");
            // Serial.println(millis());
            // Serial.println();

            byte ButtonSequence = 0;
            // 4bit shift left OR 4bit shift right = 4bit shift rotate
            ButtonSequence = (byte)packetBuffer[2] >> 4 | (byte)packetBuffer[2] << 4;

            digitalWrite(ShiftOutLatchPin, LOW);  // ready for receive data
            // shiftOut(ShiftOutDataPin, ShiftOutClockPin, LSBFIRST, packetBuffer[4]);    // encoder
            // shiftOut(ShiftOutDataPin, ShiftOutClockPin, LSBFIRST, packetBuffer[3]);    // encoder
            shiftOut(ShiftOutDataPin, ShiftOutClockPin, LSBFIRST, ButtonSequence);    // buttons
            shiftOut(ShiftOutDataPin, ShiftOutClockPin, LSBFIRST, ButtonSequence);    // buttons
            digitalWrite(ShiftOutLatchPin, HIGH);    // switch to output pin
          // }

          #if defined(SERIAL_debug)
          if(DEBUG==1){
            Serial.print(F("RX m:"));
            Serial.print((byte)packetBuffer[2], BIN);
            Serial.print(F("|"));
            Serial.print((byte)packetBuffer[3], BIN);
            Serial.print(F("|"));
            Serial.print((byte)packetBuffer[4], BIN);
            Serial.print(packetBuffer[5]);
            Serial.print(F(" > Latency: "));
            Serial.println(RemoteSwLatency[1]);
          }
          #endif
          LcdNeedRefresh = true;
        }

        memset(packetBuffer, 0, sizeof(packetBuffer));   // Clear contents of Buffer
      }

    InterruptON(1,1); // ptt, enc
  #endif
}
//-------------------------------------------------------------------------------------------------------

void TxUDP(){
  #if defined(EthModule)
    // if detect ip relay
    if(DetectedRemoteSw[BOARD_ID][4]!=0 && PTT==false){
      RemoteSwIP = DetectedRemoteSw[BOARD_ID];
      RemoteSwPort = DetectedRemoteSw[BOARD_ID][4];

      // UDP send to Switch
      // TxUdpBuffer[0] = B01100100;        // d
      // TxUdpBuffer[0] = B01110011;        // s

      TxUdpBuffer[0] = B01101101;           // m
      TxUdpBuffer[1] = B00111010;           // :
      TxUdpBuffer[2] = rxShiftInButton[0];  // buttons
      TxUdpBuffer[3] = rxShiftInButton[1];  // encoder
      TxUdpBuffer[4] = rxShiftInButton[2];  // encoder
      TxUdpBuffer[5] = B00111011;           // ;

      // Serial.print(TxUdpBuffer[2], BIN);
      // Serial.print(" txUDP   ");
      // Serial.println(millis());

      UdpCommand.beginPacket(RemoteSwIP, RemoteSwPort);
        UdpCommand.write(TxUdpBuffer, sizeof(TxUdpBuffer));   // send buffer
        RemoteSwLatency[0] = millis(); // set START time mark UDP command latency
      UdpCommand.endPacket();
      RemoteSwLatencyAnsw = 0;   // send command, wait to answer

      #if defined(SERIAL_debug)
      if(DEBUG==1){
        Serial.print(F("TX "));
        Serial.print(RemoteSwIP);
        Serial.print(F(":"));
        Serial.print(RemoteSwPort);
        Serial.print(F(" m:"));
        Serial.print(TxUdpBuffer[2], HEX);
        Serial.print(F("|"));
        Serial.print(TxUdpBuffer[3], HEX);
        Serial.print(F("|"));
        Serial.print(TxUdpBuffer[4], HEX);
        Serial.println(F(";"));
      }
      #endif
    }
  #endif
}
//---------------------------------------------------------------------------------------------------------

void PttDetector(){   // call from interupt
  digitalWrite(PttOffPin, HIGH);
  PTT = true;
  #if defined(LCD)
    LcdNeedRefresh = true;
  #endif
  PttTiming[0]=millis();
}
//-------------------------------------------------------------------------------------------------------

void EthernetCheck(){
  if(millis()-EthLinkStatusTimer[0]>EthLinkStatusTimer[1] && EnableEthernet==1){
    if ((Ethernet.linkStatus() == Unknown || Ethernet.linkStatus() == LinkOFF) && EthLinkStatus==1) {
      EthLinkStatus=0;
      #if defined(SERIAL_debug)
      if(DEBUG==1){
        Serial.println(F("Ethernet DISCONNECTED"));
      }
      #endif
    }else if (Ethernet.linkStatus() == LinkON && EthLinkStatus==0) {
      EthLinkStatus=1;
      #if defined(SERIAL_debug)
      if(DEBUG==1){
        Serial.println(F("Ethernet CONNECTED"));
      }
      #endif

      lcd.clear();
      lcd.setCursor(1, 0);
      lcd.print(F("Net-ID: "));
      lcd.print(BOARD_ID);
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

        delay(2000);
        lcd.clear();
        lcd.setCursor(1, 0);
        lcd.print(F("IP address:"));
        lcd.setCursor(1, 1);
        lcd.print(Ethernet.localIP());
        delay(2500);
        lcd.clear();

      server.begin();                     // Web
      UdpCommand.begin(UdpCommandPort);   // UDP
      SendBroadcastUdp();

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
  if(PTT==true && millis()-PttTiming[0] > PttTiming[1] && digitalRead(PttDetectorPin)==HIGH && BAND!=0){

  #if defined(EthModule)
    if(DetectedRemoteSw[BOARD_ID][4]!=0 && RemoteSwLatencyAnsw==1){
  #endif

    digitalWrite(PttOffPin, LOW);
    #if defined(EthModule) && defined(UdpBroadcastDebug_debug)
      TxBroadcastUdp("PttOff-" + String(DetectedRemoteSw[BOARD_ID][4]) + "-" + String(RemoteSwLatencyAnsw) );
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

void bandSET() {                                               // set outputs by BAND variable
  if(BAND==0 && previousBAND != 0){    // deactivate PTT
    digitalWrite(PttOffPin, HIGH);
    PTT = true;
    #if defined(LCD)
      LcdNeedRefresh = true;
    #endif
  }


  if((PTT==false && previousBAND != 0 ) || (PTT==true && previousBAND == 0)){
    ShiftByte[0] = B00000000;
    ShiftByte[1] = B00000000;

    if(BAND > 0 && BAND < 9){
      ShiftByte[0] = ShiftByte[0] | (1<<BAND-1);    // Set the n-th bit
    }
    if(BAND > 7 && BAND < 17){
      ShiftByte[1] = ShiftByte[1] | (1<<BAND-9);    // Set the n-th bit
    }
    digitalWrite(ShiftOutLatchPin, LOW);    // ready for receive data
    if(NumberOfBoards > 1){ shiftOut(ShiftOutDataPin, ShiftOutClockPin, LSBFIRST, ShiftByte[1]); }
                            shiftOut(ShiftOutDataPin, ShiftOutClockPin, LSBFIRST, ShiftByte[0]);
    digitalWrite(ShiftOutLatchPin, HIGH);    // switch to output pin

      #if !defined(YAESU_BCD)
          bcdOut();
      #endif
      #if defined(LCD)
        LcdNeedRefresh = true;
      #endif
  }

  #if defined(EthModule)
    if(DetectedRemoteSw[BOARD_ID][4]==0 || RemoteSwLatencyAnsw==0){
      //  && millis() < RemoteSwLatency[0]+RemoteSwLatency[1]*5) ){
      digitalWrite(PttOffPin, HIGH);
      #if defined(UdpBroadcastDebug_debug)
        TxBroadcastUdp("BandSet-" + String(DetectedRemoteSw[BOARD_ID][4]) + "-" + String(RemoteSwLatencyAnsw) );
      #endif
      PTT = true;
      #if defined(LCD)
      LcdNeedRefresh = true;
      #endif
    }
  #endif
  TxUDP();

  previousBAND = BAND;
  #if defined(WATCHDOG)
    WatchdogTimeout[0] = millis();                   // set time mark
  #endif
}
//---------------------------------------------------------------------------------------------------------

void remoteRelay() {
    Serial.print(1);
    Serial.print(',');
    Serial.print(BAND, DEC);
    Serial.print('\n');
    Serial.flush();
}
//---------------------------------------------------------------------------------------------------------

void serialEcho() {
    Serial.print("<");
    Serial.print(BAND);
    Serial.print(",");
    Serial.print(freq);
    Serial.println(">");
    Serial.flush();
}
//---------------------------------------------------------------------------------------------------------

#if !defined(YAESU_BCD)
    void bcdOut(){
        if (BCDmatrixOUT[0][BAND] == 1){ digitalWrite(BcdIn1Pin, HIGH); }else{ digitalWrite(BcdIn1Pin, LOW);}
        if (BCDmatrixOUT[1][BAND] == 1){ digitalWrite(BcdIn2Pin, HIGH); }else{ digitalWrite(BcdIn2Pin, LOW);}
        if (BCDmatrixOUT[2][BAND] == 1){ digitalWrite(BcdIn3Pin, HIGH); }else{ digitalWrite(BcdIn3Pin, LOW);}
        if (BCDmatrixOUT[3][BAND] == 1){ digitalWrite(BcdIn4Pin, HIGH); }else{ digitalWrite(BcdIn4Pin, LOW);}
    }
#endif
//---------------------------------------------------------------------------------------------------------

void watchDog() {
  #if defined(WATCHDOG)
    if((millis() - WatchdogTimeout[0]) > WatchdogTimeout[1]) {
      BAND=0;
      freq=0;
      bandSET();                                 // set outputs
    }
  #endif
}
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

void FreqToBandRules(){
         if (freq >=Freq2Band[0][0] && freq <=Freq2Band[0][1] )  {BAND=1;}  // 160m
    else if (freq >=Freq2Band[1][0] && freq <=Freq2Band[1][1] )  {BAND=2;}  //  80m
    else if (freq >=Freq2Band[2][0] && freq <=Freq2Band[2][1] )  {BAND=3;}  //  40m
    else if (freq >=Freq2Band[3][0] && freq <=Freq2Band[3][1] )  {BAND=4;}  //  30m
    else if (freq >=Freq2Band[4][0] && freq <=Freq2Band[4][1] )  {BAND=5;}  //  20m
    else if (freq >=Freq2Band[5][0] && freq <=Freq2Band[5][1] )  {BAND=6;}  //  17m
    else if (freq >=Freq2Band[6][0] && freq <=Freq2Band[6][1] )  {BAND=7;}  //  15m
    else if (freq >=Freq2Band[7][0] && freq <=Freq2Band[7][1] )  {BAND=8;}  //  12m
    else if (freq >=Freq2Band[8][0] && freq <=Freq2Band[8][1] )  {BAND=9;}  //  10m
    else if (freq >=Freq2Band[9][0] && freq <=Freq2Band[9][1] ) {BAND=10;}  //   6m
    else if (freq >=Freq2Band[10][0] && freq <=Freq2Band[10][1] ) {BAND=11;}  //   2m
    else {BAND=0;}                                                // out of range
}
