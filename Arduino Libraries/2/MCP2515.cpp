/*
  MCP2515.cpp - Library for Microchip MCP2515 CAN Controller
  
  Author: David Harding
  
  Created: 11/08/2010
  
  For further information see:
  
  http://ww1.microchip.com/downloads/en/DeviceDoc/21801e.pdf
  http://en.wikipedia.org/wiki/CAN_bus
*/

#include "SPI.h"
#include "MCP2515.h"
#include "MCP2515_defs.h"
#include <SD.h>


MCP2515::MCP2515(byte CS_Pin, byte INT_Pin) {
  pinMode(CS_Pin, OUTPUT);
  digitalWrite(CS_Pin,HIGH);

  pinMode(INT_Pin,INPUT);
  digitalWrite(INT_Pin,HIGH);
  
  _CS = CS_Pin;
  _INT = INT_Pin;
}

/*
  Initialize MCP2515
  
  int CAN_Bus_Speed = transfer speed in kbps
  int Freq = MCP2515 oscillator frequency in MHz
  int SJW = Synchronization Jump Width Length bits - 1 to 4 (see data sheet)
  
  returns baud rate set
  
  Sending a bus speed of 0 kbps initiates AutoBaud and returns zero if no
  baud rate could be determined.  There must be two other active nodes on the bus!
*/
int MCP2515::Init(int CAN_Bus_Speed, byte Freq) {
  if(CAN_Bus_Speed>0) {
    if(_init(CAN_Bus_Speed, Freq, 1, false)) return CAN_Bus_Speed;
  } else {
      int i=0;
      byte interruptFlags = 0;
      for(i=5; i<1000; i=i+5) {
        if(_init(i, Freq, 1, true)) {
            // check for bus activity
            Write(CANINTF,0);
            delay(500); // need the bus to be communicating within this time frame
            if(Interrupt()) {
              // determine which interrupt flags have been set
              interruptFlags = Read(CANINTF);
              if(!(interruptFlags & MERRF)) {
                // to get here we must have received something without errors
                Mode(MODE_NORMAL);
                  return i;
              }
            }
        }
      }
  }
  return 0;
}

int MCP2515::Init(int CAN_Bus_Speed, byte Freq, byte SJW) {
  if(SJW < 1) SJW = 1;
  if(SJW > 4) SJW = 4;
  if(CAN_Bus_Speed>0) {
    if(_init(CAN_Bus_Speed, Freq, SJW, false)) return CAN_Bus_Speed;
  } else {
      int i=0;
      byte interruptFlags = 0;
      for(i=5; i<1000; i=i+5) {
        if(_init(i, Freq, SJW, true)) {
            // check for bus activity
            Write(CANINTF,0);
            delay(500); // need the bus to be communicating within this time frame
            if(Interrupt()) {
              // determine which interrupt flags have been set
              interruptFlags = Read(CANINTF);
              if(!(interruptFlags & MERRF)) {
                // to get here we must have received something without errors
                Mode(MODE_NORMAL);
                  return i;
              }
            }
        }
      }
  }
  return 0;
}

bool MCP2515::_init(int CAN_Bus_Speed, byte Freq, byte SJW, bool autoBaud) {
  
  // Reset MCP2515 which puts it in configuration mode
  Reset();
  
  // Calculate bit timing registers
  byte BRP;
  float TQ;
  byte BT;
  float tempBT;

  float NBT = 1.0 / (float)CAN_Bus_Speed * 1000.0; // Nominal Bit Time
  for(BRP=0;BRP<8;BRP++) {
    TQ = 2.0 * (float)(BRP + 1) / (float)Freq;
    tempBT = NBT / TQ;
      if(tempBT<=25) {
        BT = (int)tempBT;
        if(tempBT-BT==0) break;
      }
  }
  
  byte SPT = (0.7 * BT); // Sample point
  byte PRSEG = (SPT - 1) / 2;
  byte PHSEG1 = SPT - PRSEG - 1;
  byte PHSEG2 = BT - PHSEG1 - PRSEG - 1;

  // Programming requirements
  if(PRSEG + PHSEG1 < PHSEG2) return false;
  if(PHSEG2 <= SJW) return false;
  
  byte BTLMODE = 1;
  byte SAM = 0;
  
  // Set registers
  byte data = (((SJW-1) << 6) | BRP);
  Write(CNF1, data);
  Write(CNF2, ((BTLMODE << 7) | (SAM << 6) | ((PHSEG1-1) << 3) | (PRSEG-1)));
  Write(CNF3, (B10000000 | (PHSEG2-1)));
  Write(TXRTSCTRL,0);
  
  if(!autoBaud) {
    // Return to Normal mode
      if(!Mode(MODE_NORMAL)) return false;
  } else {
    // Set to Listen Only mode
      if(!Mode(MODE_LISTEN)) return false;
  }
  // Enable all interupts
  Write(CANINTE,255);
  
  // Test that we can read back from the MCP2515 what we wrote to it
  byte rtn = Read(CNF1);
  return (rtn==data);
}

void MCP2515::Reset() {
  digitalWrite(_CS,LOW);
  SPI.transfer(CAN_RESET);
  digitalWrite(_CS,HIGH);
}

byte MCP2515::Read(byte address) {
  digitalWrite(_CS,LOW);
  SPI.transfer(CAN_READ);
  SPI.transfer(address);
  byte data = SPI.transfer(0x00);
  digitalWrite(_CS,HIGH);
  return data;
}

void MCP2515::Read(byte address, byte data[], byte bytes) {
  // allows for sequential reading of registers starting at address - see data sheet
  byte i;
  digitalWrite(_CS,LOW);
  SPI.transfer(CAN_READ);
  SPI.transfer(address);
  for(i=0;i<bytes;i++) {
    data[i] = SPI.transfer(0x00);
  }
  digitalWrite(_CS,HIGH);
}

Frame MCP2515::ReadBuffer(byte buffer) {
 
  // Reads an entire RX buffer.
  // buffer should be either RXB0 or RXB1
  
  Frame message;
  
  digitalWrite(_CS,LOW);
  SPI.transfer(CAN_READ_BUFFER | (buffer<<1));
  byte byte1 = SPI.transfer(0x00); // RXBnSIDH
  byte byte2 = SPI.transfer(0x00); // RXBnSIDL
  byte byte3 = SPI.transfer(0x00); // RXBnEID8
  byte byte4 = SPI.transfer(0x00); // RXBnEID0
  byte byte5 = SPI.transfer(0x00); // RXBnDLC

  message.srr=(byte2 & B00010000);
  message.ide=(byte2 & B00001000);

  if(message.ide) {
    message.id = (byte1>>3);
    message.id = (message.id<<8) | ((byte1<<5) | ((byte2>>5)<<2) | (byte2 & B00000011));
    message.id = (message.id<<8) | byte3;
    message.id = (message.id<<8) | byte4;
  } else {
    message.id = ((byte1>>5)<<8) | ((byte1<<3) | (byte2>>5));
  }

  message.rtr=(byte5 & B01000000);
  message.dlc = (byte5 & B00001111);  // Number of data bytes
  for(int i=0;i<message.dlc;i++) {
    message.data[i] = SPI.transfer(0x00);
  }
  digitalWrite(_CS,HIGH);

  return message;
}

void MCP2515::Write(byte address, byte data) {
  digitalWrite(_CS,LOW);
  SPI.transfer(CAN_WRITE);
  SPI.transfer(address);
  SPI.transfer(data);
  digitalWrite(_CS,HIGH);
}

void MCP2515::Write(byte address, byte data[], byte bytes) {
  // allows for sequential writing of registers starting at address - see data sheet
  byte i;
  digitalWrite(_CS,LOW);
  SPI.transfer(CAN_WRITE);
  SPI.transfer(address);
  for(i=0;i<bytes;i++) {
    SPI.transfer(data[i]);
  }
  digitalWrite(_CS,HIGH);
}

void MCP2515::SendBuffer(byte buffers) {
  // buffers should be any combination of TXB0, TXB1, TXB2 ORed together, or TXB_ALL
  digitalWrite(_CS,LOW);
  SPI.transfer(CAN_RTS | buffers);
  digitalWrite(_CS,HIGH);
}

void MCP2515::LoadBuffer(byte buffer, Frame message) {
 
  // buffer should be one of TXB0, TXB1 or TXB2
  if(buffer==TXB0) buffer = 0;

  byte byte1=0; // TXBnSIDH
  byte byte2=0; // TXBnSIDL
  byte byte3=0; // TXBnEID8
  byte byte4=0; // TXBnEID0
  byte byte5=0; // TXBnDLC

  if(message.ide) {
    byte1 = byte((message.id<<3)>>24); // 8 MSBits of SID
      byte2 = byte((message.id<<11)>>24) & B11100000; // 3 LSBits of SID
      byte2 = byte2 | byte((message.id<<14)>>30); // 2 MSBits of EID
      byte2 = byte2 | B00001000; // EXIDE
    byte3 = byte((message.id<<16)>>24); // EID Bits 15-8
    byte4 = byte((message.id<<24)>>24); // EID Bits 7-0
  } else {
    byte1 = byte((message.id<<21)>>24); // 8 MSBits of SID
      byte2 = byte((message.id<<29)>>24) & B11100000; // 3 LSBits of SID
    byte3 = 0; // TXBnEID8
    byte4 = 0; // TXBnEID0
  }
  byte5 = message.dlc;
  if(message.rtr) {
    byte5 = byte5 | B01000000;
  }
  
  digitalWrite(_CS,LOW);
  SPI.transfer(CAN_LOAD_BUFFER | buffer);  
  SPI.transfer(byte1);
  SPI.transfer(byte2);
  SPI.transfer(byte3);
  SPI.transfer(byte4);
  SPI.transfer(byte5);
 
  for(int i=0;i<message.dlc;i++) {
    SPI.transfer(message.data[i]);
  }
  digitalWrite(_CS,HIGH);
}

byte MCP2515::Status() {
  digitalWrite(_CS,LOW);
  SPI.transfer(CAN_STATUS);
  byte data = SPI.transfer(0x00);
  digitalWrite(_CS,HIGH);
  return data;
  /*
  bit 7 - CANINTF.TX2IF
  bit 6 - TXB2CNTRL.TXREQ
  bit 5 - CANINTF.TX1IF
  bit 4 - TXB1CNTRL.TXREQ
  bit 3 - CANINTF.TX0IF
  bit 2 - TXB0CNTRL.TXREQ
  bit 1 - CANINTFL.RX1IF
  bit 0 - CANINTF.RX0IF
  */
}

byte MCP2515::RXStatus() {
  digitalWrite(_CS,LOW);
  SPI.transfer(CAN_RX_STATUS);
  byte data = SPI.transfer(0x00);
  digitalWrite(_CS,HIGH);
  return data;
  /*
  bit 7 - CANINTF.RX1IF
  bit 6 - CANINTF.RX0IF
  bit 5 - 
  bit 4 - RXBnSIDL.EIDE
  bit 3 - RXBnDLC.RTR
  bit 2 | 1 | 0 | Filter Match
  ------|---|---|-------------
      0 | 0 | 0 | RXF0
        0 | 0 | 1 | RXF1
        0 | 1 | 0 | RXF2
        0 | 1 | 1 | RXF3
        1 | 0 | 0 | RXF4
        1 | 0 | 1 | RXF5
        1 | 1 | 0 | RXF0 (rollover to RXB1)
        1 | 1 | 1 | RXF1 (rollover to RXB1)
  */
}

void MCP2515::BitModify(byte address, byte mask, byte data) {
  // see data sheet for explanation
  digitalWrite(_CS,LOW);
  SPI.transfer(CAN_BIT_MODIFY);
  SPI.transfer(address);
  SPI.transfer(mask);
  SPI.transfer(data);
  digitalWrite(_CS,HIGH);
}

bool MCP2515::Interrupt() {
  return (digitalRead(_INT)==LOW);
}

bool MCP2515::Mode(byte mode) {
  /*
  mode can be one of the following:
  MODE_CONFIG
  MODE_LISTEN
  MODE_LOOPBACK
  MODE_SLEEP
  MODE_NORMAL
  */
  BitModify(CANCTRL, B11100000, mode);
  delay(10); // allow for any transmissions to complete
  byte data = Read(CANSTAT); // check mode has been set
  return ((data & mode)==mode);
}

bool MCP2515::initCAN()
{
  // Progress statement
  Serial.println("initCan:entry");
  //Setup not complete yet.
  boolean setupSuccess = false;
  //Initialize CAN shield
  pinMode(_CS, OUTPUT);

  // Initialize MCP2515 CAN controller at the specified speed and clock frequency.
  // Entering 0 as the first argument  to  init request automatic CAN bus speed detection
  // In this case 500 kbps
  int baudRate = Init(1000, 16);
  //Pause for a second
  delay(1000);
  if (baudRate > 0)
  {
    //Print current progress
    Serial.println("MCP2515 init OK");
    Serial.print("Baud: ");
    Serial.println(baudRate, DEC);
    delay(1000);

    //Print CAN bus status
    if (DEBUG == true)
    {
      Serial.print("CAN Stat: ");
      displayCanStatus();
      Serial.println(" ");
    }

    setCanStatus();
    setupSuccess = true;
  }
  else
  {
    Serial.println("MCP2515 Init Fail");
  }
  return setupSuccess;
}

void MCP2515::setCanStatus(){
  Serial.println("sCANStat:entr");
  /*
    // MCP2515 SPI Commands
   #define CAN_RESET	0xC0
   #define CAN_READ	0x03
   #define CAN_WRITE	0x02
   #define CAN_RTS	   	0x80
   #define CAN_STATUS	0xA0
   #define CAN_BIT_MODIFY  0x05
   #define CAN_RX_STATUS   0xB0
   #define CAN_READ_BUFFER 0x90
   #define CAN_LOAD_BUFFER 0X40
   
   // Register Bit Masks
   
   // CANSTAT
   #define MODE_CONFIG	0x80
   #define MODE_LISTEN	0x60
   #define MODE_LOOPBACK	0x40
   #define MODE_SLEEP	0x20
   #define MODE_NORMAL	0x00
   
   // CANINTF
   #define RX0IF		0x01
   #define RX1IF		0x02
   #define TX0IF		0x04
   #define TX1IF		0x08
   #define TX2IF		0x10
   #define ERRIF		0x20
   #define WAKIF		0x40
   #define MERRF		0x80
   
   // Configuration Registers
   #define CANSTAT	   	0x0E
   #define CANCTRL	   	0x0F
   #define BFPCTRL	   	0x0C
   #define TEC		0x1C
   #define REC		0x1D
   #define CNF3		0x28
   #define CNF2		0x29
   #define CNF1		0x2A
   #define CANINTE	   	0x2B
   #define CANINTF	   	0x2C
   #define EFLG		0x2D
   #define TXRTSCTRL	0x0D
   
   // RX Buffer 0
   #define RXB0CTRL	0x60
   #define RXB0SIDH	0x61
   #define RXB0SIDL	0x62
   #define RXB0EID8	0x63
   #define RXB0EID0	0x64
   #define RXB0DLC	   	0x65
   #define RXB0D0	    	0x66
   #define RXB0D1	    	0x67
   #define RXB0D2	    	0x68
   #define RXB0D3	    	0x69
   #define RXB0D4	    	0x6A
   #define RXB0D5	    	0x6B
   #define RXB0D6	    	0x6C
   #define RXB0D7	    	0x6D
   
   // RX Buffer 1
   #define RXB1CTRL	0x70
   #define RXB1SIDH	0x71
   #define RXB1SIDL	0x72
   #define RXB1EID8	0x73
   #define RXB1EID0	0x74
   #define RXB1DLC	   	0x75
   #define RXB1D0	    	0x76
   #define RXB1D1	    	0x77
   #define RXB1D2	    	0x78
   #define RXB1D3	    	0x79
   #define RXB1D4	    	0x7A
   #define RXB1D5	    	0x7B
   #define RXB1D6	    	0x7C
   #define RXB1D7	    	0x7D
   */

  //set configuraton mode
  int modeset = Mode(0x80); //MCP2515_CONFIG = 0x80;
  if (DEBUG == true)
  {
    Serial.println("Cfg MCP2515 Reg");
    Serial.print("Mode set: ");
    Serial.println(modeset);
  }


  //Enable reception of all messages in buffer 0.
  byte value = B01100100;
  Write(RXB0CTRL, value);
  if (DEBUG == true)
  {
    Serial.print("RXB0CTRL: ");
    Serial.println( Read(RXB0CTRL), BIN);
  }

  //enable reception of all messages in buffer 1
  value = B01100000;
  Write(RXB1CTRL, value);
  if (DEBUG == true)
  {
    Serial.print("RXB1CTRL: ");
    Serial.println( Read(RXB1CTRL), BIN);
  }

  //Set the interrupt enable flags
  value = B00000011;
  byte mask = B11111111;
  BitModify(CANINTE, mask, value);

  //Reset all interrupt flags
  value = B00000000;
  BitModify(CANINTF, mask, value);

  //finally set mode to listen only mode
  modeset = Mode(0x60); //MCP2515_LISTEN = 0x60;
  if (DEBUG == true)
  {
    Serial.print("Mode set: ");
    Serial.println(modeset);
  }
}

void MCP2515::displayCanStatus(void)
{
  Serial.println("displayCANStatus:entry");
  //Display CAN Status bits
  /*
    bit 7 - CANINTF.TX2IF
   bit 6 - TXB2CNTRL.TXREQ
   bit 5 - CANINTF.TX1IF
   bit 4 - TXB1CNTRL.TXREQ
   bit 3 - CANINTF.TX0IF
   bit 2 - TXB0CNTRL.TXREQ
   bit 1 - CANINTF.RX1IF
   bit 0 - CANINTF.RX0IF
   */
  Serial.print("CAN Status: ");
  Serial.println(Status(), BIN);

  //DISPLAY RX status bits
  /*
  bit 7 - CANINTF.RX1IF
   bit 6 - CANINTF.RX0IF
   bit 5 -
   bit 4 - RXBnSIDL.EIDE
   bit 3 - RXBnDLC.RTR
   bit 2 | 1 | 0 | Filter Match
   ------|---|---|-------------
   	0 | 0 | 0 | RXF0
   	0 | 0 | 1 | RXF1
   	0 | 1 | 0 | RXF2
   	0 | 1 | 1 | RXF3
   	1 | 0 | 0 | RXF4
   	1 | 0 | 1 | RXF5
   	1 | 1 | 0 | RXF0 (rollover to RXB1)
   	1 | 1 | 1 | RXF1 (rollover to RXB1)
   */
  Serial.print("RX Status: ");
  Serial.println(RXStatus(), BIN);

  byte helper = Read(CANCTRL);
  Serial.print("CANTRL: ");
  Serial.println(helper, BIN);

  helper = Read(CANSTAT);
  Serial.print("CANSTAT: ");
  Serial.println(helper, BIN);

  Serial.println();
  helper = Read(RXB0CTRL);
  Serial.print("RXB0CTRL: ");
  Serial.println(helper, BIN);

  helper = Read(RXB0SIDL);
  Serial.print("RFX0SIDL: ");
  Serial.println(helper, BIN);

  helper = Read(RXB0EID8);
  Serial.print("RXB0EID8: ");
  Serial.println(helper, BIN);

  helper = Read(RXB0EID0);
  Serial.print("RXB0EID0: ");
  Serial.println(helper, BIN);
  Serial.println();

  helper = Read(RXB1CTRL);
  Serial.print("RXB1CTRL: ");
  Serial.println(helper, BIN);

  helper = Read(RXB1SIDL);
  Serial.print("RXB1SIDL: ");
  Serial.println(helper, BIN);

  helper = Read(RXB1EID8);
  Serial.print("RXB1EID8: ");
  Serial.println(helper, BIN);

  helper = Read(RXB1EID0);
  Serial.print("RXB1EID0: ");
  Serial.println(helper, BIN);
  Serial.println();

  helper = Read(BFPCTRL);
  Serial.print("BFPCTRL: ");
  Serial.println(helper, BIN);

  helper = Read(CANINTE);
  Serial.print("CANINTE: ");
  Serial.println(helper, BIN);

  helper = Read(CANINTF);
  Serial.print("CANINTF: ");
  Serial.println(helper, BIN);
}

bool MCP2515::initSD(void)
{
  pinMode(_CS, OUTPUT);
  Serial.println("initSD:entry");
  boolean setupSuccess = false;
  // Initialize the SD card SPI CS Pin
  pinMode(SD_CHIP_SELECT, OUTPUT);
  //digitalWrite(SD_CHIP_SELECT, HIGH);   // Turns Sd Card communication off
  //Check whether SD card initialization is successful
  if (!SD.begin(SD_CHIP_SELECT))
  {
    Serial.println("Card failed");
  }
  else
  {
    Serial.println("Card OK");
    setupSuccess = true;
  }
  return setupSuccess;
}

void MCP2515::initSPI(void)
{
  Serial.println("initSPI:entry");
  // Set up SPI Communication
  // dataMode can be SPI_MODE0 or SPI_MODE3 only for MCP2515
  SPI.setClockDivider(SPI_CLOCK_DIV2);
  SPI.setDataMode(SPI_MODE0);
  SPI.setBitOrder(MSBFIRST);
  SPI.begin();
}

/**Determines whether a set amount of time (wait_time) has expired since a given start_time*/
bool MCP2515::hasTimeElapsed( unsigned long start_time, unsigned long wait_time)
{
  boolean ret_value = false;
  unsigned long now = millis();
  if ( now < start_time + wait_time)
  {
    ret_value = false;
  }
  else
  {
    ret_value = true;
  }
  return ret_value;
}
