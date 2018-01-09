/*
  MCP2515.h - Library for Microchip MCP2515 CAN Controller
  
  Author: David Harding
  
  Created: 11/08/2010
  
  For further information see:
  
  http://ww1.microchip.com/downloads/en/DeviceDoc/21801e.pdf
  http://en.wikipedia.org/wiki/CAN_bus
*/

#define DEBUG 1
#define SD_CHIP_SELECT 9
#define LIGHT_CAN  7
#ifndef MCP2515_h
#define MCP2515_h

#include "MCP2515_defs.h"

class MCP2515
{
  public:
      // Constructor defining which pins to use for CS and INT
    MCP2515(byte CS_Pin, byte INT_Pin);
      
      // Overloaded initialization function
      int Init(int baud, byte freq);
      int Init(int baud, byte freq, byte sjw);
      
      // Basic MCP2515 SPI Command Set
    void Reset();
    byte Read(byte address);
    void Read(byte address, byte data[], byte bytes);
      Frame ReadBuffer(byte buffer);
      void Write(byte address, byte data);
      void Write(byte address, byte data[], byte bytes);
      void LoadBuffer(byte buffer, Frame message);
      void SendBuffer(byte buffers);
      byte Status();
      byte RXStatus();
      void BitModify(byte address, byte mask, byte data);
	  bool initCAN(void);

      // Extra functions
      bool Interrupt(); // Expose state of INT pin
      bool Mode(byte mode); // Returns TRUE if mode change successful
      void setCanStatus();
	  void displayCanStatus(void);
	  bool initSD(void);
	  void initSPI(void);
	  bool hasTimeElapsed( unsigned long start_time, unsigned long wait_time);
  private:
      bool _init(int baud, byte freq, byte sjw, bool autoBaud);
    // Pin variables
      byte _CS;
      byte _INT;
};

#endif