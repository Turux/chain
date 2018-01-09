
#include canSTV.h

/**Modifies several registers that determine which methods will be processed by the receive buffers*/
void setCanStatus()
{
  Serial.println("sCANStat:entr");
  /* Skipping configuraton
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
  int modeset = CAN.Mode(MCP2515_CONFIG);
  if (DEBUG == true)
  {
    Serial.println("Cfg MCP2515 Reg");
    Serial.print("Mode set: ");
    Serial.println(modeset);
  }


  //Enable reception of all messages in buffer 0.
  byte value = B01100100;
  CAN.Write(RXB0CTRL, value);
  if (DEBUG == true)
  {
    Serial.print("RXB0CTRL: ");
    Serial.println( CAN.Read(RXB0CTRL), BIN);
  }

  //enable reception of all messages in buffer 1
  value = B01100000;
  CAN.Write(RXB1CTRL, value);
  if (DEBUG == true)
  {
    Serial.print("RXB1CTRL: ");
    Serial.println( CAN.Read(RXB1CTRL), BIN);
  }

  //Set the interrupt enable flags
  value = B00000011;
  byte mask = B11111111;
  CAN.BitModify(CANINTE, mask, value);

  //Reset all interrupt flags
  value = B00000000;
  CAN.BitModify(CANINTF, mask, value);

  //finally set mode to listen only mode
  modeset = CAN.Mode(MCP2515_LISTEN);
  if (DEBUG == true)
  {
    Serial.print("Mode set: ");
    Serial.println(modeset);
  }
}
