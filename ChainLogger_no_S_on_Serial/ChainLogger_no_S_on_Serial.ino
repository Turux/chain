#include <SD.h>
#include <SPI.h>
#include <MCP2515.h>

/** CAN/J1939 logger based on the Arduino platform using a Sparkfun CAN-BUS shield.
 * Required Hardware: 1.) Arduino Duemilanove or similar, available at http://arduino.cc/en/Main/Buy
 * 2.) CAN Shield, available at http://www.sparkfun.com/products/10039
 * 3.) Any Micro SD Card
 * 4.) USB Cable and ODB-II cable
 * 
 * Required Software: 1.) Arduino Software 0023, available at http://www.arduino.cc/   
 * 2.) Arduino SD card library
 * 3.) Arduino SPI library    
 * 
 * Approach: Continuously record CAN/J1939 messages (optional: record messages that match filter criteria). Please not that the application has been configured to log all information accessible on the CAN bus and explicitly show 
 * individual processing steps. Significant performance improvements are possbile by using selective message filters and writing storage routines that use the bandwidth available at the SD card interface.
 */
#define summary

/*  The CAN shield select pin is 10 and the SD chip select is 9 
 Note that even if it's not used as the CS pin, the hardware CS pin (10 on most Arduino boards,
 53 on the Mega) must be left as an output or the SD library functions will not work.
 */
/* CAN_CHIP_SELECT assigned to pin 10*/
const int CAN_CHIP_SELECT      = 10;
/* SD_CHIP_SELECT assigned to pin 9*/
const int SD_CHIP_SELECT       = 9;
/* LIGHT_SD assigned to pin 8*/
const int LIGHT_SD       = 7;
/* SD_CHIP_SELECT assigned to pin 9*/
const int LIGHT_CAN      = 8;
/* CAN_INTERRUPT assigned to pin 2*/
const int CAN_INTERRUPT_PIN    = 2;

/*MCP2515 CONFIG state: Enables mask configuration etc.*/
const byte MCP2515_CONFIG =0x80;
/*MCP2515 LISTEN_ONLY state: Receives messages but does not send any*/
const byte MCP2515_LISTEN =0x60;
/*MCP2515 LOOP state: Enables sending/receiving of messages without using the CAN BUS*/
const byte MCP2515_LOOP   =0x40;
/*MCP2515 SLEEP state: Reduces power consumption*/
const byte MCP2515_SLEEP  =0x20;
/*MCP2515 NORMAL state: Participates as a regular node in the CAN network*/
const byte MCP2515_NORMAL =0x00;

/*File name of log file*/
char fileName[]     = "DATA00.txt";
/**Column headers for logged data*/
char header[]       = "Msg#,Time Diff, ID,DLC, Data";
/*File objet used to access the SD card*/
File myFile;

/*Object to interact with the MCP2515 directly*/
MCP2515 CAN( CAN_CHIP_SELECT, CAN_INTERRUPT_PIN);
/* CAN message frame exposed by the MCP2515 RX0 buffer*/
Frame message0;
/* CAN message frame exposed by the MCP2515 RX1 buffer*/
Frame message1;

//Various helper variables
int counter = 0;
int msgCount =0;

/*Time at which the last message was received*/
unsigned long timeLastMessageReceived = 0;

/*Time difference between the last two received messages*/
unsigned long timeDifference =0;
/*Indicates whether or not processing should be continued*/
boolean KEEPGOING = true;

/*Variable for temporary message assembly*/
String tmpMessage;

/**Debug variable controls whether or not debug/status information will bewritten to the serial console. Setting DEBUG to true has a negative performance impact.*/
boolean DEBUG = true;


/**Initialize the CAN shield*/
boolean initCAN(void)
{
  // Progress statement
  Serial.println("initCan:entry");  
  //Setup not complete yet.
  boolean setupSuccess = false;
  //Initialize CAN shield
  pinMode(CAN_CHIP_SELECT,OUTPUT);      

  // Initialize MCP2515 CAN controller at the specified speed and clock frequency. 
  // Entering 0 as the first argument  to CAN.init request automatic CAN bus speed detection
  // In this case 500 kbps
  int baudRate=CAN.Init(1000,16);
  //Pause for a second
  delay(1000);
  if(baudRate>0) 
  { 
    //Print current progress
    Serial.println("MCP2515 init OK");
    Serial.print("Baud: ");
    Serial.println(baudRate,DEC);
    delay(1000);

    //Print CAN bus status
    if(DEBUG ==true)
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

/**Modifies several registers that determine which methods will be processed by the receive buffers*/
void setCanStatus()
{
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
  int modeset = CAN.Mode(MCP2515_CONFIG);
  if(DEBUG ==true)
  {
    Serial.println("Cfg MCP2515 Reg");
    Serial.print("Mode set: ");
    Serial.println(modeset);
  }


  //Enable reception of all messages in buffer 0. 
  byte value= B01100100;
  CAN.Write(RXB0CTRL, value);
  if(DEBUG==true)
  {
    Serial.print("RXB0CTRL: ");
    Serial.println( CAN.Read(RXB0CTRL),BIN);
  }           

  //enable reception of all messages in buffer 1
  value = B01100000;
  CAN.Write(RXB1CTRL, value);
  if(DEBUG ==true)
  {
    Serial.print("RXB1CTRL: ");
    Serial.println( CAN.Read(RXB1CTRL),BIN);
  }

  //Set the interrupt enable flags    
  value = B00000011;
  byte mask = B11111111;
  CAN.BitModify(CANINTE,mask, value);

  //Reset all interrupt flags
  value = B00000000;
  CAN.BitModify(CANINTF, mask, value); 

  //finally set mode to listen only mode
  modeset = CAN.Mode(MCP2515_LISTEN);
  if(DEBUG == true)
  {
    Serial.print("Mode set: ");
    Serial.println(modeset);
  }
}
/**Initializes SD card communication*/
boolean initSD(void)
{
    pinMode(10, OUTPUT);
    
  Serial.println("initSD:entry");
  boolean setupSuccess =false;
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

/**Initialize SPI communication*/
void initSPI(void)
{
  Serial.println("initSPI:entry");
  // Set up SPI Communication
  // dataMode can be SPI_MODE0 or SPI_MODE3 only for MCP2515
  SPI.setClockDivider(SPI_CLOCK_DIV2);
  SPI.setDataMode(SPI_MODE0);
  SPI.setBitOrder(MSBFIRST);
  SPI.begin();
}

/**Method that provides summary information on the status of the MCP2515 chip*/
void displayCanStatus(void)
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
  Serial.println(CAN.Status(), BIN);

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
  Serial.println(CAN.RXStatus(), BIN);

  byte helper = CAN.Read(CANCTRL);  
  Serial.print("CANTRL: ");    
  Serial.println(helper, BIN);

  helper = CAN.Read(CANSTAT);
  Serial.print("CANSTAT: ");   
  Serial.println(helper, BIN);

  Serial.println();
  helper = CAN.Read(RXB0CTRL);   
  Serial.print("RXB0CTRL: ");  
  Serial.println(helper,BIN);

  helper= CAN.Read(RXB0SIDL);
  Serial.print("RFX0SIDL: ");  
  Serial.println(helper,BIN);

  helper= CAN.Read(RXB0EID8);
  Serial.print("RXB0EID8: ");  
  Serial.println(helper,BIN);

  helper= CAN.Read(RXB0EID0);
  Serial.print("RXB0EID0: ");  
  Serial.println(helper,BIN);
  Serial.println();

  helper = CAN.Read(RXB1CTRL);
  Serial.print("RXB1CTRL: ");  
  Serial.println(helper,BIN);

  helper= CAN.Read(RXB1SIDL);
  Serial.print("RXB1SIDL: ");  
  Serial.println(helper,BIN);

  helper= CAN.Read(RXB1EID8);
  Serial.print("RXB1EID8: ");  
  Serial.println(helper,BIN);

  helper= CAN.Read(RXB1EID0);
  Serial.print("RXB1EID0: ");  
  Serial.println(helper,BIN);
  Serial.println();

  helper = CAN.Read(BFPCTRL);
  Serial.print("BFPCTRL: ");   
  Serial.println(helper,BIN);

  helper = CAN.Read(CANINTE);
  Serial.print("CANINTE: ");   
  Serial.println(helper,BIN);

  helper = CAN.Read(CANINTF);
  Serial.print("CANINTF: ");   
  Serial.println(helper,BIN);
}

/**Initializes all components that will be used during the continuous loop*/
void setup()
{
  pinMode(LIGHT_SD,OUTPUT);
  pinMode(LIGHT_CAN,OUTPUT);
  boolean setupSuccess = true;
  Serial.begin(115200);
  Serial.println("Setup");
  pinMode(10, OUTPUT);
  pinMode(9, OUTPUT);
  //sd
  initSPI();
  setupSuccess = initSD();
  if (setupSuccess) preparaSD();
  delay(100);
  initSPI();


  //can
  setupSuccess &= initCAN(); 

  //

  if ( setupSuccess == true)
  {
    if(DEBUG== true)
    {
      displayCanStatus();
    }    
    Serial.println("log start");
  }


}

/**Determines the delta time between the last two messages*/
unsigned long getTimeDifference()
{
  //Serial.print("getTime Difference(): ");
  unsigned long current_time = millis();
  unsigned long result = current_time - timeLastMessageReceived;
  if( DEBUG == true )
  {
    tmpMessage = String("getTimeDifference(): ");
    tmpMessage += current_time;
    tmpMessage += " - ";
    tmpMessage += timeLastMessageReceived;
    tmpMessage +=  "=";
    tmpMessage += result;
    // Serial.println( tmpMessage );
  }  
  return ( result );
}
/**Determines whether a set amount of time (wait_time) has expired since a given start_time*/
boolean hasTimeElapsed( unsigned long start_time, unsigned long wait_time)
{
  boolean ret_value = false;
  unsigned long now = millis();
  if( now < start_time + wait_time)
  { 
    ret_value = false;
  }
  else
  {
    ret_value = true;
  }
  return ret_value;
}

/**Main loop continuously reading CAN messages and writing text representaions to the SD card. Each time a interupt is generated 
 * the nessage is extracted and forearded to ``processMessage''
 */
void loop()
{
  //Begin the loop to capture CAN messgages
  byte mask;
  const byte INTERRUPT_RESET = B00000000;
  while( KEEPGOING ==true )
  {
    //time since last message has been received
    timeDifference = getTimeDifference();
    if( DEBUG == true)
    {
      tmpMessage = String("loop:timeDifference = ");
      tmpMessage.concat( timeDifference );
      Serial.println( tmpMessage );
    }
    /*After a inital message has been received, the program will shutdown afte 10 sec of idle time*/
    if( ((timeLastMessageReceived!=0)) && ( timeDifference > 10000))
    {
      if( DEBUG == true)
      {
        tmpMessage = String("Idle!!\n timeDifferenc = ");
        tmpMessage.concat( timeDifference );
        tmpMessage.concat(", timeLastMessageReceived= ");
        tmpMessage.concat( timeLastMessageReceived );
        Serial.println( tmpMessage);
      }      
      //Close file
      myFile.close();      
      KEEPGOING = false;
    }
    else
    {
      //continue
    }

    byte interruptFlags = CAN.Read(CANINTF);
    if( DEBUG == true)
    {
      Serial.println(interruptFlags,BIN);
      Serial.println("interrupt flags OK");
      Serial.println("Waiting for interrrupt");
    }
    //Wait a maximum of 10 seconds for the next message
    unsigned long start_wait_time = millis();
    while(  (KEEPGOING==true) && (! CAN.Interrupt()) && ( ! hasTimeElapsed(start_wait_time, 10000 )) )
    {
      ;
    }

    // This implementation utilizes the MCP2515 INT pin to flag received messages
    if(CAN.Interrupt()) 
    {      

      //Message in RX buffer 0
      if(interruptFlags & RX0IF) 
      {
        // Serial.println("Message on RX Buffer 0");
        if(DEBUG == true )
        {	
          Serial.println("Message on RX Buffer 0");
          interruptFlags = CAN.Read(CANINTF);
          Serial.println(interruptFlags,BIN);
        }
        //Retrieve the new message
        message0 = CAN.ReadBuffer(RXB0);
        //Process the message
        processMessage( message0);
        //Reset the CANITF flag
        mask = RX0IF;
        CAN.BitModify(CANINTF,mask, INTERRUPT_RESET);       
      }
      if(interruptFlags & RX1IF) 
      {
        //Serial.println("Message on RX Buffer 1");
        if(DEBUG == true)
        {
          Serial.println("Message on RX Buffer 1");
          interruptFlags = CAN.Read(CANINTF);
          Serial.println(interruptFlags,BIN);
        }
        //Retrieve the new message
        message1 = CAN.ReadBuffer(RXB1);
        //Process the message
        processMessage( message1 );
        //Reset the CANITF flag
        mask = RX1IF;
        CAN.BitModify(CANINTF,mask, INTERRUPT_RESET);
      }
      if(interruptFlags & TX0IF) 
      {
        // Serial.println("Sent on TX Buffer 0");
        // TX buffer 0 sent
        mask = TX0IF;
        CAN.BitModify(CANINTF,mask, INTERRUPT_RESET);
      }
      if(interruptFlags & TX1IF) 
      {
        //Serial.println("Sent on TX Buffer 1");
        // TX buffer 1 sent
        mask = TX1IF;
        CAN.BitModify(CANINTF,mask, INTERRUPT_RESET);
      }
      if(interruptFlags & TX2IF) 
      {
        //Serial.println("Sent on TX Buffer 2");
        // TX buffer 2 sent
        mask = TX2IF;
        CAN.BitModify(CANINTF,mask, INTERRUPT_RESET);
      }
      if(interruptFlags & ERRIF) 
      {
        //Serial.println("Error encountered");
        // error handling code
        mask = ERRIF;
        CAN.BitModify(CANINTF,mask, INTERRUPT_RESET);
      }
      if(interruptFlags & MERRF) 
      {
        //Serial.println("Error encountered 2");
        mask = MERRF;
        CAN.BitModify(CANINTF,mask, INTERRUPT_RESET);
        // error handling code
        // if TXBnCTRL.TXERR set then transmission error
        // if message is lost TXBnCTRL.MLOA will be set
      }
    }
  }  // end while loop 
  Serial.println("Program complete!");
  Serial.println("Waiting for reset...");
  /* myFile = SD.open(fileName);
   if (myFile) {
   Serial.println(fileName);
   
   // read from the file until there's nothing else in it:
   while (myFile.available()) {
   	Serial.write(myFile.read());
   }
   // close the file:
   myFile.close();
   } else {
   	// if the file didn't open, print an error:
   Serial.println("error opening quel cazzo di fail");
   }*/
  while( true )
  {
    digitalWrite(LIGHT_SD,HIGH);
    delay(1000);
    digitalWrite(LIGHT_SD,LOW);
    delay(1000);
    //do nothing
    ;
  }
}

/**Extracts the components from received messsage and writes them to file. This method has a considerable performance impact 
 * and is for illustration purposes only. Speedup can be achieved by ensuring the maximum amount of data (e.g. 512 byte ) is written each time ``print'' is called. 
 */
void processMessage( Frame& message )
{
  digitalWrite(LIGHT_CAN,HIGH);

  //current time 
  timeLastMessageReceived = millis();
  if(DEBUG == true)
  {
    Serial.print("Assigning= ");
    Serial.println(timeLastMessageReceived);
  }   

  if(message.id>0) 
  { 
    if(DEBUG == true)
    {      
      Serial.println(message.id,HEX);
    }

    myFile.print(msgCount++, DEC);
    myFile.print(",");
    myFile.print(getTimeDifference(),DEC);
    myFile.print(",");
    myFile.print(message.id,HEX);
    myFile.print(",");
    myFile.print(message.dlc,DEC);  
    myFile.print(",");       
    for(int i=0;i<message.dlc;i++) 
    {	
      myFile.print(message.data[i],HEX);
      myFile.print(" ");

    }

    myFile.println();     
    //flush for illustration purpose only
    myFile.flush();
  }
  digitalWrite(LIGHT_CAN,LOW);
  digitalWrite(LIGHT_SD,LOW);

}  
void preparaSD(void) {

  //Find the next available file name to prevent overwriting previously recorded sessions.
  Serial.print("Checking file name: ");
  Serial.println(fileName); 
  boolean fileExists = SD.exists( fileName );
  Serial.flush();
  //If the file exists change the name until a new one is created
  if( fileExists == true )
  {
    for (uint8_t i = 0; i < 100; i++) 
    {
      fileName[4] = i/10 + '0';
      fileName[5] = i%10 + '0';
      Serial.print("Checking file name: ");
      Serial.println(fileName);
      fileExists = SD.exists( fileName );
      Serial.flush();
      Serial.println("");
      if(fileExists == false )
      {
        break;
      }
    }
  }
  else
  {
    //file does not exist yet -> create it
  }
  Serial.flush();

  //Open file and get ready to enter data
  Serial.println("1");
  unsigned short int j=0;
  Serial.println("1");
  
  while (!myFile) {
   delay(100);
    myFile = SD.open(fileName, FILE_WRITE);
    Serial.print("teo nuo");
    Serial.println((int)j++);
    Serial.flush();
  }
  if (myFile) {
    Serial.println("Esi il fle txt:");
    myFile.println(header);
    myFile.flush();
    // close the file:
  } 
  else {
    // if the file didn't open, print an error:
    Serial.println("NO");
  }
}


