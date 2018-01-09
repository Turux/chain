#include <SD.h>
#include <SPI.h>
#include <MCP2515.h>

/*#include "LINX_Config.h"
#include "LINX_Devices.h"
#include "LINX.h"*/

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
const int CAN_CHIP_SELECT      = 53;
/* SD_CHIP_SELECT assigned to pin 9*/
// const int SD_CHIP_SELECT       = 9;
/* LIGHT_SD assigned to pin 8*/
const int LIGHT_SD       = 7;
/* SD_CHIP_SELECT assigned to pin 9*/
// const int LIGHT_CAN      = 8;
/* CAN_INTERRUPT assigned to pin 2*/
const int CAN_INTERRUPT_PIN    = 2;

/*MCP2515 CONFIG state: Enables mask configuration etc.*/
const byte MCP2515_CONFIG = 0x80;
/*MCP2515 LISTEN_ONLY state: Receives messages but does not send any*/
const byte MCP2515_LISTEN = 0x60;
/*MCP2515 LOOP state: Enables sending/receiving of messages without using the CAN BUS*/
const byte MCP2515_LOOP   = 0x40;
/*MCP2515 SLEEP state: Reduces power consumption*/
const byte MCP2515_SLEEP  = 0x20;
/*MCP2515 NORMAL state: Participates as a regular node in the CAN network*/
const byte MCP2515_NORMAL = 0x00;

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
/* CAN message frame exposed by the potsFrame */
Frame potsFrame;
//Various helper variables
int counter = 0;
int msgCount = 0;
int pots[4];

/*Time at which the last message was received*/
unsigned long timeLastMessageReceived = 0;

/*Time difference between the last two received messages*/
unsigned long timeDifference = 0;
/*Indicates whether or not processing should be continued*/
boolean KEEPGOING = true;
/*Byte generale che è bene portasse in giro */
byte interruptFlags;
byte mask;
const byte INTERRUPT_RESET = B00000000;

/*Variable for temporary message assembly*/
String tmpMessage;

/**Debug variable controls whether or not debug/status information will bewritten to the serial console. Setting DEBUG to true has a negative performance impact.*/
// boolean DEBUG = true;

/**Initialize the CAN shield*/

/**Initialize SPI communication*/

/**Method that provides summary information on the status of the MCP2515 chip*/

/**Initializes all components that will be used during the continuous loop*/
void setup()
{
  // setupLINX();
  pinMode(LIGHT_SD, OUTPUT);
  pinMode(LIGHT_CAN, OUTPUT);
  boolean setupSuccess = true;
  Serial.begin(115200);
  Serial.println("Setup");
  pinMode(10, OUTPUT);
  pinMode(9, OUTPUT);
  //sd
  CAN.initSPI();
  setupSuccess = CAN.initSD();
  if (setupSuccess) preparaSD();
  delay(100);

  //can
  setupSuccess &= CAN.initCAN();

  if ( setupSuccess == true)
  {
    if (DEBUG == true)
    {
      CAN.displayCanStatus();
    }
    Serial.println("log start");
  }


}

/**Main loop continuously reading CAN messages and writing text representaions to the SD card. Each time a interupt is generated
 * the nessage is extracted and forearded to ``processMessage''
 */
void loop()
{
  //Begin the loop to capture CAN messgages
  while ( KEEPGOING == true )
  {
    //time since last message has been received
    timeDifference = getTimeDifference();
    if ( DEBUG == true)
    {
      tmpMessage = String("loop:timeDifference = ");
      tmpMessage.concat( timeDifference );
      Serial.println( tmpMessage );
    }
    /*After a inital message has been received, the program will shutdown afte 10 sec of idle time*/
    if ( ((timeLastMessageReceived != 0)) && ( timeDifference > 10000))
    {
      if ( DEBUG == true)
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

    interruptFlags = CAN.Read(CANINTF);
    if ( DEBUG == true)
    {
      Serial.println(interruptFlags, BIN);
      Serial.println("interrupt flags OK");
      Serial.println("Waiting for interrrupt");
    }
    //Wait a maximum of 10 seconds for the next message
    unsigned long start_wait_time = millis();
    while (  (KEEPGOING == true) && (! CAN.Interrupt()) && ( ! CAN.hasTimeElapsed(start_wait_time, 10000 )) )
    {
      ;
    }

    // This implementation utilizes the MCP2515 INT pin to flag received messages
    if (CAN.Interrupt())
    {
      checkCanRx0();
      checkCanRx1();
      checkCanTx0();
      checkCanTx1();
      checkCanTx2();
      checkCanErr0();
      checkCanErr1();
      readPots();
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
  while ( true )
  {
    digitalWrite(LIGHT_SD, HIGH);
    delay(1000);
    digitalWrite(LIGHT_SD, LOW);
    delay(1000);
    //do nothing
    ;
  }
}

/**Extracts the components from received messsage and writes them to file. This method has a considerable performance impact
 * and is for illustration purposes only. Speedup can be achieved by ensuring the maximum amount of data (e.g. 512 byte ) is written each time ``print'' is called.
 */

void preparaSD(void) {

  //Find the next available file name to prevent overwriting previously recorded sessions.
  Serial.print("Checking file name: ");
  Serial.println(fileName);
  boolean fileExists = SD.exists( fileName );
  Serial.flush();
  //If the file exists change the name until a new one is created
  if ( fileExists == true )
  {
    for (uint8_t i = 0; i < 100; i++)
    {
      fileName[4] = i / 10 + '0';
      fileName[5] = i % 10 + '0';
      Serial.print("Checking file name: ");
      Serial.println(fileName);
      fileExists = SD.exists( fileName );
      Serial.flush();
      Serial.println("");
      if (fileExists == false )
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
  unsigned short int j = 0;
  while (!myFile) {
    delay(100);
    myFile = SD.open(fileName, FILE_WRITE);
    Serial.print("Try #");
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

void print_hex(int v, int num_places)
{
  int mask = 0, n, num_nibbles, digit;

  for (n = 1; n <= num_places; n++)
  {
    mask = (mask << 1) | 0x0001;
  }
  v = v & mask; // truncate v to specified number of places

  num_nibbles = num_places / 4;
  if ((num_places % 4) != 0)
  {
    ++num_nibbles;
  }

  do
  {
    digit = ((v >> (num_nibbles - 1) * 4)) & 0x0f;
    myFile.print(digit, HEX);
  }
  while (--num_nibbles);

}

void processMessage( Frame& message )
{
  digitalWrite(LIGHT_CAN, HIGH);

  //current time
  timeLastMessageReceived = millis();

  if (message.id > 0)
  {
    if (DEBUG == true)
    {
      Serial.println(message.id, HEX);
    }

    myFile.print(msgCount++, DEC);
    myFile.print(",");
    myFile.print(getTimeDifference(), DEC);
    myFile.print(",");
    myFile.print(message.id, HEX);
    myFile.print(",");
    myFile.print(message.dlc, DEC);
    myFile.print(",");
    for (int i = 0; i < message.dlc; i++)
    {
      print_hex(message.data[i], 8);
      myFile.print(" ");

    }

    myFile.println();
    //flush for illustration purpose only
    myFile.flush();
  }
  digitalWrite(LIGHT_CAN, LOW);

}

unsigned long getTimeDifference()
{
  //Serial.print("getTime Difference(): ");
  unsigned long current_time = millis();
  unsigned long result = current_time - timeLastMessageReceived;
  if ( DEBUG == true )
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


void checkCanRx0() {
  if (interruptFlags & RX0IF)
  {
    // Serial.println("Message on RX Buffer 0");
    if (DEBUG == true )
    {
      Serial.println("Message on RX Buffer 0");
      interruptFlags =  CAN.Read(CANINTF);
      Serial.println(interruptFlags, BIN);
    }
    //Retrieve the new message
    message0 =   CAN.ReadBuffer(RXB0);
    //Process the message
    processMessage( message0);
    //Reset the CANITF flag
    mask = RX0IF;
    CAN.BitModify(CANINTF, mask, INTERRUPT_RESET);
  }
}

void checkCanRx1() {
  if (interruptFlags & RX1IF)
  {
    //Serial.println("Message on RX Buffer 1");
    if (DEBUG == true)
    {
      Serial.println("Message on RX Buffer 1");
      interruptFlags =  CAN.Read(CANINTF);
      Serial.println(interruptFlags, BIN);
    }
    //Retrieve the new message
    message1 =  CAN.ReadBuffer(RXB1);
    //Process the message
    processMessage( message1 );
    //Reset the CANITF flag
    mask = RX1IF;
    CAN.BitModify(CANINTF, mask, INTERRUPT_RESET);
  }
}

void checkCanTx0() {
  if (interruptFlags & TX0IF)
  {
    // Serial.println("Sent on TX Buffer 0");
    // TX buffer 0 sent
    mask = TX0IF;
    CAN.BitModify(CANINTF, mask, INTERRUPT_RESET);
  }
}
void checkCanTx1() {
  if (interruptFlags & TX1IF)
  {
    //Serial.println("Sent on TX Buffer 1");
    // TX buffer 1 sent
    mask = TX1IF;
    CAN.BitModify(CANINTF, mask, INTERRUPT_RESET);
  }
}
void checkCanTx2() {
  if (interruptFlags & TX2IF)
  {
    //Serial.println("Sent on TX Buffer 2");
    // TX buffer 2 sent
    mask = TX2IF;
    CAN.BitModify(CANINTF, mask, INTERRUPT_RESET);
  }
}
void checkCanErr0() {
  if (interruptFlags & ERRIF)
  {
    //Serial.println("Error encountered");
    // error handling code
    mask = ERRIF;
    CAN.BitModify(CANINTF, mask, INTERRUPT_RESET);
  }
}
void checkCanErr1() {
  if (interruptFlags & MERRF)
  {
    //Serial.println("Error encountered 2");
    mask = MERRF;
    CAN.BitModify(CANINTF, mask, INTERRUPT_RESET);
    // error handling code
    // if TXBnCTRL.TXERR set then transmission error
    // if message is lost TXBnCTRL.MLOA will be set
  }
}

void readPots(void) {
    potsFrame.id = 0x110;                 // Extended ID flag
  pots[0] = analogRead(A8);
  pots[1] = analogRead(A9);
  pots[2] = analogRead(A10);
  pots[3] = analogRead(A11);
  potsFrame.srr = 0;
  potsFrame.rtr = 0;                 // Remote Transmission Request
  potsFrame.ide = 0x110;                 // Extended ID flag
  potsFrame.dlc = 15;
  int i;
  for (i = 0; i < 4; i++) {
    potsFrame.data[2 * i] = (pots[i] & 0xFF);
    potsFrame.data[2 * i + 1] = ((pots[i] >> 8) & 0xFF);
  }
  for (i = 8; i < 16; i++) {
    potsFrame.data[i] = 0x00;
  }
  processMessage(potsFrame);
}
