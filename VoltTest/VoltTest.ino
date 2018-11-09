#include <mcp_can.h>
#include <SPI.h>

//Variables - Constant
MCP_CAN CAN0(46);     // Set CS to pin 46 for CAN0
MCP_CAN CAN1(42);     // Set CS to pin 42 for CAN1

byte messageCANOpenStart[8] = {0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
byte messageCloseContact[8] = {0x00, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00};
byte messageCloseContactS[8] = {0x00, 0x00, 0x03, 0x80, 0x00, 0x00, 0x00, 0x00};
byte messageCrank[8] = {0x00, 0x00, 0x0B, 0x00, 0x00, 0x00, 0x58, 0x02};
byte messageCrankS[8] = {0x00, 0x00, 0x0B, 0x80, 0x00, 0x00, 0x58, 0x02};
byte messageCharge30[8] = {0xE1, 0xFF, 0x13, 0x00, 0x00, 0x00, 0x00, 0x00};
byte messageCharge30S[8] = {0xE1, 0xFF, 0x13, 0x80, 0x00, 0x00, 0x00, 0x00};
byte messageCharge50[8] = {0xCD, 0xFF, 0x13, 0x00, 0x00, 0x00, 0x00, 0x00};
byte messageCharge50S[8] = {0xCD, 0xFF, 0x13, 0x80, 0x00, 0x00, 0x00, 0x00};
byte messageCharge75[8] = {0xB4, 0xFF, 0x13, 0x00, 0x00, 0x00, 0x00, 0x00};
byte messageCharge75S[8] = {0xB4, 0xFF, 0x13, 0x80, 0x00, 0x00, 0x00, 0x00};
byte messageCharge100[8] = {0x9B, 0xFF, 0x13, 0x00, 0x00, 0x00, 0x00, 0x00};
byte messageCharge100S[8] = {0x9B, 0xFF, 0x13, 0x80, 0x00, 0x00, 0x00, 0x00};
byte messagePowerDown[8] = {0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00};
byte messagePowerDownS[8] = {0x00, 0x00, 0x02, 0x80, 0x00, 0x00, 0x00, 0x00};
byte messageIdle[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
byte messageIdleS[8] = {0x00, 0x00, 0x00, 0x80, 0x00, 0x00, 0x00, 0x00};

int keySwitchPin = 22; //power up the engine. Relay 1. Pin 11 - J5 connection 
int hiSpeedSwitchPin = 24; // Switch to 2800 rpm. Relay 2. Pin 8 - J5 connecion 
int fanContPin = 23; //Relay 3. Pin 11 - J6 
int vBattSensPin = 6; // vBattSense is on ADC 6

//Variables - Variable
byte engineRunStateOn = LOW; // LOW = Off; HIGH = On
byte engineHighSpeedState = LOW; // LOW = 1800 rpm; HIGH = 2800 rpm
float battLargeV = 0; // Variable for main battery voltage



//Functions
void closeContact(long wait_time);
void crankEngine(long wait_time);
void charge30(long wait_time);
void stopEngine();
void idle();

//This code reports error sending messages, but the controller will respond and close the contactor.
void setup()
{
  Serial.begin(115200);

  // Initialize MCP2515 running at 10MHz with a baudrate of 500kb/s and the masks and filters disabled.
  if (CAN0.begin(MCP_ANY, CAN_125KBPS, MCP_10MHZ) == CAN_OK) Serial.println("MCP2515 CAN0 Initialized Successfully!");
  else Serial.println("Error Initializing MCP2515 on CAN0...");

  CAN0.setMode(MCP_NORMAL);   // Change to normal mode to allow messages to be transmitted
  if (CAN1.begin(MCP_ANY, CAN_125KBPS, MCP_10MHZ) == CAN_OK) Serial.println("MCP2515 CAN1 Initialized Successfully!");
  else Serial.println("Error Initializing MCP2515 on CAN1...");

  CAN1.setMode(MCP_NORMAL);   // Change to normal mode to allow messages to be transmitted
  
  //Initial CANOpen message
  // send data:  ID = 0x00, Standard CAN Frame, Data length = 8 bytes, messageCANOpenStart = message to send
  byte sndStat1 = CAN1.sendMsgBuf(0x00, 0, 8, messageCANOpenStart);
  if (sndStat1 == CAN_OK) {
    Serial.println("CAN 1  Start Message Sent Successfully!");
  } else {
    Serial.println("CAN 1  Error Sending Start Message...");
    Serial.print("Return Code: ");
    Serial.println(sndStat1);
  }
  engineRunStateOn = LOW; // initialize engine state to off
  engineHighSpeedState = LOW; // initialize engine speed to 1800 rpm
  pinMode(keySwitchPin, OUTPUT);
  pinMode(hiSpeedSwitchPin, OUTPUT);
  pinMode(fanContPin, OUTPUT);
  digitalWrite(keySwitchPin, engineRunStateOn);
  digitalWrite(hiSpeedSwitchPin, engineHighSpeedState);
  digitalWrite(fanContPin, HIGH);
}

void loop()
{
  //battLargeV = analogRead(vBattSensPin)*(402.0+10000.0)/402.0; //Values set by voltage divider built into conroller board.
  battLargeV = (analogRead(vBattSensPin)*(402.0+10000.0)/402.0); //Values set by voltage divider built into conroller board.
  Serial.print("Main Batt V = ");
  Serial.println(battLargeV);
//  //Close contactor and hold for 1 sec (1000ms)
//  closeContact(1000); 
//  //Crank Engine for 1 sec
//  crankEngine(1000);
//  //Charge at 30A for 30 sec
//  charge30(30000);
//  //Stop charging, turn off engine
//  stopEngine();
  for(;;) idle();
}

void closeContact(long wait_time) {
  long loops = wait_time / 50;
  for (long i = 0; i < loops; i++) {
    // send data:  ID = 0x248, Standard CAN Frame, Data length = 8 bytes, 'data' = array of data bytes to send
    byte sndStat1 = CAN1.sendMsgBuf(0x248, 0, 8, messageCloseContact);
    if (sndStat1 == CAN_OK) {
      Serial.println("CAN 1 Contact Close Message Sent Successfully!");
    } else {
      Serial.println("CAN 1 Error Sending Contact Close Message...");
      Serial.print("Return Code: ");
      Serial.println(sndStat1);
    }
    delay(25); // send message every 25ms
    // send data:  ID = 0x248, Standard CAN Frame, Data length = 8 bytes, 'data' = array of data bytes to send
    sndStat1 = CAN1.sendMsgBuf(0x248, 0, 8, messageCloseContactS);
    if (sndStat1 == CAN_OK) {
      Serial.println("CAN 1 Contact Close Message S Sent Successfully!");
    } else {
      Serial.println("CAN 1 Error Sending Contact Close Message S...");
      Serial.print("Return Code: ");
      Serial.println(sndStat1);
    }
    delay(25);   // send message every 25ms
  }
}

void crankEngine(long wait_time) {
  engineRunStateOn = HIGH; // set engine on variable
  digitalWrite(keySwitchPin, engineRunStateOn); // turn on engine
  closeContact(1000); 
  long loops = wait_time / 50;
  for (long i = 0; i < loops; i++) {
    // send data:  ID = 0x248, Standard CAN Frame, Data length = 8 bytes, 'data' = array of data bytes to send
    byte sndStat1 = CAN1.sendMsgBuf(0x248, 0, 8, messageCrank);
    if (sndStat1 == CAN_OK) {
      Serial.println("CAN 1 Crank Engine Message Sent Successfully!");
    } else {
      Serial.println("CAN 1 Error Sending Crank Engine Message...");
      Serial.print("Return Code: ");
      Serial.println(sndStat1);
    }
    delay(25); // send message every 25ms
    // send data:  ID = 0x248, Standard CAN Frame, Data length = 8 bytes, 'data' = array of data bytes to send
    sndStat1 = CAN1.sendMsgBuf(0x248, 0, 8, messageCrankS);
    if (sndStat1 == CAN_OK) {
      Serial.println("CAN 1 Crank Engine Message Sent Successfully!");
    } else {
      Serial.println("CAN 1 Error Sending Crank Engine Message...");
      Serial.print("Return Code: ");
      Serial.println(sndStat1);
    }
    delay(25);   // send message every 25ms
  }
  engineHighSpeedState = HIGH; // set engine speed variable to high speed
  digitalWrite(hiSpeedSwitchPin, engineHighSpeedState); // set engine speed to 2800
  delay(1000);
}

void charge30(long wait_time) {
  long loops = wait_time / 50;
  for (long i = 0; i < loops; i++) {
    Serial.print("Main Batt V = ");
    Serial.print(battLargeV);
    // send data:  ID = 0x248, Standard CAN Frame, Data length = 8 bytes, 'data' = array of data bytes to send
    byte sndStat1 = CAN1.sendMsgBuf(0x248, 0, 8, messageCharge100);
    if (sndStat1 == CAN_OK) {
      Serial.print("\tCAN 1 Charge 30 Message Sent Successfully!\t");
    } else {
      Serial.print("\tCAN 1 Error Sending Charge 30 Message...  \t");
      Serial.print("RetC: ");
      Serial.println(sndStat1);
    }
    delay(25); // send message every 25ms
    // send data:  ID = 0x248, Standard CAN Frame, Data length = 8 bytes, 'data' = array of data bytes to send
    sndStat1 = CAN1.sendMsgBuf(0x248, 0, 8, messageCharge100S);
    if (sndStat1 == CAN_OK) {
      Serial.print("\tCAN 1 Charge 30 Message Sent Successfully!");
    } else {
      Serial.print("\tCAN 1 Error Sending Charge 30 Message...");
      Serial.print("RetC: ");
      Serial.println(sndStat1);
    }
    delay(25);   // send message every 25ms
  }
}

void stopEngine() { //This is an 11 second process.
  //Stop engine; Dissipate Power
  engineHighSpeedState = LOW; // set engine speed variable to low speed
  digitalWrite(hiSpeedSwitchPin, engineHighSpeedState); // set engine speed to 1800
  engineRunStateOn = LOW; // set engine off variable
  digitalWrite(keySwitchPin, engineRunStateOn); // turn off engine
  long wait_time = 10000; //wait 10 seconds for engine shutdown and power dissipation
  long loops = wait_time / 50;
  for (int i = 0; i < loops; i++) {
    // send data:  ID = 0x248, Standard CAN Frame, Data length = 8 bytes, 'data' = array of data bytes to send
    byte sndStat1 = CAN1.sendMsgBuf(0x248, 0, 8, messagePowerDown);
    if (sndStat1 == CAN_OK) {
      Serial.println("CAN 1 Power Down Message Sent Successfully!");
    } else {
      Serial.println("CAN 1 Error Sending Power Down Message...");
      Serial.print("Return Code: ");
      Serial.println(sndStat1);
    }
    delay(25); // send message every 25ms
    // send data:  ID = 0x248, Standard CAN Frame, Data length = 8 bytes, 'data' = array of data bytes to send
    sndStat1 = CAN1.sendMsgBuf(0x248, 0, 8, messagePowerDownS);
    if (sndStat1 == CAN_OK) {
      Serial.println("CAN 1 Power Down Message Sent Successfully!");
    } else {
      Serial.println("CAN 1 Error Sending Power Down Message...");
      Serial.print("Return Code: ");
      Serial.println(sndStat1);
    }
    delay(25);   // send message every 25ms
  }
  //Open contactor
  wait_time = 1000; //wait 1 second for contact opening
  loops = wait_time / 50;
  for (int i = 0; i < loops; i++) {
    // send data:  ID = 0x248, Standard CAN Frame, Data length = 8 bytes, 'data' = array of data bytes to send
    byte sndStat1 = CAN1.sendMsgBuf(0x248, 0, 8, messageIdle);
    if (sndStat1 == CAN_OK) {
      Serial.println("CAN 1 Idle Message Sent Successfully!");
    } else {
      Serial.println("CAN 1 Error Sending Idle Message...");
      Serial.print("Return Code: ");
      Serial.println(sndStat1);
    }
    delay(25); // send message every 25ms
    // send data:  ID = 0x248, Standard CAN Frame, Data length = 8 bytes, 'data' = array of data bytes to send
    sndStat1 = CAN1.sendMsgBuf(0x248, 0, 8, messageIdleS);
    if (sndStat1 == CAN_OK) {
      Serial.println("CAN 1 Idle Message Sent Successfully!");
    } else {
      Serial.println("CAN 1 Error Sending Idle Message...");
      Serial.print("Return Code: ");
      Serial.println(sndStat1);
    }
    delay(25);   // send message every 25ms
  }
}

void idle() { //idles for 50ms.
    // send data:  ID = 0x248, Standard CAN Frame, Data length = 8 bytes, 'data' = array of data bytes to send
    byte sndStat1 = CAN1.sendMsgBuf(0x248, 0, 8, messageIdle);
    if (sndStat1 == CAN_OK) {
      //Serial.println("CAN 1 Idle Message Sent Successfully!");
    } else {
//      Serial.println("CAN 1 Error Sending Idle Message...");
//      Serial.print("Return Code: ");
//      Serial.println(sndStat1);
    }
    delay(25); // send message every 25ms
    // send data:  ID = 0x248, Standard CAN Frame, Data length = 8 bytes, 'data' = array of data bytes to send
    sndStat1 = CAN1.sendMsgBuf(0x248, 0, 8, messageIdleS);
    if (sndStat1 == CAN_OK) {
      //Serial.println("CAN 1 Idle Message Sent Successfully!");
    } else {
      //Serial.println("CAN 1 Error Sending Idle Message...");
//      Serial.print("Return Code: ");
//      Serial.println(sndStat1);
    }
    delay(25);   // send message every 25ms
    float anaRead = analogRead(vBattSensPin);
    Serial.print("Read = ");
  Serial.println(anaRead);
  float anaV = anaRead/1024.0*5.0;
  Serial.print("Read = ");
  Serial.println(anaV);
    battLargeV = ((analogRead(vBattSensPin)/1024.0*5.0)*(402.0+10000.0)/402.0); //Values set by voltage divider built into conroller board.
  Serial.print("\tMain Batt V = ");
  Serial.println(battLargeV);
}
