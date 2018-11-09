#include <mcp_can.h>
#include <SPI.h>

//Variables - Constant
//float bulkChargeCellVoltLimit = 2.25; //Set to 2.25V to be just below gassing voltage at 50 degC.
float bulkChargeCellVoltLimit = 2.4; //Generator can spike to 82 easily on startup.
int numOfCells = 36; // number of cells in the battery pack
float maxbattLargeV = bulkChargeCellVoltLimit * numOfCells; // bulk charge limit per cell, 36 cells
float minbattLargeV = 2.00 * numOfCells; // 2V per cell, 36 cells

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
void charge30();
void stopEngine();
void idle();
void charge(int amp_level);

//This code reports error sending messages, but the controller will respond and close the contactor.
void setup()
{
  Serial.begin(115200);

  // Initialize MCP2515 running at 10MHz with a baudrate of 125kb/s and the masks and filters disabled.
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
  battLargeV = ((analogRead(vBattSensPin)/1024.0*5.0)*(402.0+10000.0)/402.0); //Values set by voltage divider built into conroller board.
  Serial.print("Main Batt V = ");
  Serial.println(battLargeV);
  if (engineRunStateOn) { // engine on
    if (battLargeV >= maxbattLargeV) {
      stopEngine();
    } else {
      //No change. Continue Charging.
      charge30();
      //charge(30);
      Serial.println("Engine On. Charging.");
    }
  } else { // engine off
    if (battLargeV <= minbattLargeV) {
      startCharing();
    } else {
      //No change. Running on battery.
      idle();
      Serial.println("Engine Off. Running on Battery.");
    }
  }
}

void startCharing() {
  closeContact(500);
  //Crank Engine for 1 sec
  crankEngine(1000);
  //Start charging
  charge30();
  //charge(30);
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
      Serial.println("CAN 1 Contact Close Message Sent Successfully!");
    } else {
      Serial.println("CAN 1 Error Sending Contact Close Message...");
      Serial.print("Return Code: ");
      Serial.println(sndStat1);
    }
    delay(25);   // send message every 25ms
  }
}

void crankEngine(long wait_time) {
  engineRunStateOn = HIGH; // set engine on variable
  digitalWrite(keySwitchPin, engineRunStateOn); // turn on engine
  closeContact(100); 
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
  //delay(1000);
}

void charge30() { //Charge at 30 amps for 50 ms.
  // send data:  ID = 0x248, Standard CAN Frame, Data length = 8 bytes, 'data' = array of data bytes to send
  byte sndStat1 = CAN1.sendMsgBuf(0x248, 0, 8, messageCharge30);
  if (sndStat1 == CAN_OK) {
    Serial.println("CAN 1 Charge 30 Message Sent Successfully!");
  } else {
    Serial.println("CAN 1 Error Sending Charge 30 Message...");
    Serial.print("Return Code: ");
    Serial.println(sndStat1);
  }
  delay(25); // send message every 25ms
  // send data:  ID = 0x248, Standard CAN Frame, Data length = 8 bytes, 'data' = array of data bytes to send
  sndStat1 = CAN1.sendMsgBuf(0x248, 0, 8, messageCharge30S);
  if (sndStat1 == CAN_OK) {
    Serial.println("CAN 1 Charge 30 Message Sent Successfully!");
  } else {
    Serial.println("CAN 1 Error Sending Charge 30 Message...");
    Serial.print("Return Code: ");
    Serial.println(sndStat1);
  }
  delay(25);   // send message every 25ms
}

void charge(int amp_level) {
  byte messageCharge[8] = {0xFF, 0xFF, 0x13, 0x00, 0x00, 0x00, 0x00, 0x00};
  messageCharge[0] = 0xFF - amp_level;
  // send data:  ID = 0x248, Standard CAN Frame, Data length = 8 bytes, 'data' = array of data bytes to send
  byte sndStat1 = CAN1.sendMsgBuf(0x248, 0, 8, messageCharge);
  if (sndStat1 == CAN_OK) {
    Serial.println("CAN 1 Charge 30 Message Sent Successfully!");
  } else {
    Serial.println("CAN 1 Error Sending Charge Message...");
    Serial.print("Return Code: ");
    Serial.println(sndStat1);
  }
  delay(25); // send message every 25ms
  byte messageChargeS[8] = {0xE1, 0xFF, 0x13, 0x80, 0x00, 0x00, 0x00, 0x00};
  messageChargeS[0] = 0xFF - amp_level;
  // send data:  ID = 0x248, Standard CAN Frame, Data length = 8 bytes, 'data' = array of data bytes to send 
  sndStat1 = CAN1.sendMsgBuf(0x248, 0, 8, messageChargeS);
  if (sndStat1 == CAN_OK) {
    Serial.println("CAN 1 Charge 30 Message Sent Successfully!");
  } else {
    Serial.println("CAN 1 Error Sending Charge Message...");
    Serial.print("Return Code: ");
    Serial.println(sndStat1);
  }
  delay(25);   // send message every 25ms
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
