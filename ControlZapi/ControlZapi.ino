#include <mcp_can.h>
#include <SPI.h>

MCP_CAN CAN0(46);     // Set CS to pin 46 for CAN0
MCP_CAN CAN1(42);     // Set CS to pin 42 for CAN1

byte messageCloseContact1[8] = {0x00, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00};
byte messageCloseContact2[8] = {0x00, 0x00, 0x03, 0x80, 0x00, 0x00, 0x00, 0x00};
byte messageCANOpenStart[8] = {0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

//This code reports error sending messages, but the controller will respond and close the contactor.
void setup()
{
  Serial.begin(115200);

  // Initialize MCP2515 running at 16MHz with a baudrate of 500kb/s and the masks and filters disabled.
  if(CAN0.begin(MCP_ANY, CAN_125KBPS, MCP_10MHZ) == CAN_OK) Serial.println("MCP2515 CAN0 Initialized Successfully!");
  else Serial.println("Error Initializing MCP2515 on CAN0...");

  CAN0.setMode(MCP_NORMAL);   // Change to normal mode to allow messages to be transmitted
  if(CAN1.begin(MCP_ANY, CAN_125KBPS, MCP_10MHZ) == CAN_OK) Serial.println("MCP2515 CAN1 Initialized Successfully!");
  else Serial.println("Error Initializing MCP2515 on CAN1...");

  CAN1.setMode(MCP_NORMAL);   // Change to normal mode to allow messages to be transmitted
}

void loop()
{
//  // send data:  ID = 0x100, Standard CAN Frame, Data length = 8 bytes, 'data' = array of data bytes to send
//  byte sndStat0 = CAN0.sendMsgBuf(0x100, 0, 8, data);
//  if(sndStat0 == CAN_OK){
//    Serial.println("CAN 0  Message Sent Successfully!");
//  } else {
//    Serial.println("CAN 0  Error Sending Message...");
//  }
//  // send data:  ID = 0x101, Standard CAN Frame, Data length = 8 bytes, 'data' = array of data bytes to send
//  byte sndStat1 = CAN1.sendMsgBuf(0x101, 0, 8, data);
//  if(sndStat1 == CAN_OK){
//    Serial.println("CAN 1  Message Sent Successfully!");
//  } else {
//    Serial.println("CAN 1  Error Sending Message...");
//  }
//  delay(100);   // send data per 100ms
  //Initial CANOpen message
  // send data:  ID = 0x101, Standard CAN Frame, Data length = 8 bytes, messageCANOpenStart = message to send
  byte sndStat1 = CAN1.sendMsgBuf(0x00, 0, 8, messageCANOpenStart);
  if(sndStat1 == CAN_OK){
    Serial.println("CAN 1  Start Message Sent Successfully!");
  } else {
    Serial.println("CAN 1  Error Sending Start Message...");
  }
//Close contactor
for(;;){
  // send data:  ID = 0x101, Standard CAN Frame, Data length = 8 bytes, 'data' = array of data bytes to send
  byte sndStat1 = CAN1.sendMsgBuf(0x248, 0, 8, messageCloseContact1);
  if(sndStat1 == CAN_OK){
    Serial.println("CAN 1 Contact Close Message Sent Successfully!");
  } else {
    Serial.println("CAN 1 Error Sending Contact Close Message...");
  }
  delay(25);
  // send data:  ID = 0x101, Standard CAN Frame, Data length = 8 bytes, 'data' = array of data bytes to send
  sndStat1 = CAN1.sendMsgBuf(0x248, 0, 8, messageCloseContact2);
  if(sndStat1 == CAN_OK){
    Serial.println("CAN 1 Contact Close Message Sent Successfully!");
  } else {
    Serial.println("CAN 1 Error Sending Contact Close Message...");
  }
  delay(25);   // send data per 100ms
}
}
