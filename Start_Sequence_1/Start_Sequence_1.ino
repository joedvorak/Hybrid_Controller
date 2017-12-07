//must include these each time
#include <stdlib.h>
#include <SPI.h>

#include "mcp_can.h"
#include "can_ext.h"

void setup()
{
    // Set the serial interface baud rate
    Serial.begin(115200);
  
    // Initialize the CAN controller
    if (canInitialize(CAN_250KBPS) == CAN_OK)  // Baud rates defined in mcp_can_dfs.h
      Serial.print("CAN Init OK.\n\r\n\r");
    else
      Serial.print("CAN Init Failed.\n\r");
}

//Code to for StartCommand for Kubota Engine

// keys and relays assigned to pin numbers
int keySwitchPin = 9;
int startSwitchPin = 8;                                 //cannot have start switch on for more than 10000 ms
int engineRPM;
pinMode(keySwitchPin, OUTPUT);
pinMode(startSwitchPin, OUTPUT);

void startEngine()
{
    digitalWrite(keySwitchPin, HIGH);
    delay(500);
}


void startSwitch()
{
    byte nPriority;
    byte nSrcAddr;
    byte nDestAddr;
    byte nData[8];
    int nDataLen;
    long lPGN;
  
    char sString[80];
  
  // Check for received J1939 messages
    if(j1939Receive(&lPGN, &nPriority, &nSrcAddr, &nDestAddr, nData, &nDataLen) == 0)
    {
       if((int)lPGN == 0xF004)
       {
          sprintf(sString, "%0b ", nData[4]);
          Serial.print(sString);
          Serial.print("\n\r");
          sprintf(sString, "%0b ", nData[3]);
          Serial.print(sString);
          Serial.print("\n\r");
          int engineRPM = ((nData[4] << 8) + nData[3]) >> 3;
          sprintf(sString, "%d", engineRPM);
          Serial.print(sString);
          Serial.print("\n\r");
      }
     int startSwitchTimer = millis();
    boolean done = false;
    while(done != true)
    {
       digitalWrite(startSwitchPin,HIGH);
       if(engineRPM >= 600)
       {
          digitalWrite(startSwitchPin, LOW);
          done = true;
       }
       else if(millis() >= startSwitchTimer+10000);
       {
          digitalWrite(startSwitchPin, LOW);
          done = true;
       }
    }
}

void loop()
{
    byte nPriority;
    byte nSrcAddr;
    byte nDestAddr;
    byte nData[8];
    int nDataLen;
    long lPGN;
  
    char sString[80];
  
    digitalWrite(keySwitchPin, HIGH);
    delay(500);

    int startSwitchTimer = millis();
  
    if(j1939Receive(&lPGN, &nPriority, &nSrcAddr, &nDestAddr, nData, &nDataLen) == 0)
    {
        if((int)lPGN == 0xF004)
        {
            sprintf(sString, "%0b ", nData[4]);
            Serial.print(sString);
            Serial.print("\n\r");
            sprintf(sString, "%0b ", nData[3]);
            Serial.print(sString);
            Serial.print("\n\r");
            int engineRPM = ((nData[4] << 8) + nData[3]) >> 3;
            sprintf(sString, "%d", engineRPM);
            Serial.print(sString);
            Serial.print("\n\r");
        }
    }
    boolean done = false;
    while(done != true)
    {
        digitalWrite(startSwitchPin,HIGH);
        if(engineRPM >= 600)
        {
            digitalWrite(startSwitchPin, LOW);
            done = true;
        }
        else if(millis() >= startSwitchTimer+10000);
        {
            digitalWrite(startSwitchPin, LOW);
            done = true;
        }
    }
}



