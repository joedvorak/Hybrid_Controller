// ------------------------------------------------------------------------
//  J1939 - Receiving Messages
// ------------------------------------------------------------------------
//
// IMPOPRTANT: Depending on the CAN shield used for this programming sample,
//             please make sure you set the proper CS pin in module can.cpp.
//
//  This Arduino program is free software; you can redistribute it and/or
//  modify it under the terms of the GNU Lesser General Public
//  License as published by the Free Software Foundation; either
//  version 2.1 of the License, or (at your option) any later version.

//  This program is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
//  Lesser General Public License for more details.

#include <stdlib.h>
#include <SPI.h>

#include "mcp_can.h"
#include "can_ext.h"

// ------------------------------------------------------------------------
//  SYSTEM: Setup routine runs on power-up or reset
// ------------------------------------------------------------------------
void setup()
{
  // Set the serial interface baud rate
  Serial.begin(115200);
  
  // Initialize the CAN controller
  if (canInitialize(CAN_250KBPS) == CAN_OK)  // Baud rates defined in mcp_can_dfs.h
    Serial.print("CAN Init OK.\n\r\n\r");
  else
    Serial.print("CAN Init Failed.\n\r");
    
}// end setup

// ------------------------------------------------------------------------
// Main Loop - Arduino Entry Point
// ------------------------------------------------------------------------
void loop() 
{
  // Declarations
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
      sprintf(sString, "Bytes = %X %X \n\r", nData[4], nData[3]);
      Serial.print(sString);
//      sprintf(sString, "%0b ", nData[3]);
//      Serial.print(sString);
//      Serial.print("\n\r");
      int engineRPM = ((nData[4] << 8) + nData[3]) >> 3;
      sprintf(sString, "engineRPM = %d", engineRPM);
      Serial.print(sString);
      Serial.print("\n\r");      
        
  }// end if

}// end loop


