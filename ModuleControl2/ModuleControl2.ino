#include <mcp_can.h>
#include <SPI.h>
#include "ModuleControl2.h"

//This code reports error sending messages, but the controller will respond and close the contactor.
void setup()
{
  Serial.begin(115200);
  Serial.println("Entering Setup");
  startCAN();
  engineRunStateOn = LOW;     // initialize engine state to off
  engineHighSpeedState = LOW; // initialize engine speed to 1800 rpm
  fanState = 0;
  pinMode(keySwitchPin, OUTPUT);
  //pinMode(hiSpeedSwitchPin, OUTPUT);
  pinMode(fanContPin, OUTPUT);
  pinMode(resetZAPIPin, OUTPUT);
  digitalWrite(keySwitchPin, engineRunStateOn);
  //digitalWrite(hiSpeedSwitchPin, engineHighSpeedState);
  digitalWrite(fanContPin, fanState);
  digitalWrite(resetZAPIPin, LOW);
  Serial.println("Pins Setup, Resetting ZAPI");
  resetZAPI();
  Serial.println("Setting Idle Mode");
  set_mode_idle();
  Serial.println("Exiting Setup");
}

void loop()
{
  delay(ZAPI_MESSAGE_PERIOD); //Keep other code short. Need to send Zapi message every 25ms
  // send data:  ID = 0x248, Standard CAN Frame, Data length = 8 bytes, 'data' = array of data bytes to send
  CANmessageStat = CAN1.sendMsgBuf(0x248, 0, 8, messagecurrent);
  messagecurrent[3] = messagecurrent[3] ^ 0x80; //toggle Message active bit

  //Check module state
  battLargeV = ((analogRead(vBattSensPin) / 1024.0 * 5.0) * (402.0 + 10000.0) / 402.0); //Values set by voltage divider built into conroller board.

  //Control Code:
  switch (module_State)
  {
  case MOD_IDLE:
    if (battLargeV <= minbattLargeV)
    {
      startCharingProcess();
    }
    break;
  case MOD_CL_CONTACT: //In the start process.
    stateCountDown--;
    if (stateCountDown == 0)
    {
      set_mode_crank();
    }
    break;
  case MOD_CRANK:
    stateCountDown--;
    if (stateCountDown == 0)
    {
      set_mode_charge();
    }
    break;
  case MOD_CHARGE:
    if (battLargeV >= maxbattLargeV)
    {
      stopEngine();
    }
    else
    {
      charge();
    }
    break;
  case MOD_POWER_DOWN:
    stateCountDown--;
    if (stateCountDown == 0)
    {
      set_mode_open_contact();
    }
    break;
  case MOD_OP_CONTACT: //In the stopping process.
    stateCountDown--;
    if (stateCountDown == 0)
    {
      set_mode_fan_cool_down();
    }
    break;
  case MOD_FAN_COOL_DOWN:
    stateCountDown--;
    if (stateCountDown == 0)
    {
      set_mode_idle();
    }
    if (battLargeV <= minbattLargeV)
    {
      startCharingProcess();
    }
    break;
  }
  print_system_status();
}

//Returns true if initialization is successfully or false otherwise.
bool startCAN()
{
  int ret_value = true;
  // Initialize MCP2515 running at 10MHz with a baudrate of 125kb/s and the masks and filters disabled.
  if (CAN0.begin(MCP_ANY, CAN_125KBPS, MCP_10MHZ) == CAN_OK)
  {
    Serial.println("MCP2515 CAN0 Initialized Successfully!");
  }
  else
  {
    Serial.println("Error Initializing MCP2515 on CAN0...");
    ret_value = false;
  }
  CAN0.setMode(MCP_NORMAL); // Change to normal mode to allow messages to be transmitted

  if (CAN1.begin(MCP_ANY, CAN_125KBPS, MCP_10MHZ) == CAN_OK)
  {
    Serial.println("MCP2515 CAN1 Initialized Successfully!");
  }
  else
  {
    Serial.println("Error Initializing MCP2515 on CAN1...");
    ret_value = false;
  }
  CAN1.setMode(MCP_NORMAL); // Change to normal mode to allow messages to be transmitted

  return ret_value;
}

//resetZAPI is the only function that contains delays!
void resetZAPI()
{
  Serial.println("Resetting");
  digitalWrite(resetZAPIPin, HIGH);
  delay(1000);
  digitalWrite(resetZAPIPin, LOW);
  Serial.println("Reset Done. Sending CAN Start");
  delay(1000);
  byte sndStat1 = CAN1.sendMsgBuf(0x00, 0, 8, messageCANOpenStart); //No Delays allowed after this.
  Serial.println("CAN Started");
}

void startCharingProcess()
{
  if (USE_ZAPI_RESET)
  {
    resetZAPI();
  }
  engineRunStateOn = HIGH;                      // set engine on variable
  digitalWrite(keySwitchPin, engineRunStateOn); // turn on engine
  fanState = 1;                                 //set fan variable to on
  digitalWrite(fanContPin, fanState);           // turn on fans
  //Close the main Battery Contactor
  set_mode_close_contact();
  //The rest of the start sequence will be finished after the close contactor count down.
}

void charge()
{
  if (CONSTANT_CHARGE_A == 0)
  {
    //Adjust charging current to maintain charge level.
  }
  else
  {
    //Use constant Charge Current
    chargingCurrent = CONSTANT_CHARGE_A;
  }
  messagecurrent[0] = 0xFF - chargingCurrent;
  messagecurrent[1] = 0xFF;
  messagecurrent[2] = 0x13;                     //charge, contactor and power enabled
  messagecurrent[3] = messagecurrent[3] & 0x80; //All 0 but stuffing bit which is not changed(3.7).
  messagecurrent[4] = 0x00;
  messagecurrent[5] = 0x00;
  messagecurrent[6] = 0x00;
  messagecurrent[7] = 0x00;
}

void stopEngine()
{
  engineRunStateOn = LOW;                       // set engine off variable
  digitalWrite(keySwitchPin, engineRunStateOn); // turn off engine
  module_State = MOD_POWER_DOWN;
  stateCountDown = POWER_DOWN_WAIT / ZAPI_MESSAGE_PERIOD;
}

void set_mode_idle()
{
  fanState = 0;                       //set fan variable to off
  digitalWrite(fanContPin, fanState); // turn off fans
  module_State = MOD_IDLE;
  messagecurrent[0] = 0x00;
  messagecurrent[1] = 0x00;
  messagecurrent[2] = 0x00;
  messagecurrent[3] = messagecurrent[3] & 0x80; //All 0 but stuffing bit which is not changed(3.7).
  messagecurrent[4] = 0x00;
  messagecurrent[5] = 0x00;
  messagecurrent[6] = 0x00;
  messagecurrent[7] = 0x00;
}
void set_mode_close_contact()
{
  module_State = MOD_CL_CONTACT;
  stateCountDown = CLOSE_CONTACT_WAIT / ZAPI_MESSAGE_PERIOD;
  messagecurrent[0] = 0x00;
  messagecurrent[1] = 0x00;
  messagecurrent[2] = 0x03;                     //contactor and power enabled
  messagecurrent[3] = messagecurrent[3] & 0x80; //All 0 but stuffing bit which is not changed(3.7).
  messagecurrent[4] = 0x00;
  messagecurrent[5] = 0x00;
  messagecurrent[6] = 0x00;
  messagecurrent[7] = 0x00;
}

void set_mode_crank()
{
  module_State = MOD_CRANK;
  stateCountDown = CRANK_WAIT / ZAPI_MESSAGE_PERIOD;
  messagecurrent[0] = 0x00;
  messagecurrent[1] = 0x00;
  messagecurrent[2] = 0x0B;                     //crank, contactor and power enabled
  messagecurrent[3] = messagecurrent[3] & 0x80; //All 0 but stuffing bit which is not changed(3.7).
  messagecurrent[4] = 0x00;
  messagecurrent[5] = 0x00;
  messagecurrent[6] = CRANK_SPEED / 20;
  messagecurrent[7] = 0x02; //Not sure about the point of this byte.
}

void set_mode_charge()
{
  module_State = MOD_CHARGE;
  charge();
}

void set_mode_power_down()
{
  module_State = MOD_POWER_DOWN;
  stateCountDown = POWER_DOWN_WAIT / ZAPI_MESSAGE_PERIOD;
  messagecurrent[0] = 0x00;
  messagecurrent[1] = 0x00;
  messagecurrent[2] = 0x02;                     //Contactor on but no power
  messagecurrent[3] = messagecurrent[3] & 0x80; //All 0 but stuffing bit which is not changed(3.7).
  messagecurrent[4] = 0x00;
  messagecurrent[5] = 0x00;
  messagecurrent[6] = 0x00;
  messagecurrent[7] = 0x00;
}

void set_mode_open_contact()
{
  //Open Contact Mode is the same as idle, but it requires a timeout before switching to fan cool down and enabling further switching
  module_State = MOD_OP_CONTACT;
  stateCountDown = OPEN_CONTACT_WAIT / ZAPI_MESSAGE_PERIOD;
  // Sending the idle message.
  messagecurrent[0] = 0x00;
  messagecurrent[1] = 0x00;
  messagecurrent[2] = 0x00;
  messagecurrent[3] = messagecurrent[3] & 0x80; //All 0 but stuffing bit which is not changed(3.7).
  messagecurrent[4] = 0x00;
  messagecurrent[5] = 0x00;
  messagecurrent[6] = 0x00;
  messagecurrent[7] = 0x00;
}

void set_mode_fan_cool_down()
{
  fanState = 1;                       //set fan variable to on
  digitalWrite(fanContPin, fanState); // turn on fans
  //Fan cool down is the same as idle but the fans are running to cool the module after charging. Can be switched to another state.
  module_State = MOD_FAN_COOL_DOWN;
  stateCountDown = FAN_COOL_DOWN_WAIT / ZAPI_MESSAGE_PERIOD;
  // Setting the idle message.
  messagecurrent[0] = 0x00;
  messagecurrent[1] = 0x00;
  messagecurrent[2] = 0x00;
  messagecurrent[3] = messagecurrent[3] & 0x80; //All 0 but stuffing bit which is not changed(3.7).
  messagecurrent[4] = 0x00;
  messagecurrent[5] = 0x00;
  messagecurrent[6] = 0x00;
  messagecurrent[7] = 0x00;
}

void print_system_status()
{
  //Print System Battery Voltage
  Serial.print("V:");
  Serial.print(battLargeV);
  //Print Engine Status
  Serial.print("\tE:");
  if (engineRunStateOn)
  {
    if (engineHighSpeedState)
    {
      Serial.print("High");
    }
    else
    {
      Serial.print("Low");
    }
  }
  else
  {
    Serial.print("Off");
  }
  //Print Fan Status
  Serial.print("\tF:");
  if (fanState)
  {
    Serial.print("On");
  }
  else
  {
    Serial.print("Off");
  }
  //Print Current Charging Current
  Serial.print("\tC:");
  Serial.print(chargingCurrent);
  //Print Current ZAPI message
  Serial.print(messagecurrent[0], HEX);
  Serial.print(" ");
  Serial.print(messagecurrent[1], HEX);
  Serial.print(" ");
  Serial.print(messagecurrent[2], HEX);
  Serial.print(" ");
  Serial.print(messagecurrent[3], HEX);
  Serial.print(" ");
  Serial.print(messagecurrent[4], HEX);
  Serial.print(" ");
  Serial.print(messagecurrent[5], HEX);
  Serial.print(" ");
  Serial.print(messagecurrent[6], HEX);
  Serial.print(" ");
  Serial.print(messagecurrent[7], HEX);
  //Print CAN message send status
  Serial.print("\tSt:");
  Serial.print(CANmessageStat);
  switch (CANmessageStat)
  {
  case CAN_OK:
    Serial.print(":CAN_OK");
    break;
  case CAN_FAILINIT:
    Serial.print(":CAN_FAILINIT");
    break;
  case CAN_FAILTX:
    Serial.print(":CAN_FAILTX");
    break;
  case CAN_MSGAVAIL:
    Serial.print(":CAN_MSGAVAIL");
    break;
  case CAN_NOMSG:
    Serial.print(":CAN_NOMSG");
    break;
  case CAN_CTRLERROR:
    Serial.print(":CAN_CTRLERROR");
    break;
  case CAN_GETTXBFTIMEOUT:
    Serial.print(":CAN_GETTXBFTIMEOUT");
    break;
  case CAN_SENDMSGTIMEOUT:
    Serial.print(":CAN_SENDMSGTIMEOUT");
    break;
  }
  //Print Module State
  Serial.print("\t");
  switch (module_State)
  {
  case MOD_IDLE:
    Serial.print("IDLE");
    break;
  case MOD_CL_CONTACT: //In the start process.
    Serial.print("CLOSE CONTACT");
    break;
  case MOD_CRANK:
    Serial.print("CRANKING");
    break;
  case MOD_CHARGE:
    Serial.print("CHARGING");
    break;
  case MOD_POWER_DOWN:
    Serial.print("POWERING DOWN");
    break;
  case MOD_OP_CONTACT: //In the stopping process.
    Serial.print("OPENING CONTACTOR");
    break;
  case MOD_FAN_COOL_DOWN:
    Serial.print("COOL DOWN");
    break;
  }
  Serial.println("");
}
