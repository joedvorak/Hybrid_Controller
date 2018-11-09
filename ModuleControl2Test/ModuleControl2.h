//Configuration
const bool USE_ZAPI_RESET  = false;
const int ZAPI_MESSAGE_PERIOD = 25;
const long CLOSE_CONTACT_WAIT = 500; //in ms
const long CRANK_WAIT = 1000; //in ms
const long POWER_DOWN_WAIT = 10000L; //in ms
const long OPEN_CONTACT_WAIT = 1000; //in ms
const long FAN_COOL_DOWN_WAIT = 300000L; //Run the fan for 5 mins (5*60*1000) after charging in ms
const long CONSTANT_CHARGE_A = 30; //30A, Set to 0 if using variable charge.
const long CRANK_SPEED = 1760;

//Variables - Constant
//Battery Specs
//float bulkChargeCellVoltLimit = 2.25; //Set to 2.25V to be just below gassing voltage at 50 degC.
const float bulkChargeCellVoltLimit = 2.4; //Generator can spike to 82 easily on startup.
const int numOfCells = 36; // number of cells in the battery pack
const float maxbattLargeV = bulkChargeCellVoltLimit * numOfCells; // bulk charge limit per cell, 36 cells
const float minbattLargeV = 2.00 * numOfCells; // 2V per cell, 36 cells
const int maxChargingCurrent = 100; //100 A

//I/O pins
MCP_CAN CAN0(46);     // Set CS to pin 46 for CAN0
MCP_CAN CAN1(42);     // Set CS to pin 42 for CAN1

int keySwitchPin = 22; //power up the engine. Relay 1. Pin 11 - J5 connection
int hiSpeedSwitchPin = 24; // Switch to 2800 rpm. Relay 2. Pin 8 - J5 connecion
int fanContPin = 23; //Relay 3. Pin 11 - J6
int vBattSensPin = 6; // vBattSense is on ADC 6
int resetZAPIPin = 12; //12V output on J2 Pin 1. HIGH causes a relay to break power to ZAPI.

//Base Zapi message
byte messagebase[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
byte messageCANOpenStart[8] = {0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

//Variables - Variable
byte messagecurrent[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
byte engineRunStateOn = LOW; // LOW = Off; HIGH = On
byte engineHighSpeedState = LOW; // LOW = 1800 rpm; HIGH = 2800 rpm
byte fanState = 0; // 1 = on; 0 = off
float battLargeV = 0; // Variable for main battery voltage
long stateCountDown = 0; //Used to countdown time in a state before switching. Based on message transmission increments.
int chargingCurrent = 0; //10 Amps

//Output message variables
byte CANmessageStat = CAN_OK;

//Module State
const int  MOD_IDLE = 0;
const int  MOD_CL_CONTACT = 1;
const int  MOD_CRANK = 2;
const int  MOD_CHARGE = 3;
const int  MOD_POWER_DOWN = 4;
const int  MOD_OP_CONTACT = 5;
const int  MOD_FAN_COOL_DOWN = 6;
int module_State = MOD_IDLE;

//Functions
void startCharingProcess();
bool startCAN();
void resetZAPI();
void stopEngine();
void charge();
void set_mode_idle();
void set_mode_close_contact();
void set_mode_crank();
void set_mode_charge();
void set_mode_power_down();
void set_mode_open_contact();
void set_mode_fan_cool_down();
void print_system_status();
