//Configuration
const bool USE_ZAPI_RESET = true;
const int ZAPI_MESSAGE_PERIOD = 20;
const long CLOSE_CONTACT_WAIT = 500;     //in ms
const long CRANK_WAIT = 3000;            //in ms
const long POWER_DOWN_WAIT = 10000L;     //in ms
const long OPEN_CONTACT_WAIT = 1000;     //in ms
const long FAN_COOL_DOWN_WAIT = 300000L; //Run the fan for 5 mins (5*60*1000) after charging in ms
const long CONSTANT_CHARGE_A = 130;       //30A, Set to 0 if using variable charge. //75 is 38A, 85 is max start at 1800rpm 
const long CONSTANT_CHARGE_HIGH_A = 110;       //The generator output floats a bit, but 120 can be supported by 2800rpm, with stagger start 1800 rpm can do 90 A which is 130 on this entry.
const long CONSTANT_CHARGE_LOW_A = 85;       //Set to 0 if using variable charge. //75 is 38A 85 is max at 1800rpm
const long LOW_RPM_COUNT_DOWN_WAIT = 3000L; //in ms (30s) 10000L
const long CRANK_SPEED = 1760;
#define VOLT_RING_SIZE 2000         // Average is over message period*ring size. For 200 and 20, about 4 to 5 s.

//Variables - Constant
//Battery Specs for TPPL
//The max voltage for charging should be 2.40VPC. This can be adjusted for temperature, 
//which will be beneficial at colder temperatures. The adjustment is -0.004VPC per degree C, 
//at nominal 25 degree C. The maximum adjustment is at 0 degree C. 
//If temperature drops lower than 0 degree C, we hold at 2.50VPC.
const float bulkChargeCellVoltLimit = 2.4;                        // 2.4 V is nominal at 25 C.
const int numOfCells = 36;                                        // number of cells in the battery pack
const float maxbattVAve = bulkChargeCellVoltLimit * numOfCells; // bulk charge limit per cell, 36 cells 86.4
const float minbattVAve = 1.96 * numOfCells;                    // 60% DOD is 1.96; 80% is 1.93V (Time Ave Volt) 70.56
const int maxChargingCurrent = 130;                               //130 A

//I/O pins
MCP_CAN CAN0(46); // Set CS to pin 46 for CAN0
MCP_CAN CAN1(42); // Set CS to pin 42 for CAN1

//Current Sense Rings are on A0, A1, A3, and A4. Current 2.5V Ref signal is on A2 and A5.

//Relay 4 is burned out and pins 7,8,9 on relay 2 (2-1)
int keySwitchPin = 22; //power up the engine. Relay 1. Pin 11 - J5 connection
//int hiSpeedSwitchPin = 24; // Switch to 2800 rpm. Relay 2. Pin 8 - J5 connecion
int fanContPin = 23;   //Relay 3. Pin 11 - J6
int vBattSensPin = 6;  // vBattSense is on ADC 6
int resetZAPIPin = 24; // Relay 2. Pin 2 - J5 connecion X Relay 4. X12V output on J2 Pin 1. HIGH causes a relay to break power to ZAPI.

//Base Zapi message
byte messagebase[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
byte messageCANOpenStart[8] = {0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

//Variables - Variable
byte messagecurrent[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
byte engineRunStateOn = LOW;     // LOW = Off; HIGH = On
byte engineHighSpeedState = LOW; // LOW = 1800 rpm; HIGH = 2800 rpm
byte fanState = 0;               // 1 = on; 0 = off
float battVAve = 0;              // Variable for main battery voltage (averaged)
float battVInstant = 0;          // Variable for instanteous main battery voltage
long stateCountDown = 0;         //Used to countdown time in a state before switching. Based on message transmission increments.
int chargingCurrent = 0;         //10 Amps
int voltRing[VOLT_RING_SIZE];
int voltAsIntIn = 0;
long voltRingPosition = 0;
long voltRingSum = 0;
float voltAve = 0;

//Output message variables
byte CANmessageStat = CAN_OK;

//Module State
const int MOD_IDLE = 0;
const int MOD_CL_CONTACT = 1;
const int MOD_CRANK = 2;
const int MOD_CHARGE = 3;
const int MOD_POWER_DOWN = 4;
const int MOD_OP_CONTACT = 5;
const int MOD_FAN_COOL_DOWN = 6;
const int MOD_CHARGE_HIGH = 7;
int module_State = MOD_IDLE;

//Functions
void startCharingProcess();
bool startCAN();
void resetZAPI();
void stopEngine();
void charge();
void charge_high();
void set_mode_idle();
void set_mode_close_contact();
void set_mode_crank();
void set_mode_charge();
void set_mode_power_down();
void set_mode_open_contact();
void set_mode_fan_cool_down();
void print_system_status();
