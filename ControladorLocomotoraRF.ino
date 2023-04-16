


#include <Arduino.h>
#include "SWLogic.h"

//
// DEF
//


//Set both remote and locomotive to 0 to disable radio
#define REMOTE 1
#define LOCOMOTIVE 0

#define DEBUGA 0  //Debug flag

#define STANDALONE (!REMOTE && !LOCOMOTIVE)    //Controller runs on local mode (no radio)
#define RADIO (REMOTE || LOCOMOTIVE)           //Radio is connected to the board
#define SABERTOOTH (STANDALONE || LOCOMOTIVE)  //Board is directly connected to a Sabertooth controller

//
// SABERTOOTH
//

#if SABERTOOTH
#include <USBSabertooth_NB.h>
USBSabertoothSerial C;    // create a SabertoothSerial
USBSabertooth ST(C, 128); // create a Sabertooth
#endif

//
// RADIO
//

#if RADIO
#include <RF24.h>
#define CE_PIN   9
#define CS_PIN  10
RF24 radio(CE_PIN, CS_PIN); // create a Radio
uint8_t remote_address[] = {"remot"};  // define remote identifier
uint8_t locomotive_address1[]  = {"loco1"};  // define locomotive identifier
uint8_t locomotive_address2[]  = {"moco1"};  // define locomotive identifier (Only the first character must be different between locos)
// NOTE: On the addresses above, the first character (LSB) must be different for each channel,
// the remaining 4 characters must be all equal and they represent an unique communication address 
#endif

//
// DISPLAY
//



#define OLED13
//#define OLED24

#ifdef OLED13
#define I2C_ADDRESS 0x3C  // 0X3C+SA0 - 0x3C or 0x3D
#endif

#ifdef OLED24
#define I2C_ADDRESS 0x3D  // 0X3C+SA0 - 0x3C or 0x3D
#endif

//#include "SSD1306AsciiAvrI2c.h"
//#ifdef SSD1306AsciiAvrI2c_h
//SSD1306AsciiAvrI2c oled; // create a display
//#endif

#include "SSD1306AsciiWire.h"
#ifdef SSD1306AsciiWire_h
SSD1306AsciiWire oled;   // create a display
#endif

#include "Verdana_digits_16x24.h"
#include "Symbol_8x8.h"
#include "Symbol_24x24.h"

//
// EEPROM
//

#include <EEPROM.h>

//
// LED
//

#define LEDPIN 18

//
// IO
//

#define IPIN0 4
#define IPIN1 5

#define OPIN0 7
#define OPIN1 8

//
// CHARGER
//

#define BATTCHARGEPIN A7
#define CHARGERPLUGPIN A8
#define BATTCHECKPIN 7
//#define BATTCHARGERPIN 8   // not used

//
//  ENCODER
//

#include <Encoder.h>
#define PINA 21
#define PINB 20
#define PINP 19
Encoder encoder( PINA, PINB, PINP );   // create an encoder

//
//  SWITCH
//

#define SWPIN 6
bool remoteSwitch = false;

//
// COMMAND
//

//#include "SWCommand.h"

#ifdef SWCommand_H
SWCommand command;
int a=0;
int b=0;
#endif



////////////////////////////////////////////////////////////////////////////////
// GLOBAL DEFS

enum MainMode
{
  MainModeNone=0,
  MainModeDefault,
  MainModeConfig,
};

enum MotorMode
{
  MotorModeNone=0,
  MotorModeStop,
  MotorModeForward,
  MotorModeReverse,
  MotorModeWait,
};

enum RadioMode
{
  RadioModeNone=0,
  RadioModeLocal,
  RadioModeRemote,
  RadioModeRemoteError,
};

enum ConfigMode
{
  ConfigModeNone=0,
  ConfigModeSelect,
  ConfigModeEdit,  
};

enum ScreenMode
{
  ScreenModeNone=0,
  ScreenModeSleep,
  ScreenModeDefault,
  ScreenModeConfig
};

struct RemoteData
{
  int motorSetPoint;
  MotorMode motorMode;
  byte button0;
  byte button1;
  RemoteData(byte b, MotorMode m) { memset( this, b, sizeof(*this) ); motorMode=m; }
};

RemoteData md(0,MotorModeStop);
RemoteData dsp_md(0xff,MotorModeNone);

struct LocomotiveData
{
  int battery;
  int motor1;
  int motor2;
  int current1;
  int current2;
  int temperature1;
  int temperature2;
  LocomotiveData(byte b) { memset( this, b, sizeof(*this) ); } 
};

LocomotiveData sd(0);
LocomotiveData dsp_sd(0xff);

struct SelfData
{
  MainMode mainMode;
  RadioMode radioMode;
  ConfigMode configMode;
  ScreenMode screenMode;
  int battCharge;  // only used for remote
  bool battIsCharging; // only used for remote
  SelfData( MainMode mm, RadioMode sm, ConfigMode cm, ScreenMode scm, int batt, bool isCharging ) : 
    mainMode(mm), radioMode(sm), configMode(cm), screenMode(scm), battCharge(batt), battIsCharging(isCharging) {};
};

SelfData d( MainModeDefault, RadioModeLocal, ConfigModeSelect, ScreenModeDefault, 100, false );
SelfData dsp_d( MainModeNone, RadioModeNone, ConfigModeNone, ScreenModeNone, 100, false );


//SETTINGS MENU
//TODO: Save configuration into non-volatile memory

 //Type of the configuration variables
 enum OptionType
 {
   OptionTypeNone=0,
   OptionTypeRotate,    // value rotates through the specified range
   OptionTypeShift,     // value bottoms or tops on the range
 };

 struct ConfigOption //Data struct for each option
 {
   OptionType valueType;     //Type of the value
   int value;                //The value of the option itself, stored in an integer (Use 0 and 1 for boolean)
   const int minv;
   const int maxv;
   const char *description;        //Option description that will show up in the screen, keep it short to prevent display glitches
   const char format;              //Size for the printf format string
   ConfigOption(OptionType vType, int val, const int mi, const int ma, const char* desc, const char fmt) : 
      valueType(vType), value(val), minv(mi), maxv(ma), description(desc), format(fmt) {}; //Constructor with parameters for the struct
 };
 
 //Declare options
 ConfigOption radioChannel      (OptionTypeRotate, 80, 0, 125,  "Canal Radio", '3');
 ConfigOption channelAddress    (OptionTypeRotate, '1', 0, 250, "Addreca",     '3');
 ConfigOption radioPALevel      (OptionTypeShift,   1, 0, 3,    "Potenc Radio",'1');
 ConfigOption activeBraking     (OptionTypeShift,   0, 0, 1,    "Fre Motor",   '1'); // TO DO: Make it work !
 ConfigOption maxBarCurrent     (OptionTypeShift,  20, 0, 32,   "Max Ampers",  '2');
 ConfigOption maxSpeedForward   (OptionTypeShift, 100, 0, 100,  "Max Endavant",'3');
 ConfigOption maxSpeedReverse   (OptionTypeShift, 100, 0, 100,  "Max Enrera",  '3');
 #if !REMOTE
 ConfigOption speedLimit        (OptionTypeShift, 100, 0, 100,  "Limit Veloc", '3');
 #endif
 ConfigOption naturalControl    (OptionTypeShift,   0, 0, 1,    "Ctrl Natural",'1'); 
// ConfigOption dummyDisplay3     (OptionTypeShift, 20, 0, 32, "Dummy3", '3');
// ConfigOption dummyDisplay4     (OptionTypeShift, 20, 0, 32, "Dummy4", '3');
// ConfigOption dummyDisplay5     (OptionTypeShift, 20, 0, 32, "Dummy5", '3');
// ConfigOption dummyDisplay6     (OptionTypeShift, 20, 0, 32, "Dummy6", '3');
// ConfigOption dummyDisplay7     (OptionTypeShift, 20, 0, 32, "Dummy7", '3');
// ConfigOption dummyDisplay8     (OptionTypeShift, 20, 0, 32, "Dummy8", '3');
// ConfigOption smoothThrottle(OptionTypeBool,0,"Accel. suau");


 //#define NUMVARS (10 + (RADIO!=0))   //Define the amount of settings in the menu
 
//Put all options in a table
 ConfigOption *varRefs[] = 
 { 
    &radioChannel, &channelAddress, &radioPALevel, 
    &activeBraking, &maxBarCurrent, 
    &maxSpeedForward, &maxSpeedReverse,
 #if !REMOTE
    &speedLimit,
 #endif 
    &naturalControl,
// &dummyDisplay3, 
// &dummyDisplay4, 
// &dummyDisplay5, &dummyDisplay6, &dummyDisplay7, &dummyDisplay8
};

#define NUMVARS (sizeof(varRefs)/sizeof(ConfigOption*))

int configVarIndex = 0; //Highlighted option in the menu
int configOffset = 0; //Config menu offset

bool userAction = false;


////////////////////////////////////////////////////////////////////////////////

int freeRAM()
{
  extern int __heap_start, *__brkval;
  int v;
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval );
}


////////////////////////////////////////////////////////////////////////////////
// Setup function

void setup() 
{
  delay(500);  // Wait for some time to initialize hardware

#if DEBUGA
  Serial.begin(9600);
#endif

//
// EEPROM
//
  loadSettings();
  
//
// LED
//
  pinMode( LEDPIN, OUTPUT );

//
// IO
//
  pinMode( IPIN0, INPUT_PULLUP ); 
  pinMode( IPIN1, INPUT_PULLUP ); 
#if !REMOTE
  pinMode( OPIN0, OUTPUT ); 
  pinMode( OPIN1, OUTPUT ); 
#endif

//
// CHARGER
//
#if REMOTE
  pinMode( BATTCHARGEPIN, INPUT );
  pinMode( CHARGERPLUGPIN, INPUT );
  pinMode( BATTCHECKPIN, OUTPUT );
//  pinMode( BATTCHARGERPIN, INPUT );  // not used
#endif

//
// SWITCH
//
  pinMode( SWPIN, INPUT_PULLUP );

//
// ENCODER
//
  EncoderInterrupt.begin( &encoder );

//
// SABERTOOTH
//
#if SABERTOOTH
   SabertoothTXPinSerial.begin(9600);
   C.setPollInterval(20);
   ST.async_getBattery( 1, 0 );
#endif

//
// RADIO
//
#if RADIO
  radio.begin();
  radio.enableAckPayload();          // We will be using the Ack Payload feature, so please enable it
  radio.enableDynamicPayloads();     // Ack payloads are dynamic payloads
  radio.setRetries( 6, 15 );         // Delay in 250us multiples and retries when connection fails
  //radio.setPALevel( RF24_PA_LOW );   // Using low power amplifier level
  radio.setPALevel( radioPALevel.value );   // Using low power amplifier level by default
  radio.setDataRate( RF24_1MBPS );   // Using 1MB/s of data rate
  radio.setChannel( radioChannel.value );            // Using channel 80 by default
  remote_address[4] = channelAddress.value;
  locomotive_address1[4] = channelAddress.value;
  locomotive_address2[4] = channelAddress.value;
  #if REMOTE
    radio.openWritingPipe(remote_address);             // communicate back and forth. Remote listens, Locomotive talks.
    radio.openReadingPipe(1, locomotive_address1);      // see starping example for multiple reading pipes (1 through 5 )
    //radio.openReadingPipe(2, locomotive_address2);    // TO DO : uncomment and find a way to set it for double traction
    radio.startListening();
  #elif LOCOMOTIVE
    radio.openWritingPipe(locomotive_address1);        // TO DO: find a way to set the right locomotive in case of double traction
    radio.openReadingPipe(1, remote_address);          // use pipe 1 so all 5 byte address bytes are set for reading from the remote
    radio.stopListening();
  #endif

#endif

//
// DISPLAY
//

#ifdef SSD1306AsciiWire_h
  Wire.setClock( 100000L ); // other possibilities are 400000L and 2500000L, but don't work
#endif

#ifdef SSD1306AsciiAvrI2c_h
  oled.setI2cClock( 100000L); // other possibilities are 400000L and 2500000L, but don't work
#endif

#ifdef OLED13
  oled.begin(&SH1106_128x64, I2C_ADDRESS);
#endif

//#ifdef OLED24
//  pinMode( 8, OUTPUT);  // RES Pin Display
//  digitalWrite( 8, LOW);
//  delay (500);
//  digitalWrite( 8, HIGH);
//  oled.begin(&Adafruit128x64, I2C_ADDRESS);
//#endif

}

////////////////////////////////////////////////////////////////////////////////

// Main loop
void loop() 
{

  #ifdef SWCommand_H
    docommand();
  #endif

  flashLed();
  readSwitch();

  #if REMOTE
    readBatteryCharge();
    readChargeStatus();
  #endif

//  #if REMOTE
//    inputIO();
//  #elif LOCOMOTIVE
//    outputIO();
//  #endif

  #if SABERTOOTH 
    readSabertooth();
  #elif REMOTE
    readFromLocomotive();
  #endif

  selectMode();
  
  #if LOCOMOTIVE
    readFromRemote();
  #endif  

  selectInputButtons();
  selectMotorSpeed();
  selectConfigValue();

  #if !REMOTE
    outputRelay();
  #endif

  #if SABERTOOTH
    sendMotorSpeed();
  #elif REMOTE
    writeToLocomotive();
  #endif

  #if LOCOMOTIVE
    writeToRemote();
  #endif

    updateScreenMode();
    updateDisplay();

 
  //Serial.print("RAM: ");
  //Serial.println( freeRAM() ) ;
  
  //Serial.print( "MILLIS: " );
  //Serial.println( millis() );
}


////////////////////////////////////////////////////////////////////////////////

#ifdef SWCommand_H

void docommand()
{
#ifdef SWCOMMAND_USESTRING
  if ( command.available() )
  {
    String var = command.var();
    
    SWCommandType type = command.type();
    if ( var == "var1" )
    {
      if ( type == SWCommandTypeGet )  command.print( var1 );
      else if ( type == SWCommandTypeSet ) onTime = command.value();
    }
    else if ( var == "var2" )
    {
      if ( type == SWCommandTypeGet ) command.print( var2 );
      else if ( type == SWCommandTypeSet ) offTime = command.value() ;
    }
  }

#else

  if ( command.available() )
  {
    char var = command.var();
    
    SWCommandType type = command.type();
    if ( var == 'a' )
    {
      if ( type == SWCommandTypeGet )  command.print( a );
      else if ( type == SWCommandTypeSet ) a = command.value();
    }
    else if ( var == 'b' )
    {
      if ( type == SWCommandTypeGet ) command.print( b );
      else if ( type == SWCommandTypeSet ) b = command.value() ;
    }
  }

#endif
}

#endif 

////////////////////////////////////////////////////////////////////////////////

#define StoreInitAddr 2
#define StoreSentinelAddr 0
const int StoreSentinelValue = 0x1966;

// Store settings to permanent memory
void storeSettings()
{
  EEPROM.put( StoreSentinelAddr, StoreSentinelValue );
  for ( int i=0, addr=StoreInitAddr ; i< NUMVARS ; i++ )
  {
    EEPROM.put( addr, varRefs[i]->value );
    addr += sizeof(varRefs[i]->value);
  }

  // radio.setChannel( radioChannel.value );  // FIX ME: Should we uncoment this?
  // radio.setPALevel( radioPALevel.value );  // FIX ME: Should we uncoment this?
}

// Load settings from permanent memory
void loadSettings()
{
  // only load if at least one previous store was performed
  int sentinel = 0;
  
  EEPROM.get( StoreSentinelAddr, sentinel );
  if ( sentinel != StoreSentinelValue )
    return;
  
  for ( int i=0, addr=StoreInitAddr ; i< NUMVARS ; i++ )
  {
    EEPROM.get( addr, varRefs[i]->value );
    addr += sizeof(varRefs[i]->value);
  }
}


////////////////////////////////////////////////////////////////////////////////

// Turns the LED on and off
void flashLed()
{
//  const long shortTime = 50;
//  const long longTime = 950;

  
  const long shortTime = 150;
  const long longTime = 2000-shortTime;

/*
  const long mildTime = 200;
  unsigned long now = millis();
  static unsigned long start = now;

  bool b0 = now - start > (unsigned long) longTime;
  bool b1 = now - start > (unsigned long) (longTime+shortTime);
  bool b2 = now - start > (unsigned long) (longTime+shortTime+mildTime);
  bool b3 = now - start > (unsigned long) (longTime+2*shortTime+mildTime);

  bool ledOn = (b0 && !b1) || (b2 && !b3);

  if ( b3 ) start = now;
  
  digitalWrite( LEDPIN, ledOn) ;
*/

  unsigned long now = millis();
  static unsigned long start = now;

  bool b0 = now - start > (unsigned long)longTime;
  bool b1 = now - start > (unsigned long)(longTime+shortTime);
  if ( b1 ) start = now;
  
  digitalWrite( LEDPIN, b0);
}


////////////////////////////////////////////////////////////////////////////////

void readSwitch()
{
  #if LOCOMOTIVE
      static Debouncer swDebouncer;
      bool b = digitalRead( SWPIN ); // will read 1 if nothing is connected
      if (swDebouncer.isDebounced( b, 3 )) remoteSwitch = b;
     
  #elif REMOTE
     remoteSwitch = true;
     
  #else // neither remote or locomotive
     remoteSwitch = false;
  #endif
}

////////////////////////////////////////////////////////////////////////////////

#if REMOTE
void  readBatteryCharge()
{
  const long shortTime = 50;
  const long longTime = 5000-shortTime;
  
  unsigned long now = millis();
  static unsigned long start = now;

  bool b0 = now - start > (unsigned long)longTime;
  bool b1 = now - start > (unsigned long)(longTime+shortTime);

  // send out signal to read battery for a short limited time
  digitalWrite( BATTCHECKPIN, b0 ); 

  // read battery voltage and update timer
  if ( b1 ) 
  {
     int rawCharge = analogRead( BATTCHARGEPIN );
     const int maxRawCharge = 332;   // this is about an input of  1.1v -> 1.1/3.3 * 1024 = 341 -> 
     const int minRawCharge = 280;   // this is about an input of  0.9v -> 0.9/3.3 * 1024 = 279
     if (rawCharge > maxRawCharge) rawCharge = maxRawCharge;
     if (rawCharge < minRawCharge) rawCharge = minRawCharge;
     int percentCharge = (rawCharge-minRawCharge)*100L/(maxRawCharge-minRawCharge);
     if (percentCharge < d.battCharge) d.battCharge = (d.battCharge*80L + percentCharge*20L)/100;
     else d.battCharge = percentCharge;

     // debug 
       //d.battCharge=rawCharge;
     start = now;
  }
}
#endif

#if REMOTE
void readChargeStatus()
{
  static SWTimer chargeTimer;
  
  int b = (analogRead( CHARGERPLUGPIN ) > 300);

  chargeTimer.timer( b, 1000 );
  d.battIsCharging = chargeTimer.value();
}
#endif


////////////////////////////////////////////////////////////////////////////////

//#if REMOTE
//void inputIO()
//{
//  static Debouncer swDebouncer0, swDebouncer1;
//  
//  bool b0 = !digitalRead( IPIN0 );
//  if ( swDebouncer0.isDebounced( b0, 3 ) ) md.io0 = b0;
//  
//  bool b1 = !digitalRead( IPIN1 );
//  if ( swDebouncer1.isDebounced( b1, 3 ) )
//  {
//    if ( b1 && dsp_md.io1 == md.io1 ) md.io1 = !md.io1;
//    if ( !b1 ) dsp_md.io1 = md.io1;
//  }
//}
//#endif

//#if LOCOMOTIVE || STANDALONE
//void outputRelay()
//{ 
//  digitalWrite( OPIN0, md.io0 );
//  digitalWrite( OPIN1, md.io1 );
//}
//#endif


////////////////////////////////////////////////////////////////////////////////

#if SABERTOOTH
// Asynchronously reads data from the motor controller
void readSabertooth()
{
  int result = 0;
  int context = 0;
  
  if ( C.reply_available( &result, &context ) )
  {
      switch ( context )
      {

        case SABERTOOTH_GET_ERROR:
        case SABERTOOTH_GET_TIMED_OUT:

           #if DEBUGA
           Serial.print( "ERROR " );
           Serial.print( millis() );
           Serial.print( " *** ");
           Serial.println( result );
           #endif
           
           ST.async_getBattery( 1, 0 );  // try again from the beginning
           break;
           
        case 0:  // get Battery  
           sd.battery = result;
           ST.async_get( 'M', 1, 1 );   // request motor 1 value voltage with context 1
           break;
           
        case 1: // get value
           sd.motor1 = result;
           ST.async_get( 'M', 2, 2 );  // request motor 2 value voltage with context 2
           break;

        case 2: // get value
           sd.motor2 = result;
           ST.async_getCurrent( 1, 3 );  // request motor 1 current with context 3
           break;
           
        case 3: // get current
           sd.current1 = (sd.current1*9 + result*1)/10;
           ST.async_getCurrent( 2, 4 );  // request motor 2 current with context 4
           break;

        case 4: // get current
           sd.current2 = (sd.current2*9 + result*1)/10;
           ST.async_getTemperature( 1, 5 );  // request temperature 1 with context 5
           break;

        case 5: // get temperature
           sd.temperature1 = result;
           ST.async_getTemperature( 2, 6 );  // request temperature 2 voltage with context 6
           break;
           
        case 6: // get temperature
           sd.temperature2 = result;
           ST.async_getBattery( 1, 0 );  // request battery voltage with context 0
           break;
     } 
  }
}

#endif


////////////////////////////////////////////////////////////////////////////////

#if REMOTE
//Receive data from the locomotive (Locomotive)
void readFromLocomotive()
{
  static SWTimer radioTimer;
  static SWKeep radioOk;
  
  if ( d.radioMode == RadioModeRemote || d.radioMode == RadioModeRemoteError )
  {
    uint8_t pipeNum;
    bool b = radio.available( &pipeNum );
    if ( b )
    { 
      radio.read(&sd, sizeof(sd));  // TO DO : use pipeNum to account for locomotive display data
    
      #if DEBUGA
      Serial.print("Got payload from locomotive from pipe:");
      Serial.println( pipeNum );
      #endif
    }

    radioTimer.timer( !b, 3000 );
    
    // bool error = radioTimer.value();
//    if ( !error ) d.radioMode = RadioModeRemote;
//    else if ( d.radioMode == RadioModeRemote ) d.radioMode = RadioModeRemoteError, md.motorMode = MotorModeStop;
    
    if ( radioOk.keep( b, radioTimer.value() ) ) d.radioMode = RadioModeRemote;
    else if ( d.radioMode == RadioModeRemote ) d.radioMode = RadioModeRemoteError, md.motorMode = MotorModeStop;
  }
}

#endif


////////////////////////////////////////////////////////////////////////////////

//Determines the mode of each component: motor, menu, radio, config and screen.
//See the list of states for each mode in GLOBAL DEFS section.
void selectMode()
{
  userAction = false;

// Button states

  bool pb = encoder.button();
  //Button timers: time between first and second push and length of the push
  static SWTimer tempsRepeticio, tempsPuls;
  //Timer that keeps track of how long the loco has been idle and with no user input
  static SWTimer tempsZeroMarxa;

  //Button works using a state machine
  static byte finite = 0;     //Current input
  static byte preFinite = 0;  //Previous input
  static bool prePb = false;
  
  bool pbOne, pbOneStrong, pbTime, pbTwo;

  if ( finite == 3 )
  {
    if ( prePb && !pb ) finite = 0 ;
  }
  
  if ( finite == 2 )
  {
    if ( !prePb && pb ) finite = 3;
    if ( tempsRepeticio.value() ) finite = 0; 
  }

  if ( finite == 1 )
  {
    if ( prePb && !pb ) finite = 2;
    if ( tempsPuls.value() ) finite = 0;
  }

  if ( finite == 0 )
  {
    if ( !prePb && pb ) finite = 1;
  }

  pbOne = pbOneStrong = pbTime = pbTwo = false;
  if ( preFinite == 1 && finite == 2 ) pbOne = true;        // Button has been pushed once and might be pushed again 
  if ( preFinite == 2 && finite == 0 ) pbOneStrong = true;  // Button has been pushed only once
  if ( preFinite == 1 && finite == 0 ) pbTime = true;       // Button has been held for some time
  if ( preFinite == 2 && finite == 3 ) pbTwo = true;        // Button has been pushed twice in succession

  // Start the timers if conditions are met
  tempsRepeticio.timer( finite == 2, 300 ); // 300ms timer that only starts if there was a previous button input
  tempsPuls.timer( finite == 1, 1000 ); // 1000ms timer counting the length of the push
  
  // Save previous state for next iteration
  preFinite = finite;
  prePb = pb;


//  #if DEBUGA
//  static bool oldpbOne = pbOne;
//  if ( oldpbOne != pbOne )
//  {
//    Serial.print( " One ");
//    Serial.println( pbOne );
//    oldpbOne = pbOne;
//  }
//
//  static bool oldpbTime = pbTime;
//  if ( oldpbTime != pbTime )
//  {
//    Serial.print( " Time ");
//    Serial.println( pbTime );
//    oldpbTime = pbTime;
//  }
//
//  static bool oldpbTwo = pbTwo;
//  if ( oldpbTwo != pbTwo )
//  {
//    Serial.print( " Two ");
//    Serial.println( pbTwo );
//    oldpbTwo = pbTwo;
//  }
//
//  static bool oldpbOneStrong = pbOneStrong;
//  if ( oldpbOneStrong != pbOneStrong )
//  {
//    Serial.print( " OneStrong ");
//    Serial.println( pbOneStrong );
//    oldpbOneStrong = pbOneStrong;
//  }
//  #endif

// Determine the mode according to user actions

  if ( !remoteSwitch || d.battIsCharging ) d.radioMode = RadioModeLocal, md.motorMode = MotorModeStop; // Turn off RC if remoteSwitch flag is off or battery is charging
  else if ( d.radioMode == RadioModeLocal ) d.radioMode = RadioModeRemoteError;  // Preserving RadioRemote if it existed, otherwise start with error

  if ( d.mainMode == MainModeConfig ) // Display mode is on config menu
  {
    if ( d.configMode == ConfigModeSelect ) // When navigating through the settings menu
    {
      // Edit the selected option
      if ( pbOne && configVarIndex < NUMVARS ) d.configMode = ConfigModeEdit;
      
      // Exit to default mode if the selected option is the last one
      if ( pbOneStrong && configVarIndex == NUMVARS /*Sortir*/ ) 
      {
          md.motorMode = MotorModeStop, d.mainMode = MainModeDefault;
          storeSettings();
      }
    }
    else if ( d.configMode == ConfigModeEdit ) // When adjusting a setting
    {
      if ( pbOne ) d.configMode = ConfigModeSelect; // Confirm setting and go back to settings menu
    }
  }

  else if ( d.mainMode == MainModeDefault ) // Normal operation display
  {
     bool autorize =   // Button will be responsible only for remote or standalone locomotive 
          ( REMOTE && d.radioMode == RadioModeRemote ) ||
          ( LOCOMOTIVE && (d.radioMode == RadioModeRemoteError || d.radioMode == RadioModeLocal) );
  
    if ( md.motorMode == MotorModeStop )  // If motor mode is neutral:
    {
      if ( naturalControl.value == 0 )
      {
        if ( pbOneStrong && autorize ) md.motorMode = MotorModeWait; // Set motor mode to wait if button is pushed once
      }
      else
      {
        if ( pbOneStrong && autorize ) md.motorMode = MotorModeForward; // Set motor mode to forward if button is pushed once
        if ( pbTwo && autorize ) md.motorMode = MotorModeReverse;       // Set motor mode to reverse if button is pushed twice
      }

      if ( pbTime ) d.mainMode = MainModeConfig;                      // Open up config display if button is pushed and held
    }
    
    else if ( (md.motorMode == MotorModeForward || md.motorMode == MotorModeReverse || md.motorMode == MotorModeWait) ) // If motor mode is not neutral:
    {
      if ( pbOneStrong || pbTwo ) md.motorMode = MotorModeStop; // Turn off motor and set the mode to neutral
    }
  }

 // Inactivity timer, set motor to neutral after a while with zero setpoint
  tempsZeroMarxa.timer( ( (md.motorMode == MotorModeForward || md.motorMode == MotorModeReverse) && md.motorSetPoint == 0 ) || md.motorMode == MotorModeWait, 10000 ); // Start 10 second timer for inactivity
  if ( tempsZeroMarxa.value() ) md.motorMode = MotorModeStop; // Set motor mode to neutral

  if ( finite != 0 ) userAction = true; // Action has been performed by the user
}


////////////////////////////////////////////////////////////////////////////////

#if LOCOMOTIVE
// Receive orders from the remote (RC) unit through ack payload
void readFromRemote()
{
  static SWTimer radioTimer;
  static SWKeep radioOk;

  if ( d.radioMode == RadioModeRemote || d.radioMode == RadioModeRemoteError )
  {
    bool b = radio.available();
    if ( b )
    { 
      radio.read(&md, sizeof(md));
    
      #if DEBUGA
      Serial.println("Got Ack payload from remote");
      #endif
    }

    radioTimer.timer( !b, 3000 );
    
    if ( radioOk.keep( b, radioTimer.value() ) ) d.radioMode = RadioModeRemote;
    else if ( d.radioMode == RadioModeRemote ) d.radioMode = RadioModeRemoteError, md.motorMode = MotorModeStop;
  }
}

#endif


////////////////////////////////////////////////////////////////////////////////

// Updates input button states.
void selectInputButtons()
{
  static Debouncer swDebouncer0, swDebouncer1;

  bool authorize =
         ( REMOTE && d.radioMode == RadioModeRemote ) ||
         ( LOCOMOTIVE && (d.radioMode == RadioModeRemoteError || d.radioMode == RadioModeLocal) );

  if ( authorize )
  {
    bool b0 = !digitalRead( IPIN0 );
    if ( swDebouncer0.isDebounced( b0, 3 ) ) md.button0 = b0;
  
    bool b1 = !digitalRead( IPIN1 );
    if ( swDebouncer1.isDebounced( b1, 3 ) )
    {
      if ( b1 && dsp_md.button1 == md.button1 ) md.button1 = !md.button1;
      if ( !b1 ) dsp_md.button1 = md.button1;
    }
  }
}

////////////////////////////////////////////////////////////////////////////////

// Updates motor speed set-point according to user input and motor states.
void selectMotorSpeed()
{
  //Get the amount of steps the wheel has been rotated by the user
  int delta = encoder.delta();

  if ( d.mainMode == MainModeDefault )
  { 
    bool authorize =
          ( REMOTE && d.radioMode == RadioModeRemote ) ||
          ( LOCOMOTIVE && (d.radioMode == RadioModeRemoteError || d.radioMode == RadioModeLocal) );
 
    if ( md.motorMode == MotorModeStop )
    {
      md.motorSetPoint = 0; //Shut down the motor
    }  

    if ( md.motorMode == MotorModeWait ) // When motor is waiting for direction
    {
      if ( authorize && delta != 0 )
      {
        if ( delta > 0 ) md.motorMode = MotorModeForward;
        if ( delta < 0 ) md.motorMode = MotorModeReverse;
      }
    }
    
    if ( md.motorMode == MotorModeForward || md.motorMode == MotorModeReverse ) //When motor is running
    {
      if ( authorize && delta != 0 )
      {
        int efectiveDelta = delta;
        if ( naturalControl.value == 0 ) efectiveDelta = (md.motorMode == MotorModeReverse?-delta:delta);
        md.motorSetPoint = md.motorSetPoint + efectiveDelta; //Change the motor speed according to user input
        if ( md.motorSetPoint < 0 ) md.motorSetPoint = 0;     //Avoid underflow
        //if ( md.motorSetPoint > 100 ) md.motorSetPoint = 100; //Avoid overflow
        int maxSetpoint = maxSpeedReverse.value ;
        if ( md.motorMode == MotorModeForward ) maxSetpoint = maxSpeedForward.value;
        if ( md.motorSetPoint > maxSetpoint ) md.motorSetPoint = maxSetpoint; //Avoid overflow
      }
    }
  }
  
  if ( delta != 0 ) userAction = true; // An action has been performed by the user
}


////////////////////////////////////////////////////////////////////////////////

// Selects the option in the config menu based on user input.
void selectConfigValue()
{
  static SWTimer tempsConfig;
  //Obtain rotation from the wheel
  int delta = encoder.deltaTick();

  if ( d.mainMode == MainModeConfig )
  { 
    if ( delta != 0 )
    {
      if ( d.configMode == ConfigModeSelect ) //Main options menu
      {
        configVarIndex = configVarIndex + delta;
        if ( configVarIndex < 0 ) configVarIndex = 0;
        if ( configVarIndex > NUMVARS ) configVarIndex = NUMVARS;
      }
      else if ( d.configMode == ConfigModeEdit ) //Option is being edited
      {
        if (configVarIndex < NUMVARS ) 
        {
          int value = varRefs[configVarIndex]->value + delta;
          const int minv = varRefs[configVarIndex]->minv;
          const int maxv = varRefs[configVarIndex]->maxv;
   
          switch ( varRefs[configVarIndex]->valueType )
          {
            case OptionTypeRotate:
              if ( value > maxv ) value = minv;
              if ( value < minv ) value = maxv;
              break;
              
            case OptionTypeShift:
              if ( value > maxv ) value = maxv;
              if ( value < minv ) value = minv;
              break;
              
            default : break;        
          }
          varRefs[configVarIndex]->value = value;
          #if (!STANDALONE)
          if ( varRefs[configVarIndex] == &radioChannel ) radio.setChannel(value); // FIX ME: should we set it right away here or wait until save settings 
          if ( varRefs[configVarIndex] == &channelAddress ) 
          {
            remote_address[4] = value;
            locomotive_address1[4] = value;
            // FIX ME: should we set it right away here or wait until save settings
            #if REMOTE
               radio.openWritingPipe(remote_address);             // communicate back and forth. Remote listens, Locomotive talks.
               radio.openReadingPipe(1, locomotive_address1);      // see starping example for multiple reading pipes (1 through 5 )
              //radio.openReadingPipe(2, locomotive_address2);    // TO DO : uncomment and find a way to set it for double traction
              radio.startListening();
            #elif LOCOMOTIVE
              radio.openWritingPipe(locomotive_address1);        // TO DO: find a way to set the right locomotive in case of double traction
              radio.openReadingPipe(1, remote_address);          // use pipe 1 for reading from the remote
            #endif
          }
          if ( varRefs[configVarIndex] == &radioPALevel ) radio.setPALevel(value); // FIX ME: should we set it right away here or wait until save settings
          
          #endif
        }
      }
    }
  }

  if ( delta != 0 ) userAction = true; //An action has been performed by the user

  //Inactivity timer: exit from config mode after 20s of inactivity
  tempsConfig.timer( !userAction && d.mainMode == MainModeConfig, 20000 );
  if ( tempsConfig.value() ) md.motorMode = MotorModeStop, d.mainMode = MainModeDefault;
}

////////////////////////////////////////////////////////////////////////////////

#if !REMOTE
void outputRelay()
{ 
  digitalWrite( OPIN0, md.button0 );
  digitalWrite( OPIN1, md.button1 );
}
#endif

////////////////////////////////////////////////////////////////////////////////

#if SABERTOOTH
// Send the specified speed to the Sabertooth controller
void sendMotorSpeed()
{
  static int preMotorSetpoint = 0;
  static SWTimer timMotor;

  bool timMotorValue = timMotor.value();
  bool sendNow = (preMotorSetpoint != md.motorSetPoint || timMotorValue);
  
  preMotorSetpoint = md.motorSetPoint;
  timMotor.timer( !sendNow, 1000 );
  
  if ( sendNow )
  {   
    int sign = 0;
    if ( md.motorMode == MotorModeForward ) sign = 1;
    if ( md.motorMode == MotorModeReverse ) sign = -1;
    ST.freewheel( '*', md.motorMode == MotorModeStop ); // Turn on freewheeling when mode is Stop.
    ST.motor    ( '*', (2047L*md.motorSetPoint*speedLimit.value)/10000L * sign ); // Set both motor speeds
#if DEBUGA
    Serial.print( "MotorSpeed: " );
    Serial.println( md.motorSetPoint );
#endif
    ST.power( 1, md.motorMode == MotorModeForward ? 2047 : -2047 ) ;  // Set power output acording to motor direction
    ST.power( 2, md.motorMode == MotorModeReverse ? 2047 : -2047 ) ;  // Set power output acording to motor direction
  }
}

#endif


////////////////////////////////////////////////////////////////////////////////

#if LOCOMOTIVE

#define RADIO_POLL_TIME 10

// Send data to remote (RC) device
void writeToRemote()
{
  unsigned long now = millis();
  static unsigned long then = 0;
  if ( now - then  > RADIO_POLL_TIME )
  {
    then = now;

    #if DEGUGA
    Serial.println( "Now sending to Remote");
    #endif
    
    radio.writeFast(  &sd, sizeof(sd), false ) ; // request ack
    radio.txStandBy();
  }
}

#endif

////////////////////////////////////////////////////////////////////////////////

#if REMOTE

// Send commands to locomotive (Locomotive) device as an ack payload
void writeToLocomotive()
{
  radio.flush_tx();
  radio.writeAckPayload( 1, &md, sizeof(md) );    
    
  #if DEGUGA
  Serial.println( "Wrote ack payload for Locomotive");
  #endif
}

//void writeToLocomotive()
//{
//  unsigned long now = millis();
//  static unsigned long then = 0;
//  if ( now - then  > RADIO_POLL_TIME )
//  {
//    then = now;
//    
//    radio.flush_tx();
//    radio.writeAckPayload( 1, &md, sizeof(md) );    
//    
//    #if DEGUGA
//    Serial.println( "Wrote ack payload for Locomotive");
//    #endif
//  }
//}

#endif


////////////////////////////////////////////////////////////////////////////////
// Display related functions
////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////

// Updates the screen based on current mode (default, sleep or config).
void updateScreenMode()
{
  static SWTimer sleepTimer;

  if ( d.mainMode == MainModeConfig ) d.screenMode = ScreenModeConfig; //Change the screen to config menu
  else d.screenMode = ScreenModeDefault; //Change screen mode to default

  //Inactivity timer that turns off the screen after 120s of inactivity
  sleepTimer.timer( !userAction && md.motorMode == MotorModeStop, 120000 );
  if ( sleepTimer.value() ) d.screenMode = ScreenModeSleep;
}

////////////////////////////////////////////////////////////////////////////////

// Updates and refreshes the screen.
void updateDisplay()
{ 
  bool refresh = (dsp_d.screenMode != d.screenMode);
  dsp_d.screenMode = d.screenMode;
  
  //Different methods with different modes
  switch ( d.screenMode )
  {
    //Inactive screen
    case ScreenModeSleep:
      updateDisplaySleep( refresh );
      break;

    //Default screen
    case ScreenModeDefault:
      updateDisplayDefault( refresh );
      break;

    //Options screen
    case ScreenModeConfig:
      updateDisplayConfig( refresh );
      break;
  }
}

////////////////////////////////////////////////////////////////////////////////

// Clears the screen when entering sleep.
void updateDisplaySleep( bool refresh )
{
  if ( refresh )
    oled.clear();
}

////////////////////////////////////////////////////////////////////////////////
// Updates the default mode screen.
void updateDisplayDefault( bool refresh )
{
//---SCREEN POSITIONS FOR EACH ELEMENT---

// Locomotive Battery
#if REMOTE
#define BATCOL 0
#define BATROW 6
#else
#define BATCOL 64
#define BATROW 0
#endif

// Remote Battery
#define BATCHARGCOL 70
#define BATCHARGROW 0

//Temperatures for both motor drivers
#define TEMP1COL 0
#define TEMP1ROW 0

//Motor effective thrust, in percentage
#define M2COL 0
#define M2ROW 7

//Motor currents, in amperes
#define C1COL 64
#define C1ROW 6
#define C2COL 64
#define C2ROW 7

//Amperimeter bars
#define BAR1COL 112
#define BAR1ROW 7
#define BAR2COL 120
#define BAR2ROW 7

//Main power indicator
#define POTCOL 2
#define POTOFFS 22
#define POTWIDTH 20
#define POTROW 2
#define POTSYMROW 4

//Radio status
#define RADIOCOL 0// 104
#define RADIOROW 2

//Icon and unit spacing
#define ICONGAP 11
#define UNITGAP 3

  //Clear previous screen
  if ( refresh )
  {
    oled.clear();
  }  
  
  //Check radio status
  bool radioChanged = false;
  if ( dsp_d.radioMode != d.radioMode )
  {
    dsp_d.radioMode = d.radioMode;
    radioChanged = true;
  }

  refresh = refresh || (REMOTE && radioChanged) ;
  bool showError = REMOTE && (d.radioMode == RadioModeRemoteError || d.radioMode == RadioModeLocal);

  // Charge levels always displayed
  #if REMOTE

  if (dsp_d.battIsCharging != d.battIsCharging || refresh )
  {
    dsp_d.battIsCharging = d.battIsCharging;
    oled.setFont(Symbol_8x8);
    oled.setCursor( BATCHARGCOL, BATCHARGROW );
    oled.print ( d.battIsCharging ? 'A' : '@' );
  }
  
  if (dsp_d.battCharge != d.battCharge || refresh )
  {
    dsp_d.battCharge = d.battCharge;
    oled.setFont(System5x7);
    oled.setCursor( BATCHARGCOL+ICONGAP, BATCHARGROW );
    oled_printf( "%3d", d.battCharge );
  } 
  #endif

  if ( showError )
  {
    if ( refresh )
    {
      // clear specific display sections
      oled.setFont( Symbol_8x8 );
      oled.clearField( TEMP1COL, TEMP1ROW, 8);

      oled.setCursor( BATCOL, BATROW );
      oled.clearToEOL();
    
      oled.setCursor( M2COL, M2ROW );
      oled.clearToEOL();

      oled.clearField( POTCOL+POTWIDTH+POTOFFS+UNITGAP+17*3, POTSYMROW, 1); // note that this will clear a 8x8 area for a 5x7 character
      
      //Radio status
      if ( radioChanged || refresh )
      {  
        oled.setCursor( RADIOCOL, RADIOROW );
        oled.print( d.radioMode == RadioModeRemoteError ? '<' : ' '  );
      }
      
      // clear stop icon
      if ( REMOTE && (radioChanged || refresh) )
      {
        oled.setFont(Symbol_24x24);
        if (d.radioMode == RadioModeLocal ) oled.clearField( POTCOL+POTWIDTH/2, POTROW, 1 ); // this clears the stop icon
        if (d.radioMode == RadioModeRemoteError )
        {
          oled.setCursor( POTCOL+POTWIDTH/2, POTROW );
          oled.print( '0' ); // this displays a stop icon
        }
      }
      
      // clear bars
      drawBar( BAR1COL, BAR1ROW, 8, 100, 0 );
      drawBar( BAR2COL, BAR2ROW, 8, 100, 0 );

      // display train set
      oled.setFont(Verdana_digits_16x24);
      oled.setCursor( POTCOL+POTWIDTH+POTOFFS, POTROW );
      oled.print( "==:" ); // this displays a train set
    }
  }
  else
  {
    if ( refresh )
    {
      //Draw the new frame static components
      oled.setFont( Symbol_8x8 );

      oled.setCursor( TEMP1COL, TEMP1ROW );
      oled.print ( ':' );   
    
      oled.setCursor( BATCOL, BATROW );
      oled.print ( ';' );
 
      oled.setCursor( M2COL, M2ROW );
      oled.print ( '=' );

      oled.setCursor( C1COL, C1ROW );
      oled.print ( '?' );
    
      oled.setCursor( C2COL, C2ROW );
      oled.print ( '?' );

      oled.setFont( System5x7 );
    
      oled.setCursor( POTCOL+POTWIDTH+POTOFFS+UNITGAP+17*3, POTSYMROW);
      oled.print ( '%' );
    
      oled.setCursor( TEMP1COL+ICONGAP+UNITGAP+5*6, TEMP1ROW );
      oled.print( 'C' );

      oled.setCursor( BATCOL+ICONGAP+UNITGAP+4*6, BATROW );
      oled.print( 'V' );

 #if REMOTE
      oled.setCursor( BATCHARGCOL+ICONGAP+UNITGAP+3*6, BATCHARGROW );
      oled.print( '%' );
 #endif

      oled.setCursor( M2COL+ICONGAP+UNITGAP+5*6, M2ROW );
      oled.print( '%' );

      oled.setCursor( C1COL+ICONGAP+UNITGAP+4*6, C1ROW );
      oled.print( 'A' );

      oled.setCursor( C2COL+ICONGAP+UNITGAP+4*6, C2ROW );
      oled.print( 'A' );
    }
     
    //Draw the new frame's dynamic components
    oled.setFont(System5x7);

    //Temperatures
    if (dsp_sd.temperature1 != sd.temperature1 || dsp_sd.temperature2 != sd.temperature2 || refresh )
    {
      dsp_sd.temperature1 = sd.temperature1;
      dsp_sd.temperature2 = sd.temperature2;
      oled.setCursor( TEMP1COL+ICONGAP, TEMP1ROW );
      oled_printf( "%02d/%02d", sd.temperature1, sd.temperature2 );
    }
  
    //Battery voltage
    if (dsp_sd.battery != sd.battery || refresh )
    {
      dsp_sd.battery = sd.battery;
      oled.setCursor( BATCOL+ICONGAP, BATROW );
      oled_printf( "%02d.%01d", sd.battery/10, sd.battery%10 );
    }

    //Motor speed
    if ( dsp_sd.motor1 != sd.motor1 || dsp_sd.motor2 != sd.motor2 || refresh )
    {
      dsp_sd.motor1 = sd.motor1;
      dsp_sd.motor2 = sd.motor2;

      int scaled1 = (abs(sd.motor1)*100L+1024)/2047;
      int scaled2 = (abs(sd.motor2)*100L+1024)/2047;
      oled.setCursor( M2COL+ICONGAP, M2ROW );
      if ( scaled1 < 100 && scaled2 < 100 ) oled_printf( "%02d/%02d", scaled1, scaled2 );
      else oled.print( "MAXIM" );
    }

    //Motor 1 current
    if (dsp_sd.current1 != sd.current1 || refresh )
    {
      dsp_sd.current1 = sd.current1;

      int scaled = abs(sd.current1); // abs(current1)*1000L/2047;
      if ( scaled > 999 ) scaled = 999;
           
      oled.setCursor( C1COL+ICONGAP, C1ROW );
      oled_printf( "%02d.%01d", scaled/10, scaled%10 );

      drawBar( BAR1COL, BAR1ROW, 8, 10*maxBarCurrent.value, scaled );
    }

    //Motor 2 current
    if ( dsp_sd.current2 != sd.current2 || refresh )
    {
      dsp_sd.current2 = sd.current2;
    
      int scaled = abs(sd.current2);
      if ( scaled > 999 ) scaled = 999;
      //Digits 
      oled.setCursor( C2COL+ICONGAP, C2ROW );
      oled_printf( "%02d.%01d", scaled/10, scaled%10 );
      //Bars
      drawBar( BAR2COL, BAR2ROW, 8, 10*maxBarCurrent.value, scaled );
    }

    //Motor speed set point (in percentage)
    if ( dsp_md.motorSetPoint != md.motorSetPoint || refresh )
    {
      dsp_md.motorSetPoint = md.motorSetPoint; 
    
      oled.setFont(Verdana_digits_16x24);
      oled.setCursor( POTCOL+POTWIDTH+POTOFFS, POTROW );
      oled_printf( "%03d", md.motorSetPoint );
    }

    //Radio status
    if ( radioChanged || refresh )
    {    
      oled.setFont(Symbol_8x8);
      oled.setCursor( RADIOCOL, RADIOROW );
      oled.print( d.radioMode == RadioModeRemoteError ? '<' : '>' );
    }
  
    //Locomotive direction
    if ( dsp_md.motorMode != md.motorMode || refresh )
    { 
      char status;
      dsp_md.motorMode = md.motorMode; 
      switch ( md.motorMode )
      {
        case MotorModeStop : status = '0'; break;
        case MotorModeForward : status = '1'; break;
        case MotorModeReverse : status = '2'; break;
        default: status = 0;
      }
    
      oled.setFont(Symbol_24x24);
      oled.setCursor( POTCOL+POTWIDTH/2, POTROW );
      if ( status ) oled.print( status );
      else oled.clearField( POTCOL+POTWIDTH/2, POTROW, 1 );
    }
  }
}


////////////////////////////////////////////////////////////////////////////////
// Updates the config mode screen
void updateDisplayConfig( bool refresh )
{
//Position of the various elements on the screen
#define NAMESELCOL 0
#define NAMECOL (NAMESELCOL+8)
#define FIRSTROW 0 /*2*/
#define ENDROW 8
#define VARCOL 92
#define VARSELCOL (VARCOL+(4*6)+3)

  //Clear the previous frame
  if ( refresh )
  {
    oled.clear();
  }

  //Draw the next frame
  oled.setFont(System5x7);

  static int dsp_configOffset = -1;
  static int dsp_configVarIndex = -1;
  
  if ( dsp_configVarIndex != configVarIndex )
  {
    if (configVarIndex < configOffset+FIRSTROW)
    {
      configOffset = configVarIndex-FIRSTROW;
      refresh = true;
    }
    else if (configVarIndex >= configOffset+ENDROW )
    {
      configOffset = configVarIndex-ENDROW+1;
      refresh = true;
    }
  }

  if ( refresh )
  {
    //Draw the various options names
    for ( int i=FIRSTROW ; i < ENDROW ; i++ )
    {
      int varIndex = i-FIRSTROW+configOffset;
      if ( varIndex >= NUMVARS ) break;
      oled.setCursor( NAMECOL, i );
      oled_printf( "%-12s", varRefs[varIndex]->description );
    }

//    if ( configOffset+ENDROW-1 == NUMVARS )
//    { 
//      // add the 'SAVE & EXIT' row
//      oled.setCursor( NAMECOL, ENDROW-1 );
//      oled_printf( "%-12s","GUARDAR" );  
//      oled.clearField( VARCOL, ENDROW-1, 4 );
//    }

    if ( ENDROW-FIRSTROW+configOffset > NUMVARS )
    { 
      // add the 'SAVE & EXIT' row
      oled.setCursor( NAMECOL, FIRSTROW+NUMVARS-configOffset );
      oled_printf( "%-12s","GUARDAR" );  
      oled.clearField( VARCOL, FIRSTROW+NUMVARS-configOffset, 4 );
    }
  }
  
  //Draw the current value for each option
  static int dsp_varValues[NUMVARS];
  char format[] = { '%', '4' ,'.' ,'4' , 'd', '\0' };
  for ( int i=FIRSTROW ; i < ENDROW ; i++ )
  {
    int varIndex = i-FIRSTROW+configOffset;
    if ( varIndex >= NUMVARS ) break;
    if ( dsp_varValues[varIndex] != varRefs[varIndex]->value || refresh )
    {
      dsp_varValues[varIndex] = varRefs[varIndex]->value;
      format[3] = varRefs[varIndex]->format;
      oled.setCursor( VARCOL, i );
      oled_printf( format, varRefs[varIndex]->value );
    }
  }

  
  //Draw the cursor for the highlighted option
  if ( dsp_configVarIndex != configVarIndex || dsp_d.configMode != d.configMode || refresh )
  {
    dsp_configVarIndex = configVarIndex;
    dsp_d.configMode = d.configMode;
    for ( int i=FIRSTROW ; i < ENDROW ; i++ )
    {  
      int varIndex = i-FIRSTROW+configOffset;
      if ( varIndex > NUMVARS ) break;
      char namecurs = (d.configMode == ConfigModeSelect && configVarIndex == varIndex ? '*' : ' ');
      oled.setCursor( NAMESELCOL, i );
      oled.print( namecurs );
      
      char varcurs = (d.configMode == ConfigModeEdit && configVarIndex == varIndex ? '<' : ' ');
      oled.setCursor( VARSELCOL, i );
      oled.print( varcurs );
    }
  }
}


//// Updates the config mode screen
//void updateDisplayConfig( bool refresh )
//{
////Position of the various elements on the screen
//#define NAMESELCOL 0
//#define NAMECOL (NAMESELCOL+8)
//#define NAMEROW 0 /*2*/
//#define VARCOL 92
//#define VARSELCOL (VARCOL+(4*6)+3)
//
//  //Clear the previous frame
//  if ( refresh )
//  {
//    oled.clear();
//  }
//
//  //Draw the next frame
//  oled.setFont(System5x7);
//
//  static int dsp_configOffset = -1;
//
//  if ( refresh )
//  {
//    //Draw the various options names
//    for ( int i=0 ; i < NUMVARS ; i++ )
//    {
//      oled.setCursor( NAMECOL, i-configOffset+NAMEROW );
//      oled.print( varRefs[i]->description );
//      //oled.print( varNames[i] );
//    }
//  
//    oled.setCursor( NAMECOL, NUMVARS+NAMEROW );
//    oled.print( "GUARDAR" );
//  }
//  
//  //Draw the current value for each option
//  static int dsp_varValues[NUMVARS];
//  for ( int i=0 ; i < NUMVARS ; i++ )
//  {
//    if ( dsp_varValues[i] != varRefs[i]->value || refresh )
//    {
//      dsp_varValues[i] = varRefs[i]->value;
//      oled.setCursor( VARCOL, i+NAMEROW );
//      oled_printf( "%04d", varRefs[i]->value );
//    }
//  }
//
//  static int dsp_configVarIndex = -1;
//  
//  //Draw the cursor for the highlighted option
//  if ( dsp_configVarIndex != configVarIndex || dsp_d.configMode != d.configMode || refresh )
//  {
//    dsp_configVarIndex = configVarIndex;
//    dsp_d.configMode = d.configMode;
//    for ( int i=0 ; i <= NUMVARS ; i++ )
//    {
//      char namecurs = (d.configMode == ConfigModeSelect && configVarIndex == i ? '*' : ' ');
//      oled.setCursor( NAMESELCOL, i+NAMEROW );
//      oled.print( namecurs );
//      
//      char varcurs = (d.configMode == ConfigModeEdit && configVarIndex == i ? '*' : ' ');
//      oled.setCursor( VARSELCOL, i+NAMEROW );
//      oled.print( varcurs );
//    }
//  }
//}

////////////////////////////////////////////////////////////////////////////////

//Draws a vertical bar in the specified (col,row) with a predefined number of segments.
void drawBar( uint8_t col, uint8_t row, uint8_t segments, int maxValue, int value )
{
  int rawValue = 8L*segments*value/maxValue;

  //Temporarily store the previous font into a constant to revert the font later
  const uint8_t* font = oled.font();
  oled.setFont( Symbol_8x8 );
  
  //Draw each segment based on the value
  for ( int i=0 ; i<segments ; i++ )
  {
    char segmentValue = '0';

    if ( rawValue > 0 )
    {
        if ( rawValue > 8 ) segmentValue = '8';
        else segmentValue = '0' + rawValue;
        
        rawValue = rawValue - 8;
    }

    oled.setCursor( col, row-i );
    oled.print( segmentValue );
  }

  //Revert the font to its previous charset
  oled.setFont( font );
}

////////////////////////////////////////////////////////////////////////////////

//Draws a given string using the given format. Replaces the default sprintf
void oled_printf(const char *format, ...)
{
  char buf[20];
  va_list args;
  va_start (args, format );
  vsnprintf(buf, sizeof(buf), format, args);
  va_end (args);
  oled.print(buf);
} 

////////////////////////////////////////////////////////////////////////////////
// End of File
