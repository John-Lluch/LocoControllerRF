


#include <Arduino.h>

//
// DEF
//

//Set both master and slave to 0 to disable radio
#define MASTER 1
#define SLAVE 0

#define DEBUGA 0  //Debug flag

#define STANDALONE (!MASTER && !SLAVE)    //Controller runs on local mode (no radio)
#define RADIO (MASTER || SLAVE)           //Radio is connected to the board
#define SABERTOOTH (STANDALONE || SLAVE)  //Board is directly connected to a Sabertooth controller

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
const uint8_t master_address[] = {"mestr"};  // define master identifier
const uint8_t slave_address[]  = {"escla"};  // define slave identifier
#endif

//
// DISPLAY
//

//#include "SSD1306AsciiAvrI2c.h"
#include "SSD1306AsciiWire.h"

#include "Verdana_digits_16x24.h"
#include "Symbol_8x8.h"
#include "Symbol_24x24.h"

#define OLED13
//#define OLED24

#ifdef OLED13
#define I2C_ADDRESS 0x3C  // 0X3C+SA0 - 0x3C or 0x3D
#endif

#ifdef OLED24
#define I2C_ADDRESS 0x3D  // 0X3C+SA0 - 0x3C or 0x3D
#endif

//#ifdef SSD1306AsciiAvrI2c_h
//SSD1306AsciiAvrI2c oled; // create a display
//#endif

#ifdef SSD1306AsciiWire_h
SSD1306AsciiWire oled;   // create a display
#endif

//
// LED
//

#define LEDPIN 18

//
//  ENCODER
//

#include <Encoder.h>
#include "SWLogic.h"

#define pinA 21
#define pinB 20
#define pinP 19
Encoder encoder( pinA, pinB, pinP );   // create an encoder

//
//  SWITCH
//

#define SWPIN 4
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

struct MasterData
{
  int motorSetPoint;
  MotorMode motorMode;
  MasterData(byte b, MotorMode m) { memset( this, b, sizeof(*this) ); motorMode=m; }
};

MasterData md(0,MotorModeStop);
MasterData dsp_md(0xff,MotorModeNone);

struct SlaveData
{
  int battery;
  int motor1;
  int motor2;
  int current1;
  int current2;
  int temperature1;
  int temperature2;
  SlaveData(byte b) { memset( this, b, sizeof(*this) ); } 
};

SlaveData sd(0);
SlaveData dsp_sd(0xff);

struct SelfData
{
  MainMode mainMode;
  RadioMode radioMode;
  ConfigMode configMode;
  ScreenMode screenMode;
  SelfData( MainMode mm, RadioMode sm, ConfigMode cm, ScreenMode scm ) : mainMode(mm), radioMode(sm), configMode(cm), screenMode(scm) {};
};

SelfData d( MainModeDefault, RadioModeLocal, ConfigModeSelect, ScreenModeDefault );
SelfData dsp_d( MainModeNone, RadioModeNone, ConfigModeNone, ScreenModeNone );

//RadioMode RadioMode = RadioModeLocal;
//ConfigMode configMode = ConfigModeSelect;

//SETTINGS MENU
//TODO: Save configuration into non-volatile memory
#define NUMVARS 3   //Define the amount of settings in the menu

//Type of the configuration variables
enum OptionType{
  OptionTypeNone=0,
  OptionTypeInt,    //Standard integer
  OptionTypeRadio,  //Integer with a range of 0-125
  OptionTypeBool    //Boolean
};

struct configOption //Data struct for each option
{
  OptionType valueType;     //Type of the value
  int value;                //The value of the option itself, stored in an integer (Use 0 and 1 for boolean)
  char *description;        //Option description that will show up in the screen, do not make it very long to avoid display issues
  configOption(OptionType vType, int val, char* desc) : valueType(vType), value(val), description(desc); //Constructor with parameters for the struct
};

//Declare options
configOption radioChannel(OptionTypeRadio,80,"Canal radio");
configOption activeBraking(OptionTypeBool,0,"Fre motor");
configOption smoothThrottle(OptionTypeBool,0,"Accel. suau");

//Put all options in a table
const configOption *varList[NUMVARS] = { &radioChannel, &activeBraking, &smoothThrottle };

int configVarIndex = 0; //Highlighted option in the menu
bool userAction = false;

int freeRAM()
{
  extern int __heap_start, *__brkval;
  int v;
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval );
}



////////////////////////////////////////////////////////////////////////////////

//Setup function
void setup() 
{
  delay(500);  // Wait for some time to initialize hardware

#if DEBUGA
  Serial.begin(9600);
#endif

//
// LED
//
  pinMode( LEDPIN, OUTPUT );

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
  radio.enableAckPayload();               // We will be using the Ack Payload feature, so please enable it
  radio.enableDynamicPayloads();          // Ack payloads are dynamic payloads
  radio.setRetries( 6, 15 );              // Delay in 250us multiples and retries when connection fails
  radio.setPALevel( RF24_PA_LOW );        // Using low power amplifier level
  radio.setDataRate( RF24_1MBPS );        // Using 1MB/s of data rate
  radio.setChannel( radioChannel.value ); // Using channel 80 by default
  #if MASTER
    radio.openWritingPipe(master_address);             // communicate back and forth.  One listens on it, the other talks to it.
    radio.openReadingPipe(1, slave_address);
  #elif SLAVE
    radio.openWritingPipe(slave_address);
    radio.openReadingPipe(1, master_address);
    radio.startListening();
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

#ifdef OLED24
  pinMode( 8, OUTPUT);  // RES Pin Display
  digitalWrite( 8, LOW);
  delay (500);
  digitalWrite( 8, HIGH);
  oled.begin(&Adafruit128x64, I2C_ADDRESS);
#endif

}

////////////////////////////////////////////////////////////////////////////////

//Main loop
void loop() 
{

  #ifdef SWCommand_H
    docommand();
  #endif

  flashLed();
  readSwitch();

  #if SABERTOOTH 
    readSabertooth();
  #elif MASTER
    readFromSlave();
  #endif

  selectMode();
  
  #if SLAVE
    readFromMaster();
  #endif  
  
  selectMotorSpeed();
  selectConfigValue();

  #if SABERTOOTH
    sendMotorSpeed();
  #elif MASTER
    writeToSlave();
  #endif

  #if SLAVE
    writeToMaster();
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

// Turns the LED on and off
void flashLed()
{
//  const long shortTime = 50;
//  const long longTime = 950;

  
  const long shortTime = 150;
  const long longTime = 1850;

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
  #if SLAVE
      static Debouncer swDebouncer;
      bool b = digitalRead( SWPIN );
      if (swDebouncer.isDebounced( b, 3 )) remoteSwitch = b;
     
  #elif MASTER
     remoteSwitch = true;
  #else
     remoteSwitch = false;
  #endif
}

////////////////////////////////////////////////////////////////////////////////

#if SABERTOOTH
// Asynchronously reads data from the engine controller
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

#if MASTER
//Receive orders from the master (RC) unit
void readFromSlave()
{
  static SWTimer radioTimer;
  
  if ( d.radioMode == RadioModeRemote || d.radioMode == RadioModeRemoteError )
  {
    bool b = radio.available();
    if ( b )
    { 
      radio.read(&sd, sizeof(sd));
    
      #if DEBUGA
      Serial.print("Got Ack payload from slave");
      #endif
    }

    radioTimer.timer( !b, 5000 ); 
    bool error = radioTimer.value();
    
    if ( !error ) d.radioMode = RadioModeRemote;
    else if ( d.radioMode == RadioModeRemote ) d.radioMode = RadioModeRemoteError, md.motorMode = MotorModeStop;
  }
}

#endif


////////////////////////////////////////////////////////////////////////////////

//Save and apply the settings
//TODO: Save configuration into non-volatile memory
void saveSettings(){
  #if RADIO
    //Change radio channel
    if (int(radio.getChannel()) != radioChannel.value){
      radio.setChannel(radioChannel.value);
      #if SLAVE
        radio.startListening();
      #endif
    }
  #endif
}

////////////////////////////////////////////////////////////////////////////////

//Determines the mode of each component: engine, menu, radio, config and screen.
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


  if ( !remoteSwitch ) d.radioMode = RadioModeLocal; // Turn off RC if remoteSwitch flag is off
  else if ( d.radioMode == RadioModeLocal ) d.radioMode = RadioModeRemote;  // Preserving RadioRemoteError if it existed

  if ( d.mainMode == MainModeConfig ) // Display mode is on config menu
  {
    if ( d.configMode == ConfigModeSelect ) // When navigating through the settings menu
    {
      // Edit the selected option
      if ( pbOne && configVarIndex < NUMVARS ) d.configMode = ConfigModeEdit;
      // Exit to normal mode if the selected option is the last one
      if ( pbOneStrong && configVarIndex == NUMVARS /*Sortir*/ ) {
        saveSettings();
        md.motorMode = MotorModeStop; 
        d.mainMode = MainModeDefault;
      }
    }
    else if ( d.configMode == ConfigModeEdit ) // When adjusting a setting
    {
      if ( pbOne ) d.configMode = ConfigModeSelect; // Confirm setting and go back to settings menu
    }
  }

/*
  //else if ( MASTER || d.radioMode == RadioModeLocal || d.radioMode == RadioModeRemoteError )
  else if ( ( d.radioMode == RadioModeLocal ) || 
            ( MASTER && d.radioMode == RadioModeRemote ) ||
            ( SLAVE  && d.radioMode == RadioModeRemoteError ) )
  {
    if ( md.motorMode == MotorModeStop )
    {
      if ( pbOneStrong ) md.motorMode = MotorModeForward;
      if ( pbTwo ) md.motorMode = MotorModeReverse;
      if ( pbTime ) d.mainMode = MainModeConfig;
    }
    
    else if ( (md.motorMode == MotorModeForward || md.motorMode == MotorModeReverse) )
    {
      if ( pbOneStrong || pbTwo ) md.motorMode = MotorModeStop;
    }
  }
*/

//else if ( MASTER || d.radioMode == RadioModeLocal || d.radioMode == RadioModeRemoteError )
  else if ( d.mainMode == MainModeDefault ) // Normal operation display
  {
     bool autorize = ( d.radioMode == RadioModeLocal ) || 
          ( MASTER && d.radioMode == RadioModeRemote ) ||
          ( SLAVE  && d.radioMode == RadioModeRemoteError );
  
    if ( md.motorMode == MotorModeStop )  // If engine mode is neutral:
    {
      if ( pbOneStrong && autorize ) md.motorMode = MotorModeForward; // Set engine mode to forward if button is pushed once
      if ( pbTwo && autorize ) md.motorMode = MotorModeReverse;       // Set engine mode to reverse if button is pushed twice
      if ( pbTime ) d.mainMode = MainModeConfig;                      // Open up config display if button is pushed and held
    }
    
    else if ( (md.motorMode == MotorModeForward || md.motorMode == MotorModeReverse) ) // If engine mode is not neutral:
    {
      if ( (pbOneStrong || pbTwo) && activeBraking.value == 0) md.motorMode = MotorModeStop; // Turn off engine and set the mode to neutral
      else if ((pbOneStrong || pbTwo) && activeBraking.value == 1) {
        if (md.motorSetPoint > 0) md.motorSetPoint = 0; //Turn off engine and use its resistance as electric brake
        else md.motorMode = MotorModeStop; //Change mode to neutral
      }
    }
  }





  

  tempsZeroMarxa.timer( (md.motorMode == MotorModeForward || md.motorMode == MotorModeReverse) && md.motorSetPoint == 0, 20000 ); // Start 20 second timer for inactivity
  if ( tempsZeroMarxa.value() ) md.motorMode = MotorModeStop; // Set engine mode to neutral

  if ( finite != 0 ) userAction = true; // Action has been performed by the user
}


////////////////////////////////////////////////////////////////////////////////

#if SLAVE
//Receive data from the master (locomotive)
void readFromMaster()
{
  static SWTimer radioTimer;

  if ( d.radioMode == RadioModeRemote || d.radioMode == RadioModeRemoteError )
  {
    bool b = radio.available();
    if ( b )
    { 
      radio.read(&md, sizeof(md));
    
      #if DEBUGA
      Serial.print("Got payload from master");
      #endif
    }

    radioTimer.timer( !b, 5000 );
    bool error = radioTimer.value();
    if ( error == false ) d.radioMode = RadioModeRemote;
    else if ( d.radioMode == RadioModeRemote ) d.radioMode = RadioModeRemoteError, md.motorMode = MotorModeStop;
  }
}

#endif

////////////////////////////////////////////////////////////////////////////////

//Changes the engine speed according to user input and engine states.
void selectMotorSpeed()
{
  /*
  if ( md.mode == ModeStop || md.mode == ModeConfig )
  */

  //Get the amount of steps the wheel has been rotated by the user
  int delta = encoder.delta();

  //Timer and target speed for smooth thrust
  static SWTimer MotorTick;
  static int targetSpeed = md.motorSetPoint + delta;

/*
  //if ( MASTER || d.radioMode == RadioModeLocal || d.radioMode == RadioModeRemoteError )
  if ( ( d.radioMode == RadioModeLocal ) || 
       ( MASTER && d.radioMode == RadioModeRemote ) ||
       ( SLAVE  && d.radioMode == RadioModeRemoteError ) )
  {
    if ( md.motorMode == MotorModeStop )
    {
      md.motorSetPoint = 0;
    }  
    
    else if ( md.motorMode == MotorModeForward || md.motorMode == MotorModeReverse )
    {
      if ( delta != 0 )
      {
        md.motorSetPoint = md.motorSetPoint + delta;
        if ( md.motorSetPoint < 0 ) md.motorSetPoint = 0;
        if ( md.motorSetPoint > 100 ) md.motorSetPoint = 100;
      }
    }
  }
*/

  if ( d.mainMode == MainModeDefault )
  { 
    bool authorize = ( d.radioMode == RadioModeLocal ) || 
          ( MASTER && d.radioMode == RadioModeRemote ) ||
          ( SLAVE  && d.radioMode == RadioModeRemoteError );
 
    if ( md.motorMode == MotorModeStop )
    {
      md.motorSetPoint = 0; //Shut down the engine
    }  
    
    else if ( md.motorMode == MotorModeForward || md.motorMode == MotorModeReverse ) //When engine is running
    {
      //Change the engine speed according to user input
      if ( authorize && delta != 0 )
      {
        //Delay the engine throttle when smooth acceleration is on
        if (smoothThrottle.value == 1 && delta > 0){
          if ( targetSpeed < 0 ) targetSpeed = 0;     //Avoid underflow
          if ( targetSpeed > 100 ) targetSpeed = 100; //Avoid overflow
          //Increase speed by 1 percent every 200ms
          MotorTick.timer(md.motorSetPoint < targetSpeed, 200);
          if (MotorTick.value()) md.motorSetPoint++;
        }
        else md.motorSetPoint = md.motorSetPoint + delta;          

        if ( md.motorSetPoint < 0 ) md.motorSetPoint = 0;     //Avoid underflow
        if ( md.motorSetPoint > 100 ) md.motorSetPoint = 100; //Avoid overflow
      }
    }
  }
  
  if ( delta != 0 ) userAction = true; // An action has been performed by the user
}

////////////////////////////////////////////////////////////////////////////////

//Selects the option in the config menu based on user input.
void selectConfigValue()
{
  static SWTimer tempsConfig;
  //Obtain rotation from the wheel
  int delta = encoder.deltaTick();
/*  
    if ( md.mode == ModeConfig )
*/
  if ( d.mainMode == MainModeConfig )
  { 
    if ( delta != 0 )
    {
      if ( d.configMode == ConfigModeSelect ) //Main options menu
      {
        configVarIndex += delta;
        if ( configVarIndex < 0 ) configVarIndex = 0;
        if ( configVarIndex > NUMVARS ) configVarIndex = NUMVARS;
      }
      else if ( d.configMode == ConfigModeEdit ) //Option is being edited
      {
        if (configVarIndex < NUMVARS )
          if (*varList[configVarIndex].valueType == OptionTypeInt) *varList[configVarIndex].value += delta; //If value is of integer type

          else if (*varList[configVarIndex].valueType == OptionTypeRadio){  //If value is of radio type
            if (*varList[configVarIndex].value < 0) *varList[configVarIndex].value = 125;
            else if (*varList[configVarIndex].value > 125) *varList[configVarIndex].value = 0;
            else *varList[configVarIndex].value += delta;
          }

          else if (*varList[configVarIndex].valueType == OptionTypeBool){ //If value is of boolean type
            if (delta > 0) *varList[configVarIndex].value = 1;
            else if (delta < 0) *varList[configVarIndex].value = 0;
          }
      }
    }
  }

  if ( delta != 0 ) userAction = true; //An action has been performed by the user

  //Inactivity timer: after 20s exit config mode
  tempsConfig.timer( !userAction && d.mainMode == MainModeConfig, 20000 );
  if ( tempsConfig.value() ) md.motorMode = MotorModeStop, d.mainMode = MainModeDefault;
}

////////////////////////////////////////////////////////////////////////////////

#if SABERTOOTH
//Send the specified speed to the Sabertooth controller
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
    int sign = 1;
    if ( md.motorMode == MotorModeReverse ) sign = -1;
    //ST.freewheel( '*', motorSetPoint == 0 ); // Turn on freewheeling when speed is zero.
    ST.freewheel( '*', md.motorMode == MotorModeStop ); // Turn on freewheeling when mode is Stop.
    ST.motor    ( '*', (md.motorSetPoint*2047L)/100 * sign ); // Set both motor speeds
#if DEBUGA
    Serial.print( "MotorSpeed: " );
    Serial.println( md.motorSetPoint );
#endif
  }
}

#endif

////////////////////////////////////////////////////////////////////////////////

#if MASTER
//Send data to slave (RC) device
void writeToSlave()
{
  unsigned long now = millis();
  static unsigned long then = 0;
  if ( now - then  > 50 )
  {
    then = now;

    #if DEGUGA
    Serial.print( "Now sending to Slave");
    #endif
    
    radio.writeFast(  &md, sizeof(md), 0 ) ;
    radio.txStandBy();
  }
}

#endif

////////////////////////////////////////////////////////////////////////////////

#if SLAVE
//Send data to master (loco) device
void writeToMaster()
{
  unsigned long now = millis();
  static unsigned long then = 0;
  if ( now - then  > 100 )
  {
    then = now;
    
    radio.flush_tx();
    radio.writeAckPayload( 1, &sd, sizeof(sd) );    
    
    #if DEGUGA
    Serial.print( "Wrote ack payload for Master");
    #endif
  }
}

#endif

////////////////////////////////////////////////////////////////////////////////

//Update the screen based on current mode (default, sleep or config).
void updateScreenMode()
{

  static SWTimer sleepTimer;

/*
  if ( md.mode == ModeConfig ) screenMode = ScreenModeConfig;
  else screenMode = ScreenModeDefault;
*/

  if ( d.mainMode == MainModeConfig ) d.screenMode = ScreenModeConfig; //Change the screen to config menu
  else d.screenMode = ScreenModeDefault; //Change screen mode to default

  //Inactivity timer that turns off the screen after 120s of inactivity
  sleepTimer.timer( !userAction && md.motorMode == MotorModeStop, 120000 );
  if ( sleepTimer.value() ) d.screenMode = ScreenModeSleep;
}

////////////////////////////////////////////////////////////////////////////////

//Update and refresh the screen.
void updateDisplay()
{

 // static ScreenMode preScreenMode = ScreenModeNone;
  
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

//Clear the screen when entering sleep.
void updateDisplaySleep( bool refresh )
{
  if ( refresh )
  {
    oled.clear();
  }
}

////////////////////////////////////////////////////////////////////////////////

//Update the screen when on main UI.
void updateDisplayDefault( bool refresh )
{
//---SCREEN POSITIONS FOR EACH ELEMENT---
//Battery
#define BATCOL 64
#define BATROW 0

//Temperatures for both engines
#define TEMP1COL 0
#define TEMP1ROW 0

//Engine effective thrust, in percentage
#define M1COL 0
#define M1ROW 6

#define M2COL 0
#define M2ROW 7

//Engine current intensities, in amperes
#define C1COL 64
#define C1ROW 6

#define C2COL 64
#define C2ROW 7

//Amperimeter bars
#define BAR1COL 112
#define BAR1ROW 7

#define BAR2COL 120
#define BAR2ROW 7

//#define POTCOL 0
//#define POTOFFS 24
//#define POTWIDTH 20
//#define POTROW 2
//#define POTSYMROW 4

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

  //Draw the new frame's static components
  if ( refresh )
  {
    oled.setFont( Symbol_8x8 );

    oled.setCursor( TEMP1COL, TEMP1ROW );
    oled.print ( ':' );   
    
    oled.setCursor( BATCOL, BATROW );
    oled.print ( ';' );

    //oled.setCursor( M1COL, M1ROW );
    //oled.print ( '=' );
    
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

    //oled.setCursor( M1COL+ICONGAP+UNITGAP+5*6, M1ROW );
    //oled.print( '%' );

    oled.setCursor( M2COL+ICONGAP+UNITGAP+5*6, M2ROW );
    oled.print( '%' );

    oled.setCursor( C1COL+ICONGAP+UNITGAP+4*6, C1ROW );
    oled.print( 'A' );

    oled.setCursor( C2COL+ICONGAP+UNITGAP+4*6, C2ROW );
    oled.print( 'A' );
  }

  //Check radio status
  bool radioChanged = false;
  if ( dsp_d.radioMode != d.radioMode )
  {
    dsp_d.radioMode = d.radioMode;
    radioChanged = true;
  }

  refresh = refresh || (MASTER && radioChanged) ;
  bool showError = MASTER && d.radioMode == RadioModeRemoteError;

  //Draw the new frame's dynamic components
  oled.setFont(System5x7);
  
  //Temperatures
  if (dsp_sd.temperature1 != sd.temperature1 || dsp_sd.temperature2 != sd.temperature2 || refresh )
  {
    dsp_sd.temperature1 = sd.temperature1;
    dsp_sd.temperature2 = sd.temperature2;
    oled.setCursor( TEMP1COL+ICONGAP, TEMP1ROW );
    if ( showError ) oled.print( "--/--" );
    else oled_printf( "%02d/%02d", sd.temperature1, sd.temperature2 );
  }
  
  //Battery voltage
  if (dsp_sd.battery != sd.battery || refresh )
  {
    dsp_sd.battery = sd.battery;
    oled.setCursor( BATCOL+ICONGAP, BATROW );
    if ( showError ) oled.print( "--.-" );
    else oled_printf( "%02d.%01d", sd.battery/10, sd.battery%10 );   
  }

//  if (dsp_sd.motor1 != sd.motor1 || refresh )
//  {
//    dsp_sd.motor1 = sd.motor1;
//    //int scaled = (abs(sd.motor1)*1000L+1024)/2047;
//    int scaled = abs(sd.motor1)*1000L/2047;
//
//    oled.setCursor( M1COL+ICONGAP, M1ROW );
//    if ( scaled < 1000 ) oled_printf( "%02d.%01d", scaled/10, scaled%10 );
//    else oled.print( " 100" );
//  }
//
//  if ( dsp_sd.motor2 != sd.motor2 || refresh )
//  {
//    dsp_sd.motor2 = sd.motor2;
//    //int scaled = (abs(sd.motor2)*1000L+1024)/2047;
//    //oled.setCursor( M2COL+ICONGAP, M2ROW );
//    //if ( scaled < 1000 ) oled_printf( "%02d.%01d", scaled/10, scaled%10 );
//    //else oled.print( " 100" );
//
//    int scaled = (abs(sd.motor2)*100L+1024)/2047;
//    oled.setCursor( M2COL+ICONGAP, M2ROW );
//    if ( scaled < 1000 ) oled_printf( " %02d", scaled );
//    else oled.print( "100" );
//  }


//Engine real power applied
if ( dsp_sd.motor1 != sd.motor1 || dsp_sd.motor2 != sd.motor2 || refresh )
  {
    dsp_sd.motor1 = sd.motor1;
    dsp_sd.motor2 = sd.motor2;

    int scaled1 = (abs(sd.motor1)*100L+1024)/2047;
    int scaled2 = (abs(sd.motor2)*100L+1024)/2047;
    oled.setCursor( M2COL+ICONGAP, M2ROW );
    if ( showError ) oled.print( "--/--" );
    else if ( scaled1 < 100 && scaled2 < 100 ) oled_printf( "%02d/%02d", scaled1, scaled2 );
    else oled.print( "MAXIM" );
  }
  
  //Engine 1 current intensity
  if (dsp_sd.current1 != sd.current1 || refresh )
  {
    dsp_sd.current1 = sd.current1;

    int scaled = abs(sd.current1); // abs(current1)*1000L/2047;
    if ( scaled > 999 ) scaled = 999;
           
    oled.setCursor( C1COL+ICONGAP, C1ROW );
    if ( showError ) oled.print( "--.-" );
    else oled_printf( "%02d.%01d", scaled/10, scaled%10 );

    if ( showError ) scaled = 0;
    drawBar( BAR1COL, BAR1ROW, 8, 50, scaled );
  }

  //Engine 2 current intensity
  if ( dsp_sd.current2 != sd.current2 || refresh )
  {
    dsp_sd.current2 = sd.current2;
    
    int scaled = abs(sd.current2);
    if ( scaled > 999 ) scaled = 999;
    //Digits 
    oled.setCursor( C2COL+ICONGAP, C2ROW );
    if ( showError ) oled.print( "--.-" );
    else oled_printf( "%02d.%01d", scaled/10, scaled%10 );
    //Bars
    if ( showError ) scaled = 0;
    drawBar( BAR2COL, BAR2ROW, 8, 50, scaled );
  }

  //Engine steps (in percentage)
  if ( dsp_md.motorSetPoint != md.motorSetPoint || refresh )
  {
    dsp_md.motorSetPoint = md.motorSetPoint; 
    
    oled.setFont(Verdana_digits_16x24);
    oled.setCursor( POTCOL+POTWIDTH+POTOFFS, POTROW );
    if ( showError ) oled.print( "==:" );
    else oled_printf( "%03d", md.motorSetPoint );
  }

  //Radio status
  if ( radioChanged || refresh )
  {
    char c = ' ';
    if ( d.radioMode == RadioModeRemote ) c = '>';
    else if ( d.radioMode == RadioModeRemoteError ) c = '<';
    oled.setFont(Symbol_8x8);
    oled.setCursor( RADIOCOL, RADIOROW );
    oled.print( c );
  }
  
  //Engine direction
  if ( dsp_md.motorMode != md.motorMode || refresh )
  {
    dsp_md.motorMode = md.motorMode; 
    
    char status = '0';
    switch ( md.motorMode )
    {
      case MotorModeForward : status = '1'; break;
      case MotorModeReverse : status = '2'; break;
    }
    
    oled.setFont(Symbol_24x24);
    oled.setCursor( POTCOL+POTWIDTH/2, POTROW );
    oled.print( status );
  }
}

////////////////////////////////////////////////////////////////////////////////

//Prints the screen when on config mode.
void updateDisplayConfig( bool refresh )
{
//Position of the various elements on the screen
#define NAMESELCOL 0
#define NAMECOL (NAMESELCOL+8)
#define NAMEROW 2
#define VARCOL 92
#define VARSELCOL (VARCOL+(4*6)+3)

//  static bool dsp_back = true;
//  refresh = refresh || dsp_back;
//  dsp_back = false;

  //Clear the previous frame
  if ( refresh )
  {
    oled.clear();
  }

  //Draw the next frame
  oled.setFont(System5x7);

  if ( refresh )
  {
    //Draw the various options names
    for ( int i=0 ; i < NUMVARS ; i++ )
    {
      oled.setCursor( NAMECOL, i+NAMEROW );
      oled.print( *varList[i].description );
    }
  
    oled.setCursor( NAMECOL, NUMVARS+NAMEROW );
    oled.print( "SORTIR" );
  }
  
  //Draw the current value for each option
  static int dsp_varValues[NUMVARS];
  for ( int i=0 ; i < NUMVARS ; i++ )
  {
    if ( dsp_varValues[i] != *varList[i].value || refresh )
    {
      dsp_varValues[i] = *varList[i].value;
      oled.setCursor( VARCOL, i+NAMEROW );

      if (*varList[i].valueType == OptionTypeInt || *varList[i].valueType == OptionTypeRadio) oled_printf( "%04d", *varList[i].value );

      else if (*varList[i].valueType == OptionTypeBool) {
        if (*varList[i].value == 0) oled.print(" Off");
        else oled.print("  On");
      }
    }
  }

  static int dsp_configVarIndex = -1;
  //static int dsp_configMode = ConfigModeNone;
  
  //Draw the cursor for the highlighted option
  if ( dsp_configVarIndex != configVarIndex || dsp_d.configMode != d.configMode || refresh )
  {
    dsp_configVarIndex = configVarIndex;
    dsp_d.configMode = d.configMode;
    for ( int i=0 ; i <= NUMVARS ; i++ )
    {
      char namecurs = (d.configMode == ConfigModeSelect && configVarIndex == i ? '*' : ' ');
      oled.setCursor( NAMESELCOL, i+NAMEROW );
      oled.print( namecurs );
      
      char varcurs = (d.configMode == ConfigModeEdit && configVarIndex == i ? '*' : ' ');
      oled.setCursor( VARSELCOL, i+NAMEROW );
      oled.print( varcurs );
    }
  }
}

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
