


#include <Arduino.h>

//
// DEF
//

#define MASTER 1
#define SLAVE 0

#define DEBUGA 0

#define STANDALONE (!MASTER && !SLAVE)
#define RADIO (MASTER || SLAVE)
#define SABERTOOTH (STANDALONE || SLAVE)

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
const uint8_t master_address[] = {"mestr"};  // define master address
const uint8_t slave_address[]  = {"escla"};  // define slave address
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


#define NUMVARS 2
int variable1 = 0;
int variable2 = 0;

const char *varNames[NUMVARS] = { "Variable1", "Variable2" };
int *varRefs[NUMVARS] = { &variable1, &variable2 };

int configVarIndex = 0;
bool userAction = false;

int freeRAM()
{
  extern int __heap_start, *__brkval;
  int v;
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval );
}



////////////////////////////////////////////////////////////////////////////////

void setup() 
{
  delay(500);  // donem un temps al hardware per inicialitzar-se

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
  radio.enableAckPayload();          // We will be using the Ack Payload feature, so please enable it
  radio.enableDynamicPayloads();     // Ack payloads are dynamic payloads
  radio.setRetries( 6, 15 );
  radio.setPALevel( RF24_PA_LOW );
  radio.setDataRate( RF24_1MBPS );
  radio.setChannel( 80 );
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
  Wire.setClock( 100000L ); // altres possibilitats son 400000L i 2500000L, pero peten
#endif

#ifdef SSD1306AsciiAvrI2c_h
  oled.setI2cClock( 100000L); // altres possibilitats son 400000L i 2500000L, pero peten
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
      Serial.print("Got Ack paylod from slave");
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

void selectMode()
{
  userAction = false;

// estats del buto

  bool pb = encoder.button();
  static SWTimer tempsRepeticio, tempsPuls;
  static SWTimer tempsZeroMarxa;

  static byte finite = 0;
  static byte preFinite = 0;
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
  if ( preFinite == 1 && finite == 2 ) pbOne = true;
  if ( preFinite == 2 && finite == 0 ) pbOneStrong = true;
  if ( preFinite == 1 && finite == 0 ) pbTime = true;
  if ( preFinite == 2 && finite == 3 ) pbTwo = true;

  tempsRepeticio.timer( finite == 2, 300 );
  tempsPuls.timer( finite == 1, 1000 ); 
  
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

// determinem el mode de la aplicació en funcio de les accions de l'usuari


  if ( remoteSwitch == false ) d.radioMode = RadioModeLocal;
  else if ( d.radioMode == RadioModeLocal ) d.radioMode = RadioModeRemote;  // preservem el RadioRemoteError si hi era

  if ( d.mainMode == MainModeConfig )
  {
    if ( d.configMode == ConfigModeSelect )
    {
      if ( pbOne && configVarIndex < NUMVARS ) d.configMode = ConfigModeEdit;
      if ( pbOneStrong && configVarIndex == NUMVARS /*Sortir*/ ) md.motorMode = MotorModeStop, d.mainMode = MainModeDefault;
    }
    else if ( d.configMode == ConfigModeEdit )
    {
      if ( pbOne ) d.configMode = ConfigModeSelect;
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
  else if ( d.mainMode == MainModeDefault ) 
  {
     bool autorize = ( d.radioMode == RadioModeLocal ) || 
          ( MASTER && d.radioMode == RadioModeRemote ) ||
          ( SLAVE  && d.radioMode == RadioModeRemoteError );
  
    if ( md.motorMode == MotorModeStop )
    {
      if ( pbOneStrong && autorize ) md.motorMode = MotorModeForward;
      if ( pbTwo && autorize ) md.motorMode = MotorModeReverse;
      if ( pbTime ) d.mainMode = MainModeConfig;
    }
    
    else if ( (md.motorMode == MotorModeForward || md.motorMode == MotorModeReverse) )
    {
      if ( pbOneStrong || pbTwo ) md.motorMode = MotorModeStop;
    }
  }





  

  tempsZeroMarxa.timer( (md.motorMode == MotorModeForward || md.motorMode == MotorModeReverse) && md.motorSetPoint == 0, 20000 );
  if ( tempsZeroMarxa.value() ) md.motorMode = MotorModeStop;

  if ( finite != 0 ) userAction = true;
}


////////////////////////////////////////////////////////////////////////////////

#if SLAVE

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
      Serial.print("Got paylod from master");
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

void selectMotorSpeed()
{
  /*
  if ( md.mode == ModeStop || md.mode == ModeConfig )
  */

  int delta = encoder.delta();

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
      md.motorSetPoint = 0;
    }  
    
    else if ( md.motorMode == MotorModeForward || md.motorMode == MotorModeReverse )
    {
      if ( authorize && delta != 0 )
      {
        md.motorSetPoint = md.motorSetPoint + delta;
        if ( md.motorSetPoint < 0 ) md.motorSetPoint = 0;
        if ( md.motorSetPoint > 100 ) md.motorSetPoint = 100;
      }
    }
  }
  
  if ( delta != 0 ) userAction = true;
}

////////////////////////////////////////////////////////////////////////////////

void selectConfigValue()
{
  static SWTimer tempsConfig;
  
  int delta = encoder.deltaTick();
/*  
    if ( md.mode == ModeConfig )
*/
  if ( d.mainMode == MainModeConfig )
  { 
    if ( delta != 0 )
    {
      if ( d.configMode == ConfigModeSelect )
      {
        configVarIndex = configVarIndex + delta;
        if ( configVarIndex < 0 ) configVarIndex = 0;
        if ( configVarIndex > NUMVARS ) configVarIndex = NUMVARS;
      }
      else if ( d.configMode == ConfigModeEdit )
      {
        if (configVarIndex < NUMVARS ) 
          *varRefs[configVarIndex] += delta;
      }
    }
  }
  
  if ( delta != 0 ) userAction = true;

  tempsConfig.timer( !userAction && d.mainMode == MainModeConfig, 20000 );
  if ( tempsConfig.value() ) md.motorMode = MotorModeStop, d.mainMode = MainModeDefault;
}

////////////////////////////////////////////////////////////////////////////////

#if SABERTOOTH

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

void updateScreenMode()
{

  static SWTimer sleepTimer;

/*
  if ( md.mode == ModeConfig ) screenMode = ScreenModeConfig;
  else screenMode = ScreenModeDefault;
*/

  if ( d.mainMode == MainModeConfig ) d.screenMode = ScreenModeConfig;
  else d.screenMode = ScreenModeDefault;

  sleepTimer.timer( !userAction && md.motorMode == MotorModeStop, 120000 );
  if ( sleepTimer.value() ) d.screenMode = ScreenModeSleep;
}

////////////////////////////////////////////////////////////////////////////////

void updateDisplay()
{

 // static ScreenMode preScreenMode = ScreenModeNone;
  
  bool refresh = (dsp_d.screenMode != d.screenMode);
  dsp_d.screenMode = d.screenMode;

  switch ( d.screenMode )
  {
    case ScreenModeSleep:
      updateDisplaySleep( refresh );
      break;

    case ScreenModeDefault:
      updateDisplayDefault( refresh );
      break;

    case ScreenModeConfig:
      updateDisplayConfig( refresh );
      break;
  }
}

////////////////////////////////////////////////////////////////////////////////

void updateDisplaySleep( bool refresh )
{
  if ( refresh )
  {
    oled.clear();
  }
}

////////////////////////////////////////////////////////////////////////////////

void updateDisplayDefault( bool refresh )
{

#define BATCOL 64
#define BATROW 0

#define TEMP1COL 0
#define TEMP1ROW 0

#define M1COL 0
#define M1ROW 6

#define M2COL 0
#define M2ROW 7

#define C1COL 64
#define C1ROW 6

#define C2COL 64
#define C2ROW 7

#define BAR1COL 112
#define BAR1ROW 7

#define BAR2COL 120
#define BAR2ROW 7

//#define POTCOL 0
//#define POTOFFS 24
//#define POTWIDTH 20
//#define POTROW 2
//#define POTSYMROW 4

#define POTCOL 2
#define POTOFFS 22
#define POTWIDTH 20
#define POTROW 2
#define POTSYMROW 4

#define RADIOCOL 0// 104
#define RADIOROW 2

#define ICONGAP 11
#define UNITGAP 3

  if ( refresh )
  {
    oled.clear();
  }

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

  bool radioChanged = false;
  if ( dsp_d.radioMode != d.radioMode )
  {
    dsp_d.radioMode = d.radioMode;
    radioChanged = true;
  }

  refresh = refresh || (MASTER && radioChanged) ;
  bool showError = MASTER && d.radioMode == RadioModeRemoteError;

  oled.setFont(System5x7);
  
  if (dsp_sd.temperature1 != sd.temperature1 || dsp_sd.temperature2 != sd.temperature2 || refresh )
  {
    dsp_sd.temperature1 = sd.temperature1;
    dsp_sd.temperature2 = sd.temperature2;
    oled.setCursor( TEMP1COL+ICONGAP, TEMP1ROW );
    if ( showError ) oled.print( "--/--" );
    else oled_printf( "%02d/%02d", sd.temperature1, sd.temperature2 );
  }
  
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
    
  if (dsp_sd.current1 != sd.current1 || refresh )
  {
    dsp_sd.current1 = sd.current1;

    int scaled = abs(sd.current1); // abs(current1)*1000L/2047;
    if ( scaled > 999 ) scaled = 999;
           
    oled.setCursor( C1COL+ICONGAP, C1ROW );
    if ( showError ) oled.print( "--.-" );
    else oled_printf( "%02d.%01d", scaled/10, scaled%10 );

    if ( showError ) scaled = 0;
    drawBar( BAR1COL, BAR1ROW, 8, 10, scaled );
  }

  if ( dsp_sd.current2 != sd.current2 || refresh )
  {
    dsp_sd.current2 = sd.current2;
    
    int scaled = abs(sd.current2);
    if ( scaled > 999 ) scaled = 999;
    
    oled.setCursor( C2COL+ICONGAP, C2ROW );
    if ( showError ) oled.print( "--.-" );
    else oled_printf( "%02d.%01d", scaled/10, scaled%10 );

    if ( showError ) scaled = 0;
    drawBar( BAR2COL, BAR2ROW, 8, 10, scaled );
  }

  if ( dsp_md.motorSetPoint != md.motorSetPoint || refresh )
  {
    dsp_md.motorSetPoint = md.motorSetPoint; 
    
    oled.setFont(Verdana_digits_16x24);
    oled.setCursor( POTCOL+POTWIDTH+POTOFFS, POTROW );
    if ( showError ) oled.print( "==:" );
    else oled_printf( "%03d", md.motorSetPoint );
  }

  if ( radioChanged || refresh )
  {
    char c = ' ';
    if ( d.radioMode == RadioModeRemote ) c = '>';
    else if ( d.radioMode == RadioModeRemoteError ) c = '<';
    oled.setFont(Symbol_8x8);
    oled.setCursor( RADIOCOL, RADIOROW );
    oled.print( c );
  }
  
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

void updateDisplayConfig( bool refresh )
{

#define NAMESELCOL 0
#define NAMECOL (NAMESELCOL+8)
#define NAMEROW 2
#define VARCOL 92
#define VARSELCOL (VARCOL+(4*6)+3)

//  static bool dsp_back = true;
//  refresh = refresh || dsp_back;
//  dsp_back = false;

  if ( refresh )
  {
    oled.clear();
  }

  oled.setFont(System5x7);

  if ( refresh )
  {
    for ( int i=0 ; i < NUMVARS ; i++ )
    {
      oled.setCursor( NAMECOL, i+NAMEROW );
      oled.print( varNames[i] );
    }
  
    oled.setCursor( NAMECOL, NUMVARS+NAMEROW );
    oled.print( "SORTIR" );
  }
  
  static int dsp_varValues[NUMVARS];
  for ( int i=0 ; i < NUMVARS ; i++ )
  {
    if ( dsp_varValues[i] != *varRefs[i] || refresh )
    {
      dsp_varValues[i] = *varRefs[i];
      oled.setCursor( VARCOL, i+NAMEROW );
      oled_printf( "%04d", *varRefs[i] );
    }
  }

  static int dsp_configVarIndex = -1;
  //static int dsp_configMode = ConfigModeNone;
  
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

void drawBar( uint8_t col, uint8_t row, uint8_t segments, int maxValue, int value )
{
  int rawValue = 8L*segments*value/maxValue;

  const uint8_t* font = oled.font();
  oled.setFont( Symbol_8x8 );
  
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

  oled.setFont( font );
}

////////////////////////////////////////////////////////////////////////////////

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
