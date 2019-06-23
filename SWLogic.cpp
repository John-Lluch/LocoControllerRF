
#include <Arduino.h>
#include "SWLogic.h"

SWTimer::SWTimer()
{
  _start = millis();
  _time = -1;  // Negative time indicates timer deactivated
}


void SWTimer::timer( bool set, long time )
{
  if ( set && _time<0 ) //Start timer if not already started
  {
    _start = millis();
    _time = time;
  }

  if ( !set ) //Stop the timer
  {
    _time = -1;
  }
}

bool SWTimer::value() 
{
  return _time>=0 && millis()-_start >= (unsigned long)_time ;
}

bool SWTimer::onDelay( bool set, long time )
{
  timer( set, time );
  return value();
}

//
//--------------------
//

bool SWKeep::keep( bool set, bool reset )
{
  if ( set ) _state = true;
  if ( reset ) _state = false;
  return _state;
}

//
//--------------------
//

bool SWDifu::difU( bool set )
{
  _state = set && !_pre;
  _pre = set;
  return ( _state );  
}


//
//--------------------
//

bool SWDifd::difD( bool set )
{
  _state = !set && _pre;
  _pre = set;
  return ( _state );  
}

//
//--------------------
//
