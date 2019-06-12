
#include <Arduino.h>
#include "SWCommand.h"


SWCommand::SWCommand()
{
  _done = false;
  _type = SWCommandTypeNone;
  _index = 0;
}

bool SWCommand::available()
{
  if ( Serial.available() > 0 )
  {
    char chr = (char)Serial.read();
    if ( chr == '\n' )
    {
      _done = _parse();
      _index = 0;
    
      if ( !_done ) Serial.println( "Syntax Error" );
      else if ( _type == SWCommandTypeSet ) print( _value );
      
      return true;  // ja ho tenim tot tornem que vale
    }
    else if ( _index < SWCommandBuffLength )
    {
      _buff[_index++] = chr;
      _buff[_index] = '\0';
    }
  }
  return false; // no ho tenim encara
}

void SWCommand::print( long value )
{
  Serial.print( _var );
  Serial.print( " = " );
  Serial.println( value );
}

bool SWCommand::_parse()
{
  // inicialització
  int i = 0;
  char ch = '\0';  ;

  // saltem espais 
  while ( _buff[i] == ' ' ) i++;

  // determinem la variable
  if ( isalpha( _buff[i] ) )
    _var = _buff[i++];
  else 
    return false;

#ifdef SWCOMMAND_USESTRING
  while ( isalnum( _buff[i]) ) 
    _var.concat( _buff[i++] );
#endif

  // saltem espais 
  while ( _buff[i] == ' ' ) i++;

  // busquem operador '=' i determinem la comanda
  _type = SWCommandTypeGet;
  if ( _buff[i] == '=' ) _type = SWCommandTypeSet, i++;

  // determinarem el valor si cal
  if ( _type == SWCommandTypeSet )
  {
    char *endPtr;
    _value = strtol( _buff+i, &endPtr, 0 );
    i = endPtr - _buff;
  }

  // saltem espais 
  while ( _buff[i] == ' ' ) i++;

  // nomes tornem amb exit si estem al final
  return ( _buff[i] == '\0' );
}
