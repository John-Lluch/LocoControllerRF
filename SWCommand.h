

#ifndef SWCommand_H
#define SWCommand_H

 //#define SWCOMMAND_USESTRING
 
typedef enum
{
  SWCommandTypeNone = 0,
  SWCommandTypeSet = 1,
  SWCommandTypeGet = 2
} SWCommandType;

#define SWCommandBuffLength 100

class SWCommand
{
  public:
    SWCommand();

  public:
    bool available();
    void print( long value );
    inline SWCommandType type() { return _done ? _type : SWCommandTypeNone; }
    inline long value() { return _done ? _value : 0; }
    
  private:
    bool _parse();

  private:
    char _buff[SWCommandBuffLength];
    int _index;
    bool _done;
    long _value;
    SWCommandType _type;

    
 #ifdef SWCOMMAND_USESTRING
 public:
    inline String var() { return _done ? _var : "?"; }
 private:
    String _var;
 #else
 public:
    inline char var() { return _done ? _var : '?'; }
 private:
    char _var;
 #endif
};


#endif
