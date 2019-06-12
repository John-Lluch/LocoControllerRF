
#ifndef SWLogic_H
#define SWLogic_H


class SWTimer
{
  public:
    SWTimer();

  public:
    bool value();
    void timer( bool set, long time );
    bool onDelay( bool set, long time );

  private:
    unsigned long  _start;
    long _time;
};


class SWKeep
{
  public:
    SWKeep() : _state(false) {};

  public:
    bool keep( bool set, bool reset );
    inline bool value() { return _state; }

  private:
    bool  _state;  
};

class SWDifu
{
  public: 
    SWDifu() : _state(false) {};

  public: 
    bool difU( bool set );
    inline bool value() { return _state; }

  private:
    bool _pre;
    bool _state;
};

class SWDifd
{
  public: 
    SWDifd() : _state(false) {};

  public: 
    bool difD( bool set );
    inline bool value() { return _state; }

  private:
    bool _pre;
    bool _state;
};



#endif
