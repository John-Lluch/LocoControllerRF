
#ifndef SWLogic_H
#define SWLogic_H


class SWTimer //Arduino-based timer
{
  public:
    SWTimer();

  public:
    //Returns true if t - t0 > timer, false otherwise. If no timer is running, returns false
    bool value();

    //Starts the n-miliseconds timer if set is true
    void timer( bool set, long time );

    //Combination of the above: Start a n-miliseconds timer and return true if t - t0 > timer
    bool onDelay( bool set, long time );

  private:
    unsigned long  _start;
    long _time;
};


class SWKeep //Simple state register
{
  public:
    SWKeep() : _state(false) {};

  public:
    //Save the state or reset the stored value, returns the stored value
    bool keep( bool set, bool reset );

    //Return the value of the register
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
