#ifndef Input_h
#define Input_h

#include "Arduino.h"

#include "Display.h"


namespace input {


using display::Display;


template <typename T>
class Input {
public:
  Input(Display* displ, const display::DisplayKey& key): 
      displ_(displ),
      disp_key_(key) {}

  void begin(T (*read_fun)()) {
    read_fun_ = read_fun;
  }

  virtual void update() = 0;

  virtual T read() const = 0;

protected:
  T (*read_fun_)();
  Display* displ_;
  display::DisplayKey disp_key_;
};


template <typename T>
class Knob : public Input<T> {
public:
  Knob(Display* displ, const display::DisplayKey& key): Input<T>(displ, key) {}

  void update() {
    value_ = read_fun_();
    this->displ_->write(this->disp_key_, read());  
  }

  T read() const {
    return value_;
  }
private:
  T value_;
};


template <typename T>
class KnobSafe : public Input<T> {
};


}


#endif

