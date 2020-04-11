#ifndef Input_h
#define Input_h

#include "Arduino.h"


namespace input {


template <typename T>
class Input {
public:
  void begin(T (*read_fun)()) {
    read_fun_ = read_fun;
  }

  virtual void update() = 0;

  virtual T read() const = 0;

private:
  T (*read_fun_)();
};


template <typename T>
class Knob : public Input<T> {
public:
  void update() {}

  T read() const {
    return read_fun_();
  }
};


template <typename T>
class KnobSafe : public Input<T> {
};


}


#endif

