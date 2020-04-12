#ifndef Input_h
#define Input_h

#include "Arduino.h"

#include "Display.h"
#include "Utilities.h"


namespace input {


using display::Display;


/**
 * Input
 * Defines the interface for different possible input methods, e.g. knobs.
 */
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

  T read() const {
    return set_value_;
  }

protected:
  T (*read_fun_)();
  Display* displ_;
  display::DisplayKey disp_key_;

  T set_value_;  // Dial value displayed and used for operation

  void display(const T& value, const bool& blank = false) {
    if (blank) {
      displ_->writeBlank(disp_key_);
    }
    else {
      displ_->write(disp_key_, value);
    }
  }
};


/**
 * Knob
 * Simplest knob interface that directly sets value set from pot.
 */
template <typename T>
class Knob : public Input<T> {
public:
  Knob(Display* displ, const display::DisplayKey& key): Input<T>(displ, key) {}

  void update() {
    this->set_value_ = read_fun_();
    this->display(read());  
  }
};


/**
 * SafeKnob
 * Safe knob that requires confirmation via 'confirm' button before setting a value.
 */
template <typename T>
class SafeKnob : public Input<T> {
public:
  SafeKnob(Display* displ, const display::DisplayKey& key, const int& confirm_pin): 
      Input<T>(displ, key),
      confirm_button_(confirm_pin),
      pulse_(1000, 0.5) {}

  void begin(T (*read_fun)()) {
    Input<T>::begin(read_fun);
    confirm_button_.begin();
  }

  void update() {
    unconfirmed_value_ = read_fun_();
    if (unconfirmed_value_ == this->set_value_) {  // TODO add tolerance
      this->display(read());
    }
    else if (confirm_button_.is_LOW()) {
      this->set_value_ = unconfirmed_value_;
      confirmed_ = true;
    }
    else {
      const unsigned long time_now = millis();
      if (confirmed_) {
        time_changed_ = time_now;
        confirmed_ = false;
      }
      this->display(unconfirmed_value_, !pulse_.read());
    }
  }

private:
  utilities::DebouncedButton confirm_button_;
  utilities::Pulse pulse_;
  T unconfirmed_value_;
  unsigned long time_changed_ = 0;
  bool confirmed_ = true;
};


}


#endif

