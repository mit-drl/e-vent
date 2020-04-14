#ifndef Input_h
#define Input_h

#include "Arduino.h"

#include "Alarms.h"
#include "Display.h"
#include "Utilities.h"


namespace input {


using alarms::AlarmManager;
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
    // don't require confirmation the on the very first reading
    set_value_ = read_fun_();
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

  inline String toString(const T& val) const { return displ_->toString(disp_key_, val); }

  inline String getLabel() const { return displ_->getLabel(disp_key_); }
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
 * TODO move definitions to cpp
 */
template <typename T>
class SafeKnob : public Input<T> {

  // Time to wait to sound alarm after knob is changed if not confirmed
  static const unsigned long kAlarmTime = 5 * 1000UL;

public:
  SafeKnob(Display* displ, const display::DisplayKey& key,
           const int& confirm_pin, AlarmManager* alarms): 
      Input<T>(displ, key),
      confirm_button_(confirm_pin),
      alarms_(alarms),
      pulse_(1000, 0.5) {}

  void begin(T (*read_fun)()) {
    Input<T>::begin(read_fun);
    confirm_button_.begin();
  }

  void update() {
    unconfirmed_value_ = read_fun_();
    if (!isSignificant(unconfirmed_value_ - this->set_value_)) {
      this->display(read());
      if (!confirmed_) {
        confirmed_ = true;
        alarms_->unconfirmedChange(false);
      }
    }
    else if (confirm_button_.is_LOW()) {
      this->set_value_ = unconfirmed_value_;
    }
    else {
      const unsigned long time_now = millis();
      if (confirmed_) {
        time_changed_ = time_now;
        confirmed_ = false;
      }
      this->display(unconfirmed_value_, !pulse_.read());
      if (time_now - time_changed_ > kAlarmTime) {
        alarms_->unconfirmedChange(true, getConfirmPrompt());
      }
    }
  }

private:
  utilities::DebouncedButton confirm_button_;
  AlarmManager* alarms_;
  utilities::Pulse pulse_;
  T unconfirmed_value_;
  unsigned long time_changed_ = 0;
  bool confirmed_ = true;

  String getConfirmPrompt() const {
    char buff[display::kWidth + 1];
    sprintf(buff, "Set %s(%s)->%s?", this->getLabel().c_str(), 
            this->toString(this->set_value_).c_str(), this->toString(unconfirmed_value_).c_str());
    String s(buff);
    while (s.length() < display::kWidth) s += " ";
    return s;
  }

  // Check if a setting change is big enough to request confirmation
  inline bool isSignificant(const T& change) { return abs(change) > 0.2; }
};


}


#endif

