#include "Input.h"


namespace input {


/// Input ///

template <typename T>
void Input<T>::begin(T (*read_fun)()) {
  read_fun_ = read_fun;
  // don't require confirmation the on the very first reading
  set_value_ = read_fun_();
}

template <typename T>
void Input<T>::display(const T& value, const bool& blank) {
  // throttle display rate
  const unsigned long time_now = millis();
  if (time_now - last_display_update_time_ < kDisplayUpdatePeriod) return;
  last_display_update_time_ = time_now;

  if (blank) {
    displ_->writeBlank(disp_key_);
  }
  else {
    displ_->write(disp_key_, value);
  }
}


/// Knob ///

template <typename T>
void Knob<T>::update() {
  this->set_value_ = read_fun_();
  this->display(read());  
}


/// SafeKnob ///

template <typename T>
void SafeKnob<T>::begin(T (*read_fun)()) {
  Input<T>::begin(read_fun);
  confirm_button_.begin();
}

template <typename T>
void SafeKnob<T>::update() {
  unconfirmed_value_ = discretize(read_fun_());
  if (unconfirmed_value_ == this->set_value_) {
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

template <typename T>
String SafeKnob<T>::getConfirmPrompt() const {
  char buff[display::kWidth + 1];
  sprintf(buff, "Set %s(%s)->%s?", this->getLabel().c_str(), 
          this->toString(this->set_value_).c_str(), this->toString(unconfirmed_value_).c_str());
  String s(buff);
  while (s.length() < display::kWidth) s += " ";
  return s;
}


}
