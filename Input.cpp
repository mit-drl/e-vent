/**
 * MIT Emergency Ventilator Controller
 * 
 * MIT License:
 * 
 * Copyright (c) 2020 MIT
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

/**
 * Input.cpp
 */

#include "Input.h"


namespace input {


/// Input ///

template <typename T>
void Input<T>::begin(float (*read_fun)()) {
  read_fun_ = read_fun;
  set_value_ = discretize(read_fun_());
}

template <typename T>
T Input<T>::discretize(const float& raw_value) const {
  return round(raw_value / resolution_) * resolution_;
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
  this->set_value_ = discretize(read_fun_());
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
  if (abs(unconfirmed_value_ - this->set_value_) <= this->resolution_ / 2) {
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
