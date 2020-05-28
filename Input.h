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
 * Input.h
 * Defines interface for knobs used in the E-Vent. There are two implementations:
 *   - `Knob` the simplest of the two with a `read()` method that just returns the value
 *     currently dialed.
 *   - `SafeKnob`, with a `read()` method that returns the last value confirmed but also
 *     sets off an alarm if a changed value is not confirmed.
 */

#ifndef Input_h
#define Input_h

#include "Arduino.h"

#include "Alarms.h"
#include "Buttons.h"
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
  static const unsigned long kDisplayUpdatePeriod = 250;

public:
  Input(Display* displ, const display::DisplayKey& key, const T& resolution):
      displ_(displ),
      disp_key_(key),
      resolution_(resolution) {}

  // Setup during arduino setup()
  void begin(float (*read_fun)());

  // Update during arduino loop()
  virtual void update() = 0;

  // Read confirmed value to use for operation
  inline T read() const& { return set_value_; }

protected:
  float (*read_fun_)();
  Display* displ_;
  display::DisplayKey disp_key_;
  T resolution_;

  T set_value_;  // Dial value displayed and used for operation
  unsigned long last_display_update_time_ = 0;

  // Discretize value into closest multiple of resolution
  T discretize(const float& raw_value) const;

  void display(const T& value, const bool& blank = false);

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
  Knob(Display* displ, const display::DisplayKey& key, const T& resolution):
      Input<T>(displ, key, resolution) {}

  // Update during arduino loop()
  void update();
};

// Instantiate for types used
template class Knob<int>;
template class Knob<float>;


/**
 * SafeKnob
 * Safe knob that requires confirmation via 'confirm' button before setting a value.
 */
template <typename T>
class SafeKnob : public Input<T> {

  // Time to wait to sound alarm after knob is changed if not confirmed
  static const unsigned long kAlarmTime = 5 * 1000UL;

public:
  SafeKnob(Display* displ, const display::DisplayKey& key,
           const int& confirm_pin, AlarmManager* alarms, const T& resolution): 
      Input<T>(displ, key, resolution),
      confirm_button_(confirm_pin),
      alarms_(alarms),
      pulse_(1000, 0.5) {}

  // Setup during arduino setup()
  void begin(T (*read_fun)());

  // Update during arduino loop()
  void update();

private:
  buttons::DebouncedButton confirm_button_;
  AlarmManager* alarms_;
  utils::Pulse pulse_;
  T unconfirmed_value_;
  unsigned long time_changed_ = 0;
  bool confirmed_ = true;

  String getConfirmPrompt() const;
};

// Instantiate for types used
template class SafeKnob<int>;
template class SafeKnob<float>;


}


#endif

