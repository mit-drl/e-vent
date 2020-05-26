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
 * Buttons.h
 * Defines methods for quickly and reliably checking different button actions
 * such as pressing and pressing and holding.
 */

#ifndef Buttons_h
#define Buttons_h

#include "Arduino.h"


namespace buttons {


/**
 * PressHoldButton
 * Button abstraction capable of detecting if a given button has been held
 * pressed for a given ammount of time.
 */
class PressHoldButton {

  const unsigned long kDebounceDelay = 100;

public:
  PressHoldButton(const int& pin, const unsigned long& hold_duration):
    pin_(pin),
    hold_duration_(hold_duration) {}

  // Setup during arduino setup()
  inline void begin() { pinMode(pin_, INPUT_PULLUP); }

  // Update during arduino loop()
  void update();

  // Check if button was just held for hold_duration ms.
  inline bool wasHeld() { return current_hold_time_ > hold_duration_; }

private:
  int pin_;
  unsigned long hold_duration_;
  unsigned long last_low_time_ = 0;
  unsigned long current_hold_time_ = 0;
};


/**
 * DebouncedButton
 * Represents a pullup button that filters out unintended LOW readings.
 */
class DebouncedButton {

  const unsigned long kDebounceDelay = 100;

public:
  DebouncedButton(const int& pin);

  // Setup during arduino setup()
  void begin();

  // Check if button is low
  bool is_LOW();

private:
  int pin_;
  unsigned long last_low_time_ = 0;
};


}  // namespace buttons


#endif

