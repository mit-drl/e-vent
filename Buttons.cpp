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
 * Buttons.cpp
 */

#include "Buttons.h"


namespace buttons {


/// PressHoldButton ///

void PressHoldButton::update() {
  const unsigned long time_now = millis();
  const unsigned long time_since_last_low = time_now - last_low_time_;
  const bool press_lost = time_since_last_low > kDebounceDelay;
  if (digitalRead(pin_) == LOW) {
    if (!press_lost) {
      current_hold_time_ += time_since_last_low;
    }
    last_low_time_ = time_now;
  }
  else if (press_lost) {
    current_hold_time_ = 0;
  }
}


/// DebouncedButton ///

DebouncedButton::DebouncedButton(const int& pin): pin_(pin) {}

void DebouncedButton::begin() {
  pinMode(pin_, INPUT_PULLUP);
}

bool DebouncedButton::is_LOW() {
  int reading = digitalRead(pin_);

  bool low_value = false;
  const unsigned long time_now = millis();
  if (reading == LOW) {
    if ((time_now - last_low_time_) > kDebounceDelay) {
      low_value = true;
    }
    last_low_time_ = time_now;
  }
  return low_value;
}


}  // namespace buttons

