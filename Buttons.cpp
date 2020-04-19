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

