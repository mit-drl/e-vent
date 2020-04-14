#include "Utilities.h"


namespace utilities {


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


}  // namespace utilities

